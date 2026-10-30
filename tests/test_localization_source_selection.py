"""Preserve useful information when correlated localization sources switch."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from ee3033_project.coverage import CoveragePlanner
from ee3033_project.target_acquisition import SingleTarget
from ee3033_project.target_estimate import MapMMSE, map_signature


def tracker():
    t = MapMMSE()
    t.set_map(np.zeros((80, 80), dtype=int), .05, (0, 0, 0))
    return t


def floor(stamp, sigma=.05, **changes):
    return dict(stamp=stamp, x=2., y=2., sigma=sigma, confidence=.9, label='cup', **changes)


def factor(stamp=11., covariance=None, **changes):
    covariance = np.diag([.0025, .16]) if covariance is None else np.asarray(covariance)
    value = dict(stamp=stamp, x=2., y=2., sigma=float(np.sqrt(np.linalg.eigvalsh(covariance).max())),
        covariance=covariance.tolist(), confidence=.9, label='cup', sites=2,
        source='bearing_intersection', source_frames=[stamp-.7+i*.1 for i in range(8)])
    value.update(changes)
    return value


def test_joint_factor_preserves_directional_covariance_without_video_error_reinflation():
    t = tracker()
    f = factor()
    assert t.update_factor(f, 11.)
    r = t.snapshot(11.)
    assert r['likelihood_covariance'] == pytest.approx(np.asarray(f['covariance']))
    assert r['likelihood_model'] == 'joint_bearing_factor'
    assert len(t.samples) == 1 and r['frames'] == 8
    assert r['covariance'][0][0] < .003
    assert r['covariance'][1][1] > .1


def test_factor_weaker_in_one_direction_cannot_erase_fresh_floor_evidence():
    t = tracker()
    for i in range(4):
        d = floor(10.+i*.2)
        assert t.update(d, d['stamp'])
    before = json.dumps(t.snapshot(10.6), sort_keys=True)
    samples = list(t.samples)
    f = factor(covariance=np.diag([.001, .16]))  # Better X, much worse Y.
    assert not t.update_factor(f, 11.)
    assert t.last_factor_rejection == 'weaker_covariance'
    assert t.factor is None and len(t.samples) == 4
    assert json.dumps(t.snapshot(10.6), sort_keys=True) == before
    assert all(a[0] == b[0] and np.array_equal(a[3], b[3]) for a, b in zip(t.samples, samples))


def test_stronger_floor_replaces_factor_without_counting_shared_evidence_twice():
    t = tracker()
    f = factor(covariance=np.array([[.0025, .0005], [.0005, .004]]))
    assert t.update_factor(f, 11.)
    assert not t.update(floor(11.1, sigma=.2), 11.1)
    assert t.factor is not None and t.last_update_rejection == 'weaker_covariance'
    assert t.update(floor(11.2, sigma=.01), 11.2)
    assert t.factor is None and len(t.samples) == 1
    assert not t.snapshot(11.2)['stable']
    for stamp in (11.3, 11.4, 11.5):
        assert t.update(floor(stamp, sigma=.01), stamp)
    assert t.snapshot(11.5)['stable'] and t.snapshot(11.5)['frames'] == 4


def test_precise_but_inconsistent_replacement_retains_the_original_source():
    t = tracker()
    assert t.update_factor(factor(covariance=np.eye(2)*.0025), 11.)
    before = t.result.copy()
    d = floor(11.1, sigma=.01)
    d['x'] = 3.
    assert not t.update(d, 11.1)
    assert t.last_update_rejection == 'inconsistent_measurement'
    assert t.result == before and t.factor is not None


def test_new_factor_replaces_stale_floor_even_when_it_is_less_precise():
    t = tracker()
    assert t.update(floor(10.), 10.)
    assert t.update_factor(factor(stamp=15.), 15.)
    assert t.snapshot(15.)['source'] == 'bearing_intersection'


def test_reported_estimate_cannot_be_changed_by_either_source():
    t = tracker()
    assert t.update_factor(factor(), 11.)
    t.freeze(t.snapshot(11.))
    before = t.snapshot(11.)
    assert not t.update(floor(11.2, sigma=.01), 11.2)
    assert not t.update_factor(factor(stamp=11.3, covariance=np.eye(2)*.0001), 11.3)
    assert t.snapshot(11.) == before


def test_default_acquisition_advances_without_requesting_lateral_ab():
    p = CoveragePlanner(np.zeros((80, 80), dtype=int), .05, (0, 0, 0))
    t = SingleTarget()
    t.candidate = dict(stamp=10., label='cup', confidence=.9, bearing=0., origin=[2., 2.])
    goal = t.observation_goal(p, (2., 2., 0.), force_baseline=True)
    assert not t.enable_ab_maneuver
    assert goal is not None and .04 < goal[0]-2. <= .100001
    assert goal[1] == pytest.approx(2.) and t.last_goal_action == 'approach'
    active = SingleTarget(enable_ab_maneuver=True)
    active.candidate = t.candidate.copy()
    lateral = active.observation_goal(p, (2., 2., 0.), force_baseline=True)
    assert lateral is not None and abs(lateral[1]-2.) >= .3


def test_default_approach_checks_every_part_of_the_allowed_connector():
    p = CoveragePlanner(np.zeros((100, 100), dtype=int), .025, (0, 0, 0))
    t = SingleTarget()
    t.candidate = dict(stamp=10., label='cup', confidence=.9, bearing=0., origin=[1., 1.])
    # Each possible endpoint is allowed; an intermediate strip is blocked.
    assert t.observation_goal(p, (1., 1., 0.), force_baseline=True,
        allowed=lambda x, y: not 1.015 < x < 1.04) is None
    assert not t.visited


def test_worse_second_site_does_not_end_localization_before_budget_or_precision():
    source = Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/mission.py'
    cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'on_estimate')
    namespace = dict(json=json, math=math, np=np, map_signature=map_signature)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    grid = np.zeros((40, 40), dtype=int)
    signature = map_signature(grid, .1, (0, 0, 0))
    endings = []
    node = NS(current_target_estimate=None, state='LOCALIZING', target=None,
        p=dict(map_frame='map', target_class='cup', max_target_age=2., max_target_variance=.01),
        started_stamp=10., get_clock=lambda: NS(now=lambda: NS(nanoseconds=12_000_000_000)),
        readiness_error=lambda: None, planner=NS(grid=grid, resolution=.1, origin=(0, 0, 0)),
        acquisition=NS(positions=[]), local_sigmas=[], target_detected=True,
        finish_localization=lambda reason: endings.append(reason),
        report_target=lambda *args: endings.append('report'))
    for sites, sigma in ((1, .2), (2, .38)):
        estimate = dict(method='map-constrained MMSE', stable=True, frame_id='map',
            label='cup', mission_stamp=10., stamp=11.9, x=1., y=1., sigma=sigma,
            variance=sigma*sigma, covariance=(np.eye(2)*sigma*sigma).tolist(),
            map_signature=signature, sites=sites)
        namespace['on_estimate'](node, NS(data=json.dumps(estimate)))
    assert not endings and node.best_target['sigma'] == .38
