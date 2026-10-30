"""Comparison runner safety boundaries and reference-frame arithmetic."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location('localization_comparison', ROOT/'scripts/compare_localization_modes.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.mark.parametrize('url', ['https://127.0.0.1:8794', 'http://example.com:8794',
    'http://user:pass@localhost:8794', 'http://localhost:8794/something',
    'http://localhost:8794?token=secret', 'http://localhost:8794/#/workflow'])
def test_runner_refuses_remote_or_ambiguous_origins(url):
    with pytest.raises(ValueError):
        runner.local_base(url)


@pytest.mark.parametrize('state', [{}, {'simulated': True},
    {'simulated': False, 'preview_backend': 'main-ros'},
    {'simulated': 'true', 'preview_backend': 'main-ros'},
    {'simulated': True, 'preview_backend': 'static-preview'}])
def test_runner_requires_actual_ros_simulation_marker(state):
    with pytest.raises(RuntimeError):
        runner.require_simulated(state)


@pytest.mark.parametrize('path', ['manual', 'navigate', 'simulation_scenario', 'target_estimate'])
def test_runner_control_surface_excludes_manual_goals_and_target_injection(tmp_path, path):
    api = runner.Api('http://localhost:8794', tmp_path)
    api.state = lambda: pytest.fail('Unapproved paths must fail before reading or sending anything')
    with pytest.raises(ValueError):
        api.post(path, {'x': 1., 'y': 2.})


def test_both_simple_scenes_pass_original_world_validation(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT/'scripts'))
    from simulation_scenarios import validate_scenario
    scenes = runner.scenarios()
    assert set(scenes) == {'normal', 'far_edge'}
    for scene in scenes.values():
        result = validate_scenario(scene)
        assert len(result['walls']) == 4
        assert result['start'] == [0., 0., 0.]
        assert result['bounds'] == [-.7, -.9, 1.3, .9]
    assert scenes['normal']['target'] == [.65, 0., 0.]
    assert scenes['far_edge']['target'] == [1., .35, 0.]


def test_ab_override_changes_only_its_explicit_parameter():
    config = 'mission:\n  enable_ab_maneuver: false # Opt-in only\n  speed: 0.12\n  precision: 0.1\n'
    enabled = runner.switch_ab(config, True)
    assert enabled == config.replace('false #', 'true #')
    assert runner.switch_ab(enabled, False) == config
    with pytest.raises(ValueError):
        runner.switch_ab('mission:\n  speed: 0.12\n', True)


def test_reference_transform_includes_map_rotation_and_translation():
    assert runner.map_point([1., 2., math.pi/2], [.5, -.25, 0.]) == pytest.approx([1.25, 2.5])


@pytest.mark.parametrize('age, expected', [(0.1, .05), (2., None)])
def test_reference_error_uses_only_fresh_recorded_tf(tmp_path, age, expected):
    scene = {'target': [.5, -.25, 0.]}
    row = dict(captured_wall=100., ros_time=100., transform_stamp=100.-age,
               transform=[1., 2., math.pi/2])
    runner.save(tmp_path/'map-to-odom-tf.jsonl', row)
    # A JSON-lines observer emits each record on one line.
    import json
    (tmp_path/'map-to-odom-tf.jsonl').write_text(json.dumps(row)+'\n')
    api = SimpleNamespace(first_report_wall=100.,
                          first_report={'mission': {'target': {'x': 1.3, 'y': 2.5}}})
    result = runner.truth_metrics(tmp_path, scene, api, {})
    if expected is None:
        assert result['target_error_metres'] is None
        assert 'stale' in result['truth_unavailable_reason']
    else:
        assert result['target_error_metres'] == pytest.approx(expected)


def test_state_artifacts_remove_nested_session_tokens():
    value = dict(token='secret', mission=dict(target={'x': 1.}, token='nested'))
    assert runner.sanitized(value) == {'mission': {'target': {'x': 1.}}}
