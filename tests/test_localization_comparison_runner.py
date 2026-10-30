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
    # A report-time TF alone cannot establish a fixed map/world reference.
    assert result['target_error_metres'] is None
    if expected is None:
        assert result['conditional_geometry_error_metres'] is None
        assert 'stale' in result['conditional_truth_unavailable_reason']
    else:
        assert result['conditional_geometry_error_metres'] == pytest.approx(expected)


def test_state_artifacts_remove_nested_session_tokens():
    value = dict(token='secret', mission=dict(target={'x': 1.}, token='nested'))
    assert runner.sanitized(value) == {'mission': {'target': {'x': 1.}}}


def test_parameter_diagnostic_wait_records_discovery_failure_before_success(monkeypatch, tmp_path):
    import subprocess
    results = [subprocess.CalledProcessError(1, ['ros2'], output='', stderr='Node not found'),
               SimpleNamespace(stdout='Double value is: 0.05\n', stderr='')]
    def call(*args, **kwargs):
        result = results.pop(0)
        if isinstance(result, Exception):
            raise result
        return result
    monkeypatch.setattr(runner, 'docker', call)
    monkeypatch.setattr(runner.time, 'sleep', lambda _: None)
    assert runner.parameter(None, 'container', '/slam_toolbox', 'resolution', tmp_path) == 'Double value is: 0.05'
    text = (tmp_path/'parameter-readbacks.jsonl').read_text()
    assert 'Node not found' in text and '0.05' in text


def test_model_label_readiness_populates_cache_before_mission_setup(monkeypatch, tmp_path):
    replies = [dict(ready=False, labels=[], error='Reading model labels'), dict(ready=True, labels=['cup'])]
    api = SimpleNamespace(output=tmp_path, state=lambda: dict(simulated=True, preview_backend='main-ros'),
                          get=lambda path: replies.pop(0) if path == 'labels' else pytest.fail(path))
    monkeypatch.setattr(runner.time, 'sleep', lambda _: None)
    assert runner.wait_labels(api) == dict(ready=True, labels=['cup'])
    assert len((tmp_path/'model-labels.jsonl').read_text().splitlines()) == 2


def gauge_fixture(tmp_path, change=None, legacy=False):
    import json
    scene = {'target': [.5, -.25, 0.], 'start': [0., 0., 0.]}
    initial_tf = dict(captured_wall=10., ros_time=10., transform_stamp=9.9,
                      transform=[1., 2., math.pi/2])
    report_tf = dict(captured_wall=100., ros_time=100., transform_stamp=99.9,
                     transform=[1.5, 2., math.pi/2])
    state = dict(captured_wall=10., state=dict(phase='mapping', mission={'state': 'WAITING'},
        simulation_motion=dict(ground_truth=[0., 0., 0.], command=[0., 0.], actual_velocity=[0., 0.])))
    controls = [dict(path='start', captured_wall=20.1, request_started_wall=20.)]
    if change == 'wrong_phase':
        state['state']['phase'] = 'mission'
    elif change == 'wrong_pose':
        state['state']['simulation_motion']['ground_truth'][0] = .1
    elif change == 'nonzero_command':
        state['state']['simulation_motion']['command'][0] = .1
    elif change == 'nonzero_velocity':
        state['state']['simulation_motion']['actual_velocity'][0] = .1
    elif change == 'after_start':
        initial_tf['captured_wall'] = state['captured_wall'] = 21.
    elif change == 'no_start_evidence':
        controls = []
    states = [state]
    if legacy:
        controls[0].pop('request_started_wall')
        later = json.loads(json.dumps(state))
        later['captured_wall'] = 19.
        states.append(later)
    for name, rows in [('map-to-odom-tf.jsonl', [initial_tf, report_tf]),
                       ('states.jsonl', states), ('controls.jsonl', controls)]:
        (tmp_path/name).write_text(''.join(json.dumps(row)+'\n' for row in rows))
    api = SimpleNamespace(first_report_wall=100.,
        first_report={'mission': {'target': {'x': 1.75, 'y': 2.5}}})
    return scene, api


def test_primary_map_error_keeps_localization_drift_when_dynamic_alignment_cancels_it(tmp_path):
    scene, api = gauge_fixture(tmp_path)
    result = runner.truth_metrics(tmp_path, scene, api, {})
    assert result['target_error_metres'] == pytest.approx(.5)
    assert result['conditional_geometry_error_metres'] == pytest.approx(0.)
    assert result['map_truth'] == pytest.approx([1.25, 2.5])
    assert result['frozen_initial_gauge']['mission_state'] == 'WAITING'


@pytest.mark.parametrize('change', ['wrong_phase', 'wrong_pose', 'nonzero_command',
                                  'nonzero_velocity', 'after_start', 'no_start_evidence'])
def test_primary_accuracy_unavailable_without_proven_stationary_pre_start_gauge(tmp_path, change):
    scene, api = gauge_fixture(tmp_path, change=change)
    result = runner.truth_metrics(tmp_path, scene, api, {})
    assert result['target_error_metres'] is None
    assert result['conditional_geometry_error_metres'] == pytest.approx(0.)
    assert 'not a fallback' in result['truth_unavailable_reason']


def test_legacy_serial_pre_start_snapshot_preserves_frozen_gauge_evidence(tmp_path):
    scene, api = gauge_fixture(tmp_path, legacy=True)
    result = runner.truth_metrics(tmp_path, scene, api, {})
    assert result['target_error_metres'] == pytest.approx(.5)
    assert 'legacy' in result['frozen_initial_gauge']['evidence_kind']
