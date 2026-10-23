import copy
import math
from types import SimpleNamespace

import numpy as np
import pytest

from ee3033_project.calibration import (camera_model, fit_profile, floor_point,
    load_profile, mounting_rotation, predicted_points, save_profile, validate_holdout,
    validate_sample)
from ee3033_project.calibration_motion import CalibrationMotion
from ee3033_project.assessment import StageProcess


SETUP = dict(bottle_height=.25, bottle_diameter=.065, camera_height=.21,
             camera_x=.07, camera_y=-.015)
CAMERA = camera_model(640, 480, 'usb_camera', [520., 0., 320., 0., 510., 240., 0., 0., 1.], [0.]*5)
TRUE = [555., 548., .13, .035]
POSITIONS = [(.65, -.09), (.8, .1), (1., -.2), (1.15, .22), (1.35, -.18), (1.5, .12), (.95, 0.)]


def sample(distance, lateral, noise=0., parameters=TRUE, camera=CAMERA):
    s = dict(distance=distance, lateral=lateral)
    p = predicted_points(parameters, SETUP, camera, s)
    # Detector boxes approximate an upright bottle's centreline and silhouette.
    box = [p[2:, 0].min(), p[1, 1], p[2:, 0].max(), p[0, 1]]
    if noise:
        box = np.array(box)+np.random.default_rng(int(distance*1000)).normal(0, noise, 4)
    return {**s, 'box': list(box)}


def profile():
    return fit_profile(SETUP, CAMERA, [sample(d, y, .15) for d, y in POSITIONS])


def test_fit_recovers_effective_camera_geometry_and_unseen_floor_positions():
    p = profile()
    np.testing.assert_allclose([p['camera']['k'][0], p['camera']['k'][4]], TRUE[:2], atol=10)
    assert abs(p['pitch']-TRUE[2]) < .01
    assert abs(p['yaw']-TRUE[3]) < .01
    for d, y in [(.9, -.08), (1.25, .09)]:
        result = validate_holdout(p, sample(d, y))
        assert result['passed'] and result['error_m'] < .025
        p['validation'].append(result)


def test_focal_fit_does_not_estimate_or_overwrite_lens_distortion():
    c = camera_model(640, 480, 'usb_camera', CAMERA['k'], [.08, -.03, .001, -.001, 0.])
    p = fit_profile(SETUP, c, [sample(d, y, camera=c) for d, y in POSITIONS])
    assert p['camera']['d'] == c['d']
    assert p['camera']['k'][2:3] == c['k'][2:3]
    result = validate_holdout(p, sample(1.25, .09, camera=c))
    assert result['error_m'] < .03


@pytest.mark.parametrize('positions,message', [
    (POSITIONS[:5], 'six'),
    ([(1.+i*.02, y) for i, (_, y) in enumerate(POSITIONS)], 'span'),
    ([(d, 0.) for d, _ in POSITIONS], 'offsets'),
])
def test_insufficient_measurements_are_rejected(positions, message):
    with pytest.raises(ValueError, match=message):
        fit_profile(SETUP, CAMERA, [sample(d, y) for d, y in positions])


def test_bad_measurement_fit_is_rejected():
    samples = [sample(d, y) for d, y in POSITIONS]
    samples[2]['box'][1] -= 35
    with pytest.raises(ValueError, match='rejected'):
        fit_profile(SETUP, CAMERA, samples)


@pytest.mark.parametrize('box', [[0, 20, 50, 100], [10, 10, 14, 100], [10, 10, 100, 481], [10, 50, 100, 40], [float('nan'), 10, 100, 100]])
def test_clipped_or_invalid_bottle_boxes_are_rejected(box):
    with pytest.raises(ValueError):
        validate_sample(dict(distance=1., lateral=0., box=box), CAMERA)


def test_save_requires_independent_passing_holdouts_and_preserves_previous_profiles(tmp_path):
    p = profile()
    with pytest.raises(ValueError, match='Two independent'):
        save_profile(p, tmp_path)
    with pytest.raises(ValueError, match='independently'):
        validate_holdout(p, sample(*POSITIONS[0]))
    with pytest.raises(ValueError, match='range'):
        validate_holdout(p, sample(2., 0.))
    for d, y in [(.9, -.08), (1.25, .09)]:
        p['validation'].append(validate_holdout(p, sample(d, y)))
    one, two = save_profile(p, tmp_path), save_profile(p, tmp_path)
    assert one != two
    loaded = load_profile(one)
    assert loaded['validated'] is True
    assert loaded['training'] == p['training']
    assert len(list(tmp_path.glob('*.json'))) == 2
    p['validation'][0]['passed'] = False
    with pytest.raises(ValueError):
        save_profile(p, tmp_path)


def test_validation_failure_cannot_be_hidden_by_adding_two_successes(tmp_path):
    p = profile()
    bad = sample(.9, -.08)
    bad['box'][3] += 40
    p['validation'].append(validate_holdout(p, bad))
    assert not p['validation'][0]['passed']
    for d, y in [(1.25, .09), (1.1, -.1)]:
        p['validation'].append(validate_holdout(p, sample(d, y)))
    with pytest.raises(ValueError):
        save_profile(p, tmp_path)


def test_profile_rotation_and_mount_translation_use_ros_axes():
    np.testing.assert_allclose(mounting_rotation(0., 0.) @ [0, 0, 1], [1, 0, 0])
    np.testing.assert_allclose(mounting_rotation(0., 0.) @ [1, 0, 0], [0, -1, 0])
    p = profile()
    pixel = predicted_points(TRUE, SETUP, CAMERA, dict(distance=1., lateral=.1))[0]
    point = floor_point(p, pixel)
    np.testing.assert_allclose(point, [1.+SETUP['camera_x'], .1+SETUP['camera_y'], 0.], atol=.02)
    with pytest.raises(ValueError, match='horizon'):
        floor_point(p, [320, 0])


def test_missing_intrinsics_are_explicit_and_unsupported_lenses_are_rejected():
    assert 'assumed' in camera_model(640, 480, 'camera')['source']
    with pytest.raises(ValueError, match='lens models'):
        camera_model(640, 480, 'camera', CAMERA['k'], [0.]*4, 'equidistant')


def moving(mode='forward'):
    m = CalibrationMotion()
    m.start(mode, 10., [0, 0, 0], m.generation)
    return m


def test_distance_move_stops_at_ten_centimetres_without_odometry_as_calibration_truth():
    m = moving()
    assert m.tick(10.1, [.01, 0, 0], dict(error=0.), True) == (.035, 0.)
    assert m.tick(10.2, [.101, 0, 0], dict(error=0.), True) == (0., 0.)
    assert m.mode == 'idle'


@pytest.mark.parametrize('now,pose,target,healthy', [
    (10.7, [0, 0, 0], dict(error=0.), True),
    (10.1, [0, 0, 0], None, True),
    (10.1, [0, 0, 0], dict(error=0.), False),
    (10.1, [0, 0, .46], dict(error=0.), True),
])
def test_motion_stops_on_heartbeat_target_sensor_or_turn_limits(now, pose, target, healthy):
    m = moving()
    assert m.tick(now, pose, target, healthy) == (0., 0.)
    assert m.mode == 'idle'


def test_movement_time_limit_stops_even_if_obstacle_gate_prevents_translation():
    m = moving()
    m.renew(15., m.generation)
    assert m.tick(15.1, [0, 0, 0], dict(error=0.), True) == (0., 0.)
    assert m.mode == 'idle'


def test_center_turn_direction_and_speed_limits():
    m = moving('center')
    assert m.tick(10.1, [0, 0, 0], dict(error=.5), True) == (0., -.15)
    assert m.tick(10.2, [0, 0, 0], dict(error=-.1), True)[1] > 0
    for i in range(6):
        m.renew(10.3+i*.05, m.generation)
        assert m.tick(10.3+i*.05, [0, 0, 0], dict(error=0.), True) == (0., 0.)
    assert m.mode == 'idle'


def test_stop_invalidates_queued_motion_and_old_heartbeats():
    m = moving()
    previous = m.generation
    m.stop()
    with pytest.raises(ValueError, match='context'):
        m.start('forward', 11., [0, 0, 0], previous)
    m.start('backward', 11., [0, 0, 0], m.generation)
    m.renew(11.5, previous)
    assert m.lease == 11.
    assert m.tick(11.7, [0, 0, 0], dict(error=0.), True) == (0., 0.)


def test_assessment_forwards_camera_profile_only_to_mission_as_a_single_argument(tmp_path):
    calls = []
    def factory(args, **kwargs):
        calls.append(args)
        return SimpleNamespace(poll=lambda: None)
    manager = StageProcess(tmp_path, calibration_file='/tmp/camera profile.json', popen=factory)
    assert manager.launch('mission', '/tmp/map.yaml', {}, tmp_path)
    assert 'calibration_file:=/tmp/camera profile.json' in calls[-1]
    manager = StageProcess(tmp_path, calibration_file='/tmp/camera profile.json', popen=factory)
    assert manager.launch('mapping', map_directory=tmp_path)
    assert not any(arg.startswith('calibration_file:=') for arg in calls[-1])


def test_simulated_profiles_cannot_be_used_by_the_robot(tmp_path):
    p = profile()
    p['simulated'] = True
    for d, y in [(.9, -.08), (1.25, .09)]:
        p['validation'].append(validate_holdout(p, sample(d, y)))
    path = save_profile(p, tmp_path)
    with pytest.raises(ValueError, match='validated measured-object'):
        load_profile(path)
