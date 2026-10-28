import math
import numpy as np
import pytest

from ee3033_project.attitude_geometry import (AttitudeBuffer, ATTITUDE_TOPIC,
    compensated_mount, default_config, mount_fingerprint, reference_valid,
    tilt_rotation, validate_config)
from ee3033_project.calibration import mounting_rotation
from ee3033_project.vision_geometry import FloorCamera


def test_reference_pose_preserves_measured_mount_and_pivot_lever():
    rotation = mounting_rotation(math.radians(28.6), 0.)
    camera, pivot = np.array([-.01, 0., .33]), [0., 0., .033]
    r, t = compensated_mount(rotation, camera, pivot, (.01, .04), (.01, .04))
    assert r == pytest.approx(rotation)
    assert t == pytest.approx(camera)
    r, t = compensated_mount(rotation, camera, pivot, (0., 0.), (0., .1))
    assert t == pytest.approx(np.array(pivot)+tilt_rotation(0., .1)@[-.01, 0., .297])
    assert t[0] > 0. and t[2] != pytest.approx(.33)


@pytest.mark.parametrize('roll,pitch', [(0., .12), (.15, 0.), (-.1, -.08), (.08, .09)])
def test_tilted_camera_recovers_same_physical_floor_contact(roll, pitch):
    k = np.array([[350., 0., 320.], [0., 350., 240.], [0., 0., 1.]])
    rotation, translation = compensated_mount(mounting_rotation(.499, 0.), [-.01, 0., .33],
                                               [0., 0., .033], (0., 0.), (roll, pitch))
    model = FloorCamera(640, 480, k, np.zeros(5), rotation, translation)
    point = np.array([.65, .10, 0.])
    ray = rotation.T@(point-translation)
    pixel = (k@ray)[:2]/ray[2]
    assert model.intersection(pixel) == pytest.approx(point, abs=1e-10)


def test_capture_interpolation_ignores_yaw_and_uses_exposure_not_latest():
    history = AttitudeBuffer()
    history.add(10., .0, .1)
    history.add(10.08, .08, .3)
    sample = history.at(10.04)
    assert (sample['roll'], sample['pitch']) == pytest.approx((.04, .2))
    assert sample['rate'] > 0
    assert 'yaw' not in sample
    with pytest.raises(ValueError, match='unbracketed'):
        history.at(10.09)


def test_stale_duplicate_future_and_gapped_sources_fail_closed():
    history = AttitudeBuffer()
    history.add(10., 0., 0.)
    assert not history.add(10., .1, .1)
    with pytest.raises(ValueError, match='stale'):
        history.latest(10.21)
    with pytest.raises(ValueError, match='stale'):
        history.latest(9.99)
    history.add(10.3, 0., 0.)
    with pytest.raises(ValueError, match='gap'):
        history.at(10.1)
    assert not history.add(10.4, math.nan, 0.)
    assert not history.samples


def test_wrong_frame_and_clock_reset_clear_history():
    history = AttitudeBuffer()
    history.add(10., 0., 0.)
    assert not history.add(9., 0., 0.)
    assert not history.samples
    assert not history.add(11., 0., 0., 'gyro_link')


def test_reference_needs_complete_stable_fresh_window_not_message_count():
    history = AttitudeBuffer()
    for stamp in np.arange(10., 11.101, .05):
        history.add(float(stamp), .02, .08)
    ref = history.reference(11.12)
    assert (ref['roll'], ref['pitch']) == pytest.approx((.02, .08))
    assert ref['samples'] >= 20
    with pytest.raises(ValueError, match='stale'):
        history.reference(12.)
    history.add(11.15, .2, .08)
    with pytest.raises(ValueError, match='stable'):
        history.reference(11.16)


def test_measured_ten_hertz_jitter_allows_reference_and_exposure_interpolation():
    history = AttitudeBuffer()
    stamp = 10.
    history.add(stamp, .02, .08)
    for index in range(10):
        stamp += .12051 if index == 4 else .10018
        history.add(stamp, .02, .08)
    ref = history.reference(stamp+.05)
    assert ref['samples'] == 11
    assert ref['pitch'] == pytest.approx(.08)
    assert history.at(10.+4*.10018+.06)['pitch'] == pytest.approx(.08)
    history.add(stamp+.151, .02, .08)
    with pytest.raises(ValueError, match='capture_gap'):
        history.at(stamp+.07)
    with pytest.raises(ValueError, match='missing-sample gap'):
        history.reference(stamp+.16)


def test_config_requires_fresh_only_topic_and_mount_change_invalidates_reference():
    config = default_config()
    assert validate_config(config)['mode'] == 'fixed'
    with pytest.raises(ValueError, match='fresh-packet'):
        validate_config({**config, 'topic': '/robot_euler'})
    config.update(pivot=[0., 0., .033], reference={'mount_fingerprint': mount_fingerprint({'camera_height': .33})})
    assert reference_valid(config, {'camera_height': .33})
    assert not reference_valid(config, {'camera_height': .34})
    assert not reference_valid({**config, 'pivot': None}, {'camera_height': .33})


def test_attitude_uncertainty_rotates_origin_and_survives_camera_metadata():
    model = FloorCamera(640, 480, np.array([[350., 0., 320.], [0., 350., 240.], [0., 0., 1.]]),
                        np.zeros(5), mounting_rotation(.499, 0.), [-.01, 0., .33],
                        attitude_pivot=[0., 0., .033], roll_sigma=.035)
    point, sigma, distance = model.project_box([280., 160., 330., 260.])
    assert np.isfinite(point).all() and sigma > 0 and distance > 0
    copy = FloorCamera.from_metadata(model.metadata())
    assert copy.attitude_pivot == [0., 0., .033]
    assert copy.project_box([280., 160., 330., 260.])[1] == pytest.approx(sigma)
