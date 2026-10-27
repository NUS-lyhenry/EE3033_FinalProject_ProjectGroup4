import math
from types import SimpleNamespace
import numpy as np
import pytest
from ee3033_project.core import (
    Observation, TargetTracker, angle_difference, associate_lidar,
    ground_intersection, quaternion_yaw, rotation_matrix,
)


def obs(t, x=1.0, y=2.0, confidence=0.9, label='bottle'):
    return Observation(t, x, y, confidence, label)


def test_confirmation_and_robust_coordinate_estimate():
    tracker = TargetTracker(frames=4)
    for i, x in enumerate([1.0, 1.02, 0.99]):
        assert tracker.update(obs(i*0.2, x)) is None
    estimate = tracker.update(obs(0.6, 1.01))
    assert estimate['x'] == pytest.approx(1.005)
    assert estimate['y'] == 2.0
    assert estimate['variance'] >= 0.08**2
    assert estimate['frames'] == 4


def test_stable_high_uncertainty_estimate_preserves_correlated_sensor_floor():
    tracker = TargetTracker(frames=4)
    for i in range(4):
        estimate = tracker.update(Observation(i*.2, 1., 2., .9, 'cup', .25))
    assert estimate is not None
    assert estimate['variance'] == pytest.approx(.25**2)


@pytest.mark.parametrize('break_value', [None, obs(0.6, confidence=0.2),
    obs(0.6, 3.0), obs(0.6, label='cup'), obs(4.0), obs(0.1)])
def test_miss_or_association_change_breaks_confirmation(break_value):
    tracker = TargetTracker(frames=4)
    for i in range(3):
        assert tracker.update(obs(i*0.2)) is None
    assert tracker.update(break_value) is None


def test_duplicate_frame_cannot_confirm():
    tracker = TargetTracker(frames=4)
    for _ in range(10):
        assert tracker.update(obs(1.0)) is None
    assert len(tracker.samples) == 1


def test_spatial_inconsistency_cannot_report():
    tracker = TargetTracker(frames=4, association_radius=0.4, max_spread=0.03)
    for i, x in enumerate([1.0, 1.1, 0.9, 1.1, 0.95]):
        assert tracker.update(obs(i*0.2, x)) is None


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_nonfinite_observations_rejected(bad):
    tracker = TargetTracker()
    assert tracker.update(obs(1, bad)) is None
    assert not tracker.samples


def test_rotation_and_angle_wrap():
    r = rotation_matrix(0, 0, math.sin(math.pi/4), math.cos(math.pi/4))
    assert r @ np.array([1., 0, 0]) == pytest.approx([0, 1, 0])
    assert angle_difference(-math.pi+0.1, math.pi-0.1) == pytest.approx(0.2)
    q = SimpleNamespace(x=0, y=0, z=math.sin(0.3), w=math.cos(0.3))
    assert quaternion_yaw(q) == pytest.approx(0.6)
    with pytest.raises(ValueError):
        rotation_matrix(0, 0, 0, 0)


def test_floor_projection_rejects_horizon_and_points_behind_camera():
    assert ground_intersection([1, 0, 0], np.eye(3), [0, 0, 0.3]) is None
    assert ground_intersection([1, 0, 1], np.eye(3), [0, 0, 0.3]) is None
    assert ground_intersection([1, 0, -0.3], np.eye(3), [0, 0, 0.3]) == pytest.approx([1,0,0])
    assert ground_intersection([10, 0, -0.3], np.eye(3), [0,0,0.3], max_range=3) is None


def test_lidar_bbox_association_rejects_vertical_mismatch_and_background():
    k = np.array([[300., 0, 320], [0,300,240], [0,0,1]])
    camera = np.array([[-0.02, 0.0, 1.0], [0.02,0,1.0],
                       [-0.04, 0.0, 2.0], [0.04,0,2.0]])
    laser = np.array([[1,-0.02,0], [1,0.02,0], [2,-0.04,0], [2,0.04,0]])
    indices = np.array([1,2,10,11])
    point = associate_lidar(camera, laser, indices, [290,200,350,280], k, np.zeros(5))
    assert point == pytest.approx([1,0,0])
    assert associate_lidar(camera, laser, indices, [290,300,350,400], k, np.zeros(5)) is None
    assert associate_lidar(camera[:1], laser[:1], indices[:1], [290,200,350,280],
                           k, np.zeros(5)) is None


def test_lidar_association_does_not_take_points_behind_camera():
    k = np.array([[300.,0,320],[0,300,240],[0,0,1]])
    camera = np.array([[0.,0,-1], [0.01,0,-1]])
    assert associate_lidar(camera, camera, np.array([0,1]), [0,0,640,480],
                           k, np.zeros(5)) is None
