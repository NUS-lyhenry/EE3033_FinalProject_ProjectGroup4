import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from ee3033_project.calibration import mounting_rotation
from ee3033_project.vision_geometry import FloorCamera

spec = importlib.util.spec_from_file_location('simulation_geometry', Path(__file__).parents[1]/'scripts/simulation_geometry.py')
world = importlib.util.module_from_spec(spec)
spec.loader.exec_module(world)


def test_lidar_wall_intersections_and_divider_occlusion():
    distances = world.raycast([0, 0], np.array([0., math.pi/2, math.pi, -math.pi/2]))
    assert distances == pytest.approx([1.4, 1.3, 1.2, 1.0])
    delta = world.TARGET[:2]
    angle = math.atan2(delta[1], delta[0])
    assert world.raycast([0, 0], [angle])[0] < np.linalg.norm(delta)
    assert world.clearance([.55, .5]) == 0
    assert world.clearance([0, 0]) > .14


def test_rendered_camera_ground_contact_matches_real_floor_projection():
    pose = [.95, .1, math.pi/2]
    translation, rotation = world.camera_pose(pose)
    pixel = world.project([world.TARGET], translation, rotation)[0]
    # Same camera-to-base optical transform as the real TF/Perception path.
    camera = FloorCamera(640, 480, world.K, np.zeros(5), mounting_rotation(world.PITCH, 0), world.CAMERA_OFFSET)
    base_point = camera.intersection(pixel)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    recovered = np.array([[c, -s], [s, c]])@base_point[:2]+pose[:2]
    assert recovered == pytest.approx(world.TARGET[:2], abs=1e-5)


def test_camera_target_is_occluded_and_absent_during_mapping():
    texture = np.full((80, 80, 3), (10, 240, 20), np.uint8)
    hidden = world.render_camera([0, 0, 0], texture, True)
    no_target = world.render_camera([0, 0, 0], texture, False)
    assert np.array_equal(hidden, no_target)
    pose = [.95, .1, math.pi/2]
    visible = world.render_camera(pose, texture, True)
    absent = world.render_camera(pose, texture, False)
    assert not np.array_equal(visible, absent)
