"""Mission projection compatibility and calibration launch isolation.

Run ONLY on an isolated domain. No real camera, weights, or robot are used.
"""
import importlib.util
import math
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Image, LaserScan

from ee3033_project.calibration import camera_model, fit_profile, predicted_points
from ee3033_project.perception import Perception
from ee3033_project.ros_utils import set_yaw


SETUP = dict(bottle_height=.25, bottle_diameter=.065, camera_height=.21, camera_x=.07, camera_y=-.015)
CAMERA = camera_model(640, 480, 'usb_camera', [520., 0., 320., 0., 510., 240., 0., 0., 1.], [0.]*5)
TRUE = [555., 548., .13, .035]
POSITIONS = [(.65, -.09), (.8, .1), (1., -.2), (1.15, .22), (1.35, -.18), (1.5, .12)]


def sample(d, y):
    s = dict(distance=d, lateral=y)
    points = predicted_points(TRUE, SETUP, CAMERA, s)
    return {**s, 'box': [float(points[2:, 0].min()), float(points[1, 1]),
                         float(points[2:, 0].max()), float(points[0, 1])]}


class MissionProjection(unittest.TestCase):
    def image(self, frame=CAMERA['frame_id']):
        image = Image()
        image.width, image.height = 640, 480
        image.header.stamp.sec, image.header.frame_id = 1, frame
        return image

    def make_perception(self):
        profile = fit_profile(SETUP, CAMERA, [sample(d, y) for d, y in POSITIONS])
        t = TransformStamped()
        t.transform.translation.x, t.transform.translation.y = 2., 3.
        set_yaw(t.transform.rotation, math.pi/2)
        base_from_laser = TransformStamped()
        base_from_laser.transform.translation.z, base_from_laser.transform.rotation.w = .1, 1.
        return SimpleNamespace(calibration=profile, p=dict(localization_method='ground_plane',
            camera_optical_frame='unused', map_frame='map', max_range=3., max_scan_delta=.25,
            min_lidar_points=2, cluster_gap=.12, max_cluster_width=.5),
            buffer=SimpleNamespace(lookup_transform=lambda *args: t,
                                   lookup_transform_full=lambda *args: base_from_laser))

    def test_saved_geometry_projects_to_map_at_image_time_without_camera_info(self):
        node = self.make_perception()
        image = self.image()
        point = Perception.locate(node, sample(1., 0.)['box'], image, None, [])
        np.testing.assert_allclose(point, [2.-SETUP['camera_y'], 4.+SETUP['camera_x'], 0.], atol=.025)
        image.width = 320
        with self.assertRaises(ValueError):
            Perception.locate(node, sample(1., 0.)['box'], image, None, [])

    def test_saved_geometry_rejects_wrong_camera_frame_and_out_of_range_floor(self):
        node = self.make_perception()
        image = self.image('wrong_camera')
        with self.assertRaises(ValueError):
            Perception.locate(node, sample(1., 0.)['box'], image, None, [])
        image.header.frame_id = CAMERA['frame_id']
        self.assertIsNone(Perception.locate(node, sample(2., 0.)['box'], image, None, []))

    def test_lidar_association_uses_corrected_intrinsics_and_measured_mount(self):
        node = self.make_perception()
        node.p['localization_method'] = 'lidar'
        image = self.image()
        scan = LaserScan()
        scan.header.stamp.sec, scan.header.frame_id = 1, 'laser'
        scan.angle_min, scan.angle_increment = -.025, .005
        scan.range_min, scan.range_max, scan.ranges = .05, 5., [1.]*11
        # The synthetic scan plane intersects the bottle; the saved camera's
        # measured x offset explains its position relative to the scan origin.
        point = Perception.locate(node, sample(.93, .015)['box'], image, None, [scan])
        self.assertIsNotNone(point)
        np.testing.assert_allclose(point[:2], [2., 4.], atol=.025)


class CalibrationLaunch(unittest.TestCase):
    def module(self):
        path = Path(__file__).resolve().parents[1]/'src/ee3033_project/launch/calibration.launch.py'
        spec = importlib.util.spec_from_file_location('calibration_launch_fixture', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def context(self):
        from launch import LaunchContext
        context = LaunchContext()
        context.launch_configurations.update(base='false', lidar='false', camera='false', map='',
            seed_pose='false', initial_x='0', initial_y='0', initial_yaw='0')
        return context

    def test_preflight_works_without_an_initialized_default_ros_context(self):
        # Construct actions only; do not execute drivers, gate, or detector.
        self.assertFalse(rclpy.ok())
        with self.assertRaisesRegex(RuntimeError, 'requires a saved map'):
            self.module().start(self.context())
        self.assertFalse(rclpy.ok())

    def test_mapped_calibration_launch_adds_only_localization_servers(self):
        context=self.context()
        with tempfile.NamedTemporaryFile(suffix='.yaml') as map_file:
            context.launch_configurations.update(map=map_file.name, seed_pose='true',
                initial_x='1.0', initial_y='2.0', initial_yaw='.3')
            actions=self.module().start(context)
        self.assertEqual(len(actions),6)  # map server, AMCL, lifecycle + existing 3
        self.assertFalse(rclpy.ok())

    def test_preflight_rejects_existing_command_publisher(self):
        rclpy.init()
        other = Node('calibration_conflicting_publisher')
        other.create_publisher(Twist, '/cmd_vel', 10)
        try:
            with self.assertRaisesRegex(RuntimeError, 'existing robot stack'):
                self.module().start(self.context())
        finally:
            other.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    if os.environ.get('ROS_DOMAIN_ID') in (None, '0', '6'):
        raise SystemExit('Use an isolated ROS_DOMAIN_ID, for example 106')
    unittest.main(verbosity=2)
