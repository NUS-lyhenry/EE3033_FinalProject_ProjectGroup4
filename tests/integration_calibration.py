"""Real ROS/HTTP calibration flow with synthetic images and detector proposals.

Run ONLY on an isolated domain. No real camera, weights, or robot are used.
"""
import json
import importlib.util
import hashlib
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from ee3033_project.calibration import camera_model, fit_profile, load_profile, predicted_points
from ee3033_project.calibration_node import CalibrationNode
from ee3033_project.perception import Perception
from ee3033_project.ros_utils import set_yaw
from ee3033_project.velocity_gate import VelocityGate
from ee3033_project.console import Console


SETUP = dict(bottle_height=.25, bottle_diameter=.065, camera_height=.21, camera_x=.07, camera_y=-.015)
CAMERA = camera_model(640, 480, 'usb_camera', [520., 0., 320., 0., 510., 240., 0., 0., 1.], [0.]*5)
TRUE = [555., 548., .13, .035]
POSITIONS = [(.65, -.09), (.8, .1), (1., -.2), (1.15, .22), (1.35, -.18), (1.5, .12)]


def sample(d, y):
    s = dict(distance=d, lateral=y)
    points = predicted_points(TRUE, SETUP, CAMERA, s)
    return {**s, 'box': [float(points[2:, 0].min()), float(points[1, 1]),
                         float(points[2:, 0].max()), float(points[0, 1])]}


def wait(predicate, timeout=8.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.03)
    return False


class Proposal:
    cls = np.array(0)

    def __init__(self, values):
        self.values, self.xyxy = values, [self]

    def cpu(self):
        return self

    def numpy(self):
        return np.array(self.values)


class Detector:
    names = {0: 'bottle'}

    def __init__(self):
        self.box = sample(1., 0.)['box']
        self.delay = 0.
        self.entered = threading.Event()
        self.completed_delay = threading.Event()

    def to(self, *args):
        return self

    def predict(self, *args, **kwargs):
        if self.delay:
            delay = self.delay
            self.entered.set()
            time.sleep(delay)
            self.completed_delay.set()
        return [SimpleNamespace(names=self.names, boxes=[Proposal(self.box)] if self.box else [])]


class Sensors(Node):
    def __init__(self):
        super().__init__('calibration_synthetic_sensors')
        self.image_pub = self.create_publisher(Image, '/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/camera_info', qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_fixed', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        self.commands = []
        self.frames = 0
        self.create_subscription(Twist, '/cmd_vel', lambda msg: self.commands.append((time.monotonic(), msg.linear.x, msg.angular.z)), 10)
        self.create_timer(.05, self.tick)

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        from cv_bridge import CvBridge
        self.frames += 1
        image = CvBridge().cv2_to_imgmsg(np.full((480, 640, 3), self.frames%255, dtype=np.uint8), 'bgr8')
        image.header.stamp, image.header.frame_id = stamp, CAMERA['frame_id']
        self.image_pub.publish(image)
        info = CameraInfo()
        info.header = image.header
        info.width, info.height, info.k, info.d, info.distortion_model = 640, 480, CAMERA['k'], CAMERA['d'], 'plumb_bob'
        self.info_pub.publish(info)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'laser'
        scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/360
        scan.range_min, scan.range_max, scan.ranges = .05, 5., [2.]*360
        self.scan_pub.publish(scan)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.child_frame_id, odom.pose.pose.orientation.w = 'base_footprint', 1.
        # Balancing corrections exceed the mission verifier's limits. The
        # calibration-specific settling policy must accept their bounded pose.
        t = self.frames*.05
        odom.pose.pose.position.x = .02*math.sin(8*t)
        odom.pose.pose.position.y = .02*math.sin(11*t)
        set_yaw(odom.pose.pose.orientation, .07*math.sin(9*t))
        odom.twist.twist.linear.x = .10 if self.frames%5 == 0 else .035
        odom.twist.twist.angular.z = .22 if self.frames%5 == 0 else .12
        self.odom_pub.publish(odom)


class CalibrationHTTP(unittest.TestCase):
    def test_full_capture_fit_validate_save_and_bounded_motion(self):
        rclpy.init()
        with tempfile.TemporaryDirectory(prefix='group4_calibration_') as tmp:
            detector = Detector()
            with patch.dict(sys.modules, {'torch': SimpleNamespace(set_num_threads=lambda n: None),
                                          'ultralytics': SimpleNamespace(YOLO=lambda path: detector)}):
                node = CalibrationNode(parameter_overrides=[Parameter('port', value=0),
                    Parameter('allow_motion', value=True), Parameter('output_directory', value=tmp)])
            gate = VelocityGate(parameter_overrides=[Parameter('allow_motion', value=True),
                Parameter('max_linear', value=.04), Parameter('max_angular', value=.15)])
            console = Console(parameter_overrides=[Parameter('port',value=0),
                Parameter('phase',value='calibration'),Parameter('session_directory',value=tmp+'/assessment'),
                Parameter('calibration_directory',value=tmp),Parameter('calibration_port',value=node.p['port'])])
            console.stage=SimpleNamespace(process=True,loading=False,cancel=threading.Event(),
                calibration_file='',stop=lambda:None,
                snapshot=lambda:dict(managed=True,phase='calibration',running=True,transitioning=False,fault=None))
            node.p['parent_port']=console.p['port']
            sensors = Sensors()
            executor = MultiThreadedExecutor(num_threads=5)
            for item in (node, gate, sensors, console):
                executor.add_node(item)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            base = f'http://127.0.0.1:{node.p["port"]}'

            def post(name, body=None, token=None, origin=None):
                headers = {'Content-Type': 'application/json', 'X-Project-Token': node.token if token is None else token}
                if origin:
                    headers['Origin'] = origin
                data = dict(generation=node.motion.generation, **(body or {}))
                deadline = time.monotonic()+8.
                while True:
                    try:
                        return json.load(urlopen(Request(base+'/api/calibration/'+name, json.dumps(data).encode(), headers, method='POST'), timeout=8))
                    except HTTPError as exc:
                        error = exc.read().decode()
                        # DDS/scheduling can invalidate readiness between a GET
                        # and POST. These rejections happen before any mutation.
                        transient = ('Checking bounded balancing motion' in error
                                     or 'Fresh sensors, zero final commands' in error
                                     or 'Finish commanded driving/turning' in error)
                        if exc.code == 400 and transient and time.monotonic() < deadline:
                            time.sleep(.05)
                            continue
                        exc.msg += ': '+error
                        raise

            def capture(d, y, validation=False):
                detector.box = sample(d, y)['box']
                def ready():
                    with node.lock:
                        frames = list(node.detections)[-5:]
                        return (len(frames) == 5 and all(f['box'] is not None and
                            np.allclose(f['box'], detector.box) for f in frames)
                            and all(node.health().values()) and node.graph_ok
                            and node.stationary.evaluate(node.get_clock().now().nanoseconds*1e-9)['verified'])
                self.assertTrue(wait(ready, 12.), 'Fresh stationary frames were not ready: '+str(node.snapshot()))
                try:
                    post('freeze')
                except HTTPError as exc:
                    raise AssertionError('Freeze rejected: '+exc.read().decode()) from exc
                snapshot = json.load(urlopen(base+'/api/calibration/state'))
                post('sample', dict(freeze_id=snapshot['frozen']['id'], box=snapshot['frozen']['box'],
                                    distance=d, lateral=y, validation=validation))

            try:
                self.assertTrue(wait(lambda: node.graph_ok and node.stationary.evaluate(node.get_clock().now().nanoseconds*1e-9)['verified']))
                self.assertIn(b'Give the camera', urlopen(base+'/').read())
                with self.assertRaises(HTTPError) as denied:
                    post('setup', SETUP, token='wrong')
                self.assertEqual(denied.exception.code, 403)
                with self.assertRaises(HTTPError):
                    post('setup', SETUP, origin='https://example.invalid')
                post('setup', SETUP)
                self.assertEqual(node.stationary.seconds, .35)
                self.assertTrue(node.capture_status()['balancing_allowed'])
                detector.delay = 2.
                self.assertTrue(wait(detector.entered.is_set))
                first_sequence = node.snapshot()['preview']['sequence']
                first_jpeg = urlopen(base+'/api/calibration/image.jpg').read()
                self.assertTrue(wait(lambda: node.snapshot()['preview']['sequence'] >= first_sequence+3, 2.))
                second_jpeg = urlopen(base+'/api/calibration/image.jpg').read()
                self.assertNotEqual(hashlib.sha256(first_jpeg).digest(), hashlib.sha256(second_jpeg).digest())
                detector.delay = 0.
                self.assertTrue(wait(detector.completed_delay.is_set, 4.))
                for d, y in POSITIONS:
                    capture(d, y)
                post('fit')
                with self.assertRaises(HTTPError):
                    post('save')
                for d, y in [(.9, -.08), (1.25, .09)]:
                    capture(d, y, validation=True)
                post('save')
                saved = load_profile(node.saved)
                self.assertTrue(saved['validated'])
                self.assertLess(max(v['error_m'] for v in saved['validation']), .03)
                self.assertTrue(wait(lambda:console.calibration_status['ready']))
                self.assertEqual(console.stage.calibration_file,node.saved)
                self.assertIsNone(console.record.snapshot()['started'])
                self.assertIn(b'Camera setup',urlopen(f'http://127.0.0.1:{console.p["port"]}/').read())
                self.assertIn(str(console.p['port']),urlopen(base+'/').headers['Content-Security-Policy'])
                detector.box = sample(1., 0.)['box']
                time.sleep(.4)
                before = time.monotonic()
                post('move', dict(mode='forward'))
                self.assertTrue(wait(lambda: any(t > before and v > 0 for t, v, _ in sensors.commands), 2.))
                self.assertTrue(wait(lambda: node.motion.mode == 'idle', 2.))
                time.sleep(.3)
                self.assertTrue(all(v == 0 and w == 0 for _, v, w in sensors.commands[-3:]))
                self.assertTrue(all(abs(v) <= .04 and abs(w) <= .15 for _, v, w in sensors.commands))
                self.assertIn('heartbeat', node.motion.reason)
                old_generation = node.motion.generation
                post('stop')
                with self.assertRaises(ValueError):
                    node.command('/api/calibration/move', dict(mode='forward', generation=old_generation))
                request=Request(f'http://127.0.0.1:{console.p["port"]}/api/stop',b'{}',
                    {'Content-Type':'application/json','X-Project-Token':console.token},method='POST')
                self.assertTrue(json.load(urlopen(request))['success'])
                self.assertEqual(node.motion.mode,'idle')
                print('PASS: live preview during blocked inference, balancing-tolerant settling, real HTTP/ROS calibration and browser-loss stop')
            finally:
                executor.shutdown(timeout_sec=3)
                thread.join(timeout=3)
                for item in (console, node, gate, sensors):
                    item.destroy_node()
                rclpy.shutdown()


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
        actions = self.module().start(self.context())
        self.assertEqual(len(actions), 3)
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
