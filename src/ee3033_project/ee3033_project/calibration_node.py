"""Preparation-only camera calibration with a local guided browser console."""
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import copy
import json
import math
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool

from .calibration import (camera_model, fit_profile, save_profile, validate_holdout,
                          validate_sample, validate_setup)
from .calibration_motion import CalibrationMotion
from .core import quaternion_yaw, stamp_seconds
from .model_labels import validate_target
from .ros_utils import LATCHED, parameters
from .stationary import StationaryWindow


class CalibrationNode(Node):
    def __init__(self, **kwargs):
        super().__init__('project_camera_calibration', **kwargs)
        self.p = parameters(self, dict(port=8780, allow_motion=False,
            model='/home/ros2_ws/weights/yolo26n.pt', target_class='bottle',
            image_topic='/image_raw', camera_info_topic='/camera_info',
            output_directory='~/ee3033_calibrations', image_size=320,
            preview_fps=8., preview_quality=65,
            settle_seconds=1., settle_linear_limit=.06, settle_angular_limit=.20,
            settle_position_span=.08, settle_heading_span=.20))
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(1)
        self.model = YOLO(self.p['model'])
        validate_target(self.model.names, self.p['target_class'])
        self.model.to('cpu')
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.motion = CalibrationMotion()
        self.stationary = StationaryWindow(seconds=self.p['settle_seconds'],
            linear_limit=self.p['settle_linear_limit'], angular_limit=self.p['settle_angular_limit'],
            position_limit=self.p['settle_position_span'], heading_limit=self.p['settle_heading_span'])
        self.token = secrets.token_urlsafe(24)
        self.image = self.info = self.pose = None
        self.image_stamp = None
        self.preview_stamp = None
        self.preview_time = -math.inf
        self.preview_sequence = 0
        self.inference_latency = None
        self.detection_error = None
        self.detections = deque(maxlen=8)
        self.jpeg = self.frozen = self.camera = None
        self.setup = self.profile = None
        self.samples = []
        self.revision = 0
        self.saved = None
        self.reason = 'Enter the measured bottle dimensions and camera mounting position.'
        self.fresh = {}
        self.last_moving = time.monotonic()
        self.raw_speed = [math.inf, math.inf]
        self.final = None
        self.graph_ok = False
        self.command_pub = self.create_publisher(Twist, '/project/nav_cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, '/project/drive_enabled', LATCHED)
        self.create_subscription(Image, self.p['image_topic'], self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self.on_info, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_fixed', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_final, 10)
        self.create_timer(.05, self.tick)
        self.create_timer(.25, self.detect, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1./self.p['preview_fps'], self.preview,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(.3, self.check_graph, callback_group=MutuallyExclusiveCallbackGroup())
        self.start_server()

    def fresh_stamp(self, msg, limit):
        age = self.get_clock().now().nanoseconds*1e-9-stamp_seconds(msg.header.stamp)
        return 0 <= age <= limit

    def on_image(self, msg):
        if self.fresh_stamp(msg, 1.):
            with self.lock:
                self.image = msg
                self.fresh['camera'] = time.monotonic()

    def on_info(self, msg):
        with self.lock:
            self.info = msg

    def on_odom(self, msg):
        if not self.fresh_stamp(msg, .3):
            return
        p = msg.pose.pose
        pose = [p.position.x, p.position.y, quaternion_yaw(p.orientation)]
        speed = [abs(msg.twist.twist.linear.x), abs(msg.twist.twist.angular.z)]
        if not all(math.isfinite(v) for v in pose+speed):
            return
        with self.lock:
            self.pose, self.raw_speed = pose, speed
            self.fresh['odometry'] = time.monotonic()
            self.stationary.add(stamp_seconds(msg.header.stamp), *pose,
                                msg.twist.twist.linear.x, msg.twist.twist.angular.z, msg.header.frame_id)

    def on_scan(self, msg):
        if self.fresh_stamp(msg, .4) and any(not math.isnan(v) and v >= msg.range_min for v in msg.ranges):
            with self.lock:
                self.fresh['lidar'] = time.monotonic()

    def on_final(self, msg):
        values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        with self.lock:
            self.final = values
            self.fresh['output'] = time.monotonic()
            if any(not math.isfinite(v) or abs(v) > 1e-5 for v in values):
                self.last_moving = time.monotonic()
                self.stationary.reset()

    def check_graph(self):
        def owned(topic, node):
            publishers = self.get_publishers_info_by_topic(topic)
            return len(publishers) == 1 and publishers[0].node_name == node
        ok = (owned('/cmd_vel', 'project_velocity_gate')
              and owned('/project/nav_cmd_vel', self.get_name())
              and owned('/project/drive_enabled', self.get_name())
              and self.count_publishers('/project/mission_status') == 0)
        with self.lock:
            self.graph_ok = ok
            self.fresh['ownership'] = time.monotonic()

    def health(self):
        now = time.monotonic()
        return {name: now-self.fresh.get(name, -math.inf) < limit for name, limit in
                [('camera', 1.), ('odometry', .3), ('lidar', .4), ('output', .3), ('ownership', .8)]}

    def require_stationary(self):
        settled = self.stationary.evaluate(self.get_clock().now().nanoseconds*1e-9)
        if self.motion.mode != 'idle' or not settled['verified'] or time.monotonic()-self.last_moving < 1.:
            raise ValueError('Wait for the calibration settling window: '+settled['reason'])
        if (not all(self.health().values()) or not self.graph_ok or self.final is None
                or any(not math.isfinite(v) or abs(v) > 1e-5 for v in self.final)):
            raise ValueError('Fresh camera, odometry, lidar, zero output and exclusive control are required')

    def preview(self):
        """Encode live JPEGs independently of the slower detection callback."""
        with self.lock:
            image, info = self.image, self.info
        if image is None or not self.fresh_stamp(image, 1.):
            return
        stamp = stamp_seconds(image.header.stamp)
        if stamp == self.preview_stamp:
            return
        self.preview_stamp = stamp
        try:
            raw = self.bridge.imgmsg_to_cv2(image, 'bgr8')
            k = d = None
            if info is not None and info.width == image.width and info.height == image.height:
                k, d = list(info.k), list(info.d)
            camera = camera_model(image.width, image.height, image.header.frame_id, k, d,
                                  info.distortion_model if info else 'plumb_bob')
            ok, jpeg = cv2.imencode('.jpg', raw, [cv2.IMWRITE_JPEG_QUALITY, self.p['preview_quality']])
            if not ok or not self.fresh_stamp(image, 1.):
                return
            now = time.monotonic()
            with self.lock:
                # A changed stream cannot inherit samples or an earlier fit.
                if self.camera and camera != self.camera:
                    self.motion.stop('Camera settings changed; start a new calibration')
                    self.samples, self.profile, self.frozen, self.saved = [], None, None, None
                    self.detections.clear()
                    self.revision += 1
                    self.reason = 'Camera settings changed; training samples were reset.'
                self.camera, self.jpeg = camera, jpeg.tobytes()
                self.preview_time = now
                self.preview_sequence += 1
        except Exception as exc:
            with self.lock:
                self.reason = 'Camera preview error: '+str(exc)

    def detect(self):
        with self.lock:
            image, camera = self.image, self.camera
        if image is None or camera is None or not self.fresh_stamp(image, 1.):
            return
        stamp = stamp_seconds(image.header.stamp)
        if stamp == self.image_stamp:
            return
        self.image_stamp = stamp
        started = time.monotonic()
        captured = started-(self.get_clock().now().nanoseconds*1e-9-stamp)
        try:
            raw = self.bridge.imgmsg_to_cv2(image, 'bgr8')
            # Freeze uses the exact detected image, with higher quality than the
            # live preview. Never attach a delayed box to a newer live JPEG.
            ok, jpeg = cv2.imencode('.jpg', raw, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                return
            result = self.model.predict(raw, imgsz=self.p['image_size'], conf=.6,
                                        device='cpu', verbose=False)[0]
            boxes = [b.xyxy[0].cpu().numpy().astype(float).tolist() for b in result.boxes
                     if result.names[int(b.cls.item())].casefold() == self.p['target_class'].casefold()]
            now = time.monotonic()
            with self.lock:
                self.inference_latency = now-started
                if now-captured > 1.5 or camera != self.camera:
                    self.detection_error = 'Detector is delayed; live video continues. Wait for a fresh detection.'
                    return
                self.detection_error = None
                self.detections.append(dict(time=captured, box=boxes[0] if len(boxes) == 1 else None,
                    count=len(boxes), jpeg=jpeg.tobytes()))
                self.fresh['detection'] = now
        except Exception as exc:
            with self.lock:
                self.detection_error = 'Detector error: '+str(exc)

    def target(self):
        if not self.detections:
            return None
        d = self.detections[-1]
        if time.monotonic()-d['time'] > 1.5 or d['box'] is None:
            return None
        return dict(error=((d['box'][0]+d['box'][2])/2-self.camera['k'][2])/self.camera['width'])

    def tick(self):
        with self.lock:
            healthy = self.p['allow_motion'] and all(self.health().values()) and self.graph_ok
            linear, angular = self.motion.tick(time.monotonic(), self.pose, self.target(), healthy)
            if linear or angular:
                self.last_moving = time.monotonic()
                self.stationary.reset()
            msg = Twist()
            msg.linear.x, msg.angular.z = linear, angular
            self.command_pub.publish(msg)
            self.enable_pub.publish(Bool(data=healthy and self.motion.mode != 'idle'))

    def snapshot(self):
        with self.lock:
            last = self.detections[-1] if self.detections else {}
            return dict(token=self.token, generation=self.motion.generation,
                allow_motion=self.p['allow_motion'], read_only=False, health=self.health(),
                ownership=self.graph_ok, motion=self.motion.mode, motion_reason=self.motion.reason,
                stationary=self.stationary.evaluate(self.get_clock().now().nanoseconds*1e-9),
                preview=dict(sequence=self.preview_sequence,
                    age=round(time.monotonic()-self.preview_time, 2) if math.isfinite(self.preview_time) else None,
                    fresh=time.monotonic()-self.preview_time < 1., fps=self.p['preview_fps']),
                inference_latency=self.inference_latency, detection_error=self.detection_error,
                reason=self.reason, camera=self.camera, setup=self.setup, samples=self.samples,
                profile=self.profile, saved=self.saved,
                detected_count=last.get('count', 0), box=last.get('box') if self.target() else None,
                detection_fresh=self.target() is not None,
                frozen={k:v for k,v in self.frozen.items() if k != 'jpeg'} if self.frozen else None)

    def command(self, path, body):
        # Fit uses a snapshot outside the control lock; Stop and the velocity
        # deadline can always run while SciPy is working.
        if path == '/api/calibration/fit':
            with self.lock:
                if body.get('generation') != self.motion.generation:
                    raise ValueError('Control context changed; refresh before retrying')
                self.require_stationary()
                revision = self.revision
                args = copy.deepcopy((self.setup, self.camera, self.samples))
            profile = fit_profile(*args)
            with self.lock:
                if revision != self.revision:
                    raise ValueError('Calibration changed while fitting; retry')
                self.profile = profile
                self.reason = 'Fit ready. Capture two NEW measured validation positions.'
                return dict(success=True)
        with self.lock:
            if path == '/api/calibration/stop':
                self.motion.stop('Stopped by operator')
                self.command_pub.publish(Twist())
                self.enable_pub.publish(Bool(data=False))
                self.revision += 1
                return dict(success=True)
            if path == '/api/calibration/heartbeat':
                self.motion.renew(time.monotonic(), body.get('generation'))
                return dict(success=True)
            if body.get('generation') != self.motion.generation:
                raise ValueError('Control context changed; refresh before retrying')
            if path == '/api/calibration/unfreeze':
                self.frozen = None
                return dict(success=True)
            if path == '/api/calibration/move':
                self.require_stationary()
                if not self.p['allow_motion']:
                    raise ValueError('Motion disabled; launch with allow_motion:=true to use movement')
                if self.target() is None:
                    raise ValueError('Exactly one fresh detected target is needed for movement')
                self.frozen = None
                self.motion.start(body.get('mode'), time.monotonic(), self.pose, body['generation'])
                self.revision += 1
                return dict(success=True)
            self.require_stationary()
            if path == '/api/calibration/setup':
                self.setup = validate_setup(body)
                self.samples, self.profile, self.frozen, self.saved = [], None, None, None
                self.reason = 'Capture six or more positions with different distances and left/right offsets.'
            elif path == '/api/calibration/freeze':
                if not self.setup or not self.camera:
                    raise ValueError('Save the setup and wait for a camera image first')
                frames = [d for d in list(self.detections)[-3:]
                          if time.monotonic()-d['time'] < 4. and d['time'] > self.last_moving+.2]
                if len(frames) < 3 or any(d['box'] is None for d in frames) or self.target() is None:
                    raise ValueError('Keep exactly one bottle visible for three recent detections')
                boxes = np.array([d['box'] for d in frames])
                if float(np.ptp(boxes, axis=0).max()) > 18:
                    raise ValueError('Bottle image is moving; wait for a stable view')
                # Use the last frame's box and JPEG together. Stability is checked
                # across frames; never attach median pixels to a different image.
                self.frozen = dict(id=secrets.token_hex(8), time=time.monotonic(),
                                   box=frames[-1]['box'], jpeg=frames[-1]['jpeg'])
            elif path == '/api/calibration/sample':
                if not self.frozen or body.get('freeze_id') != self.frozen['id'] or time.monotonic()-self.frozen['time'] > 120:
                    raise ValueError('Freeze a fresh image before recording a measurement')
                sample = validate_sample(body, self.camera)
                self.saved = None
                if body.get('validation') is True:
                    if not self.profile:
                        raise ValueError('Fit the training samples before validation')
                    self.profile['validation'].append(validate_holdout(self.profile, sample))
                    self.reason = 'Validation recorded. Every validation must pass before saving.'
                else:
                    if len(self.samples) >= 40:
                        raise ValueError('Maximum 40 training samples; reset setup to start again')
                    if any(math.hypot(sample['distance']-s['distance'], sample['lateral']-s['lateral']) < .08 for s in self.samples):
                        raise ValueError('Move to a new position at least 8 cm from the earlier measurements')
                    self.samples.append(sample)
                    self.profile, self.saved = None, None
                self.frozen = None
            elif path == '/api/calibration/remove':
                kind = body.get('kind')
                entries = self.samples if kind == 'training' else self.profile['validation'] if self.profile and kind == 'validation' else None
                index = body.get('index')
                if entries is None or type(index) is not int or not 0 <= index < len(entries):
                    raise ValueError('Select an existing sample')
                entries.pop(index)
                if kind == 'training':
                    self.profile = None
                self.saved = None
            elif path == '/api/calibration/save':
                if not self.profile:
                    raise ValueError('Fit and validate before saving')
                self.saved = save_profile(self.profile, self.p['output_directory'])
                self.reason = 'Saved. Stop this calibration launch before starting mapping or mission.'
            else:
                raise ValueError('Unknown calibration command')
            self.revision += 1
            return dict(success=True, saved=self.saved)

    def start_server(self):
        node = self
        root = Path(get_package_share_directory('ee3033_project'))/'web'
        assets = {'/': ('calibration.html', 'text/html; charset=utf-8'),
                  '/calibration.js': ('calibration.js', 'text/javascript'),
                  '/calibration.css': ('calibration.css', 'text/css')}

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def reply(self, status, data, kind='application/json'):
                body = json.dumps(data, allow_nan=False).encode() if kind == 'application/json' else data
                self.send_response(status)
                for name, value in [('Content-Type', kind), ('Content-Length', str(len(body))),
                                    ('Cache-Control', 'no-store'), ('X-Content-Type-Options', 'nosniff'),
                                    ('X-Frame-Options', 'DENY')]:
                    self.send_header(name, value)
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = urlparse(self.path).path
                if path in assets:
                    name, kind = assets[path]
                    return self.reply(200, (root/name).read_bytes(), kind)
                if path == '/api/calibration/state':
                    return self.reply(200, node.snapshot())
                if path == '/api/calibration/image.jpg':
                    with node.lock:
                        jpeg = node.frozen['jpeg'] if node.frozen else node.jpeg
                        fresh = node.frozen is not None or time.monotonic()-node.preview_time < 1.
                    return self.reply(200, jpeg, 'image/jpeg') if jpeg and fresh else self.reply(503, dict(error='Waiting for a fresh camera preview'))
                return self.reply(404, dict(error='Unknown endpoint'))

            def do_POST(self):
                allowed = {f'http://127.0.0.1:{node.p["port"]}', f'http://localhost:{node.p["port"]}'}
                if (not secrets.compare_digest(self.headers.get('X-Project-Token', ''), node.token)
                        or self.headers.get('Origin', next(iter(allowed))) not in allowed):
                    return self.reply(403, dict(error='Invalid session token or origin'))
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 <= length <= 8192:
                        raise ValueError('Request too large')
                    body = json.loads(self.rfile.read(length) or b'{}')
                    if not isinstance(body, dict):
                        raise ValueError('JSON object required')
                    result = node.command(urlparse(self.path).path, body)
                    return self.reply(200, result)
                except (ValueError, TypeError, KeyError) as exc:
                    return self.reply(400, dict(error=str(exc)))
                except OSError as exc:
                    return self.reply(503, dict(error=str(exc)))

        self.server = ThreadingHTTPServer(('127.0.0.1', self.p['port']), Handler)
        self.p['port'] = self.server.server_address[1]
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(f'Preparation calibration: http://127.0.0.1:{self.p["port"]}')

    def destroy_node(self):
        self.motion.stop('Shutdown')
        if self.context.ok():
            self.command_pub.publish(Twist())
            self.enable_pub.publish(Bool(data=False))
        self.server.shutdown()
        self.server.server_close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = CalibrationNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
