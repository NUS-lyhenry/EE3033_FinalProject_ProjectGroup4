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
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

from .calibration import (camera_model, fit_profile, save_profile, validate_holdout,
                          validate_sample, validate_setup)
from .calibration_motion import CalibrationMotion
from .calibration_auto import AutomaticRoute, fit_automatic, relative_pose, corridor_clear, wrap, FreshDataPause, BottleReacquisition
from .core import quaternion_yaw, stamp_seconds
from .model_labels import validate_target
from .ros_utils import LATCHED, parameters
from .stationary import StationaryWindow


class CalibrationNode(Node):
    def __init__(self, **kwargs):
        super().__init__('project_camera_calibration', **kwargs)
        self.p = parameters(self, dict(port=8780, allow_motion=False,
            model='/home/ros2_ws/weights/yolo26n.pt', target_class='bottle',
            image_topic='/image_raw', camera_info_topic='/camera_info', mount_frame='camera_link',
            output_directory='~/ee3033_calibrations', image_size=320,
            preview_fps=8., preview_quality=65, pitch_hint_deg=14.,
            settle_seconds=.35, settle_linear_limit=.18, settle_angular_limit=.60,
            settle_position_span=.15, settle_heading_span=.45, parent_port=8765))
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(1)
        self.model = YOLO(self.p['model'])
        validate_target(self.model.names, self.p['target_class'])
        self.model.to('cpu')
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.motion = CalibrationMotion()
        self.auto = None
        self.closing = False
        self.auto_status = dict(state='idle', reason='Load a saved map and establish localization before starting.')
        self.auto_views = []
        self.odom_history = deque(maxlen=200)
        self.odom_frame = None
        self.grid = self.amcl = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
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
        self.reason = 'Enter bottle height, diameter and one floor distance, then start automatic calibration in the mapped area.'
        self.fresh = {}
        self.last_moving = time.monotonic()
        self.raw_speed = [math.inf, math.inf]
        self.final = None
        self.graph_ok = False
        self.command_pub = self.create_publisher(Twist, '/project/nav_cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, '/project/drive_enabled', LATCHED)
        self.create_service(Trigger, '/project/stop', self.on_stop)
        self.create_subscription(Image, self.p['image_topic'], self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self.on_info, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_fixed', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_final, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_timer(.05, self.tick)
        self.start_server()
        # Keep image encoding/inference and graph discovery out of the ROS
        # executor. Its safety timer and odometry callbacks must not compete
        # with queued image work in a Python executor thread pool.
        self.worker_stop = threading.Event()
        self.workers = []
        for callback, interval in ((self.detect, .25), (self.preview, 1./self.p['preview_fps']),
                                   (self.check_graph, .3)):
            worker = threading.Thread(target=self.camera_worker, args=(callback, interval), daemon=True)
            worker.start()
            self.workers.append(worker)

    def camera_worker(self, callback, interval):
        while not self.worker_stop.is_set():
            began = time.monotonic()
            try:
                callback()
            except Exception as exc:
                self.get_logger().error('Camera worker error: '+str(exc))
            self.worker_stop.wait(max(.01, interval-(time.monotonic()-began)))

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
            if self.odom_frame is not None and self.odom_frame != msg.header.frame_id:
                self.cancel_auto('Odometry frame changed')
                self.odom_history.clear()
            self.odom_frame = msg.header.frame_id
            self.odom_history.append((stamp_seconds(msg.header.stamp), pose))
            self.pose, self.raw_speed = pose, speed
            self.fresh['odometry'] = time.monotonic()
            self.stationary.add(stamp_seconds(msg.header.stamp), *pose,
                                msg.twist.twist.linear.x, msg.twist.twist.angular.z, msg.header.frame_id)

    def on_map(self, msg):
        with self.lock:
            self.grid = msg

    def on_amcl(self, msg):
        if self.fresh_stamp(msg, 3.) and msg.header.frame_id == 'map':
            with self.lock:
                self.amcl = list(msg.pose.covariance)
                self.fresh['localization'] = time.monotonic()

    def pose_at(self, stamp):
        history = list(self.odom_history)
        for (a, p), (b, q) in zip(history, history[1:]):
            if a <= stamp <= b and 0 < b-a <= .2:
                t = (stamp-a)/(b-a)
                return [p[0]+t*(q[0]-p[0]), p[1]+t*(q[1]-p[1]),
                        p[2]+t*wrap(q[2]-p[2])]
        if history and abs(history[-1][0]-stamp) <= .05:
            return list(history[-1][1])
        return None

    def map_context(self):
        if self.grid is None:
            raise ValueError('Load a saved map in the main camera setup page')
        if (self.amcl is None or time.monotonic()-self.fresh.get('localization', -math.inf) > 30
                or any(not math.isfinite(self.amcl[i]) or self.amcl[i] < 0 or self.amcl[i] > limit
                       for i, limit in ((0, .04), (7, .04), (35, .09)))):
            raise ValueError('Establish reliable map localization before automatic calibration')
        try:
            transform = self.tf.lookup_transform('map', self.odom_frame or 'odom_combined', Time())
            age = self.get_clock().now().nanoseconds*1e-9-stamp_seconds(transform.header.stamp)
            if not -.75 <= age <= 3.:
                raise ValueError('Map localization transform is stale')
            t = transform.transform
            alignment = [t.translation.x, t.translation.y, quaternion_yaw(t.rotation)]
            c, s = math.cos(alignment[2]), math.sin(alignment[2])
            pose = [alignment[0]+c*self.pose[0]-s*self.pose[1],
                    alignment[1]+s*self.pose[0]+c*self.pose[1], alignment[2]+self.pose[2]]
            return pose, alignment
        except ValueError:
            raise
        except Exception as exc:
            raise ValueError('Waiting for map-to-odometry localization') from exc

    def auto_ready(self):
        try:
            self.map_context()
            if not self.camera:
                raise ValueError('Waiting for camera geometry')
            transform = self.tf.lookup_transform('base_footprint', self.p['mount_frame'], Time())
            t = transform.transform.translation
            validate_setup(dict(bottle_height=.25, bottle_diameter=.065,
                                camera_height=t.z, camera_x=t.x, camera_y=t.y))
            return dict(ready=True, reason='Map localization and camera mounting available')
        except Exception as exc:
            return dict(ready=False, reason=str(exc))

    def cancel_auto(self, reason):
        if self.auto is not None:
            last = self.detections[-1] if self.detections else {}
            self.auto_status.update(state='stopped', reason=reason, failure=dict(
                detected_count=last.get('count'), phase=self.auto.phase,
                view_index=self.auto.index, captured_views=len(self.auto_views),
                last_box=last.get('box')))
            self.auto = None
        if self.motion.mode == 'auto':
            self.motion.stop(reason)
        self.revision += 1

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

    def capture_status(self, automatic=False):
        settled = self.stationary.evaluate(self.get_clock().now().nanoseconds*1e-9)
        reason = 'Gentle balancing motion is allowed; capture a clear bottle image.'
        ready = True
        if (self.motion.mode != 'idle' and not automatic) or time.monotonic()-self.last_moving < .2:
            ready, reason = False, 'Finish commanded driving/turning before capture'
        elif (not all(self.health().values()) or not self.graph_ok or self.final is None
              or any(not math.isfinite(v) or abs(v) > 1e-5 for v in self.final)):
            ready, reason = False, 'Fresh sensors, zero final commands and exclusive control are required'
        elif not settled['verified']:
            ready, reason = False, 'Checking bounded balancing motion: '+settled['reason']
        return dict(ready=ready, reason=reason, balancing_allowed=True, window=settled)

    def require_stationary(self):
        status = self.capture_status()
        if not status['ready']:
            raise ValueError(status['reason'])

    def on_stop(self, request, response):
        self.command('/api/calibration/stop', {})
        response.success, response.message = True, 'Camera positioning stopped'
        return response

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
                    self.cancel_auto('Camera settings changed; start a new calibration')
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
                self.detections.append(dict(time=captured, stamp=stamp, pose=self.pose_at(stamp), box=boxes[0] if len(boxes) == 1 else None,
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

    def automatic_tick(self, now, healthy):
        try:
            lease_age = now-self.motion.lease
            if lease_age > 1.5:
                raise ValueError('Browser heartbeat lost')
            if lease_age > .6:
                self.auto_pause.waiting(now, True, False)
                self.reason = 'Paused with zero commands; waiting for the browser heartbeat'
                self.auto_status.update(state='collecting', reason=self.reason)
                return 0., 0.
            if not self.graph_ok:
                raise ValueError('Exclusive command ownership lost')
            target = self.target()
            frame = self.detections[-1] if self.detections else {}
            reacquiring = self.auto_bottle.waiting(frame.get('stamp'), frame.get('count'), frame.get('box'))
            delayed = reacquiring or target is None or now-frame.get('time', -math.inf) > .8
            if self.auto_pause.waiting(now, not healthy or delayed,
                                       self.capture_status(automatic=True)['ready']):
                self.reason = ('Paused with zero commands; reacquiring the same bottle (two matching views required)'
                               if reacquiring else 'Paused with zero commands; waiting for fresh data and a clear settled view')
                self.auto_status.update(state='collecting', reason=self.reason)
                return 0., 0.
            map_pose, alignment = self.map_context()
            baseline = self.auto_alignment
            if math.hypot(alignment[0]-baseline[0], alignment[1]-baseline[1]) > .15 or abs(wrap(alignment[2]-baseline[2])) > .12:
                raise ValueError('Localization shifted; establish the pose and restart calibration')
            route = self.auto
            if route.phase == 'align' and not corridor_clear(self.grid, map_pose, map_pose[2], length=0.):
                raise ValueError('Robot footprint is not clear in the saved map')
            if route.phase != 'align':
                # Transform the fixed odometry corridor into the current map.
                c, s = math.cos(alignment[2]), math.sin(alignment[2])
                origin = [alignment[0]+c*route.origin[0]-s*route.origin[1],
                          alignment[1]+s*route.origin[0]+c*route.origin[1]]
                if not corridor_clear(self.grid, origin, alignment[2]+route.heading):
                    raise ValueError('The 0.78 m calibration corridor is not clear in the saved map')
            linear, angular, capture = route.tick(now, self.pose, target['error'], self.capture_status(automatic=True)['ready'])
            self.reason = route.reason
            self.auto_status.update(state='collecting', reason=route.reason, collected=len(self.auto_views), total=18)
            if capture:
                frame = self.detections[-1]
                if frame.get('pose') is not None and frame['time'] > self.last_moving+.2 and frame['stamp'] != self.auto_last_stamp:
                    validate_sample(dict(distance=1., lateral=0., box=frame['box']), self.camera)
                    self.auto_views.append(dict(pose=relative_pose(self.auto_anchor, frame['pose']),
                        box=list(frame['box']), stamp=frame['stamp'], holdout=route.goals[route.index][2]))
                    self.auto_last_stamp = frame['stamp']
                    route.captured(now)
            if route.phase == 'done':
                self.motion.stop('Collection complete; checking camera model')
                self.auto = None
                self.auto_status.update(state='fitting', reason='Fitting and checking two held-out positions')
                revision = self.revision
                args = copy.deepcopy((self.setup, self.camera, self.auto_range, self.auto_views, self.p['pitch_hint_deg']))
                threading.Thread(target=self.finish_automatic, args=(revision, args), daemon=True).start()
                return 0., 0.
            return linear, angular
        except ValueError as exc:
            self.cancel_auto(str(exc))
            self.reason = 'Automatic calibration stopped: '+str(exc)
            return 0., 0.

    def finish_automatic(self, revision, args):
        try:
            profile = fit_automatic(*args)
            with self.lock:
                if revision != self.revision:
                    return
                self.profile = profile
                self.samples = profile['training']
                self.saved = save_profile(profile, self.p['output_directory'])
                self.reason = 'Automatic calibration passed both floor checks and was saved for future missions.'
                self.auto_status.update(state='complete', reason=self.reason)
        except Exception as exc:
            with self.lock:
                if revision == self.revision:
                    self.reason = 'Automatic calibration failed: '+str(exc)
                    self.auto_status.update(state='failed', reason=self.reason)

    def tick(self):
        with self.lock:
            healthy = self.p['allow_motion'] and all(self.health().values()) and self.graph_ok
            linear, angular = (self.automatic_tick(time.monotonic(), healthy) if self.auto is not None
                               else self.motion.tick(time.monotonic(), self.pose, self.target(), healthy))
            if linear or angular:
                self.last_moving = time.monotonic()
                self.stationary.reset()
            msg = Twist()
            msg.linear.x, msg.angular.z = linear, angular
            self.command_pub.publish(msg)
            self.enable_pub.publish(Bool(data=healthy and self.motion.mode != 'idle'
                and (self.motion.mode != 'auto' or time.monotonic()-self.motion.lease <= .6)))

    def snapshot(self):
        with self.lock:
            last = self.detections[-1] if self.detections else {}
            return dict(token=self.token, generation=self.motion.generation,
                allow_motion=self.p['allow_motion'], read_only=False, health=self.health(),
                automatic={**self.auto_status, 'pitch_hint_deg': self.p['pitch_hint_deg'], 'prerequisites': self.auto_ready()},
                ownership=self.graph_ok, motion=self.motion.mode, motion_reason=self.motion.reason,
                stationary=self.stationary.evaluate(self.get_clock().now().nanoseconds*1e-9),
                capture=self.capture_status(),
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
            if self.closing and path != '/api/calibration/stop':
                raise ValueError('Calibration services are shutting down')
            if path == '/api/calibration/stop':
                self.cancel_auto('Stopped by operator')
                self.auto_status.update(state='stopped', reason='Stopped by operator')
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
            if self.auto_status['state'] == 'fitting':
                raise ValueError('Automatic fit is running; use Stop to cancel it')
            if path == '/api/calibration/automatic':
                self.require_stationary()
                if not self.p['allow_motion']:
                    raise ValueError('Motion is disabled')
                ready = self.auto_ready()
                if not ready['ready']:
                    raise ValueError(ready['reason'])
                if self.target() is None:
                    raise ValueError('Keep exactly one bottle fully visible')
                frame = self.detections[-1]
                if frame.get('pose') is None or frame['time'] < self.last_moving+.2:
                    raise ValueError('Wait for a fresh bottle image paired with odometry')
                t = self.tf.lookup_transform('base_footprint', self.p['mount_frame'], Time()).transform.translation
                setup = validate_setup(dict(bottle_height=body.get('bottle_height'),
                    bottle_diameter=body.get('bottle_diameter'), camera_height=t.z, camera_x=t.x, camera_y=t.y))
                from .calibration import number
                distance = number(body.get('distance'), 'Measured floor distance (m)', .45, 2.5)
                validate_sample(dict(distance=1., lateral=0., box=frame['box']), self.camera)
                map_pose, alignment = self.map_context()
                if not corridor_clear(self.grid, map_pose, map_pose[2], length=0.):
                    raise ValueError('Robot footprint is not clear in the saved map')
                self.setup = setup
                self.auto_range = distance
                self.auto_anchor = list(frame['pose'])
                self.auto_alignment = alignment
                self.auto_views = [dict(pose=[0., 0., 0.], box=list(frame['box']), stamp=frame['stamp'], holdout=False)]
                self.auto_last_stamp = frame['stamp']
                self.samples, self.profile, self.saved, self.frozen = [], None, None, None
                self.auto = AutomaticRoute(time.monotonic(), self.pose)
                self.auto_pause = FreshDataPause()
                self.auto_bottle = BottleReacquisition(frame['stamp'], frame['box'], self.camera['width'], self.camera['height'])
                self.auto_status = dict(state='collecting', reason=self.auto.reason, collected=1, total=18, anchor_range_m=distance)
                self.motion.mode, self.motion.lease = 'auto', time.monotonic()
                self.motion.reason = 'Automatic calibration running'
                self.revision += 1
                return dict(success=True)
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
                self.auto_status = dict(state='idle', reason='Manual setup active; automatic calibration is available above.')
                self.samples, self.profile, self.frozen, self.saved = [], None, None, None
                self.reason = 'Capture six or more positions with different distances and left/right offsets.'
            elif path == '/api/calibration/freeze':
                if not self.setup or not self.camera:
                    raise ValueError('Save the setup and wait for a camera image first')
                if self.target() is None or self.detections[-1]['time'] < self.last_moving+.2:
                    raise ValueError('Keep exactly one bottle fully visible for a fresh detection')
                frame = self.detections[-1]
                # A single timestamped image avoids requiring a fixed silhouette
                # while the body balances. Its box and JPEG stay paired.
                self.frozen = dict(id=secrets.token_hex(8), time=time.monotonic(),
                                   box=frame['box'], jpeg=frame['jpeg'])
                self.reason = 'Captured with balancing allowed. Check the outline and measure this position.'
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
                                    ('Content-Security-Policy', "frame-ancestors 'self' http://127.0.0.1:"+str(node.p['parent_port'])+' http://localhost:'+str(node.p['parent_port']))]:
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
        with self.lock:
            self.closing = True
            self.cancel_auto('Shutdown')
            self.motion.stop('Shutdown')
            if self.context.ok():
                self.command_pub.publish(Twist())
                self.enable_pub.publish(Bool(data=False))
        self.worker_stop.set()
        for worker in self.workers:
            worker.join(timeout=3.)
        self.server.shutdown()
        self.server.server_close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = CalibrationNode()
    executor = SingleThreadedExecutor()
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
