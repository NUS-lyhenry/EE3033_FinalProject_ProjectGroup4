"""YOLO image detections with timestamped camera/lidar-to-map projection."""
from collections import deque
import json
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException

from .core import associate_lidar, stamp_seconds
from .ros_utils import parameters, transform_arrays
from .model_labels import validate_target
from .calibration import load_profile, mounting_rotation, floor_point, in_envelope
from .vision_geometry import FloorCamera, camera_matrix


class Perception(Node):
    def __init__(self, **kwargs):
        super().__init__('project_perception', **kwargs)
        self.p = parameters(self, {
            'model': '/home/ros2_ws/weights/yolo26n.pt', 'target_class': 'cup',
            'confidence': 0.60, 'image_topic': '/image_raw',
            'camera_info_topic': '/camera_info', 'scan_topic': '/scan_fixed',
            'camera_optical_frame': 'project_camera_optical', 'map_frame': 'map',
            'localization_method': 'ground_plane', 'max_scan_delta': 0.25,
            'max_image_age': 1.5, 'inference_period': 0.25, 'image_size': 416,
            'max_range': 3.0, 'ground_z': 0.0, 'position_sigma': 0.10,
            'min_lidar_points': 2, 'cluster_gap': 0.12, 'max_cluster_width': 0.5,
            'cpu_threads': 1,
            'calibration_file': '',
            'base_frame': 'base_footprint', 'camera_height': 0., 'camera_pitch': 0.,
            'camera_yaw': 0., 'camera_x': 0., 'camera_y': 0.,
            'horizontal_fov': 0., 'camera_min_range': .35, 'camera_max_range': 2.,
            'pixel_margin': 8., 'pixel_sigma': 2., 'height_sigma': .015,
            'pitch_sigma': .034906585, 'max_report_sigma': .10,
        })
        if self.p['localization_method'] not in ('lidar', 'ground_plane'):
            raise ValueError('localization_method must be lidar or ground_plane')
        self.calibration = load_profile(self.p['calibration_file']) if self.p['calibration_file'] else None
        self.model = self.load_detector()
        validate_target(self.model.names,self.p['target_class'])
        self.bridge = CvBridge()
        self.buffer = Buffer(cache_time=Duration(seconds=20))
        self.listener = TransformListener(self.buffer, self)
        self.lock = threading.Lock()
        self.latest_image = None
        self.camera = None
        self.scans = deque(maxlen=40)
        self.last_processed = None
        self.last_warning = 0.0
        self.floor_camera = None
        self.projection_sigma = self.p['position_sigma']
        self.geometric_sigma = self.p['position_sigma']
        self.projection_in_range = True
        self.pose_variance = (.0025, .0025)
        self.quiet_since = None
        self.odometry_time = -math.inf
        self.last_odometry_age = None
        group = ReentrantCallbackGroup()
        self.create_subscription(Image, self.p['image_topic'], self.on_image,
                                 qos_profile_sensor_data, callback_group=group)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self.on_camera,
                                 qos_profile_sensor_data, callback_group=group)
        if self.p['localization_method'] == 'lidar':
            self.create_subscription(LaserScan, self.p['scan_topic'], self.on_scan,
                                     qos_profile_sensor_data, callback_group=group)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10, callback_group=group)
        self.create_subscription(Odometry, '/odom', self.on_odometry, qos_profile_sensor_data, callback_group=group)
        self.image_pub = self.create_publisher(Image, '/project/detection_image',
                                                qos_profile_sensor_data)
        self.obs_pub = self.create_publisher(String, '/project/target_observations', 10)
        self.status_pub = self.create_publisher(String, '/project/perception_status', 10)
        self.detections_pub = self.create_publisher(String, '/project/target_detections', 10)
        self.timer = self.create_timer(self.p['inference_period'], self.process)
        self.get_logger().info('YOLO ready; target=' + self.p['target_class'])

    def load_detector(self):
        if not os.path.isfile(self.p['model']):
            raise FileNotFoundError('Set model to an existing YOLO weights file')
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(self.p['cpu_threads'])
        model = YOLO(self.p['model'])
        model.to('cpu')
        return model

    def on_image(self, msg):
        with self.lock:
            self.latest_image = msg

    def on_camera(self, msg):
        with self.lock:
            self.camera = msg

    def on_scan(self, msg):
        with self.lock:
            self.scans.append(msg)

    def on_pose(self, msg):
        c = msg.pose.covariance
        if msg.header.frame_id == self.p['map_frame'] and all(math.isfinite(v) and v >= 0 for v in (c[0], c[7], c[35])):
            self.pose_variance = (max(c[0], c[7]), c[35])

    def on_odometry(self, msg):
        now = self.get_clock().now().nanoseconds*1e-9
        self.last_odometry_age = now-stamp_seconds(msg.header.stamp)
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        values = (v.x, v.y, w.z)
        if (not all(math.isfinite(a) for a in values)
                or not 0 <= now-stamp_seconds(msg.header.stamp) <= .25
                or math.hypot(v.x, v.y) > .035 or abs(w.z) > .10):
            self.quiet_since = None
            return
        self.odometry_time = time.monotonic()
        self.quiet_since = self.quiet_since or self.odometry_time

    def settled(self):
        now = time.monotonic()
        return self.quiet_since is not None and now-self.quiet_since >= .25 and now-self.odometry_time < .25

    def camera_geometry(self, image, camera):
        if not image.header.frame_id:
            raise ValueError('Camera image needs a frame ID')
        profile = self.calibration
        c = profile['camera'] if profile else None
        if c:
            if (c['width'], c['height'], c['frame_id']) != (image.width, image.height, image.header.frame_id):
                raise ValueError('Calibration image resolution/frame does not match this camera')
            k, d, source = camera_matrix(image.width, image.height, c['k'], c['d'])
            s = profile['setup']
            rotation = mounting_rotation(profile['pitch'], profile['yaw'])
            translation = np.array([s['camera_x'], s['camera_y'], s['camera_height']])
            source = 'explicit legacy profile'
        else:
            if camera is not None and (camera.width != image.width or camera.height != image.height):
                raise ValueError('CameraInfo resolution does not match the image')
            if camera is not None and camera.header.frame_id != image.header.frame_id:
                raise ValueError('CameraInfo frame does not match the image')
            if camera is not None and camera.distortion_model not in ('', 'plumb_bob', 'rational_polynomial'):
                raise ValueError('Unsupported camera distortion model')
            k, d, source = camera_matrix(image.width, image.height,
                camera.k if camera is not None else None, camera.d if camera is not None else None,
                self.p['horizontal_fov'])
            if self.p['camera_height'] > 0:
                rotation = mounting_rotation(self.p['camera_pitch'], self.p['camera_yaw'])
                translation = np.array([self.p['camera_x'], self.p['camera_y'], self.p['camera_height']])
                source += ' + measured mount'
            else:
                transform = self.buffer.lookup_transform(self.p['base_frame'],
                    self.p['camera_optical_frame'], Time.from_msg(image.header.stamp))
                rotation, translation = transform_arrays(transform)
                source += ' + timestamped TF mount'
        geometry = FloorCamera(image.width, image.height, k, d, rotation, translation, source,
            self.p['camera_min_range'], self.p['camera_max_range'], self.p['pixel_margin'],
            self.p['pixel_sigma'], self.p['height_sigma'], self.p['pitch_sigma'])
        if profile:
            geometry.min_range = max(geometry.min_range, profile['range_m'][0]-.05)
            geometry.max_range = min(geometry.max_range, profile['range_m'][1]+.05)
            geometry.bearing_limits = [profile['bearing_rad'][0]-.02, profile['bearing_rad'][1]+.02]
            geometry.__post_init__()
        return geometry

    def warn(self, text):
        if time.monotonic() - self.last_warning > 5:
            self.get_logger().warning(text)
            self.last_warning = time.monotonic()

    def locate(self, box, image, camera, scans):
        if self.p['localization_method'] == 'ground_plane' and self.calibration is not None:
            profile = self.calibration
            c = profile['camera']
            if (c['width'], c['height'], c['frame_id']) != (image.width, image.height, image.header.frame_id):
                raise ValueError('Calibration image resolution/frame does not match this camera')
            x1, _, x2, y2 = box
            point = floor_point(profile, [(x1+x2)/2, y2], self.p['max_range'])
            s = profile['setup']
            if not in_envelope(profile, point[0]-s['camera_x'], point[1]-s['camera_y']):
                return None
            transform = self.buffer.lookup_transform(self.p['map_frame'], profile['base_frame'], Time.from_msg(image.header.stamp))
            rotation, translation = transform_arrays(transform)
            return rotation@point+translation
        if self.p['localization_method'] == 'ground_plane':
            geometry = self.floor_camera or self.camera_geometry(image, camera)
            point, sigma, distance = geometry.project_box(box, coarse_max_range=self.p['max_range'])
            self.geometric_sigma = sigma
            self.projection_in_range = distance <= geometry.max_range
            transform = self.buffer.lookup_transform(self.p['map_frame'], self.p['base_frame'],
                                                      Time.from_msg(image.header.stamp))
            rotation, translation = transform_arrays(transform)
            self.projection_sigma = math.sqrt(sigma*sigma+self.pose_variance[0]+distance*distance*self.pose_variance[1])
            return rotation@point+translation
        profile = self.calibration
        c = profile['camera'] if profile else None
        if c and (c['width'] != image.width or c['height'] != image.height or c['frame_id'] != image.header.frame_id):
            raise ValueError('Calibration image resolution/frame does not match this camera')
        k = np.array(c['k'] if c else camera.k, dtype=float).reshape(3, 3)
        d = np.array(c['d'] if c else camera.d, dtype=float)
        if not c and (camera.width != image.width or camera.height != image.height or k[0,0] <= 0):
            raise ValueError('CameraInfo resolution/intrinsics do not match the image')
        image_time = Time.from_msg(image.header.stamp)
        frame = self.p['camera_optical_frame']
        if not scans:
            return None
        stamp = stamp_seconds(image.header.stamp)
        scan = min(scans, key=lambda s: abs(stamp_seconds(s.header.stamp)-stamp))
        if abs(stamp_seconds(scan.header.stamp)-stamp) > self.p['max_scan_delta']:
            return None
        ranges = np.array(scan.ranges)
        indices = np.flatnonzero(np.isfinite(ranges) & (ranges >= max(0.08, scan.range_min))
                                 & (ranges <= min(self.p['max_range'], scan.range_max)))
        angles = scan.angle_min + indices * scan.angle_increment
        points = np.column_stack([ranges[indices]*np.cos(angles),
                                  ranges[indices]*np.sin(angles), np.zeros(len(indices))])
        scan_time = Time.from_msg(scan.header.stamp)
        # Full lookup compensates robot motion between scan and image capture.
        transform = self.buffer.lookup_transform_full(
            profile['base_frame'] if profile else frame, image_time,
            scan.header.frame_id, scan_time, self.p['map_frame'])
        rotation, translation = transform_arrays(transform)
        camera_points = points @ rotation.T + translation
        if profile:
            setup = profile['setup']
            camera_points = (camera_points-np.array([setup['camera_x'], setup['camera_y'], setup['camera_height']])) @ mounting_rotation(profile['pitch'], profile['yaw'])
        point = associate_lidar(camera_points, points, indices, box, k, d,
                                self.p['min_lidar_points'], self.p['cluster_gap'],
                                self.p['max_cluster_width'])
        if point is None:
            return None
        transform = self.buffer.lookup_transform(self.p['map_frame'],
                                                  scan.header.frame_id, scan_time)
        rotation, translation = transform_arrays(transform)
        return rotation @ point + translation

    def process(self):
        with self.lock:
            image, camera, scans = self.latest_image, self.camera, list(self.scans)
        if image is None:
            return
        stamp = stamp_seconds(image.header.stamp)
        if stamp == self.last_processed:
            return
        self.last_processed = stamp
        age = self.get_clock().now().nanoseconds*1e-9 - stamp
        if not 0 <= age <= self.p['max_image_age']:
            self.warn('Stale image or unsynchronized clocks; refusing target coordinates')
            return
        observations = []
        detections = []
        detected_confidences=[]
        geometry_error = None
        started = time.monotonic()
        try:
            frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            result = self.model.predict(frame, imgsz=self.p['image_size'],
                                        conf=self.p['confidence'], device='cpu',
                                        verbose=False)[0]
            self.floor_camera = None
            if self.p['localization_method'] == 'ground_plane':
                try:
                    self.floor_camera = self.camera_geometry(image, camera)
                except (TransformException, ValueError, cv2.error) as exc:
                    geometry_error = str(exc)
            annotated = result.plot()
            for b in result.boxes:
                label = result.names[int(b.cls.item())]
                if label.casefold() != self.p['target_class'].casefold():
                    continue
                detected_confidences.append(float(b.conf.item()))
                box = b.xyxy[0].cpu().numpy().astype(float)
                detection = {'stamp': stamp, 'label': label, 'confidence': float(b.conf.item()),
                             'box': box.tolist(), 'reason': geometry_error}
                point = None
                self.projection_sigma = self.p['position_sigma']
                self.geometric_sigma = self.p['position_sigma']
                self.projection_in_range = True
                if self.floor_camera is not None or (self.p['localization_method'] == 'lidar' and (camera is not None or self.calibration is not None)):
                    try:
                        point = self.locate(box, image, camera, scans)
                    except (TransformException, ValueError, cv2.error) as exc:
                        detection['reason'] = str(exc)
                        self.warn('Target projection unavailable: ' + str(exc))
                if self.floor_camera is not None:
                    try:
                        tr = self.buffer.lookup_transform(self.p['map_frame'], self.p['base_frame'], Time.from_msg(image.header.stamp))
                        rot, origin = transform_arrays(tr)
                        ray = cv2.undistortPoints(np.array([[[(box[0]+box[2])/2, (box[1]+box[3])/2]]]), self.floor_camera.k, self.floor_camera.d).reshape(2)
                        direction = rot@self.floor_camera.rotation@np.r_[ray, 1.]
                        detection.update(bearing=math.atan2(direction[1], direction[0]),
                            origin=(rot@self.floor_camera.translation+origin)[:2].tolist())
                    except (TransformException, ValueError, cv2.error):
                        pass
                if point is not None and np.all(np.isfinite(point)):
                    detection.update(x=float(point[0]), y=float(point[1]), sigma=self.projection_sigma,
                                     close_view=self.projection_in_range and self.geometric_sigma <= self.p['max_report_sigma'])
                    reliable = self.projection_in_range and self.projection_sigma <= self.p['max_report_sigma']
                    if not reliable:
                        detection['reason'] = 'Position uncertainty is high; seek a closer view'
                    if self.p['localization_method'] == 'ground_plane' and not self.settled():
                        reliable = False
                        detection['reason'] = 'Waiting for settled odometry before reporting floor coordinates'
                    if reliable:
                        observations.append({'stamp': stamp, 'x': float(point[0]),
                                         'y': float(point[1]), 'label': label,
                                         'confidence': float(b.conf.item()),
                                         'sigma': self.projection_sigma,
                                         'method': self.p['localization_method']})
                    caption = (f'map: ({point[0]:.2f}, {point[1]:.2f}) m' if reliable else
                               f'candidate: ({point[0]:.2f}, {point[1]:.2f}) +/- {self.projection_sigma:.2f} m')
                    cv2.putText(annotated, caption,
                                (int(box[0]), max(20, int(box[3])+20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
                detections.append(detection)
            out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out.header = image.header
            self.image_pub.publish(out)
            # Publish even an empty result, so misses break confirmation streaks.
            self.obs_pub.publish(String(data=json.dumps({'stamp': stamp,
                                                          'observations': observations})))
            self.detections_pub.publish(String(data=json.dumps({'stamp': stamp, 'detections': detections})))
            self.status_pub.publish(String(data=json.dumps({
                'stamp': stamp, 'ready': True, 'latency': time.monotonic()-started,
                'localized_candidates': len(observations),
                'detected_candidates':len(detected_confidences),
                'highest_confidence':max(detected_confidences) if detected_confidences else None,
                'target_class': self.p['target_class'],
                'geometry_ready': self.floor_camera is not None if self.p['localization_method'] == 'ground_plane' else camera is not None,
                'geometry_error': geometry_error,
                'observation_ready': self.settled() if self.p['localization_method'] == 'ground_plane' else True,
                'odometry_age': self.last_odometry_age,
                'quiet_seconds': time.monotonic()-self.quiet_since if self.quiet_since is not None else None,
                'camera_model': self.floor_camera.metadata() if self.floor_camera is not None else None,
                'calibration_file': self.p['calibration_file'] or None,
                'localization_method': self.p['localization_method']})))
        except Exception as exc:
            self.warn('Perception failed: ' + str(exc))
            self.obs_pub.publish(String(data=json.dumps({'stamp': stamp,
                                                          'observations': []})))


def main():
    rclpy.init()
    node = Perception()
    executor = MultiThreadedExecutor(num_threads=2)
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
