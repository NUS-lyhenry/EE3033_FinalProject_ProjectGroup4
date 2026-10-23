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
from std_msgs.msg import String
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException

from .core import associate_lidar, ground_intersection, stamp_seconds
from .ros_utils import parameters, transform_arrays
from .model_labels import validate_target
from .calibration import load_profile, mounting_rotation, floor_point, in_envelope


class Perception(Node):
    def __init__(self):
        super().__init__('project_perception')
        self.p = parameters(self, {
            'model': '/home/ros2_ws/weights/yolo26n.pt', 'target_class': 'cup',
            'confidence': 0.60, 'image_topic': '/image_raw',
            'camera_info_topic': '/camera_info', 'scan_topic': '/scan_fixed',
            'camera_optical_frame': 'project_camera_optical', 'map_frame': 'map',
            'localization_method': 'lidar', 'max_scan_delta': 0.25,
            'max_image_age': 1.5, 'inference_period': 0.25, 'image_size': 416,
            'max_range': 3.0, 'ground_z': 0.0, 'position_sigma': 0.10,
            'min_lidar_points': 2, 'cluster_gap': 0.12, 'max_cluster_width': 0.5,
            'cpu_threads': 1,
            'calibration_file': '',
        })
        if self.p['localization_method'] not in ('lidar', 'ground_plane'):
            raise ValueError('localization_method must be lidar or ground_plane')
        self.calibration = load_profile(self.p['calibration_file']) if self.p['calibration_file'] else None
        if not os.path.isfile(self.p['model']):
            raise FileNotFoundError('Set model to an existing YOLO weights file')
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(self.p['cpu_threads'])
        self.model = YOLO(self.p['model'])
        validate_target(self.model.names,self.p['target_class'])
        self.model.to('cpu')
        self.bridge = CvBridge()
        self.buffer = Buffer(cache_time=Duration(seconds=20))
        self.listener = TransformListener(self.buffer, self)
        self.lock = threading.Lock()
        self.latest_image = None
        self.camera = None
        self.scans = deque(maxlen=40)
        self.last_processed = None
        self.last_warning = 0.0
        group = ReentrantCallbackGroup()
        self.create_subscription(Image, self.p['image_topic'], self.on_image,
                                 qos_profile_sensor_data, callback_group=group)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self.on_camera,
                                 qos_profile_sensor_data, callback_group=group)
        self.create_subscription(LaserScan, self.p['scan_topic'], self.on_scan,
                                 qos_profile_sensor_data, callback_group=group)
        self.image_pub = self.create_publisher(Image, '/project/detection_image',
                                                qos_profile_sensor_data)
        self.obs_pub = self.create_publisher(String, '/project/target_observations', 10)
        self.status_pub = self.create_publisher(String, '/project/perception_status', 10)
        self.timer = self.create_timer(self.p['inference_period'], self.process)
        self.get_logger().info('YOLO ready; target=' + self.p['target_class'])

    def on_image(self, msg):
        with self.lock:
            self.latest_image = msg

    def on_camera(self, msg):
        with self.lock:
            self.camera = msg

    def on_scan(self, msg):
        with self.lock:
            self.scans.append(msg)

    def warn(self, text):
        if time.monotonic() - self.last_warning > 5:
            self.get_logger().warning(text)
            self.last_warning = time.monotonic()

    def locate(self, box, image, camera, scans):
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
        if self.p['localization_method'] == 'ground_plane':
            x1, _, x2, y2 = box
            if profile:
                point = floor_point(profile, [0.5*(x1+x2), y2], self.p['max_range'])
                setup = profile['setup']
                if not in_envelope(profile, point[0]-setup['camera_x'], point[1]-setup['camera_y']):
                    return None
                transform = self.buffer.lookup_transform(self.p['map_frame'], profile['base_frame'], image_time)
                rotation, translation = transform_arrays(transform)
                return rotation @ point + translation
            ray2d = cv2.undistortPoints(np.array([[[0.5*(x1+x2), y2]]],
                                                dtype=np.float64), k, d).reshape(2)
            transform = self.buffer.lookup_transform(self.p['map_frame'], frame,
                                                      image_time)
            rotation, translation = transform_arrays(transform)
            return ground_intersection([ray2d[0], ray2d[1], 1.0], rotation, translation,
                                       self.p['ground_z'], self.p['max_range'])
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
        detected_confidences=[]
        started = time.monotonic()
        try:
            frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            result = self.model.predict(frame, imgsz=self.p['image_size'],
                                        conf=self.p['confidence'], device='cpu',
                                        verbose=False)[0]
            annotated = result.plot()
            for b in result.boxes:
                label = result.names[int(b.cls.item())]
                if label.casefold() != self.p['target_class'].casefold():
                    continue
                detected_confidences.append(float(b.conf.item()))
                box = b.xyxy[0].cpu().numpy().astype(float)
                point = None
                if camera is not None or self.calibration is not None:
                    try:
                        point = self.locate(box, image, camera, scans)
                    except (TransformException, ValueError, cv2.error) as exc:
                        self.warn('Target projection unavailable: ' + str(exc))
                if point is not None and np.all(np.isfinite(point)):
                    observations.append({'stamp': stamp, 'x': float(point[0]),
                                         'y': float(point[1]), 'label': label,
                                         'confidence': float(b.conf.item()),
                                         'sigma': self.p['position_sigma'],
                                         'method': self.p['localization_method']})
                    cv2.putText(annotated, f'map: ({point[0]:.2f}, {point[1]:.2f}) m',
                                (int(box[0]), max(20, int(box[3])+20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
            out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out.header = image.header
            self.image_pub.publish(out)
            # Publish even an empty result, so misses break confirmation streaks.
            self.obs_pub.publish(String(data=json.dumps({'stamp': stamp,
                                                          'observations': observations})))
            self.status_pub.publish(String(data=json.dumps({
                'stamp': stamp, 'ready': True, 'latency': time.monotonic()-started,
                'localized_candidates': len(observations),
                'detected_candidates':len(detected_confidences),
                'highest_confidence':max(detected_confidences) if detected_confidences else None,
                'target_class': self.p['target_class'],
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
