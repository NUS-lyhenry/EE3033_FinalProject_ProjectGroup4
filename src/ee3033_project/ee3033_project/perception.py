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

from .core import associate_lidar, stamp_seconds, quaternion_yaw, angle_difference
from .observation_motion import ObservationMotion
from .ros_utils import parameters, transform_arrays
from .model_labels import validate_target
from .calibration import load_profile, mounting_rotation, floor_point, in_envelope
from .vision_geometry import FloorCamera, camera_matrix, override_pitch
from .capture_frame import CaptureFrameGate


class Perception(Node):
    def __init__(self, **kwargs):
        super().__init__('project_perception', **kwargs)
        self.p = parameters(self, {
            'model': '/home/ros2_ws/weights/yolo26n.pt', 'target_class': 'cup',
            'confidence': 0.60, 'image_topic': '/image_raw',
            'camera_info_topic': '/camera_info', 'scan_topic': '/scan_fixed',
            'camera_optical_frame': 'project_camera_optical', 'map_frame': 'map',
            'localization_method': 'hybrid', 'max_scan_delta': 0.25,
            'max_image_age': 1.5, 'inference_period': 0.25, 'image_size': 416,
            'max_range': 3.0, 'ground_z': 0.0, 'position_sigma': 0.10,
            'min_lidar_points': 2, 'cluster_gap': 0.12, 'max_cluster_width': 0.5,
            'cpu_threads': 1,
            'calibration_file': '',
            'base_frame': 'base_footprint', 'camera_height': 0., 'camera_pitch': 0.,
            'camera_yaw': 0., 'use_pitch_override': False, 'odom_frame': 'odom_combined', 'camera_x': 0., 'camera_y': 0.,
            'horizontal_fov': 0., 'camera_min_range': .35, 'camera_max_range': 2.,
            'pixel_margin': 8., 'pixel_sigma': 2., 'height_sigma': .015,
            'pitch_sigma': .034906585, 'max_report_sigma': .10,
            'observation_window': .6, 'observation_linear_mean': .05,
            'observation_angular_mean': .12, 'observation_position_span': .04,
            'observation_heading_span': .10,
        })
        if self.p['localization_method'] not in ('lidar', 'ground_plane', 'hybrid'):
            raise ValueError('localization_method must be lidar, ground_plane or hybrid')
        self.calibration = load_profile(self.p['calibration_file']) if self.p['calibration_file'] else None
        self.model = self.load_detector()
        validate_target(self.model.names,self.p['target_class'])
        self.bridge = CvBridge()
        self.buffer = Buffer(cache_time=Duration(seconds=20))
        self.listener = TransformListener(self.buffer, self)
        self.lock = threading.Lock()
        self.latest_image = None
        self.capture_frame_gate = CaptureFrameGate()
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
        self.motion = ObservationMotion(self.p['observation_window'], self.p['observation_linear_mean'],
            self.p['observation_angular_mean'], self.p['observation_position_span'],
            self.p['observation_heading_span'])
        self.motion_check = None
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
        p = msg.pose.pose.position
        with self.lock:
            self.motion.add(stamp_seconds(msg.header.stamp), p.x, p.y,
                            quaternion_yaw(msg.pose.pose.orientation), math.hypot(v.x, v.y),
                            w.z, msg.header.frame_id)
        values = (v.x, v.y, w.z)
        if (not all(math.isfinite(a) for a in values)
                or not 0 <= now-stamp_seconds(msg.header.stamp) <= .25
                or math.hypot(v.x, v.y) > .035 or abs(w.z) > .10):
            self.quiet_since = None
            return
        self.odometry_time = time.monotonic()
        self.quiet_since = self.quiet_since or self.odometry_time

    def settled(self):
        return bool(self.motion_check and self.motion_check['verified'])

    def diagnostic_frame(self,image,stamp,detections=(),reason_codes=(),error=None):
        return dict(image_frame=image.header.frame_id,width=image.width,height=image.height,
            frame_age=self.get_clock().now().nanoseconds*1e-9-stamp,
            target_class=self.p['target_class'],localization_method=self.p['localization_method'],
            detector_confidence_threshold=self.p['confidence'],motion_check=self.motion_check,
            odometry_age=self.last_odometry_age,detections=list(detections)[:16],
            detection_count=len(detections),detections_truncated=len(detections)>16,
            reason_codes=list(reason_codes),error=error)

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
                if self.p.get('use_pitch_override', False):
                    rotation = override_pitch(rotation, self.p['camera_pitch'])
                    source += ' + explicit pitch override (unverified)'
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
        if self.p['localization_method'] in ('ground_plane', 'hybrid') and self.calibration is not None:
            profile = self.calibration
            c = profile['camera']
            if (c['width'], c['height'], c['frame_id']) != (image.width, image.height, image.header.frame_id):
                raise ValueError('Calibration image resolution/frame does not match this camera')
            x1, y1, x2, y2 = map(float,box)
            margin=self.p['pixel_margin']
            if (not np.isfinite([x1,y1,x2,y2]).all()
                    or not margin<=x1<x2<=image.width-margin
                    or not margin<=y1<y2<=image.height-margin
                    or x2-x1<4 or y2-y1<8):
                raise ValueError('Target is too small or clipped; seek a better view')
            point = floor_point(profile, [(x1+x2)/2, y2], self.p['max_range'])
            s = profile['setup']
            distance=math.hypot(point[0]-s['camera_x'],point[1]-s['camera_y'])
            if distance<self.p['camera_min_range']:
                raise ValueError('Target contact is outside the reliable camera range')
            self.projection_in_range=distance<=self.p['camera_max_range']
            if not in_envelope(profile, point[0]-s['camera_x'], point[1]-s['camera_y']):
                return None
            transform = self.buffer.lookup_transform(self.p['map_frame'], profile['base_frame'], Time.from_msg(image.header.stamp))
            rotation, translation = transform_arrays(transform)
            return rotation@point+translation
        if self.p['localization_method'] in ('ground_plane', 'hybrid'):
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
        if (self.capture_frame_gate.pending is None and image is not None
                and stamp_seconds(image.header.stamp) == self.last_processed):
            return
        def capture_transform_ready(frame):
            if self.p['localization_method'] == 'lidar':
                return True
            stamp = stamp_seconds(frame.header.stamp)
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if not 0 <= age <= self.p['max_image_age']:
                return True  # Preserve the existing stale-frame diagnostics.
            try:
                return self.buffer.can_transform(self.p['map_frame'], self.p['base_frame'],
                                                 Time.from_msg(frame.header.stamp))
            except (TransformException, ValueError):
                return True  # The normal detection path records the error.
        image = self.capture_frame_gate.select(image, capture_transform_ready)
        if image is None:
            return
        stamp = stamp_seconds(image.header.stamp)
        if stamp == self.last_processed:
            return
        self.last_processed = stamp
        with self.lock:
            self.motion_check = self.motion.evaluate(stamp)
        age = self.get_clock().now().nanoseconds*1e-9 - stamp
        if not 0 <= age <= self.p['max_image_age']:
            self.warn('Stale image or unsynchronized clocks; refusing target coordinates')
            self.obs_pub.publish(String(data=json.dumps(dict(stamp=stamp,observations=[],bearings=[],
                diagnostics=self.diagnostic_frame(image,stamp,reason_codes=['perception_image_stale_or_future'])))))
            return
        observations = []
        detections = []
        bearings = []
        detected_confidences=[]
        deferred = False
        geometry_error = None
        frame_codes=[]
        started = time.monotonic()
        try:
            frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            result = self.model.predict(frame, imgsz=self.p['image_size'],
                                        conf=self.p['confidence'], device='cpu',
                                        verbose=False)[0]
            self.floor_camera = None
            if self.p['localization_method'] in ('ground_plane', 'hybrid'):
                try:
                    self.floor_camera = self.camera_geometry(image, camera)
                except (TransformException, ValueError, cv2.error) as exc:
                    geometry_error = str(exc)
                    frame_codes.append('camera_geometry_tf_error' if isinstance(exc,TransformException) else 'camera_geometry_invalid')
            annotated = result.plot()
            for b in result.boxes:
                label = result.names[int(b.cls.item())]
                if label.casefold() != self.p['target_class'].casefold():
                    continue
                detected_confidences.append(float(b.conf.item()))
                box = b.xyxy[0].cpu().numpy().astype(float)
                detection = {'stamp': stamp, 'label': label, 'confidence': float(b.conf.item()),
                             'box': box.tolist(), 'reason': geometry_error,'reason_codes':[],
                             'settled':self.settled(),'motion_check':self.motion_check,
                             'frame_age':self.get_clock().now().nanoseconds*1e-9-stamp}
                if box[0]<self.p['pixel_margin'] or box[2]>image.width-self.p['pixel_margin']:
                    detection['reason_codes'].append('bearing_horizontal_clipping')
                if box[1]<self.p['pixel_margin'] or box[3]>image.height-self.p['pixel_margin']:
                    detection['reason_codes'].append('detection_vertical_clipping')
                if not detection['settled']:detection['reason_codes'].append('capture_motion_unsettled')
                if self.floor_camera is not None:
                    detection.update(self.floor_camera.view_requirements(box))
                point = None
                self.projection_sigma = self.p['position_sigma']
                self.geometric_sigma = self.p['position_sigma']
                self.projection_in_range = True
                if self.floor_camera is not None or (self.p['localization_method'] == 'lidar' and (camera is not None or self.calibration is not None)):
                    try:
                        point = self.locate(box, image, camera, scans)
                    except (TransformException, ValueError, cv2.error) as exc:
                        detection['reason'] = str(exc)
                        detection['projection_error']=str(exc)
                        detection['reason_codes'].append('projection_tf_error' if isinstance(exc,TransformException) else 'projection_geometry_rejected')
                        self.warn('Target projection unavailable: ' + str(exc))
                if self.floor_camera is not None:
                    detection['bearing_stage']='map_transform'
                    try:
                        tr = self.buffer.lookup_transform(self.p['map_frame'], self.p['base_frame'], Time.from_msg(image.header.stamp))
                        rot, origin = transform_arrays(tr)
                        ray = cv2.undistortPoints(np.array([[[(box[0]+box[2])/2, (box[1]+box[3])/2]]]), self.floor_camera.k, self.floor_camera.d).reshape(2)
                        direction = rot@self.floor_camera.rotation@np.r_[ray, 1.]
                        if not np.isfinite(direction).all() or np.linalg.norm(direction[:2])<=1e-8:
                            raise ValueError('Nonfinite or vertical bearing ray')
                        horizontal_valid = (box[0] >= self.p['pixel_margin']
                            and box[2] <= image.width-self.p['pixel_margin'] and box[2]-box[0] >= 4)
                        # A cropped horizontal centre can guide a bounded
                        # retreat but is never geometric localization evidence.
                        detection['steering_bearing'] = math.atan2(direction[1], direction[0])
                        detection['steering_origin']=(rot@self.floor_camera.translation+origin)[:2].tolist()
                        detection['bearing_stage']='map_ray'
                        if not horizontal_valid:
                            raise ValueError('Horizontal clipping; reframe before measuring a bearing')
                        detection.update(bearing=math.atan2(direction[1], direction[0]),
                            origin=(rot@self.floor_camera.translation+origin)[:2].tolist())
                        optical_direction = self.floor_camera.rotation@np.r_[ray, 1.]
                        center_error = angle_difference(math.atan2(optical_direction[1], optical_direction[0]),self.floor_camera.yaw)
                        detection.update(center_error=center_error, settled=self.settled(),
                            bearing_sigma=math.sqrt((self.p['pixel_sigma']/self.floor_camera.k[0,0])**2+self.pose_variance[1]),
                            position_sigma=math.sqrt(max(.0004, self.pose_variance[0])))
                        if abs(center_error)>math.radians(2.):detection['reason_codes'].append('bearing_not_centered')
                        detection['bearing_stage']='odom_transform'
                        odom_tf = self.buffer.lookup_transform(self.p['odom_frame'], self.p['base_frame'], Time.from_msg(image.header.stamp))
                        odom_rot, odom_translation = transform_arrays(odom_tf)
                        map_tf = self.buffer.lookup_transform(self.p['map_frame'], self.p['odom_frame'], Time.from_msg(image.header.stamp))
                        map_rot, map_translation = transform_arrays(map_tf)
                        odom_direction = odom_rot@optical_direction
                        detection.update(odom_origin=(odom_rot@self.floor_camera.translation+odom_translation)[:2].tolist(),
                            odom_bearing=math.atan2(odom_direction[1], odom_direction[0]),
                            map_odom=[float(map_translation[0]), float(map_translation[1]), math.atan2(map_rot[1,0], map_rot[0,0])])
                        if not np.isfinite([*detection['odom_origin'],detection['odom_bearing'],*detection['map_odom']]).all():
                            raise ValueError('Nonfinite continuous-odometry bearing geometry')
                        detection['bearing_stage']='complete'
                        detection['bearing_valid']=True
                        if self.p['localization_method']=='hybrid':
                            bearings.append(dict(detection))
                    except (TransformException, ValueError, cv2.error) as exc:
                        detection['bearing_valid']=False
                        detection['bearing_error']=str(exc)
                        detection['reason_codes'].append('bearing_tf_error' if isinstance(exc,TransformException) else 'bearing_geometry_rejected')
                elif self.p['localization_method']=='hybrid':
                    detection['bearing_valid']=False
                    detection['reason_codes'].append('bearing_camera_geometry_unavailable')
                if point is not None and np.all(np.isfinite(point)):
                    reliable = self.projection_in_range and self.projection_sigma <= self.p['max_report_sigma']
                    detection.update(x=float(point[0]), y=float(point[1]), sigma=self.projection_sigma,
                                     close_view=reliable)
                    if not reliable:
                        detection['reason'] = 'Position uncertainty exceeds preferred precision; seeking a better view'
                        detection['reason_codes'].append('projection_uncertain_or_out_of_range')
                    usable = self.projection_in_range
                    if self.p['localization_method'] in ('ground_plane', 'hybrid') and not self.settled():
                        usable = False
                        deferred = True
                        detection['reason'] = 'Collecting a bounded capture-time motion window'
                    if self.p['localization_method'] in ('ground_plane', 'hybrid') and self.motion_check:
                        span = self.motion_check.get('position_span') or 0.
                        heading = self.motion_check.get('heading_span') or 0.
                        # Balance motion contributes uncertainty even when the window passes.
                        distance = float(np.linalg.norm(point[:2]-np.asarray(detection.get('origin', point[:2]))))
                        self.projection_sigma = math.sqrt(self.projection_sigma**2+(span/2)**2+(distance*heading/2)**2)
                        detection['sigma'] = self.projection_sigma
                        reliable = self.projection_in_range and self.projection_sigma <= self.p['max_report_sigma']
                        detection['close_view'] = reliable
                    if usable:
                        observations.append({'stamp': stamp, 'x': float(point[0]),
                                         'y': float(point[1]), 'label': label,
                                         'confidence': float(b.conf.item()),
                                         'sigma': self.projection_sigma,
                                         'box': box.tolist(), 'source_frame': stamp,
                                         'method': self.p['localization_method']})
                    detection['projection_valid']=bool(usable)
                    caption = (f'map: ({point[0]:.2f}, {point[1]:.2f}) m' if reliable and usable else
                               f'candidate: ({point[0]:.2f}, {point[1]:.2f}) +/- {self.projection_sigma:.2f} m')
                    cv2.putText(annotated, caption,
                                (int(box[0]), max(20, int(box[3])+20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
                else:
                    detection['projection_valid']=False
                detections.append(detection)
            out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out.header = image.header
            self.image_pub.publish(out)
            # Publish even an empty result, so misses break confirmation streaks.
            self.obs_pub.publish(String(data=json.dumps({'stamp': stamp,
                'observations': observations, 'bearings': bearings, 'deferred': deferred,
                'diagnostics':self.diagnostic_frame(image,stamp,detections,
                    frame_codes+([] if detections else ['detector_no_target_above_threshold']))})))
            self.detections_pub.publish(String(data=json.dumps({'stamp': stamp, 'detections': detections})))
            self.status_pub.publish(String(data=json.dumps({
                'stamp': stamp, 'ready': True, 'latency': time.monotonic()-started,
                'localized_candidates': len(observations),
                'detected_candidates':len(detected_confidences),
                'highest_confidence':max(detected_confidences) if detected_confidences else None,
                'target_class': self.p['target_class'],
                'geometry_ready': self.floor_camera is not None if self.p['localization_method'] in ('ground_plane', 'hybrid') else camera is not None,
                'geometry_error': geometry_error,
                'observation_ready': self.settled() if self.p['localization_method'] in ('ground_plane', 'hybrid') else True,
                'motion_check': self.motion_check,
                'frame_age': self.get_clock().now().nanoseconds*1e-9-stamp,
                'projection_failures': [d['reason'] for d in detections if 'x' not in d],
                'odometry_age': self.last_odometry_age,
                'quiet_seconds': time.monotonic()-self.quiet_since if self.quiet_since is not None else None,
                'camera_model': self.floor_camera.metadata() if self.floor_camera is not None else None,
                'calibration_file': self.p['calibration_file'] or None,
                'localization_method': self.p['localization_method']})))
        except Exception as exc:
            self.warn('Perception failed: ' + str(exc))
            self.obs_pub.publish(String(data=json.dumps({'stamp': stamp,
                'observations': [],'bearings':[],
                'diagnostics':self.diagnostic_frame(image,stamp,detections,
                    frame_codes+['perception_processing_error'],error=type(exc).__name__+': '+str(exc))})))


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
