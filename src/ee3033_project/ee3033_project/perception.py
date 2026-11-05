"""YOLO image detections with timestamped camera/lidar-to-map projection."""
from collections import deque
import json
import hashlib
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
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException

from .core import associate_lidar, stamp_seconds, quaternion_yaw, angle_difference
from .observation_motion import ObservationMotion
from .ros_utils import parameters, transform_arrays
from .model_labels import validate_target
from .calibration import load_profile, mounting_rotation, floor_point, in_envelope
from .vision_geometry import FloorCamera, camera_matrix, override_pitch
from .bottom_range import BottomCamera, attitude_window, CALIBRATION_K, CALIBRATION_D, MODEL_SHA256
from .capture_frame import CaptureFrameGate
from .capture_covariance import capture_covariance
from .attitude_geometry import (ATTITUDE_TOPIC, AttitudeBuffer, load_config,
                               reference_valid, compensated_mount)


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
            'attitude_config': '', 'bottle_min_range': .35, 'bottle_max_range': .95,
        })
        if self.p['localization_method'] not in ('lidar', 'ground_plane', 'hybrid', 'bottom_inverse'):
            raise ValueError('localization_method must be lidar, ground_plane or hybrid')
        self.empirical = self.p['target_class'].casefold() == 'bottle'
        if self.p['localization_method'] == 'bottom_inverse' and not self.empirical:
            raise ValueError('Bottom inverse calibration is for bottle only')
        if getattr(self,'empirical',False):
            self.p['localization_method'] = 'bottom_inverse'
            # Ignore old pitch profiles, controller zero and camera offsets.
            self.p['calibration_file'] = self.p['attitude_config'] = ''
        self.calibration = load_profile(self.p['calibration_file']) if self.p['calibration_file'] else None
        self.attitude_config = load_config(self.p['attitude_config'])
        self.attitude_buffer = AttitudeBuffer()
        self.attitude_check = {'mode': self.attitude_config['mode'], 'valid': self.attitude_config['mode'] == 'fixed'}
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
        self.pose_history = deque(maxlen=80)
        self.covariance_odometry = deque(maxlen=8000)
        self.covariance_generation=0
        self.covariance_cutoff=0.
        self.covariance_map_signature=None
        self.covariance_map_odom=None
        self.capture_pose=None
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
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, 10, callback_group=group)
        self.create_subscription(OccupancyGrid, '/map', self.on_covariance_map, 1, callback_group=group)
        self.create_subscription(Odometry, '/odom', self.on_odometry, qos_profile_sensor_data, callback_group=group)
        if self.empirical or self.attitude_config['mode'] == 'controller':
            self.create_subscription(Vector3Stamped, ATTITUDE_TOPIC, self.on_attitude,
                                     qos_profile_sensor_data, callback_group=group)
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
        if getattr(self,'empirical',False):
            with open(self.p['model'],'rb') as stream:
                hasher=hashlib.sha256()
                for chunk in iter(lambda:stream.read(1024*1024),b''): hasher.update(chunk)
                digest=hasher.hexdigest()
            if digest!=MODEL_SHA256 or self.p['image_size']!=416:
                raise ValueError('Bottle calibration requires the collected YOLO weights and image_size=416')
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

    def on_attitude(self, msg):
        with self.lock:
            self.attitude_buffer.add(stamp_seconds(msg.header.stamp), msg.vector.x,
                                     msg.vector.y, msg.header.frame_id)

    def capture_attitude(self, stamp):
        if getattr(self,'empirical',False):
            try:
                with self.lock:
                    self.attitude_buffer.at(stamp)
                    self.attitude_check = attitude_window(list(self.attitude_buffer.samples), stamp)
                return None
            except ValueError as exc:
                with self.lock:
                    points=[p for p in self.attitude_buffer.samples if stamp-.6<=p[0]<=stamp]
                span=np.ptp(np.asarray([p[1:] for p in points]),axis=0).tolist() if len(points)>=2 else None
                self.attitude_check = dict(valid=False,mode='stability_only',compensation=False,reason=str(exc),
                    capture_stamp=stamp,**({'span':span} if span is not None else {}))
                raise
        config = self.attitude_config
        self.attitude_check = dict(mode=config['mode'], valid=config['mode'] == 'fixed',
                                   topic=ATTITUDE_TOPIC, capture_stamp=stamp)
        if config['mode'] == 'fixed':
            return None
        try:
            if not reference_valid(config, self.p):
                raise ValueError('attitude_reference_missing')
            with self.lock:
                self.attitude_buffer.latest(self.get_clock().now().nanoseconds*1e-9)
                sample = self.attitude_buffer.at(stamp)
            self.attitude_check.update(sample, valid=True)
            return sample
        except ValueError as exc:
            self.attitude_check.update(valid=False, reason=str(exc))
            raise

    def attitude_ready(self, stamp):
        try:
            self.capture_attitude(stamp)
            return True
        except ValueError:
            return False

    def reset_covariance_evidence(self, cutoff=None):
        with self.lock:
            self.pose_history.clear()
            self.covariance_odometry.clear()
            self.covariance_generation+=1
            self.covariance_cutoff=max(getattr(self,'covariance_cutoff',0.),
                self.get_clock().now().nanoseconds*1e-9 if cutoff is None else cutoff)
            self.covariance_map_odom=None

    def on_initial_pose(self,msg):
        self.reset_covariance_evidence()

    def on_covariance_map(self,msg):
        signature=(msg.info.width,msg.info.height,msg.info.resolution,
            msg.info.origin.position.x,msg.info.origin.position.y,
            quaternion_yaw(msg.info.origin.orientation),hash(bytes(v+1 for v in msg.data)))
        if self.covariance_map_signature is not None and signature!=self.covariance_map_signature:
            self.reset_covariance_evidence()
        self.covariance_map_signature=signature

    def capture_pose_for_image(self,image):
        stamp=stamp_seconds(image.header.stamp)
        if stamp<getattr(self,'covariance_cutoff',0.):return None
        try:
            transform=self.buffer.lookup_transform(self.p['map_frame'],self.p['base_frame'],Time.from_msg(image.header.stamp))
            rotation,translation=transform_arrays(transform)
            if not np.isfinite([*translation,*rotation.ravel()]).all():return None
            correction=self.buffer.lookup_transform(self.p['map_frame'],self.p['odom_frame'],Time.from_msg(image.header.stamp))
            r,t=transform_arrays(correction)
            current=(float(t[0]),float(t[1]),math.atan2(r[1,0],r[0,0]))
            previous=getattr(self,'covariance_map_odom',None)
            if previous is not None and (math.dist(current[:2],previous[:2])>.15 or abs(angle_difference(current[2],previous[2]))>.15):
                self.reset_covariance_evidence(cutoff=stamp)
            self.covariance_map_odom=current
            return dict(stamp=stamp,x=float(translation[0]),y=float(translation[1]),
                yaw=math.atan2(rotation[1,0],rotation[0,0]),frame_id=self.p['map_frame'])
        except (TransformException,ValueError):return None

    def on_pose(self, msg):
        c = msg.pose.covariance
        if stamp_seconds(msg.header.stamp)<getattr(self,'covariance_cutoff',0.):return
        if msg.header.frame_id == self.p['map_frame'] and all(math.isfinite(v) and v >= 0 for v in (c[0], c[7], c[35])):
            self.pose_variance = (max(c[0], c[7]), c[35])
            with self.lock:
                self.pose_history.append((stamp_seconds(msg.header.stamp),self.pose_variance))

    def on_odometry(self, msg):
        now = self.get_clock().now().nanoseconds*1e-9
        if stamp_seconds(msg.header.stamp)<getattr(self,'covariance_cutoff',0.):return
        self.last_odometry_age = now-stamp_seconds(msg.header.stamp)
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        p = msg.pose.pose.position
        with self.lock:
            stamp=stamp_seconds(msg.header.stamp)
            if not hasattr(self,'covariance_odometry'):self.covariance_odometry=deque(maxlen=8000)
            sample=(stamp,p.x,p.y,quaternion_yaw(msg.pose.pose.orientation),math.hypot(v.x,v.y),w.z,msg.header.frame_id)
            if all(math.isfinite(value) for value in sample[:6]) and sample[6]:
                if self.covariance_odometry and (stamp<self.covariance_odometry[-1][0] or sample[6]!=self.covariance_odometry[-1][6]):
                    self.pose_history.clear();self.covariance_odometry.clear()
                    self.covariance_generation=getattr(self,'covariance_generation',0)+1
                    self.covariance_cutoff=max(getattr(self,'covariance_cutoff',0.),stamp)
                    self.covariance_map_odom=None
                if not self.covariance_odometry or stamp>self.covariance_odometry[-1][0]:self.covariance_odometry.append(sample)
                while self.covariance_odometry and stamp-self.covariance_odometry[0][0]>20.:self.covariance_odometry.popleft()
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
            reason_codes=list(reason_codes),error=error,
            attitude=getattr(self, 'attitude_check', None),capture_pose=getattr(self,'capture_pose',None))

    def camera_geometry(self, image, camera):
        if getattr(self,'empirical',False):
            if (self.p['image_topic'] != '/image_raw' or camera is None
                    or camera.header.frame_id != image.header.frame_id
                    or (camera.width,camera.height)!=(image.width,image.height)):
                raise ValueError('Use original /image_raw and matching CameraInfo')
            k,d,_=camera_matrix(image.width,image.height,camera.k,camera.d)
            if (not np.allclose(k.ravel(),CALIBRATION_K,atol=1e-5,rtol=0)
                    or d.shape!=(5,) or not np.allclose(d,CALIBRATION_D,atol=1e-7,rtol=0)
                    or getattr(getattr(camera,'roi',None),'do_rectify',False)):
                raise ValueError('Camera intrinsics or rectification differ from the 17-point collection')
            # Stability is checked separately; moving frames still supply bearings.
            return BottomCamera(image.width,image.height,k,d,self.p['pixel_sigma'],self.p['pixel_margin'],
                self.p.get('bottle_min_range',.35),self.p.get('bottle_max_range',.95))
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
        sample = self.capture_attitude(stamp_seconds(image.header.stamp))
        pivot = None
        if sample is not None:
            reference = self.attitude_config['reference']
            pivot = self.attitude_config['pivot']
            # Use the currently loaded camera settings, rather than freezing
            # mounting geometry to the settings used at reference capture.
            translation = np.array(translation, dtype=float, copy=True)
            translation[0] = -.01
            rotation, translation = compensated_mount(rotation,
                translation, pivot, (reference['roll'], reference['pitch']),
                (sample['roll'], sample['pitch']))
            source += ' + capture-time controller roll/pitch and measured wheel pivot'
        geometry = FloorCamera(image.width, image.height, k, d, rotation, translation, source,
            self.p['camera_min_range'], self.p['camera_max_range'], self.p['pixel_margin'],
            self.p['pixel_sigma'], self.p['height_sigma'], self.p['pitch_sigma'])
        if pivot is not None:
            # Controller interpolation does not remove mounting uncertainty.
            # Add a timing term and roll sensitivity, rotating the lever too.
            geometry.attitude_pivot = list(pivot)
            geometry.roll_sigma = self.p['pitch_sigma']
            geometry.pitch_sigma = math.hypot(geometry.pitch_sigma, sample['rate']*.02)
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
        if getattr(self,'empirical',False):
            geometry=self.floor_camera or self.camera_geometry(image,camera)
            point,sigma,distance=geometry.project_box(box)
            self.geometric_sigma=sigma
            self.projection_in_range=True
            self.empirical_distance=distance
            self.empirical_view=geometry.view_requirements(box)
            transform=self.buffer.lookup_transform(self.p['map_frame'],self.p['base_frame'],Time.from_msg(image.header.stamp))
            rotation,translation=transform_arrays(transform)
            # Axle origin appears once, via map<-base. No optical TF translation.
            stamp=stamp_seconds(image.header.stamp)
            if stamp<getattr(self,'covariance_cutoff',0.):raise ValueError('Capture before localization reset')
            with self.lock:
                history=list(self.pose_history)
                odometry=list(getattr(self,'covariance_odometry',()))
            pose_variance,self.capture_pose_diagnostic=capture_covariance(history,stamp,odometry=odometry)
            self.capture_pose_diagnostic['generation']=getattr(self,'covariance_generation',0)
            if pose_variance is None:
                raise ValueError('Capture-time localization covariance missing or stale')
            swing=max(self.attitude_check.get('span',[0.,0.]))
            with self.lock:
                motion=(self.motion.uncertainty_at(stamp,distance) if hasattr(getattr(self,'motion',None),'uncertainty_at')
                    else dict(sigma=.05,verified=False,reason='capture_motion_unavailable'))
            self.capture_motion_diagnostic=motion
            if not self.attitude_check.get('valid'): sigma=math.hypot(sigma,.05)
            self.projection_sigma=math.sqrt(sigma*sigma+motion['sigma']**2+pose_variance[0]+
                distance*distance*(pose_variance[1]+swing*swing))
            return rotation@point+translation
        if (self.p['localization_method'] in ('ground_plane', 'hybrid', 'bottom_inverse') and self.calibration is not None
                and getattr(self, 'attitude_config', {'mode': 'fixed'})['mode'] == 'fixed'):
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
        if self.p['localization_method'] in ('ground_plane', 'hybrid', 'bottom_inverse'):
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
                                                 Time.from_msg(frame.header.stamp)) and (self.empirical or self.attitude_ready(stamp))
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
        if getattr(self,'empirical',False):
            self.attitude_ready(stamp)
        age = self.get_clock().now().nanoseconds*1e-9 - stamp
        if not 0 <= age <= self.p['max_image_age']:
            self.warn('Stale image or unsynchronized clocks; refusing target coordinates')
            self.obs_pub.publish(String(data=json.dumps(dict(stamp=stamp,observations=[],bearings=[],
                diagnostics=self.diagnostic_frame(image,stamp,reason_codes=['perception_image_stale_or_future'])))))
            return
        self.capture_pose=self.capture_pose_for_image(image)
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
            if self.empirical and (tuple(frame.shape[:2])!=(480,640)
                    or tuple(result.orig_shape)!=(480,640)):
                raise ValueError('YOLO boxes must be expressed in original 640x480 pixels')
            self.floor_camera = None
            if self.p['localization_method'] in ('ground_plane', 'hybrid', 'bottom_inverse'):
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
                box = b.xyxy[0].cpu().numpy().astype(float)
                detection = {'stamp': stamp, 'label': label, 'confidence': float(b.conf.item()),
                             'box': box.tolist(), 'image_width': image.width, 'image_height':image.height,
                             'reason': geometry_error,'reason_codes':[],
                             'settled':self.settled(),'motion_check':self.motion_check,'capture_pose':self.capture_pose,
                             'frame_age':self.get_clock().now().nanoseconds*1e-9-stamp,
                             'attitude':dict(self.attitude_check)}
                if box[0]<self.p['pixel_margin'] or box[2]>image.width-self.p['pixel_margin']:
                    detection['reason_codes'].append('bearing_horizontal_clipping')
                if box[1]<self.p['pixel_margin'] or box[3]>image.height-self.p['pixel_margin']:
                    detection['reason_codes'].append('detection_vertical_clipping')
                if not detection['settled']:detection['reason_codes'].append('capture_motion_unsettled')
                if self.floor_camera is not None:
                    detection.update(self.floor_camera.view_requirements(box))
                if self.empirical and not self.attitude_check.get('valid'):
                    detection['reason_codes'].append(self.attitude_check.get('reason','attitude_unverified'))
                if detection.get('search_eligible',True): detected_confidences.append(float(b.conf.item()))
                point = None
                self.projection_sigma = self.p['position_sigma']
                self.geometric_sigma = self.p['position_sigma']
                self.projection_in_range = True
                self.capture_pose_diagnostic = None
                if self.floor_camera is not None or (self.p['localization_method'] == 'lidar' and (camera is not None or self.calibration is not None)):
                    try:
                        point = self.locate(box, image, camera, scans)
                    except (TransformException, ValueError, cv2.error) as exc:
                        detection['reason'] = str(exc)
                        detection['projection_error']=str(exc)
                        detection['reason_codes'].append('projection_tf_error' if isinstance(exc,TransformException) else 'projection_geometry_rejected')
                        self.warn('Target projection unavailable: ' + str(exc))
                if self.capture_pose_diagnostic is not None:
                    detection['capture_pose_covariance'] = dict(self.capture_pose_diagnostic)
                if self.floor_camera is not None:
                    detection['bearing_stage']='map_transform'
                    try:
                        tr = self.buffer.lookup_transform(self.p['map_frame'], self.p['base_frame'], Time.from_msg(image.header.stamp))
                        rot, origin = transform_arrays(tr)
                        if getattr(self,'empirical',False):
                            base_ray=self.floor_camera.bearing_ray(box)
                        else:
                            ray = cv2.undistortPoints(np.array([[[(box[0]+box[2])/2, (box[1]+box[3])/2]]]), self.floor_camera.k, self.floor_camera.d).reshape(2)
                            base_ray=self.floor_camera.rotation@np.r_[ray, 1.]
                        direction = rot@base_ray
                        detection['capture_heading']=math.atan2(rot[1,0],rot[0,0])
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
                        optical_direction = base_ray
                        center_error = angle_difference(math.atan2(optical_direction[1], optical_direction[0]),self.floor_camera.yaw)
                        detection.update(center_error=center_error, settled=self.settled(),
                            bearing_sigma=math.sqrt((self.p['pixel_sigma']/self.floor_camera.k[0,0])**2+self.pose_variance[1]+
                                (self.floor_camera.pitch_sigma**2+self.floor_camera.roll_sigma**2
                                 if self.floor_camera.attitude_pivot is not None else 0.)),
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
                elif self.p['localization_method'] in ('hybrid','bottom_inverse'):
                    detection['bearing_valid']=False
                    detection['reason_codes'].append('bearing_camera_geometry_unavailable')
                if point is not None and np.all(np.isfinite(point)):
                    reliable = self.projection_in_range and self.projection_sigma <= self.p['max_report_sigma']
                    if getattr(self,'empirical',False):
                        detection['range_m']=self.empirical_distance
                    detection.update(x=float(point[0]), y=float(point[1]), sigma=self.projection_sigma,
                                     close_view=reliable)
                    if not reliable:
                        detection['reason'] = 'Position uncertainty exceeds preferred precision; seeking a better view'
                        detection['reason_codes'].append('projection_uncertain_or_out_of_range')
                    usable = self.projection_in_range
                    if not self.empirical and self.p['localization_method'] in ('ground_plane', 'hybrid') and not self.settled():
                        usable = False
                        deferred = True
                        detection['reason'] = 'Collecting a bounded capture-time motion window'
                    if self.p['localization_method'] in ('ground_plane', 'hybrid', 'bottom_inverse') and self.motion_check:
                        span = self.motion_check.get('position_span') or 0.
                        heading = self.motion_check.get('heading_span') or 0.
                        # Balance motion contributes uncertainty even when the window passes.
                        distance = float(np.linalg.norm(point[:2]-np.asarray(detection.get('origin', point[:2]))))
                        self.projection_sigma = math.sqrt(self.projection_sigma**2+(span/2)**2+(distance*heading/2)**2)
                        detection['sigma'] = self.projection_sigma
                        reliable = self.projection_in_range and self.projection_sigma <= self.p['max_report_sigma']
                        detection['close_view'] = reliable
                    if self.empirical and not detection.get('bearing_valid'):
                        usable=False
                    quality_flags=[]
                    if self.empirical:
                        if abs(detection.get('center_error_px',math.inf))>10.:quality_flags.append('off_center_estimate')
                        if not detection.get('settled'):quality_flags.append('capture_motion_uncertainty')
                        if getattr(self,'capture_pose_diagnostic',{}).get('propagated'):quality_flags.append('covariance_propagated')
                        if not self.attitude_check.get('valid'): quality_flags.append('attitude_unverified')
                        if detection.get('ranging_mode')=='near_guess': quality_flags.append('near_field_guess')
                        detection['quality_flags']=quality_flags
                    if usable:
                        observations.append({'stamp': stamp, 'x': float(point[0]),
                                         'y': float(point[1]), 'label': label,
                                         'confidence': float(b.conf.item()),
                                         'sigma': self.projection_sigma,
                                         'box': box.tolist(), 'source_frame': stamp,
                                         'method': self.p['localization_method'], 'quality_flags':quality_flags, 'capture_pose':self.capture_pose})
                    detection['projection_valid']=bool(usable)
                    caption = (f'map: ({point[0]:.2f}, {point[1]:.2f}) m' if reliable and usable else
                               f'candidate: ({point[0]:.2f}, {point[1]:.2f}) +/- {self.projection_sigma:.2f} m')
                    cv2.putText(annotated, caption,
                                (int(box[0]), max(20, int(box[3])+20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
                else:
                    detection['projection_valid']=False
                detections.append(detection)
                if self.empirical and detection.get('bearing_valid'):
                    bearings.append(dict(detection))
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
                'raw_detected_candidates':len(detections),
                'localized_candidates': len(observations),
                'detected_candidates':len(detected_confidences),
                'highest_confidence':max(detected_confidences) if detected_confidences else None,
                'target_class': self.p['target_class'],
                'geometry_ready': self.floor_camera is not None if self.p['localization_method'] in ('ground_plane', 'hybrid', 'bottom_inverse') else camera is not None,
                'geometry_error': geometry_error,
                'observation_ready': self.capture_pose is not None if self.empirical else self.settled() if self.p['localization_method'] in ('ground_plane', 'hybrid') else True,
                'capture_pose':self.capture_pose,
                'motion_check': self.motion_check,
                'frame_age': self.get_clock().now().nanoseconds*1e-9-stamp,
                'projection_failures': [d['reason'] for d in detections if 'x' not in d],
                'odometry_age': self.last_odometry_age,
                'quiet_seconds': time.monotonic()-self.quiet_since if self.quiet_since is not None else None,
                'range_model': 'bottom_inverse_17pt_20261031' if self.empirical else None,
                'camera_model': self.floor_camera.metadata() if self.floor_camera is not None else None,
                'attitude': self.attitude_check,
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
