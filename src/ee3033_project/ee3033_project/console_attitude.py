"""Stopped-only body-attitude setup; captures angles from the controller."""
import math
from pathlib import Path
import threading

import numpy as np
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import TransformException

from .assessment import ACTIVE, atomic_json
from .attitude_geometry import (ATTITUDE_TOPIC, AttitudeBuffer, default_config,
    load_config, mount_fingerprint, reference_valid, validate_config)
from .calibration import mounting_rotation
from .core import quaternion_yaw, stamp_seconds
from .ros_utils import transform_arrays
from .stationary import StationaryWindow
from .vision_geometry import override_pitch


class AttitudeSettings:
    def init_attitude(self):
        self.attitude_lock = threading.RLock()
        self.attitude_path = Path(self.p['session_directory']).expanduser()/'attitude.yaml'
        self.attitude_config = load_config(self.attitude_path) if self.attitude_path.is_file() else default_config()
        self.attitude_samples = AttitudeBuffer()
        # Permit bounded balance oscillation, using the existing verified-stop
        # mean speed limit. Position/heading spans still reject sustained drift.
        self.attitude_stationary = StationaryWindow(seconds=1., linear_limit=.035,
            angular_limit=.06, position_limit=.04, heading_limit=.05, max_gap=.20)
        if self.stage and self.attitude_path.is_file():
            self.stage.attitude_config = str(self.attitude_path.resolve())
        self.create_subscription(Vector3Stamped, ATTITUDE_TOPIC, self.on_body_attitude,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_attitude_odometry,
                                 qos_profile_sensor_data)

    def on_body_attitude(self, msg):
        with self.attitude_lock:
            self.attitude_samples.add(stamp_seconds(msg.header.stamp), msg.vector.x,
                                      msg.vector.y, msg.header.frame_id)

    def on_attitude_odometry(self, msg):
        p, v, w = msg.pose.pose.position, msg.twist.twist.linear, msg.twist.twist.angular
        with self.attitude_lock:
            self.attitude_stationary.add(stamp_seconds(msg.header.stamp), p.x, p.y,
                quaternion_yaw(msg.pose.pose.orientation), math.hypot(v.x, v.y), w.z,
                msg.header.frame_id)

    def attitude_snapshot(self):
        # Console.snapshot already owns self.lock. Never reacquire it here.
        with self.attitude_lock:
            config = self.attitude_config
            ref = config.get('reference')
            valid = reference_valid(config, self.vision_settings)
            now = self.get_clock().now().nanoseconds*1e-9
            sample, codes = None, []
            try:
                sample = self.attitude_samples.latest(now)
            except ValueError:
                codes.append('attitude_missing_or_stale')
            if config.get('pivot') is None:
                codes.append('attitude_pivot_missing')
            if not valid:
                codes.append('attitude_reference_missing_or_mount_changed')
            editable = self.state.get('state') not in ACTIVE and not (self.stage and self.stage.loading)
            ready = config['mode'] == 'fixed' or not codes
            return dict(success=True, mode=config['mode'], topic=ATTITUDE_TOPIC,
                pivot=config.get('pivot'), reference={**ref, 'valid': valid} if ref else None,
                sample=sample, ready=ready, editable=editable,
                reason='Fixed mounting compatibility mode' if config['mode'] == 'fixed' else
                    'Attitude reference and fresh source ready' if ready else ', '.join(codes),
                reason_codes=codes, requires_reload=True,
                message='Saved settings apply when the mission is loaded again. No motion is started.')

    def attitude_settings(self, body=None, generation=None):
        if body is None:
            return self.attitude_snapshot()
        if not isinstance(body, dict) or body.get('action') not in ('configure', 'reference'):
            raise ValueError('Choose configure or reference for body attitude')
        self.require_idle()
        if self.p['phase'] == 'calibration':
            raise ValueError('Finish camera calibration before changing body attitude')
        if body['action'] == 'reference':
            self.require_zero_command()
        with self.attitude_lock:
            config = dict(self.attitude_config)
            if body['action'] == 'configure':
                if set(body) != {'action', 'mode', 'pivot'}:
                    raise ValueError('Provide mode and the measured pivot [x,y,z]')
                config.update(mode=body['mode'], pivot=body['pivot'])
                if config['pivot'] != self.attitude_config.get('pivot'):
                    config['reference'] = None
                validate_config(config)
            else:
                if set(body) != {'action'}:
                    raise ValueError('Reference angles must be captured, not entered')
                if config.get('pivot') is None:
                    raise ValueError('Save the measured wheel pivot before capturing a reference')
                now = self.get_clock().now().nanoseconds*1e-9
                stationary = self.attitude_stationary.evaluate(now)
                if not stationary['verified']:
                    raise ValueError(stationary['reason'])
                ref = self.attitude_samples.reference(now)
                # Measured mounting angles belong to this stationary reference.
                mount = self.vision_settings
                if mount['camera_height'] > 0:
                    rotation = mounting_rotation(mount['camera_pitch'], mount['camera_yaw'])
                    translation = np.array([mount['camera_x'], mount['camera_y'], mount['camera_height']])
                else:
                    try:
                        transform = self.tf.lookup_transform('base_footprint', 'project_camera_optical',
                                                            Time(seconds=ref['stamp']))
                        rotation, translation = transform_arrays(transform)
                    except TransformException as exc:
                        raise ValueError('Camera mounting transform is unavailable at the reference capture time; '
                                         'wait for camera TF before retrying') from exc
                    if mount.get('use_pitch_override'):
                        rotation = override_pitch(rotation, mount['camera_pitch'])
                # Operator-measured camera is 1 cm behind the wheel axle. This
                # belongs only to the controller reference, never fixed mode.
                translation[0] = -.01
                ref.update(rotation=rotation.tolist(), translation=translation.tolist(),
                           mount_fingerprint=mount_fingerprint(mount))
                config['reference'] = ref
                validate_config(config)
        # Do not hold attitude_lock while acquiring Console.lock (snapshot uses
        # the opposite order). Save only if Stop has not superseded this request.
        if body['action'] == 'reference':
            self.require_zero_command()
        with self.lock:
            if generation is not None and generation != self.control_generation:
                raise ValueError('Stop superseded the attitude settings update')
            if self.state.get('state') in ACTIVE or (self.stage and self.stage.loading):
                raise ValueError('Robot state changed; stop before saving attitude settings')
            with self.attitude_lock:
                atomic_json(self.attitude_path, config)
                self.attitude_config = config
                if self.stage:
                    self.stage.attitude_config = str(self.attitude_path.resolve())
        return self.attitude_snapshot()
