"""2-D chassis, ray-cast lidar and rendered camera. No algorithm output publishers."""
import json
import math
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from ee3033_project.ros_utils import LATCHED, set_yaw
from simulation_geometry import CAMERA_OFFSET, K, PITCH, clearance, raycast, render_camera


class SimulatedRobot(Node):
    def __init__(self, texture_path):
        super().__init__('preview_sensor_world')
        self.pose = [0., 0., 0.]
        self.command = (0., 0.)
        self.command_time = -math.inf
        self.last_tick = time.monotonic()
        self.phase = 'mapping'
        self.texture = cv2.imread(texture_path)
        if self.texture is None:
            raise ValueError('A real target photograph is required for camera rendering')
        self.bridge = CvBridge()
        self.tf = TransformBroadcaster(self)
        static = StaticTransformBroadcaster(self)
        transforms = []
        for child, xyz, rotation in [('preview_laser', [0., 0., .12], [0., 0., 0.]),
                                      ('camera_link', CAMERA_OFFSET, [0., PITCH, 0.])]:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = 'base_footprint', child
            t.header.stamp = self.get_clock().now().to_msg()
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
            q = Rotation.from_euler('xyz', rotation).as_quat()
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, q)
            transforms.append(t)
        self.static = static
        static.sendTransform(transforms)
        self.scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom = self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        from sensor_msgs.msg import Image
        self.image = self.create_publisher(Image, '/image_raw', qos_profile_sensor_data)
        self.info = self.create_publisher(CameraInfo, '/camera_info', qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_subscription(String, '/project/mission_status', self.on_status, LATCHED)
        self.create_timer(.02, self.tick)
        self.create_timer(.1, self.scan_tick)
        self.create_timer(.2, self.camera_tick)

    def on_command(self, msg):
        self.command = (msg.linear.x, msg.angular.z)
        self.command_time = time.monotonic()

    def on_status(self, msg):
        try:
            self.phase = json.loads(msg.data).get('phase', self.phase)
        except (ValueError, TypeError):
            pass

    def tick(self):
        now = time.monotonic()
        dt = min(.05, now-self.last_tick)
        self.last_tick = now
        v, w = self.command if now-self.command_time < .5 else (0., 0.)
        x, y, yaw = self.pose
        heading = yaw+w*dt
        new_xy = np.array([x+v*math.cos(heading)*dt, y+v*math.sin(heading)*dt])
        if clearance(new_xy) < .14:
            v = 0.
            new_xy = np.array([x, y])
        self.pose = [*map(float, new_xy), heading]
        stamp = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id, t.child_frame_id = 'odom_combined', 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = self.pose[:2]
        set_yaw(t.transform.rotation, heading)
        self.tf.sendTransform(t)
        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.pose[:2]
        set_yaw(odom.pose.pose.orientation, heading)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = float(v), float(w)
        self.odom.publish(odom)

    def scan_tick(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'preview_laser'
        n = 450
        msg.angle_min, msg.angle_increment = -math.pi, math.tau/n
        msg.angle_max = msg.angle_min+(n-1)*msg.angle_increment
        msg.range_min, msg.range_max, msg.scan_time = .08, 8., .1
        angles = msg.angle_min+np.arange(n)*msg.angle_increment+self.pose[2]
        msg.ranges = raycast(self.pose[:2], angles).astype(float).tolist()
        self.scan.publish(msg)

    def camera_tick(self):
        frame = render_camera(self.pose, self.texture, self.phase == 'mission')
        image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = 'project_camera_optical'
        info = CameraInfo()
        info.header = image.header
        info.width, info.height = 640, 480
        info.distortion_model = 'plumb_bob'
        info.k, info.d = K.ravel().tolist(), [0.]*5
        self.info.publish(info)
        self.image.publish(image)
