"""2-D chassis, ray-cast lidar and rendered camera. No algorithm output publishers."""
import json
import math
import time
import threading

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
from simulation_geometry import clearance, raycast, render_camera
from simulation_scenarios import default_scenario,validate_scenario
from simulation_camera import default_camera_settings,validate_camera_settings,camera_intrinsics


class SimulatedRobot(Node):
    def __init__(self, texture_path,scenario=None):
        super().__init__('preview_sensor_world')
        self.pose = [0., 0., 0.]
        self.command = (0., 0.)
        self.command_time = -math.inf
        self.last_tick = time.monotonic()
        self.phase = 'mapping'
        self.world_lock=threading.RLock()
        self.accept_commands=True
        self.camera=default_camera_settings()
        self.camera_k=camera_intrinsics(self.camera)
        self.camera_k.setflags(write=False)
        self.scene_status_after=-math.inf
        self.scene=validate_scenario(scenario or default_scenario())
        self.walls=np.asarray(self.scene['walls'],dtype=float)
        self.target=np.asarray(self.scene['target'],dtype=float)
        self.pose=list(self.scene['start'])
        self.texture = cv2.imread(texture_path)
        if self.texture is None:
            raise ValueError('A real target photograph is required for camera rendering')
        self.bridge = CvBridge()
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.publish_camera_transforms(self.camera)
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

    def camera_settings(self):
        with self.world_lock:return dict(self.camera)

    def publish_camera_transforms(self,camera):
        # Re-emit every transform owned by this static publisher. Updating the
        # camera must never drop the laser transform for late subscribers.
        transforms=[]
        mount=[camera['camera_x'],camera['camera_y'],camera['camera_height']]
        for child,xyz,euler in [('preview_laser',[0.,0.,.12],[0.,0.,0.]),
                ('camera_link',mount,[0.,camera['camera_pitch'],camera['camera_yaw']])]:
            t=TransformStamped()
            t.header.frame_id,t.child_frame_id='base_footprint',child
            t.header.stamp=self.get_clock().now().to_msg()
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,xyz)
            q=Rotation.from_euler('xyz',euler).as_quat()
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
            transforms.append(t)
        self.static.sendTransform(transforms)

    def apply_camera(self,settings):
        camera=validate_camera_settings(settings)
        k=camera_intrinsics(camera)
        k.setflags(write=False)
        with self.world_lock:
            if self.accept_commands:
                raise RuntimeError('Pause simulated motion before replacing the camera')
            self.publish_camera_transforms(camera)
            self.camera,self.camera_k=dict(camera),k
            return dict(camera)

    def on_command(self, msg):
        with self.world_lock:
            if not self.accept_commands:return
            self.command = (msg.linear.x, msg.angular.z)
            self.command_time = time.monotonic()

    def stationary(self):
        with self.world_lock:
            return (time.monotonic()-self.command_time>=.5
                or all(math.isfinite(v) and abs(v)<=1e-5 for v in self.command))

    def pause_commands(self):
        with self.world_lock:
            self.accept_commands=False
            self.command=(0.,0.);self.command_time=-math.inf

    def resume_commands(self):
        with self.world_lock:
            self.command=(0.,0.);self.command_time=-math.inf
            self.accept_commands=True

    def apply_scenario(self,scenario,reset_pose=False):
        scene=validate_scenario(scenario)
        with self.world_lock:
            if self.accept_commands:raise RuntimeError('Pause simulated motion before replacing the scene')
            self.scene=scene
            self.walls=np.asarray(scene['walls'],dtype=float)
            self.target=np.asarray(scene['target'],dtype=float)
            if reset_pose:
                self.pose=list(scene['start'])
                self.phase='mapping'
            self.command=(0.,0.);self.command_time=-math.inf
            self.last_tick=time.monotonic()
            self.scene_status_after=self.get_clock().now().nanoseconds*1e-9

    def on_status(self, msg):
        try:
            data=json.loads(msg.data)
            with self.world_lock:
                if float(data.get('stamp',-math.inf))<self.scene_status_after:return
                self.phase = data.get('phase', self.phase)
        except (ValueError, TypeError):
            pass

    def tick(self):
        with self.world_lock:self.physics_tick()

    def physics_tick(self):
        now = time.monotonic()
        dt = min(.05, now-self.last_tick)
        self.last_tick = now
        v, w = self.command if now-self.command_time < .5 else (0., 0.)
        x, y, yaw = self.pose
        heading = yaw+w*dt
        new_xy = np.array([x+v*math.cos(heading)*dt, y+v*math.sin(heading)*dt])
        if not self.accept_commands:v=w=0.;new_xy=np.array([x,y]);heading=yaw
        if clearance(new_xy,self.walls) < .14:
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
        with self.world_lock:self.publish_scan()

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'preview_laser'
        n = 450
        msg.angle_min, msg.angle_increment = -math.pi, math.tau/n
        msg.angle_max = msg.angle_min+(n-1)*msg.angle_increment
        msg.range_min, msg.range_max, msg.scan_time = .08, 8., .1
        angles = msg.angle_min+np.arange(n)*msg.angle_increment+self.pose[2]
        msg.ranges = raycast(self.pose[:2], angles,self.walls).astype(float).tolist()
        self.scan.publish(msg)

    def camera_tick(self):
        # Keep rendered pixels and their CameraInfo in one camera generation,
        # including publication, while a concurrent form save changes the mount.
        with self.world_lock:self.publish_camera()

    def publish_camera(self):
        frame = render_camera(self.pose, self.texture, self.phase == 'mission',self.walls,self.target,self.camera)
        image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = 'project_camera_optical'
        info = CameraInfo()
        info.header = image.header
        info.width, info.height = 640, 480
        info.distortion_model = 'plumb_bob'
        info.k, info.d = self.camera_k.ravel().tolist(), [0.]*5
        self.info.publish(info)
        self.image.publish(image)
