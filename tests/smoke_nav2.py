"""Bring up real Humble Nav2, AMCL, YOLO and project nodes without hardware.

Synthetic scan/image/odometry only. Motion stays disabled. A separate ROS domain
is mandatory. Run after building the overlay, with the existing model installed.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class World(Node):
    def __init__(self):
        super().__init__('synthetic_world')
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        transforms = []
        for child,x,z in [('laser',0.002,0.1875),('camera_link',-0.035,0.2655)]:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id,t.child_frame_id = 'base_footprint',child
            t.transform.translation.x,t.transform.translation.z = x,z
            t.transform.rotation.w = 1.0
            transforms.append(t)
        self.static.sendTransform(transforms)
        self.scan = self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.odom = self.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
        self.image = self.create_publisher(Image,'/image_raw',qos_profile_sensor_data)
        self.info = self.create_publisher(CameraInfo,'/camera_info',qos_profile_sensor_data)
        self.initial = self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.amcl_seen = self.yolo_seen = self.image_seen = False
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',
                                 lambda m: setattr(self,'amcl_seen',True),10)
        self.create_subscription(String,'/project/perception_status',self.on_perception,10)
        self.create_subscription(Image,'/project/detection_image',
                                 lambda m: setattr(self,'image_seen',True),qos_profile_sensor_data)
        self.motor = []
        self.create_subscription(Twist,'/cmd_vel',
                                 lambda m: self.motor.append((m.linear.x,m.angular.z)),10)
        self.bridge = CvBridge()
        self.create_timer(0.1,self.tick)
        self.create_timer(0.5,self.video)
        self.create_timer(1.0,self.initialize)

    def on_perception(self,msg):
        self.yolo_seen = json.loads(msg.data)['ready']

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id,t.child_frame_id = 'odom_combined','base_footprint'
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)
        odom = Odometry()
        odom.header.stamp,odom.header.frame_id,odom.child_frame_id = stamp,'odom_combined','base_footprint'
        odom.pose.pose.orientation.w = 1.0
        self.odom.publish(odom)
        scan = LaserScan()
        scan.header.stamp,scan.header.frame_id = stamp,'laser'
        scan.angle_min,scan.angle_increment = -math.pi,2*math.pi/450
        scan.angle_max = scan.angle_min+449*scan.angle_increment
        scan.range_min,scan.range_max = 0.08,8.0
        scan.scan_time = 0.1
        scan.ranges = [min(1.95/max(abs(math.sin(a)),1e-9),
                           1.95/max(abs(math.cos(a)),1e-9))
                       for a in [scan.angle_min+i*scan.angle_increment for i in range(450)]]
        self.scan.publish(scan)

    def initialize(self):
        if not self.amcl_seen:
            msg = PoseWithCovarianceStamped()
            msg.header.stamp,msg.header.frame_id = self.get_clock().now().to_msg(),'map'
            msg.pose.pose.orientation.w = 1.0
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
            msg.pose.covariance[35] = 0.01
            self.initial.publish(msg)

    def video(self):
        stamp = self.get_clock().now().to_msg()
        image = self.bridge.cv2_to_imgmsg(np.zeros((480,640,3),np.uint8),'bgr8')
        image.header.stamp,image.header.frame_id = stamp,'camera_link'
        self.image.publish(image)
        info = CameraInfo()
        info.header = image.header
        info.width,info.height = 640,480
        info.k = [300.,0.,320.,0.,300.,240.,0.,0.,1.]
        info.d = [0.]*5
        info.distortion_model = 'plumb_bob'
        self.info.publish(info)


def main():
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6'):
        raise SystemExit('Set a separate ROS_DOMAIN_ID, e.g. 89')
    with tempfile.TemporaryDirectory(prefix='group4_nav2_') as tmp:
        root = Path(tmp)
        image = np.full((80,80),254,np.uint8)
        image[[0,-1],:] = image[:,[0,-1]] = 0
        (root/'map.pgm').write_bytes(b'P5\n80 80\n255\n'+image.tobytes())
        (root/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [-2.0,-2.0,0.0]\n'
                                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        log_path = Path(os.environ.get('GROUP4_SMOKE_LOG','/tmp/group4_nav2_smoke.log'))
        with log_path.open('w') as log:
            proc = subprocess.Popen(['ros2','launch','ee3033_project','mission.launch.py',
                                     'map:='+str(root/'map.yaml'),'base:=false','lidar:=false',
                                     'camera:=false','allow_motion:=false','target_class:=bottle',
                                     'console:=false'],
                                     stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            rclpy.init()
            world = World()
            servers = ['controller_server','planner_server','smoother_server',
                       'behavior_server','bt_navigator','map_server','amcl']
            clients = {name:world.create_client(GetState,'/'+name+'/get_state')
                       for name in servers}
            pending = {}
            requested = {}
            executor = MultiThreadedExecutor(num_threads=3)
            executor.add_node(world)
            thread = threading.Thread(target=executor.spin,daemon=True)
            thread.start()
            try:
                active = set()
                deadline = time.monotonic()+65
                while time.monotonic() < deadline:
                    if proc.poll() is not None:
                        raise AssertionError('Launch exited early; see '+str(log_path))
                    for server in set(servers)-active:
                        client = clients[server]
                        future = pending.get(server)
                        if future is not None and future.done():
                            if future.result().current_state.id == 3:
                                active.add(server)
                            pending.pop(server,None)
                        elif future is not None and time.monotonic()-requested[server]>3:
                            # Fast DDS can lose an early discovery-time service
                            # response. Retry the read, without restarting Nav2.
                            client.remove_pending_request(future)
                            pending.pop(server,None)
                        if server not in active and server not in pending and client.service_is_ready():
                            pending[server]=client.call_async(GetState.Request())
                            requested[server]=time.monotonic()
                    if len(active)==len(servers) and world.amcl_seen and world.yolo_seen and world.image_seen:
                        break
                    time.sleep(0.1)
                assert set(servers)==active, 'Inactive lifecycle nodes: '+str(set(servers)-active)
                assert world.amcl_seen,'AMCL did not localize from synthetic scan'
                assert world.yolo_seen and world.image_seen,'YOLO did not process test images'
                assert world.motor and all(v==(0.,0.) for v in world.motor),'Unexpected motion output'
                assert world.count_publishers('/cmd_vel')==1,'Velocity gate must be sole cmd_vel publisher'
                publishers = world.get_publishers_info_by_topic('/project/nav_cmd_vel')
                names = {p.node_name for p in publishers}
                # behavior_server creates a publisher for each behavior plugin.
                assert names == {'controller_server','behavior_server'}, str(names)
                # Exercise the actual Humble controller/shim parameter callbacks,
                # with motion still disabled, not only a fake parameter server.
                setter=world.create_client(SetParametersAtomically,'/controller_server/set_parameters_atomically')
                getter=world.create_client(GetParameters,'/controller_server/get_parameters')
                assert setter.wait_for_service(timeout_sec=3) and getter.wait_for_service(timeout_sec=3)
                for linear,angular in [(.18,.6),(.12,.4)]:
                    entries={'FollowPath.desired_linear_vel':linear,'FollowPath.max_linear_vel':linear,
                             'FollowPath.rotate_to_heading_angular_vel':angular,'FollowPath.max_angular_vel':angular}
                    future=setter.call_async(SetParametersAtomically.Request(parameters=[
                        Parameter(key,value=value).to_parameter_msg() for key,value in entries.items()]))
                    until=time.monotonic()+4
                    while not future.done() and time.monotonic()<until:
                        time.sleep(.02)
                    assert future.done() and future.result().result.successful, 'Nav2 trial update rejected'
                    future=getter.call_async(GetParameters.Request(names=list(entries)))
                    until=time.monotonic()+4
                    while not future.done() and time.monotonic()<until:
                        time.sleep(.02)
                    assert future.done()
                    assert all(math.isclose(msg.double_value,value,abs_tol=1e-8)
                               for msg,value in zip(future.result().values,entries.values()))
                start = world.create_client(Trigger,'/project/start')
                assert start.wait_for_service(timeout_sec=3)
                future = start.call_async(Trigger.Request())
                until = time.monotonic()+4
                while not future.done() and time.monotonic()<until:
                    time.sleep(0.02)
                assert future.done() and not future.result().success
                assert 'Motion disabled' in future.result().message, future.result().message
                print('PASS: 7 active lifecycle nodes; AMCL, YOLO and images live; '
                      'sole gated velocity output; motion-disabled start rejected; native Nav2 trial updates verified.')
            finally:
                import signal
                os.killpg(proc.pid,signal.SIGINT)
                try:
                    proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    proc.wait(timeout=5)
                executor.shutdown(timeout_sec=3)
                thread.join(timeout=3)
                world.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
