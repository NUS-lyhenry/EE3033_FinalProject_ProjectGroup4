"""Real ROS gate with offset laser, known map, covariance and moving TF stamps.

No hardware drivers. Run with ROS_LOCALHOST_ONLY=1 in an isolated ROS_DOMAIN_ID.
"""
import math
import os
import threading
import time
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool,String
from tf2_ros import StaticTransformBroadcaster,TransformBroadcaster
from ee3033_project.velocity_gate import VelocityGate
from ee3033_project.ros_utils import LATCHED


def until(predicate,seconds=5.):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.02)
    raise AssertionError('Timed out waiting for safety output')


class Sensors(Node):
    def __init__(self):
        super().__init__('motion_safety_fixture')
        self.mode='clear';self.command=(.12,.8);self.enabled=True;self.outputs=[];self.status=[]
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        transforms=[]
        for parent,child,x in [('map','odom_combined',0.),('base_footprint','offset_laser',-.2)]:
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
            t.transform.translation.x=x;t.transform.rotation.w=1.;transforms.append(t)
        self.static.sendTransform(transforms)
        self.scans=self.create_publisher(LaserScan,'/scan_fixed',10)
        self.commands=self.create_publisher(Twist,'/project/nav_cmd_vel',10)
        self.drive=self.create_publisher(Bool,'/project/drive_enabled',LATCHED)
        self.maps=self.create_publisher(OccupancyGrid,'/map',LATCHED)
        self.poses=self.create_publisher(PoseWithCovarianceStamped,'/amcl_pose',10)
        self.create_subscription(Twist,'/test/safe_output',lambda m:self.outputs.append((m.linear.x,m.angular.z)),10)
        import json
        self.create_subscription(String,'/project/velocity_gate_status',lambda m:self.status.append(json.loads(m.data)),10)
        self.create_timer(.05,self.tick)

    def tick(self):
        stamp=self.get_clock().now().to_msg()
        t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='odom_combined';t.child_frame_id='base_footprint'
        t.transform.rotation.w=1.;self.tf.sendTransform(t)
        pose=PoseWithCovarianceStamped();pose.header.frame_id='map';pose.header.stamp=stamp;pose.pose.pose.orientation.w=1.
        for idx in (0,7,35):pose.pose.covariance[idx]=.0004
        self.poses.publish(pose)
        m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=stamp
        m.info.width=m.info.height=100;m.info.resolution=.05
        m.info.origin.position.x=m.info.origin.position.y=-2.5;m.info.origin.orientation.w=1.
        grid=np.zeros((100,100),np.int8)
        if self.mode=='unknown_rear':grid[50,43]=-1
        if self.mode=='forward_map':grid[50,54]=25
        m.data=grid.ravel().tolist();self.maps.publish(m)
        scan=LaserScan();scan.header.frame_id='offset_laser';scan.header.stamp=stamp
        scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360
        scan.angle_max=scan.angle_min+359*scan.angle_increment
        scan.range_min=.08;scan.range_max=8.;scan.ranges=[5.]*360
        if self.mode=='offset_obstacle':scan.ranges[180]=.3 # +0.3 -0.2 = +0.1 m in base.
        if self.mode in ('reverse_blind','unknown_rear'):scan.ranges=[float('nan')]*360;scan.ranges[180]=5.
        self.scans.publish(scan)
        command=Twist();command.linear.x,command.angular.z=self.command;self.commands.publish(command)
        self.drive.publish(Bool(data=self.enabled))


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','6')
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    rclpy.init();fixture=Sensors()
    gate=VelocityGate(parameter_overrides=[Parameter('allow_motion',value=True),Parameter('output_topic',value='/test/safe_output')])
    executor=MultiThreadedExecutor(num_threads=4)
    for n in (fixture,gate):executor.add_node(n)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    def output(v,w):return fixture.outputs and math.isclose(fixture.outputs[-1][0],v,abs_tol=1e-5) and math.isclose(fixture.outputs[-1][1],w,abs_tol=1e-5)
    try:
        until(lambda:output(.06,.4))
        assert gate.output_arc_frame=='map'
        print('PASS native curvature-preserving output limit and known-map sweep',flush=True)
        fixture.mode='offset_obstacle';until(lambda:gate.scan and gate.scan.ranges[180]<.31)
        until(lambda:output(0.,0.))
        assert gate.laser_hits()[0][:,0].min()<.101
        print('PASS native offset laser collision stops both velocity components',flush=True)
        fixture.mode='forward_map';fixture.command=(.12,0.)
        until(lambda:gate.map.data[50*100+54]==25)
        until(lambda:output(0.,0.))
        print('PASS native full footprint detects map obstacle beyond current centre',flush=True)
        fixture.mode='reverse_blind';fixture.command=(-.12,0.)
        until(lambda:output(-.04,0.))
        assert gate.reverse_envelope>.19
        print('PASS native bounded reverse uses known map despite absent rear returns',flush=True)
        fixture.mode='unknown_rear'
        until(lambda:gate.map.data[50*100+43]==-1)
        until(lambda:output(0.,0.))
        print('PASS native unknown rear sweep denies reverse',flush=True)
        fixture.mode='clear';fixture.command=(.12,.8);until(lambda:output(.06,.4))
        fixture.enabled=False;until(lambda:output(0.,0.))
        print('PASS native Stop clears complete final command',flush=True)
    finally:
        fixture.enabled=False;fixture.command=(0.,0.);time.sleep(.1)
        executor.shutdown(timeout_sec=3);thread.join(timeout=3)
        gate.destroy_node();fixture.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
