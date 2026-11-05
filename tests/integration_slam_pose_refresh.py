"""Stationary SLAM pose freshness, isolated domain and no motion publishers."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data
from ee3033_project.ros_utils import LATCHED
from ee3033_project.core import stamp_seconds, quaternion_yaw
from ee3033_project.coverage import CoveragePlanner
from ee3033_project.frontiers import reachable_space


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '6')
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    config=Path(sys.argv[1]).resolve()
    rclpy.init()
    node=rclpy.create_node('stationary_slam_pose_fixture')
    poses=[];maps=[]
    node.create_subscription(PoseWithCovarianceStamped,'/pose',poses.append,10)
    node.create_subscription(OccupancyGrid,'/map',maps.append,LATCHED)
    scans=node.create_publisher(LaserScan,'/scan_fixed',qos_profile_sensor_data)
    dynamic=TransformBroadcaster(node);static=StaticTransformBroadcaster(node)
    mount=TransformStamped();mount.header.frame_id='base_footprint';mount.child_frame_id='test_laser'
    mount.transform.rotation.w=1.;static.sendTransform(mount)
    ranges=[]
    for angle in np.linspace(-math.pi,math.pi,360,endpoint=False):
        ranges.append(min(1.5/max(abs(math.cos(angle)),1e-9),
                          1.2/max(abs(math.sin(angle)),1e-9)))
    with tempfile.TemporaryFile(mode='w+') as log:
        process=subprocess.Popen(['/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node',
            '--ros-args','--params-file',str(config),'-p','resolution:=0.05'],
            stdout=log,stderr=subprocess.STDOUT)
        try:
            started=time.monotonic();last_scan=-math.inf
            while time.monotonic()-started<10.:
                assert process.poll() is None,'SLAM exited unexpectedly'
                stamp=node.get_clock().now().to_msg()
                odom=TransformStamped();odom.header.stamp=stamp
                odom.header.frame_id='odom_combined';odom.child_frame_id='base_footprint'
                odom.transform.rotation.w=1.;dynamic.sendTransform(odom)
                if time.monotonic()-last_scan>=.1:
                    scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='test_laser'
                    scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360
                    scan.angle_max=scan.angle_min+359*scan.angle_increment
                    scan.range_min=.08;scan.range_max=8.;scan.scan_time=.1
                    scan.ranges=ranges;scans.publish(scan);last_scan=time.monotonic()
                rclpy.spin_once(node,timeout_sec=.02)
            assert poses and maps,'No SLAM pose or map'
            span=stamp_seconds(poses[-1].header.stamp)-stamp_seconds(poses[0].header.stamp)
            age=node.get_clock().now().nanoseconds*1e-9-stamp_seconds(poses[-1].header.stamp)
            assert span>=6.,(len(poses),span)
            assert 0<=age<1.5,age
            m=maps[-1];o=m.info.origin
            grid=np.asarray(m.data).reshape(m.info.height,m.info.width)
            planner=CoveragePlanner(grid,m.info.resolution,
                (o.position.x,o.position.y,quaternion_yaw(o.orientation)),clearance=.14)
            reach=reachable_space(planner,(0.,0.,0.),np.ones_like(grid,dtype=bool))
            assert reach.sum()>100,int(reach.sum())
            print(json.dumps(dict(stationary_seconds=10,pose_samples=len(poses),
                pose_stamp_span=span,last_pose_age=age,reachable_cells=int(reach.sum()),
                physical_motion=False)),flush=True)
        except Exception:
            log.seek(0);print(log.read(),file=sys.stderr);raise
        finally:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=3)
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
