"""Real Humble rotation shim, pose progress checker and gate; synthetic chassis.

No physical drivers or /cmd_vel writer. Run on an isolated ROS domain. The
fixture reports zero angular odometry to reproduce weak/noisy velocity feedback;
TF still reports the actual simulated heading. A small simulated motor deadband
exposes commands that cannot build speed with velocity feedback stuck near zero.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
from urllib.request import Request, urlopen

import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import FollowPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from ee3033_project.console import Console
from ee3033_project.ros_utils import LATCHED, set_yaw
from ee3033_project.velocity_gate import VelocityGate


def wait(future, seconds=15):
    deadline=time.monotonic()+seconds
    while not future.done() and time.monotonic()<deadline: time.sleep(.02)
    assert future.done(), 'ROS request/action timed out'
    return future.result()


class Chassis(Node):
    def __init__(self):
        super().__init__('turning_test_chassis')
        self.xyz=[0.,0.,2.]
        self.running=False
        self.frozen=False
        self.close=False
        self.deadband=0.
        self.command=(0.,0.)
        self.commands=[]
        self.headings=[]
        self.last=time.monotonic()
        self.tf=TransformBroadcaster(self)
        self.static=StaticTransformBroadcaster(self)
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom_combined'
        t.header.stamp=self.get_clock().now().to_msg();t.transform.rotation.w=1.
        self.static.sendTransform(t)
        self.scan_pub=self.create_publisher(LaserScan,'/scan_fixed',10)
        self.odom_pub=self.create_publisher(Odometry,'/odom',10)
        self.drive=self.create_publisher(Bool,'/project/drive_enabled',LATCHED)
        self.maps=self.create_publisher(OccupancyGrid,'/map',LATCHED)
        self.status=self.create_publisher(String,'/project/mission_status',LATCHED)
        self.create_subscription(Twist,'/test/turn_output',self.on_command,10)
        self.create_timer(.02,self.tick)
        m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=t.header.stamp
        m.info.width=m.info.height=100;m.info.resolution=.05
        m.info.origin.position.x=m.info.origin.position.y=-2.5;m.info.origin.orientation.w=1.
        grid=np.zeros((100,100),np.int8);grid[[0,-1],:]=100;grid[:,[0,-1]]=100
        m.data=grid.ravel().tolist();self.maps.publish(m)

    def on_command(self,msg):
        self.command=(msg.linear.x,msg.angular.z)
        if self.running:self.commands.append(self.command)

    def tick(self):
        now=time.monotonic();dt=min(now-self.last,.05);self.last=now
        v,w=self.command
        if self.running and not self.frozen:
            if abs(w)<self.deadband:w=0.
            self.xyz[0]+=v*math.cos(self.xyz[2])*dt
            self.xyz[1]+=v*math.sin(self.xyz[2])*dt
            self.xyz[2]+=w*dt
        if self.running:self.headings.append(self.xyz[2])
        stamp=self.get_clock().now().to_msg()
        t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='odom_combined';t.child_frame_id='base_footprint'
        t.transform.translation.x,t.transform.translation.y=self.xyz[:2]
        set_yaw(t.transform.rotation,self.xyz[2]);self.tf.sendTransform(t)
        odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom_combined';odom.child_frame_id='base_footprint'
        odom.pose.pose.position.x,odom.pose.pose.position.y=self.xyz[:2]
        set_yaw(odom.pose.pose.orientation,self.xyz[2])
        odom.twist.twist.linear.x=v # Deliberately under-report angular velocity.
        self.odom_pub.publish(odom)
        scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_footprint'
        scan.angle_min=-math.pi;scan.angle_max=math.pi;scan.angle_increment=2*math.pi/99
        scan.range_min=.08;scan.range_max=8.;scan.ranges=[5.]*100
        if self.close:scan.ranges[90]=.13
        self.scan_pub.publish(scan)
        self.drive.publish(Bool(data=self.running))
        self.status.publish(String(data=json.dumps(dict(phase='mapping',state='WAITING',reason='Synthetic fixture',ready=True))))


def main():
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6'):
        raise SystemExit('Use an isolated ROS_DOMAIN_ID, e.g. 97')
    root=Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='group4_turning_') as tmp:
        config=yaml.safe_load((root/'src/ee3033_project/config/navigation.yaml').read_text())
        # Speed up the stall regression; production retains its 18 s allowance.
        config['controller_server']['ros__parameters']['progress_checker']['movement_time_allowance']=2.
        for name in ('local_costmap','global_costmap'):
            p=config[name][name]['ros__parameters']
            p['plugins']=['static_layer','inflation_layer']
            p['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True}
        path=Path(tmp)/'nav.yaml';path.write_text(yaml.safe_dump(config))
        with open('/tmp/group4_turning_nav2.log','w') as log:
            proc=subprocess.Popen(['ros2','launch','ee3033_project','navigation.launch.py','nav_config:='+str(path),
                'mapping_mode:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            rclpy.init()
            chassis=Chassis()
            mapper=Node('project_mapper');mapper.declare_parameter('clearance',.14)
            mapper.declare_parameter('goal_timeout',60.)
            console=Console(parameter_overrides=[Parameter('port',value=0),Parameter('phase',value='mapping'),
                Parameter('session_directory',value=tmp)])
            gate=VelocityGate(parameter_overrides=[Parameter('allow_motion',value=True),
                Parameter('output_topic',value='/test/turn_output')])
            executor=MultiThreadedExecutor(num_threads=5)
            for node in (chassis,mapper,console,gate):executor.add_node(node)
            thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
            client=ActionClient(chassis,FollowPath,'/follow_path')
            base=f'http://127.0.0.1:{console.p["port"]}'
            handle=None
            try:
                lifecycle=chassis.create_client(GetState,'/controller_server/get_state')
                assert lifecycle.wait_for_service(timeout_sec=25)
                deadline=time.monotonic()+30
                while wait(lifecycle.call_async(GetState.Request())).current_state.id!=3:
                    assert time.monotonic()<deadline;time.sleep(.1)
                assert client.wait_for_server(timeout_sec=10)
                assert chassis.count_publishers('/cmd_vel')==0
                for level,frozen,close,deadband,succeeds in [
                        ('cautious',False,False,0.,False),('balanced',False,False,.08,True),
                        ('bold',False,False,.08,True),('bold',True,False,.08,False),
                        ('bold',False,True,.08,False)]:
                    chassis.running=False;chassis.drive.publish(Bool(data=False));time.sleep(.15)
                    chassis.xyz=[0.,0.,2.];chassis.command=(0.,0.);chassis.commands=[];chassis.headings=[]
                    chassis.frozen,chassis.close,chassis.deadband=frozen,close,deadband
                    req=Request(base+'/api/gap',json.dumps({'enabled':False,'audacity':level}).encode(),
                        {'Content-Type':'application/json','X-Project-Token':console.token},method='POST')
                    policy=json.load(urlopen(req,timeout=15));assert policy['audacity']==level and policy['synchronized']
                    time.sleep(.3)
                    goal=FollowPath.Goal();goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
                    goal.path.header.frame_id='map';goal.path.header.stamp=chassis.get_clock().now().to_msg()
                    for x in np.linspace(0,.4,17):
                        p=PoseStamped();p.header=goal.path.header;p.pose.position.x=float(x);p.pose.orientation.w=1.
                        goal.path.poses.append(p)
                    chassis.running=True
                    handle=wait(client.send_goal_async(goal));assert handle.accepted
                    result=wait(handle.get_result_async(),25)
                    chassis.running=False;chassis.drive.publish(Bool(data=False))
                    assert (result.status==4)==succeeds,(level,frozen,close,result.status,chassis.xyz)
                    if succeeds:
                        assert chassis.xyz[0]>.25,(level,chassis.xyz)
                        assert max(abs(w) for v,w in chassis.commands)>.15
                    if close:
                        assert chassis.commands and all(abs(v)<1e-6 and abs(w)<1e-6 for v,w in chassis.commands)
                    if frozen:
                        assert chassis.xyz==[0.,0.,2.]
                    print(f'PASS native turning: level={level} frozen={frozen} obstacle={close} status={result.status} pose={chassis.xyz}',flush=True)
                    handle=None
                # Explicit fast trial updates the installed DWB/shim and gate.
                chassis.close=chassis.frozen=False;chassis.deadband=.08
                chassis.xyz=[0.,0.,2.];chassis.command=(0.,0.);chassis.commands=[]
                req=Request(base+'/api/navigation',json.dumps({'profile':'fast'}).encode(),
                    {'Content-Type':'application/json','X-Project-Token':console.token},method='POST')
                policy=json.load(urlopen(req,timeout=15));assert policy['synchronized']
                assert policy['values']['max_linear']==.16 and policy['values']['max_angular']==.60
                time.sleep(.3);chassis.running=True
                handle=wait(client.send_goal_async(goal));assert handle.accepted
                result=wait(handle.get_result_async(),25)
                chassis.running=False;chassis.drive.publish(Bool(data=False))
                assert result.status==4 and max(abs(w) for v,w in chassis.commands)>.45
                handle=None
                # Home must finish without rotating to the path's final yaw.
                chassis.xyz=[0.,0.,0.];chassis.command=(0.,0.);chassis.commands=[]
                goal.goal_checker_id='home_goal_checker'
                for p in goal.path.poses:set_yaw(p.pose.orientation,2.8)
                time.sleep(.3);chassis.running=True
                handle=wait(client.send_goal_async(goal));assert handle.accepted
                result=wait(handle.get_result_async(),20)
                chassis.running=False;chassis.drive.publish(Bool(data=False))
                assert result.status==4 and abs(chassis.xyz[2])<.4, chassis.xyz
                handle=None
                print('PASS: fast native turning, XY-only home, heading progress, stall abort and obstacle stop; no chassis output',flush=True)
            finally:
                chassis.running=False;chassis.drive.publish(Bool(data=False))
                if handle:handle.cancel_goal_async()
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=12)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
                executor.shutdown(timeout_sec=3);thread.join(timeout=3)
                for node in (console,gate,mapper,chassis):node.destroy_node()
                rclpy.shutdown()


if __name__=='__main__':main()
