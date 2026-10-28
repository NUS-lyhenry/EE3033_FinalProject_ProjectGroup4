"""Direct native Humble RPP turn-start comparison, with physical slew limits.

The controller is launched without project launch/planning/supervisor nodes.
It uses the real final gate, full known-map footprint and TF checks. Comparative
failures are retained in JSON; the selected acceleration must pass both a .08
rad/s motor-input deadband and angular-twist odometry under-reporting.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped,TransformStamped,Twist
from lifecycle_msgs.srv import GetState,ChangeState
from lifecycle_msgs.msg import Transition
from nav_msgs.msg import OccupancyGrid,Odometry
from nav2_msgs.action import FollowPath
from rcl_interfaces.srv import SetParametersAtomically,GetParameters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
import yaml

from ee3033_project.velocity_gate import VelocityGate
from ee3033_project.ros_utils import LATCHED,set_yaw


def wait(future,seconds=25.):
    deadline=time.monotonic()+seconds
    while not future.done() and time.monotonic()<deadline:time.sleep(.01)
    if not future.done():raise TimeoutError('Native ROS response timed out')
    return future.result()


def approach(actual,target,step):return min(target,actual+step) if target>actual else max(target,actual-step)


class Chassis(Node):
    def __init__(self):
        super().__init__('rpp_deadband_chassis')
        self.mode='ideal';self.running=False;self.xyz=[0.,0.,2.]
        self.command=(0.,0.);self.actual=(0.,0.);self.queue=deque()
        self.last=time.monotonic();self.samples=[];self.first=time.monotonic()
        self.commands=[];self.raw_commands=[];self.max_accel=0.
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        tr=TransformStamped();tr.header.frame_id='map';tr.child_frame_id='odom_combined';tr.transform.rotation.w=1.
        self.static.sendTransform(tr)
        self.maps=self.create_publisher(OccupancyGrid,'/map',LATCHED)
        self.scans=self.create_publisher(LaserScan,'/scan_fixed',10)
        self.odoms=self.create_publisher(Odometry,'/odom',10)
        self.drive=self.create_publisher(Bool,'/project/drive_enabled',LATCHED)
        self.create_subscription(Twist,'/test/deadband_output',self.on_command,10)
        self.create_subscription(Twist,'/project/nav_cmd_vel',self.on_raw_command,10)
        self.create_timer(.02,self.tick)
        m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
        m.info.width=m.info.height=100;m.info.resolution=.05
        m.info.origin.position.x=m.info.origin.position.y=-2.5;m.info.origin.orientation.w=1.
        grid=np.zeros((100,100),np.int8);grid[[0,-1],:]=100;grid[:,[0,-1]]=100
        m.data=grid.ravel().tolist();self.maps.publish(m)

    def on_command(self,msg):
        cmd=(msg.linear.x,msg.angular.z)
        self.queue.append((time.monotonic(),cmd))
        if self.running:self.commands.append(cmd)

    def on_raw_command(self,msg):
        if self.running:self.raw_commands.append((time.monotonic()-self.first,msg.linear.x,msg.angular.z))

    def reset(self,mode):
        self.running=False;self.mode=mode;self.xyz=[0.,0.,2.]
        self.queue.clear();self.command=self.actual=(0.,0.)
        self.samples=[];self.commands=[];self.raw_commands=[];self.max_accel=0.;self.first=time.monotonic()

    def tick(self):
        now=time.monotonic();dt=min(.05,now-self.last);self.last=now
        delay=0. if self.mode=='ideal' else .15
        while self.queue and self.queue[0][0]<=now-delay:
            _,self.command=self.queue.popleft()
        v,w=self.command if self.running else (0.,0.)
        if self.mode!='ideal' and abs(w)<.08:w=0.
        if self.mode=='frozen':v=w=0.
        old_w=self.actual[1]
        v=approach(self.actual[0],v,.30*dt)
        w=approach(old_w,w,.80*dt)
        self.max_accel=max(self.max_accel,abs(w-old_w)/max(dt,1e-9))
        self.actual=(v,w)
        self.xyz[0]+=v*math.cos(self.xyz[2]+w*dt/2)*dt
        self.xyz[1]+=v*math.sin(self.xyz[2]+w*dt/2)*dt
        self.xyz[2]+=w*dt
        stamp=self.get_clock().now().to_msg()
        t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='odom_combined';t.child_frame_id='base_footprint'
        t.transform.translation.x,t.transform.translation.y=self.xyz[:2]
        set_yaw(t.transform.rotation,self.xyz[2]);self.tf.sendTransform(t)
        od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom_combined';od.child_frame_id='base_footprint'
        od.pose.pose.position.x,od.pose.pose.position.y=self.xyz[:2]
        set_yaw(od.pose.pose.orientation,self.xyz[2]);od.twist.twist.linear.x=v
        od.twist.twist.angular.z=0. if self.mode=='angular_odom_zero' else w
        self.odoms.publish(od)
        scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_footprint'
        scan.angle_min=-math.pi;scan.angle_increment=math.tau/360;scan.angle_max=scan.angle_min+359*scan.angle_increment
        scan.range_min=.08;scan.range_max=8.;scan.ranges=[5.]*360;self.scans.publish(scan)
        self.drive.publish(Bool(data=self.running))
        if self.running and (not self.samples or now-self.first-self.samples[-1]['t']>=.10):
            self.samples.append(dict(t=now-self.first,pose=list(self.xyz),command=list(self.command),actual=list(self.actual),
                                     reported_angular=od.twist.twist.angular.z))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='/tmp/rpp-deadband-results.json')
    parser.add_argument('--accelerations',nargs='+',type=float,default=[.8,1.5,3.2])
    parser.add_argument('--modes',nargs='+',default=['ideal','delayed_deadband','angular_odom_zero'])
    parser.add_argument('--plugin',choices=['native','wrapper'],default='native')
    parser.add_argument('--replan-hz',type=float,default=0.)
    parser.add_argument('--max-linear',type=float,default=.12)
    parser.add_argument('--max-angular',type=float,default=.4)
    args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','6')
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    root=Path(__file__).resolve().parents[1];results=[]
    with tempfile.TemporaryDirectory(prefix='rpp_deadband_') as directory:
        config=yaml.safe_load((root/'src/ee3033_project/config/navigation.yaml').read_text())
        c=config['controller_server']['ros__parameters']
        c['FollowPath']['max_angular_accel']=args.accelerations[0]
        c['FollowPath']['plugin']=('ee3033_navigation::RegulatedPathFollower' if args.plugin=='wrapper' else
            'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
        if args.plugin=='wrapper':c['FollowPath'].update(max_linear_vel=.12,max_angular_vel=.4)
        c['FollowPath']['rotate_to_heading_angular_vel']=.4
        c['progress_checker']['movement_time_allowance']=18.
        p=config['local_costmap']['local_costmap']['ros__parameters']
        p['plugins']=['static_layer','inflation_layer']
        p['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True}
        path=Path(directory)/'nav.yaml';path.write_text(yaml.safe_dump(config))
        output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
        log=output.with_suffix('.nav2.log').open('w')
        proc=subprocess.Popen(['ros2','run','nav2_controller','controller_server','--ros-args',
            '--params-file',str(path),'-r','cmd_vel:=/project/nav_cmd_vel'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();chassis=Chassis()
        gate=VelocityGate(parameter_overrides=[Parameter('allow_motion',value=True),
            Parameter('output_topic',value='/test/deadband_output')])
        executor=MultiThreadedExecutor(num_threads=4)
        for n in (chassis,gate):executor.add_node(n)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        handle=None
        try:
            state=chassis.create_client(GetState,'/controller_server/get_state')
            change=chassis.create_client(ChangeState,'/controller_server/change_state')
            assert state.wait_for_service(timeout_sec=25) and change.wait_for_service(timeout_sec=5)
            for transition in (Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE):
                request=ChangeState.Request();request.transition.id=transition
                assert wait(change.call_async(request)).success
            client=ActionClient(chassis,FollowPath,'/follow_path');assert client.wait_for_server(timeout_sec=10)
            setter=chassis.create_client(SetParametersAtomically,'/controller_server/set_parameters_atomically')
            getter=chassis.create_client(GetParameters,'/controller_server/get_parameters')
            assert setter.wait_for_service(timeout_sec=5) and getter.wait_for_service(timeout_sec=5)
            if args.plugin=='wrapper':
                bad=SetParametersAtomically.Request(parameters=[Parameter('FollowPath.max_linear_vel',value=0.).to_parameter_msg()])
                assert not wait(setter.call_async(bad)).result.successful
                unchanged=wait(getter.call_async(GetParameters.Request(names=['FollowPath.max_linear_vel'])))
                assert unchanged.values[0].double_value==.12
                update=SetParametersAtomically.Request(parameters=[
                    Parameter('FollowPath.max_linear_vel',value=args.max_linear).to_parameter_msg(),
                    Parameter('FollowPath.max_angular_vel',value=args.max_angular).to_parameter_msg()])
                assert wait(setter.call_async(update)).result.successful
                read=wait(getter.call_async(GetParameters.Request(names=['FollowPath.max_linear_vel','FollowPath.max_angular_vel'])))
                assert [v.double_value for v in read.values]==[args.max_linear,args.max_angular]
            for acceleration in args.accelerations:
                req=SetParametersAtomically.Request(parameters=[Parameter('FollowPath.max_angular_accel',value=acceleration).to_parameter_msg()])
                answer=wait(setter.call_async(req));assert answer.result.successful,answer.result.reason
                read=wait(getter.call_async(GetParameters.Request(names=['FollowPath.max_angular_accel'])))
                assert read.values[0].double_value==acceleration
                for mode in args.modes:
                    chassis.reset(mode);time.sleep(.5)
                    goal=FollowPath.Goal();goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
                    goal.path.header.frame_id='map';goal.path.header.stamp=chassis.get_clock().now().to_msg()
                    for x in np.linspace(0.,.4,17):
                        pose=PoseStamped();pose.header=goal.path.header;pose.pose.position.x=float(x);pose.pose.orientation.w=1.
                        goal.path.poses.append(pose)
                    chassis.running=True;chassis.first=time.monotonic()
                    handle=wait(client.send_goal_async(goal));assert handle.accepted
                    result_future=handle.get_result_async();last_replan=time.monotonic();replans=0
                    while not result_future.done() and time.monotonic()-chassis.first<25.:
                        if args.replan_hz>0 and time.monotonic()-last_replan>=1./args.replan_hz:
                            goal.path.header.stamp=chassis.get_clock().now().to_msg()
                            handle=wait(client.send_goal_async(goal));assert handle.accepted
                            result_future=handle.get_result_async();last_replan=time.monotonic();replans+=1
                        time.sleep(.01)
                    result=wait(result_future,1.)
                    duration=time.monotonic()-chassis.first;chassis.running=False
                    record=dict(plugin=args.plugin,replans=replans,acceleration=acceleration,mode=mode,status=result.status,reached=result.status==4,
                        seconds=duration,pose=list(chassis.xyz),first_nonzero_command=next((list(cmd) for cmd in chassis.commands if abs(cmd[1])>1e-6),None),
                        max_command_angular=max((abs(w) for v,w in chassis.commands),default=0.),
                        max_command_linear=max((abs(v) for v,w in chassis.commands),default=0.),
                        max_actual_angular=max((abs(s['actual'][1]) for s in chassis.samples),default=0.),
                        max_actual_angular_acceleration=chassis.max_accel,trajectory=list(chassis.samples))
                    early=[r for r in chassis.raw_commands if r[0]<3. and abs(r[1])<1e-6 and abs(r[2])>1e-6]
                    record['max_early_rotation_command_step']=max((abs(b[2]-a[2]) for a,b in zip(early,early[1:])),default=0.)
                    if args.plugin=='wrapper':
                        assert record['max_early_rotation_command_step']<=acceleration/15.+.001,record
                    assert record['max_command_angular']<=.40001 and chassis.max_accel<=.80001
                    if args.plugin=='wrapper':
                        assert record['max_command_angular']<=args.max_angular+1e-5
                        assert record['max_command_linear']<=args.max_linear+1e-5
                    results.append(record);handle=None
                    output.write_text(json.dumps(dict(plugin=args.plugin,max_linear=args.max_linear,max_angular=args.max_angular,physical_angular_accel=.8,motor_deadband=.08,command_delay=.15,cases=results),indent=2)+'\n')
                    print(json.dumps({k:v for k,v in record.items() if k!='trajectory'}),flush=True)
            selected=[r for r in results if args.plugin=='wrapper' or r['acceleration']==3.2]
            assert not selected or all(r['reached']==(r['mode']!='frozen') for r in selected),selected
            print('PASS selected RPP turn-start acceleration with physical .8 rad/s^2 limit and .4 rad/s cap',flush=True)
        finally:
            chassis.running=False;chassis.drive.publish(Bool(data=False))
            if handle:handle.cancel_goal_async()
            executor.shutdown(timeout_sec=3);thread.join(timeout=3)
            gate.destroy_node();chassis.destroy_node();rclpy.shutdown()
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
            log.close()


if __name__=='__main__':main()
