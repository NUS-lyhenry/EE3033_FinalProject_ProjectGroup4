"""Measure observation-goal accuracy AFTER physical angular motion settles.

The actual Humble FollowPath server and full production gate drive a chassis
with .15 s command delay, .08 rad/s input deadband, .8 rad/s^2 angular slew and
.30 m/s^2 linear slew. An action success is recorded separately from accuracy
0.8 seconds later. Actual pose/settling are measured from TF, never odom.twist.
Every failure remains in the output JSON; no production limits are disabled.
"""
import argparse
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
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState,GetState
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters,SetParametersAtomically
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
import yaml

from ee3033_project.core import quaternion_yaw,stamp_seconds
from ee3033_project.velocity_gate import VelocityGate
from integration_rpp_deadband import Chassis,wait


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))


def tf_pose(chassis,buffer):
    tr=buffer.lookup_transform('map','base_footprint',Time())
    stamp=stamp_seconds(tr.header.stamp)
    now=chassis.get_clock().now().nanoseconds*1e-9
    if not -.1<=now-stamp<=.2:raise AssertionError('Measured pose TF is stale')
    p=tr.transform.translation
    return dict(stamp=stamp,pose=[p.x,p.y,quaternion_yaw(tr.transform.rotation)])


def errors(measured,target):
    p=measured['pose']
    return dict(xy=math.dist(p[:2],target[:2]),yaw=abs(wrap(p[2]-target[2])))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='/tmp/observation-settling.json')
    parser.add_argument('--repeats',type=int,default=1)
    parser.add_argument('--modes',nargs='+',choices=['delayed_deadband','angular_odom_zero'],
                        default=['delayed_deadband','angular_odom_zero'])
    parser.add_argument('--cases',nargs='+',choices=['near_backoff','pure_yaw'],
                        default=['near_backoff','pure_yaw'])
    args=parser.parse_args()
    assert args.repeats>=1
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','6')
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    root=Path(__file__).resolve().parents[1]
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    record=dict(simulation_only=True,synthetic_chassis=True,real_nav2=True,
        physical_model=dict(command_delay=.15,angular_deadband=.08,
                            angular_acceleration=.8,linear_acceleration=.30),
        checker=dict(id='observation_goal_checker',xy_tolerance=.10,yaw_tolerance=.03),
        settle_seconds=.8,actual_pose_source='map->base_footprint TF',cases=[])
    with tempfile.TemporaryDirectory(prefix='observation_settle_') as tmp:
        config=yaml.safe_load((root/'src/ee3033_project/config/navigation.yaml').read_text())
        c=config['controller_server']['ros__parameters']
        assert c['FollowPath']['plugin']=='ee3033_navigation::RegulatedPathFollower'
        assert c['observation_goal_checker']['xy_goal_tolerance']==.10
        assert c['observation_goal_checker']['yaw_goal_tolerance']==.03
        c['FollowPath']['max_angular_accel']=.8
        p=config['local_costmap']['local_costmap']['ros__parameters']
        p['plugins']=['static_layer','inflation_layer']
        p['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True}
        path=Path(tmp)/'nav.yaml';path.write_text(yaml.safe_dump(config))
        log=output.with_suffix('.nav2.log').open('w')
        proc=subprocess.Popen(['ros2','run','nav2_controller','controller_server','--ros-args',
            '--params-file',str(path),'-r','cmd_vel:=/project/nav_cmd_vel'],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();chassis=Chassis()
        gate=VelocityGate(parameter_overrides=[Parameter('allow_motion',value=True),
            Parameter('output_topic',value='/test/deadband_output')])
        assert gate.p['enforce_swept_map'] is True
        buffer=Buffer();listener=TransformListener(buffer,chassis)
        gate_events=[]
        chassis.create_subscription(String,'/project/velocity_gate_status',
            lambda msg:gate_events.append(json.loads(msg.data)),10)
        executor=MultiThreadedExecutor(num_threads=4)
        for node in (chassis,gate):executor.add_node(node)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        handle=None
        try:
            state=chassis.create_client(GetState,'/controller_server/get_state')
            change=chassis.create_client(ChangeState,'/controller_server/change_state')
            assert state.wait_for_service(timeout_sec=25) and change.wait_for_service(timeout_sec=5)
            for transition in (Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE):
                request=ChangeState.Request();request.transition.id=transition
                assert wait(change.call_async(request)).success
            client=ActionClient(chassis,FollowPath,'/follow_path')
            assert client.wait_for_server(timeout_sec=10)
            getter=chassis.create_client(GetParameters,'/controller_server/get_parameters')
            setter=chassis.create_client(SetParametersAtomically,'/controller_server/set_parameters_atomically')
            assert getter.wait_for_service(timeout_sec=5) and setter.wait_for_service(timeout_sec=5)
            names=['FollowPath.max_linear_vel','FollowPath.max_angular_vel','FollowPath.max_angular_accel',
                   'FollowPath.rotation_command_delay','FollowPath.rotation_braking_accel',
                   'FollowPath.rotation_stop_margin','FollowPath.rotation_motor_deadband']
            params=wait(getter.call_async(GetParameters.Request(names=names)))
            record['controller_parameters']={k:v.double_value for k,v in zip(names,params.values)}
            assert list(record['controller_parameters'].values())==[.12,.4,.8,.15,.8,.005,.08]
            for name,value in [('rotation_command_delay',-1.),('rotation_braking_accel',1.),
                               ('rotation_stop_margin',.1),('rotation_motor_deadband',.5)]:
                rejected=wait(setter.call_async(SetParametersAtomically.Request(parameters=[
                    Parameter('FollowPath.'+name,value=value).to_parameter_msg(),
                    Parameter('FollowPath.desired_linear_vel',value=.10).to_parameter_msg()])))
                assert not rejected.result.successful
                unchanged=wait(getter.call_async(GetParameters.Request(names=['FollowPath.desired_linear_vel'])))
                assert unchanged.values[0].double_value==.12
            for repetition in range(args.repeats):
                for mode in args.modes:
                    for name in args.cases:
                        chassis.reset(mode);chassis.xyz=[0.,0.,0.];time.sleep(.5)
                        target=(-.30,0.,0.) if name=='near_backoff' else (0.,0.,1.57)
                        goal=FollowPath.Goal();goal.controller_id='FollowPath'
                        goal.goal_checker_id='observation_goal_checker'
                        goal.path.header.frame_id='map';goal.path.header.stamp=chassis.get_clock().now().to_msg()
                        for f in np.linspace(0.,1.,17):
                            p=PoseStamped();p.header=goal.path.header
                            p.pose.position.x=float(f*target[0]);p.pose.position.y=float(f*target[1])
                            p.pose.orientation.z=math.sin(target[2]/2);p.pose.orientation.w=math.cos(target[2]/2)
                            goal.path.poses.append(p)
                        gate_start=len(gate_events)
                        chassis.running=True;chassis.first=time.monotonic()
                        handle=wait(client.send_goal_async(goal));assert handle.accepted
                        result=wait(handle.get_result_async(),40.)
                        at_result=tf_pose(chassis,buffer)
                        error_at_result=errors(at_result,target)
                        # Keep the physical model and owner heartbeat running. Nav2's
                        # zero command must actually traverse delay and deceleration.
                        settle=[];end=time.monotonic()+.8
                        while time.monotonic()<end:
                            settle.append(tf_pose(chassis,buffer));time.sleep(.02)
                        measured=tf_pose(chassis,buffer)
                        settled_error=errors(measured,target)
                        recent=[p for p in settle if measured['stamp']-p['stamp']<=.2]
                        if recent and measured['stamp']>recent[0]['stamp']:
                            dt=measured['stamp']-recent[0]['stamp']
                            linear=math.dist(measured['pose'][:2],recent[0]['pose'][:2])/dt
                            angular=abs(wrap(measured['pose'][2]-recent[0]['pose'][2]))/dt
                        else:linear=angular=math.inf
                        passed=(result.status==4 and settled_error['xy']<=.10+1e-6
                                and settled_error['yaw']<=.03+1e-6 and linear<=.005 and angular<=.01)
                        case=dict(name=name,mode=mode,repetition=repetition+1,target=list(target),raw_action_status=result.status,
                            action_seconds=at_result['stamp']-stamp_seconds(goal.path.header.stamp),
                            at_result=at_result,error_at_result=error_at_result,
                            after_settling=measured,error_after_settling=settled_error,
                            actual_settled_velocity_from_tf=dict(linear=linear,angular=angular),
                            final_command=list(chassis.command),actual_velocity_model=list(chassis.actual),
                            maximum_actual_angular_acceleration=chassis.max_accel,
                            passed=passed,gate_events=gate_events[gate_start:],trajectory=list(chassis.samples),
                            raw_commands=list(chassis.raw_commands),settling_tf=settle)
                        rotations=[r for r in chassis.raw_commands if abs(r[1])<1e-6 and abs(r[2])>1e-6]
                        case['maximum_nonzero_rotation_command_step']=max((abs(b[2]-a[2]) for a,b in zip(rotations,rotations[1:])
                            if b[0]-a[0]<=.10),default=0.)
                        case['passed']=(case['passed'] and chassis.max_accel<=.80001
                            and case['maximum_nonzero_rotation_command_step']<=.8/15.+.004)
                        record['cases'].append(case)
                        record['passed']=all(c['passed'] for c in record['cases'])
                        output.write_text(json.dumps(record,indent=2)+'\n')
                        print(json.dumps({k:v for k,v in case.items() if k not in (
                            'gate_events','trajectory','raw_commands','settling_tf')}),flush=True)
                        assert chassis.count_publishers('/cmd_vel')==0
                        chassis.running=False;handle=None
        finally:
            chassis.running=False
            if handle:handle.cancel_goal_async()
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
            executor.shutdown(timeout_sec=3);thread.join(timeout=3)
            chassis.destroy_node();gate.destroy_node();rclpy.shutdown();log.close()
    if not record.get('passed'):raise SystemExit(1)


if __name__=='__main__':main()
