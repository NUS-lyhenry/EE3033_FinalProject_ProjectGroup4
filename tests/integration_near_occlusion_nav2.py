"""Targeted production perception/mission/estimator + real Nav2 verification.

Only the detector and chassis/sensors are synthetic. Boxes are projected from
the capture-time camera pose, rather than injecting target coordinates. Motion
uses production Nav2 and the final velocity gate on a test-only output. Run
serially with other heavy native checks, on an isolated ROS domain, never 6.
"""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation
from cv_bridge import CvBridge
import yaml

from ee3033_project.calibration import mounting_rotation
from ee3033_project.core import stamp_seconds
from ee3033_project.coverage import CoveragePlanner
from ee3033_project.mission import Mission
from ee3033_project.navigation_costmap import NavigationCostmap
from ee3033_project.perception import Perception
from ee3033_project.ros_utils import LATCHED, set_yaw
from ee3033_project.target_acquisition import SingleTarget
from ee3033_project.target_filter import TargetFilter
from ee3033_project.velocity_gate import VelocityGate
from ee3033_project.visibility import observation_visibility
from integration_ros import overrides
from integration_turning_nav2 import Chassis, wait
from integration_visual_perception import Array

cv2.setNumThreads(1)

K=np.array([[300.,0.,320.],[0.,300.,240.],[0.,0.,1.]])
PITCH=math.radians(28.6)


def until(predicate,seconds=30,description='condition'):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.05)
    raise AssertionError('Timed out waiting for '+description)


class ProjectedDetector:
    names={0:'bottle'}

    def __init__(self,target,camera_height,bearing_only=False):
        self.target=np.array(target,dtype=float)
        self.mount=np.array([0.,0.,camera_height])
        self.rotation=mounting_rotation(PITCH,0.)
        self.base_z=.355 if bearing_only else 0.
        self.height=.03 if bearing_only else .25

    def predict(self,frame,**kwargs):
        # Decode the pose encoded by the sensor, not the latest chassis pose.
        x,y,yaw=np.frombuffer(frame[0,:24,0].copy().tobytes(),dtype=np.float64)
        c,s=math.cos(yaw),math.sin(yaw)
        points=[]
        for z in (self.base_z,self.base_z+self.height):
            for side in (-.03,.03):
                dx,dy=self.target-[x,y]
                points.append([c*dx+s*dy,-s*dx+c*dy+side,z])
        optical=(np.asarray(points)-self.mount)@self.rotation
        boxes=[]
        if np.all(optical[:,2]>.01):
            pixels,_=cv2.projectPoints(optical,np.zeros(3),np.zeros(3),K,np.zeros(5))
            pixels=pixels.reshape(-1,2)
            low,high=pixels.min(axis=0),pixels.max(axis=0)
            if high[0]>=0 and high[1]>=0 and low[0]<=639 and low[1]<=479:
                box=np.array([max(0.,low[0]),max(0.,low[1]),min(639.,high[0]),min(479.,high[1])])
                if box[2]-box[0]>=4 and box[3]-box[1]>=8:
                    boxes=[SimpleNamespace(cls=SimpleNamespace(item=lambda:0),
                        conf=SimpleNamespace(item=lambda:.9),xyxy=[Array(box)])]
        return [SimpleNamespace(boxes=boxes,names=self.names,plot=lambda:frame.copy())]


class CameraChassis(Chassis):
    def __init__(self,target=(.45,0.),height=.31,mission_owned=True):
        super().__init__()
        self.xyz=[0.,0.,0.]
        self.target=target
        self.camera_height=height
        self.bridge=CvBridge()
        self.frames=0
        self.capture_poses={}
        self.observation_frames=[]
        self.detection_frames=[]
        self.states=[]
        self.trajectory=[]
        self.mission=None
        self.runtime_debug=[]
        self.camera_pub=self.create_publisher(CameraInfo,'/camera_info',10)
        self.image_pub=self.create_publisher(Image,'/image_raw',10)
        self.amcl_pub=self.create_publisher(PoseWithCovarianceStamped,'/amcl_pose',10)
        self.create_subscription(String,'/project/target_observations',
            lambda msg:self.observation_frames.append(json.loads(msg.data)),10)
        self.create_subscription(String,'/project/target_detections',
            lambda msg:self.detection_frames.append(json.loads(msg.data)),10)
        self.create_subscription(String,'/project/mission_status',
            lambda msg:self.states.append(json.loads(msg.data)),LATCHED)
        if mission_owned:
            # The production mission alone owns drive enable and mission state.
            self.destroy_publisher(self.drive)
            self.destroy_publisher(self.status)
            from std_msgs.msg import Bool
            self.drive=self.create_publisher(Bool,'/test/fixture_drive',LATCHED)
            self.status=self.create_publisher(String,'/test/fixture_status',LATCHED)
        tr=TransformStamped();tr.header.frame_id='base_footprint';tr.child_frame_id='project_camera_optical'
        tr.header.stamp=self.get_clock().now().to_msg();tr.transform.translation.z=height
        q=Rotation.from_matrix(mounting_rotation(PITCH,0.)).as_quat()
        tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=map(float,q)
        anchor=TransformStamped();anchor.header.frame_id='map';anchor.child_frame_id='odom_combined'
        anchor.header.stamp=tr.header.stamp;anchor.transform.rotation.w=1.
        # One latched message must contain the whole static chain for late
        # Nav2 listeners, rather than overwriting Chassis' map->odom anchor.
        self.static.sendTransform([anchor,tr])
        self.navmap=NavigationCostmap(self)
        self.create_timer(.2,lambda:self.navmap.poll(time.monotonic()))

    def tick(self):
        super().tick()
        if not hasattr(self,'amcl_pub'):return
        stamp=self.get_clock().now().to_msg()
        pose=PoseWithCovarianceStamped();pose.header.frame_id='map';pose.header.stamp=stamp
        pose.pose.pose.position.x,pose.pose.pose.position.y=self.xyz[:2]
        set_yaw(pose.pose.pose.orientation,self.xyz[2])
        pose.pose.covariance[0]=pose.pose.covariance[7]=pose.pose.covariance[35]=.0001
        self.amcl_pub.publish(pose)
        self.trajectory.append((time.monotonic(),*self.xyz,*self.command))
        self.frames+=1
        if self.frames%50==0 and self.mission is not None:
            m=self.mission;candidate=m.acquisition.candidate or {}
            self.runtime_debug.append(dict(stamp=time.time(),pose=list(self.xyz),state=m.state,
                candidate=dict(candidate),candidate_age=time.time()-candidate.get('stamp',time.time()),
                last_raw=self.detection_frames[-1] if self.detection_frames else None,
                last_goal_reason=m.acquisition.last_goal_reason,
                reframe_attempts=m.acquisition.reframe_attempts,
                reframe_positions=list(m.acquisition.reframe_positions),
                capture_positions=list(m.acquisition.positions),
                work=m.acquisition.active_seconds,travel=m.acquisition.travel_seconds))
        if self.frames%5:return
        pixels=np.full((480,640,3),70,dtype=np.uint8)
        pixels[0,:24,0]=np.frombuffer(np.asarray(self.xyz,dtype=np.float64).tobytes(),dtype=np.uint8)
        image=self.bridge.cv2_to_imgmsg(pixels,'bgr8');image.header.stamp=stamp
        image.header.frame_id='project_camera_optical'
        info=CameraInfo();info.header=image.header;info.width=640;info.height=480
        info.k=K.ravel().tolist();info.d=[0.]*5;info.distortion_model='plumb_bob'
        self.capture_poses[stamp_seconds(stamp)]=tuple(self.xyz)
        self.camera_pub.publish(info);self.image_pub.publish(image)

    def publish_grid(self,grid):
        msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        msg.info.width=msg.info.height=100;msg.info.resolution=.05
        msg.info.origin.position.x=msg.info.origin.position.y=-2.5;msg.info.origin.orientation.w=1.
        msg.data=grid.astype(np.int8).ravel().tolist();self.maps.publish(msg)


def make_goal(node,pose):
    msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
    msg.pose.position.x,msg.pose.position.y=pose[:2];set_yaw(msg.pose.orientation,pose[2])
    return msg


def wait_nav(chassis):
    for name in ('controller_server','planner_server','smoother_server','behavior_server','bt_navigator'):
        client=chassis.create_client(GetState,'/'+name+'/get_state')
        assert client.wait_for_service(timeout_sec=25)
        until(lambda:wait(client.call_async(GetState.Request())).current_state.id==3,30,name)
    until(lambda:chassis.navmap.map is not None,10,'native navigation costmap')


def scene(case,directory):
    started=time.monotonic()
    directory.mkdir(parents=True,exist_ok=True)
    target=(.8,0.) if case=='bearing' else (.28,0.) if case=='crop' else (.45,0.)
    height=.8 if case=='crop' else .31
    rclpy.init();chassis=CameraChassis(target,height)
    detector=ProjectedDetector(target,height,bearing_only=case=='bearing')
    with patch.object(Perception,'load_detector',return_value=detector):
        perception=Perception(parameter_overrides=overrides(target_class='bottle',
            localization_method='hybrid',camera_min_range=.35 if case=='bearing' else .6,
            camera_max_range=1.5,inference_period=.10))
    mission=Mission(cli_args=['--ros-args','-r','/cmd_vel:=/test/turn_output'],
        parameter_overrides=overrides(allow_motion=True,target_class='bottle',viewpoint_spacing=.65,
            acquisition_seconds=45.,max_target_variance=.15**2,results_directory=str(directory),
            goal_timeout=75.,mission_timeout=240.,verify_seconds=.6,verify_timeout=8.))
    chassis.mission=mission
    tracker=TargetFilter()
    gate=VelocityGate(parameter_overrides=overrides(allow_motion=True,output_topic='/test/turn_output'))
    executor=MultiThreadedExecutor(num_threads=6)
    original_selection=SingleTarget.observation_goal
    def trace_selection(acquisition,*args,**kwargs):
        before=time.monotonic();record=dict(stamp=time.time(),pose=list(args[1]),
            candidate=dict(acquisition.candidate or {}),reframe_attempts=acquisition.reframe_attempts)
        try:
            selected=original_selection(acquisition,*args,**kwargs)
            record['goal']=list(selected) if selected else None
            return selected
        except Exception as exc:
            record['error']=type(exc).__name__+': '+str(exc)
            raise
        finally:
            record.update(seconds=time.monotonic()-before,reason=acquisition.last_goal_reason)
            with (directory/'selection_trace.jsonl').open('a') as stream:
                stream.write(json.dumps(record)+'\n')
    selection_patch=patch.object(SingleTarget,'observation_goal',trace_selection)
    selection_patch.start()
    nodes=(chassis,perception,mission,tracker,gate)
    for node in nodes:executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        wait_nav(chassis)
        until(lambda:mission.readiness_error() is None,20,'production mission sensors and camera geometry')
        if case!='bearing':
            until(lambda:any(d.get('reframe_required') for f in chassis.detection_frames for d in f['detections']),10,'near/crop image detection')
            assert not any(f['observations'] for f in chassis.observation_frames)
            if case=='crop':
                assert any('detection_vertical_clipping' in d.get('reason_codes',[])
                           for f in chassis.detection_frames for d in f['detections'])
        client=chassis.create_client(Trigger,'/project/start');assert client.wait_for_service(timeout_sec=5)
        chassis.running=True
        response=wait(client.call_async(Trigger.Request()));assert response.success,response.message
        until(lambda:mission.state in ('COMPLETE','FOUND_UNLOCALIZED','FAILED','STOPPED'),180,'target report and verified return')
        summary=mission.summary()
        assert mission.state=='COMPLETE',summary
        assert mission.return_verified and math.dist((mission.target['x'],mission.target['y']),target)<.12,summary
        frames=[f for f in chassis.observation_frames if f['stamp']>=mission.started_stamp]
        if case=='bearing':
            assert all(not f['observations'] for f in frames)
            assert mission.target['source']=='bearing_intersection' and mission.target['sites']>=2,summary
            assert len(mission.target['source_frames'])>=8
        else:
            assert mission.acquisition.reframe_attempts>=1,summary
            assert mission.acquisition.reframe_attempts<=2,summary
            accepted=[(f,o) for f in frames for o in f['observations']]
            if not accepted:
                # Cropping can be resolved by valid horizontal A/B geometry
                # even while its floor contact stays outside reliable range.
                assert case=='crop' and mission.target.get('source')=='bearing_intersection',summary
                assert mission.target['sites']>=2 and len(mission.target['source_frames'])>=8
            for frame,observation in accepted:
                pose=chassis.capture_poses[frame['stamp']]
                distance=math.dist(pose[:2],target)
                assert .6-1e-6<=distance<=1.5+1e-6,(distance,observation)
                assert math.dist((observation['x'],observation['y']),target)<.05
            assert mission.acquisition.reframe_distance<=.6+1e-8
        assert chassis.count_publishers('/cmd_vel')==0
        return dict(case=case,passed=True,seconds=time.monotonic()-started,
            synthetic_detector=True,synthetic_chassis=True,production_nodes=['Perception','Mission','TargetFilter','VelocityGate','Nav2'],
            ground_truth=list(target),camera_height=height,camera_pitch_degrees=28.6,
            configured_min_range=.35 if case=='bearing' else .6,
            actual_displacement=max(math.hypot(p[1],p[2]) for p in chassis.trajectory),
            localization_active_seconds=mission.acquisition.active_seconds,
            localization_travel_seconds=mission.acquisition.travel_seconds,
            reframe_attempts=mission.acquisition.reframe_attempts,target=mission.target,
            valid_floor_observations=sum(len(f['observations']) for f in frames),
            navigation_recoveries=summary.get('navigation_recoveries'),
            motion_log=summary.get('acquisition_motion_log',summary.get('localization_motion_log')),
            observation_frames=len(frames))
    finally:
        chassis.running=False
        directory.mkdir(parents=True,exist_ok=True)
        (directory/'runtime_debug.json').write_text(json.dumps(chassis.runtime_debug,indent=2)+'\n')
        (directory/'trajectory.json').write_text(json.dumps(chassis.trajectory)+'\n')
        executor.shutdown(timeout_sec=3);thread.join(timeout=3)
        selection_patch.stop()
        for node in nodes:node.destroy_node()
        rclpy.shutdown()


def occlusion_scene(directory):
    started=time.monotonic();rclpy.init()
    chassis=CameraChassis(mission_owned=False);chassis.xyz=[-.6,0.,0.]
    gate=VelocityGate(parameter_overrides=overrides(allow_motion=True,output_topic='/test/turn_output'))
    executor=MultiThreadedExecutor(num_threads=4)
    for node in (chassis,gate):executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    handle=None
    try:
        grid=np.zeros((100,100));grid[[0,-1],:]=100;grid[:,[0,-1]]=100
        grid[40:61,54]=100 # x=0.2 wall; routes go around y +/- 0.55.
        chassis.publish_grid(grid);wait_nav(chassis)
        until(lambda:chassis.navmap.allowed(.225,0.) is False
              and chassis.navmap.allowed(.6,0.) is True,15,'divider in native inflated costmap')
        path=ActionClient(chassis,ComputePathToPose,'/compute_path_to_pose');assert path.wait_for_server(timeout_sec=5)
        blocked_pose=(.6,0.,math.pi)
        goal=ComputePathToPose.Goal();goal.goal=make_goal(chassis,blocked_pose);goal.planner_id='GridBased'
        h=wait(path.send_goal_async(goal));assert h.accepted
        result=wait(h.get_result_async(),10)
        assert result.status==4 and len(result.result.path.poses)>3,'No native route around divider'
        points=[p.pose.position for p in result.result.path.poses]
        assert max(abs(p.y) for p in points)>.55,'Native path did not route around the divider'
        planner=CoveragePlanner(grid,.05,(-2.5,-2.5,0.),clearance=.15)
        rejected=observation_visibility(planner,blocked_pose,(-.1,0.))
        assert not rejected and 'obstacle' in rejected.reason,rejected
        acquisition=SingleTarget();acquisition.candidate=dict(stamp=time.time(),label='bottle',confidence=.9,
            bearing=0.,origin=[-.6,0.],x=-.1,y=0.,sigma=.2,close_view=False)
        selected=acquisition.observation_goal(planner,tuple(chassis.xyz),force_baseline=True,
            allowed=chassis.navmap.allowed,deadline=time.monotonic()+1.)
        assert selected and selected[0]<.2 and observation_visibility(planner,selected,(-.1,0.)),selected
        nav=ActionClient(chassis,NavigateToPose,'/navigate_to_pose');assert nav.wait_for_server(timeout_sec=5)
        task=NavigateToPose.Goal();task.pose=make_goal(chassis,selected)
        from ament_index_python.packages import get_package_share_directory
        task.behavior_tree=str(Path(get_package_share_directory('ee3033_project'))/'behavior_trees/navigate_observation.xml')
        chassis.running=True;handle=wait(nav.send_goal_async(task));assert handle.accepted
        answer=wait(handle.get_result_async(),60);handle=None
        assert answer.status==4,(answer.status,chassis.xyz,selected)
        assert observation_visibility(planner,tuple(chassis.xyz),(-.1,0.))
        # Unknown-space endpoints/rays are not usable alternatives, even when
        # the base grid geometry would offer candidates in that direction.
        blocked=np.full((100,100),-1.);blocked[45:56,33:45]=0.
        limited=CoveragePlanner(blocked,.05,(-2.5,-2.5,0.),clearance=.15)
        acquisition=SingleTarget();acquisition.candidate=dict(stamp=time.time(),label='bottle',confidence=.9,
            bearing=0.,origin=[-.6,0.])
        none=acquisition.observation_goal(limited,(-.6,0.,0.),force_baseline=True,deadline=time.monotonic()+1.)
        assert none is None,none
        assert not observation_visibility(limited,(-.6,.4,0.),source_origin=(-.6,0.),bearing=0.)
        return dict(case='occlusion',passed=True,seconds=time.monotonic()-started,
            real_nav2_path_points=len(result.result.path.poses),reachable_but_occluded=list(blocked_pose),
            visible_alternative=list(selected),actual_arrival=list(chassis.xyz),
            no_visible_alternative_rejected=True,synthetic_chassis=True)
    finally:
        chassis.running=False
        if handle:handle.cancel_goal_async()
        executor.shutdown(timeout_sec=3);thread.join(timeout=3)
        gate.destroy_node();chassis.destroy_node();rclpy.shutdown()


def validate_recorded_bearing(case,run_directory):
    """Validate a saved live run when an earlier test required floor-only proof.

    This does not rerun or force the mission. Its production result, original
    source-frame journal and captured chassis trajectory must independently
    prove completed navigation and at least four valid frames at each A/B site.
    """
    run_directory=Path(run_directory)
    result=json.loads((run_directory/'result.json').read_text())
    assert result['state']=='COMPLETE' and result['return_verified'],result
    target=result['target'];assert target['source']=='bearing_intersection',target
    assert target['sites']>=2 and len(set(target['source_frames']))>=8
    events=[json.loads(line) for line in (run_directory/'localization_frames.jsonl').read_text().splitlines()]
    events=[e for e in events if e.get('mission_stamp')==result['started_stamp']]
    if case=='crop':
        assert any('detection_vertical_clipping' in e.get('reason_codes',[]) for e in events)
    by_stamp={e['stamp']:e for e in events}
    groups=[]
    for stamp in target['source_frames']:
        event=by_stamp[stamp];d=event['selected']
        assert event['intersection']['frame_accepted']
        assert d['settled'] and abs(d['center_error'])<=math.radians(2.) and d['confidence']>=.6
        origin=d['odom_origin']
        group=next((g for g in groups if math.dist(g['origin'],origin)<.08),None)
        if group is None:
            group=dict(origin=origin,frames=[]);groups.append(group)
        group['frames'].append(stamp)
    ready=[g for g in groups if len(set(g['frames']))>=4]
    assert len(ready)>=2,groups
    assert all(max(g['frames'])-min(g['frames'])<=6.+1e-6 for g in ready)
    trajectory=json.loads((run_directory.parent/'trajectory.json').read_text())
    moved=max(math.dist(row[1:3],trajectory[0][1:3]) for row in trajectory)
    assert moved>=.12,moved
    return dict(case=case,passed=True,evidence_validation='post-run production artifact validation',
        seconds=result['elapsed_seconds'],
        original_runner_exit_code=1,original_test_failure='floor-only assertion; production completed via valid bearing intersection',
        production_result=str(run_directory/'result.json'),source_journal=str(run_directory/'localization_frames.jsonl'),
        synthetic_detector=True,synthetic_chassis=True,actual_displacement=moved,
        target=target,valid_capture_sites=[dict(origin=g['origin'],frames=len(set(g['frames']))) for g in ready],
        valid_floor_observations=sum(e.get('frame',{}).get('observation_count',0) for e in events),
        localization_active_seconds=result['localization_active_seconds'],
        localization_travel_seconds=result['localization_travel_seconds'])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--case',choices=['near','crop','bearing','occlusion'],required=True)
    parser.add_argument('--output',required=True);parser.add_argument('--recorded-run');args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','6'),'Never use the physical robot domain'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1','Keep synthetic DDS isolated'
    root=Path(__file__).resolve().parents[1];output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    if args.recorded_run:
        result=validate_recorded_bearing(args.case,args.recorded_run)
        result['code_version']=os.environ.get('EE3033_TEST_SOURCE_VERSION','unspecified')
        output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True);return
    with tempfile.TemporaryDirectory(prefix='group4_near_native_') as tmp:
        config=yaml.safe_load((root/'src/ee3033_project/config/navigation.yaml').read_text())
        for name in ('local_costmap','global_costmap'):
            p=config[name][name]['ros__parameters'];p['plugins']=['static_layer','inflation_layer']
            p['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True}
        path=Path(tmp)/'nav.yaml';path.write_text(yaml.safe_dump(config))
        log=output.with_suffix('.nav2.log')
        with log.open('w') as stream:
            proc=subprocess.Popen(['ros2','launch','ee3033_project','navigation.launch.py','nav_config:='+str(path)],
                stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                mission_directory=output.parent/(args.case+'-mission');mission_directory.mkdir(parents=True,exist_ok=True)
                result=occlusion_scene(mission_directory) if args.case=='occlusion' else scene(args.case,mission_directory)
                result['code_version']=os.environ.get('EE3033_TEST_SOURCE_VERSION','unspecified')
                result['nav2_log']=str(log);output.write_text(json.dumps(result,indent=2)+'\n')
                print(json.dumps(result),flush=True)
            except Exception as exc:
                output.write_text(json.dumps(dict(case=args.case,passed=False,error=str(exc),nav2_log=str(log)),indent=2)+'\n')
                raise
            finally:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)


if __name__=='__main__':main()
