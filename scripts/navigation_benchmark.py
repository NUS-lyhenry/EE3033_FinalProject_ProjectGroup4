#!/usr/bin/env python3
"""Run real Nav2/Mapper/Console against repeatable physical sensor fixtures.

Every run uses a private DDS domain and an explicit saved scene. Known-map
cases use an assigned map/odometry origin with fixed covariance; SLAM cases
use slam_toolbox outputs. Physical truth is recorded only by the evaluator.
The JSON report never turns a failed or unreachable run into a navigation pass.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
from urllib.request import Request,urlopen

sys.path.insert(0,str(Path(__file__).resolve().parent))
from navigation_benchmark_geometry import physically_reachable,farthest_goal,occupancy_grid,observed_free_coverage,geometry_profile
from simulation_scenarios import generate_scenario
from navigation_motion_metrics import MotionMetrics, SPEED_PROFILES, configure_speed, arrival_limits, select_navigation_phase

ROOT=Path(__file__).resolve().parents[1]
CASES=ROOT/'validation/navigation_cases'


def write_json(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(data,indent=2)+'\n')


def prepare_cases(directory=CASES):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    result=[]
    for seed in range(20):
        path=directory/f'random_{seed:02d}.json'
        if not path.exists():
            scene=generate_scenario(seed);goal=farthest_goal(scene)
            write_json(path,dict(case_id=f'random_{seed:02d}',scenario=scene,start_pose=scene['start'],goal=goal,
                oracle=dict(reachable=physically_reachable(scene,goal),radius=.15,resolution=.025),
                provenance='Frozen generated geometry; oracle independent of navigation implementation'))
        result.append(str(path))
    return result


def map_to_world(pose, transform):
    if not transform:return list(pose)
    tx,ty,_=transform['translation'];qx,qy,qz,qw=transform['rotation_xyzw']
    a=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
    dx,dy=pose[0]-tx,pose[1]-ty
    return [math.cos(a)*dx+math.sin(a)*dy,-math.sin(a)*dx+math.cos(a)*dy,pose[2]-a]


def action_graph_snapshot(node,actions):
    """Use this fixture's DDS graph, including unique server endpoint GIDs."""
    result={}
    for action in actions:
        pubs=node.get_publishers_info_by_topic(action+'/_action/status')
        endpoints=[]
        for info in pubs:
            services=node.get_service_names_and_types_by_node(info.node_name,info.node_namespace)
            endpoints.append(dict(node=info.node_namespace.rstrip('/')+'/'+info.node_name,
                endpoint_gid=bytes(info.endpoint_gid).hex(),
                services=[dict(name=name,types=types) for name,types in services if name.startswith(action+'/_action/')]))
        result[action]=dict(server_count=len(pubs),servers=endpoints,
            feedback_publishers=node.count_publishers(action+'/_action/feedback'))
    return result


def partition_cases(cases,shard=None):
    if shard is None:return list(enumerate(cases)),(0,1)
    try:index,total=map(int,shard.split('/'))
    except (AttributeError,ValueError):raise ValueError('shard must be INDEX/TOTAL, e.g. 0/3')
    if not 1<=total<=20 or not 0<=index<total:raise ValueError('Invalid shard index/total')
    return [(i,c) for i,c in enumerate(cases) if i%total==index],(index,total)


def aggregate_matrices(matrices):
    rows=[row for matrix in matrices for row in matrix.get('rows',[])]
    identities=[(r.get('case_id',r.get('case')),r.get('map_mode',r.get('mode')),r.get('task'),r.get('repetition'),r.get('controller'),r.get('motion'),r.get('geometry',{}).get('name'),r.get('speed_profile','original'),r.get('goal_kind','standard'),r.get('navigation_phase','mapping')) for r in rows]
    expected_values={m.get('full_matrix_expected_runs',m.get('expected_runs')) for m in matrices}
    totals={m.get('shard',[0,1])[1] for m in matrices};shards={m.get('shard',[0,1])[0] for m in matrices}
    expected=next(iter(expected_values)) if len(expected_values)==1 else None
    complete_shards=len(totals)==1 and shards==set(range(next(iter(totals))))
    versions={json.dumps(dict(source=r.get('source_sha256'),configuration=r.get('runtime_configuration_sha256',r.get('source_config_sha256')),geometry=r.get('geometry'),library=r.get('controller_library_sha256')),sort_keys=True) for r in rows if r.get('source_sha256')}
    complete=bool(expected and len(rows)==expected and len(set(identities))==len(rows) and complete_shards)
    return dict(rows=rows,expected_runs=expected,completed_runs=len(rows),complete=complete,
        source_versions=len(versions),shards=sorted(shards),duplicate_runs=len(identities)-len(set(identities)),
        positive_arrivals=sum(bool(r.get('passed')) for r in rows),
        safe_negative_cases=sum(bool(r.get('safe_negative') or r.get('safe_stop')) for r in rows),
        passed=complete and len(versions)==1 and all(r.get('passed') or r.get('safe_negative') or r.get('safe_stop') for r in rows))


def future_result(f,timeout=20.):
    end=time.monotonic()+timeout
    while not f.done() and time.monotonic()<end:time.sleep(.01)
    if not f.done():raise TimeoutError('ROS request or action acceptance timed out')
    return f.result()


def run_case(args):
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6') or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Refusing a non-isolated DDS environment')
    phase=select_navigation_phase(args.task,args.map_mode,args.navigation_phase)
    mapping_phase=phase=='mapping'
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.action import ActionClient
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.srv import GetParameters,SetParametersAtomically
    from lifecycle_msgs.srv import GetState
    from geometry_msgs.msg import PoseStamped,TransformStamped,PoseWithCovarianceStamped,Twist
    from nav_msgs.msg import OccupancyGrid,Odometry
    from nav2_msgs.action import NavigateToPose,ComputePathToPose
    from nav2_msgs.srv import GetCostmap
    from std_msgs.msg import Bool,String
    from tf2_ros import StaticTransformBroadcaster,Buffer,TransformListener,TransformException
    from rclpy.time import Time
    from ee3033_project.ros_utils import LATCHED,set_yaw
    from ee3033_project.velocity_gate import VelocityGate
    from ee3033_project.mapper import Mapper
    from ee3033_project.console import Console
    from ee3033_project.scan_resampler import ScanResampler
    from ee3033_project.robot_geometry import DEFAULT_STOP_DISTANCE,NAVIGATION_RADIUS,geometry_metadata
    from simulation_robot import SimulatedRobot
    from capture_navigation_case import NavigationRecorder
    case_bytes=Path(args.case).read_bytes()
    case=json.loads(case_bytes);scene=copy.deepcopy(case['scenario'])
    start_mode='stalled' if args.start=='stalled' or (args.start=='auto' and case.get('stalled_pose')) else 'origin'
    start=case.get('stalled_pose') if start_mode=='stalled' else case.get('start_pose',scene['start'])
    if start is None:raise ValueError('Case does not record a stalled pose')
    goal=case.get('navigation_goal',case['goal']) if args.task=='navigate' else case['goal']
    geometry=geometry_profile(args.geometry_profile)
    initial_transform=case.get('map_from_odom') if args.map_mode=='known' and args.captured_map else None
    world_start=map_to_world(start,initial_transform)
    world_goal=map_to_world(goal,initial_transform)
    oracle=physically_reachable(scene,world_goal,start=world_start,radius=geometry['footprint_radius'])
    directory=Path(args.output);directory.mkdir(parents=True,exist_ok=True)
    record=dict(case_id=case['case_id'],case_sha256=__import__('hashlib').sha256(case_bytes).hexdigest(),controller=args.controller,map_mode=args.map_mode,task=args.task,
        speed_profile=args.speed_profile,goal_kind=args.goal_kind,
        navigation_phase=phase,
        allow_reverse_recovery=args.reverse_recovery=='on',
        motion=args.motion,start=start,start_mode=start_mode,goal=goal,world_start=world_start,world_goal=world_goal,
        initial_map_from_odom=initial_transform,oracle_reachable=oracle,geometry=geometry,passed=False,
        simulation_only=True,physical_robot_contacted=False,ros_domain_id=int(os.environ['ROS_DOMAIN_ID']),
        benchmark_pid=os.getpid(),session_id=__import__('uuid').uuid4().hex)
    record['production_geometry']=geometry_metadata()
    record['scan_mask_radius']=args.scan_mask_radius
    write_json(directory/'case.json',case)
    processes=[];nodes=[];recorder=None;executor=None;handle=None;console=None;started=time.monotonic()
    metrics=None;motion_started=None;motion_finished=None
    rclpy.init()
    try:
        import simulation_robot as physical_simulation
        if not math.isclose(physical_simulation.PHYSICAL_RADIUS,geometry['physical_radius'],rel_tol=0,abs_tol=1e-10):
            raise RuntimeError('Simulator geometry import disagrees with requested profile; no run started')
        robot=SimulatedRobot(args.texture,scene,motion=args.motion);robot.pose=list(world_start)
        # Camera inference is independent of this navigation-only benchmark.
        for timer in robot.timers:
            if timer.timer_period_ns==200000000:timer.cancel()
        scan=ScanResampler(parameter_overrides=[Parameter('self_mask_radius',value=args.scan_mask_radius)]);nodes.extend([robot,scan])
        record['scan_mask_readback']=scan.get_parameter('self_mask_radius').value
        if record['scan_mask_readback']!=args.scan_mask_radius:raise RuntimeError('Scan mask did not match its requested setting')
        config=ROOT/'src/ee3033_project/config'/({'dwb':'navigation_dwb.yaml','rpp-native':'navigation_rpp_native.yaml'}.get(args.controller,'navigation.yaml'))
        # Give both controllers identical dependency-startup allowances.
        import yaml,hashlib
        if args.nav_config:config=Path(args.nav_config)
        source_config=config
        configuration=yaml.safe_load(config.read_text())
        limits=configure_speed(configuration,args.controller,args.speed_profile)
        record['speed_limits']=limits
        metrics=MotionMetrics(limits)
        configuration['bt_navigator']['ros__parameters'].update(wait_for_service_timeout=10000,default_server_timeout=100)
        for costmap_name in ('local_costmap','global_costmap'):
            configuration[costmap_name][costmap_name]['ros__parameters'].update(
                robot_radius=NAVIGATION_RADIUS,footprint_padding=geometry['padding'],footprint='[]')
        config=directory/'navigation_runtime.yaml';config.write_text(yaml.safe_dump(configuration))
        record['runtime_configuration_sha256']=hashlib.sha256(config.read_bytes()).hexdigest()
        record['source_config_sha256']=hashlib.sha256(source_config.read_bytes()).hexdigest()
        record['dependency_timeouts_ms']={'wait_for_service_timeout':10000,'default_server_timeout':100}
        command=['ros2','launch','ee3033_project',
                 'mapping.launch.py' if args.map_mode=='slam' else 'navigation.launch.py',
                 'nav_config:='+str(config),'mapping_mode:='+str(mapping_phase).lower()]
        if args.map_mode=='slam':
            command=command[:-1]+['base:=false','lidar:=false','console:=false','allow_motion:=true',
                'map_directory:='+str(directory/'maps')]
        command+=['allow_reverse_recovery:='+str(args.reverse_recovery=='on').lower()]
        stream=(directory/'launch.log').open('w')
        processes.append(subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
        fixture=Node('navigation_benchmark_fixture');nodes.append(fixture)
        service_group=ReentrantCallbackGroup()
        frame_buffer=Buffer();frame_listener=TransformListener(frame_buffer,fixture)
        if args.map_mode=='known':
            static=StaticTransformBroadcaster(fixture)
            tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom_combined'
            tf.header.stamp=fixture.get_clock().now().to_msg();tf.transform.rotation.w=1.
            # This is an assigned initial frame relationship, never corrected
            # using ongoing physical truth. SLAM runs own this transform instead.
            transform=case.get('map_from_odom') if args.captured_map else None
            if transform:
                for key,value in zip(('x','y','z'),transform['translation']):setattr(tf.transform.translation,key,value)
                for key,value in zip(('x','y','z','w'),transform['rotation_xyzw']):setattr(tf.transform.rotation,key,value)
                # Convert the assigned map start to the fixed odometry frame.
                q=tf.transform.rotation;a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                dx=start[0]-tf.transform.translation.x;dy=start[1]-tf.transform.translation.y
                robot.pose=[math.cos(a)*dx+math.sin(a)*dy,-math.sin(a)*dx+math.cos(a)*dy,start[2]-a]
            static.sendTransform(tf)
            data=case['map'] if args.captured_map and case.get('map') else occupancy_grid(scene,
                args.known_map_resolution or configuration['global_costmap']['global_costmap']['ros__parameters']['resolution'])
            record['known_map_resolution']=data['resolution']
            record['captured_map_unmodified']=bool(args.captured_map and case.get('map'))
            m=OccupancyGrid();m.header.frame_id='map';m.info.width=data['width'];m.info.height=data['height'];m.info.resolution=data['resolution']
            m.info.origin.position.x,m.info.origin.position.y=data['origin'][:2];set_yaw(m.info.origin.orientation,data['origin'][2]);m.data=data['cells']
            maps=fixture.create_publisher(OccupancyGrid,'/map',LATCHED)
            pose_topic='/pose' if mapping_phase else '/amcl_pose'
            pose_pub=fixture.create_publisher(PoseWithCovarianceStamped,pose_topic,10)
            record['localization_fixture']=dict(topic=pose_topic,
                source='Assigned map-odometry origin plus published odometry TF; fixed calibrated-origin covariance',
                xy_variance=.000025,yaw_variance=.0001,oracle_correction=False,actual_amcl_filter=False)
            initial_pose_published=False
            def known_sensors():
                nonlocal initial_pose_published
                m.header.stamp=fixture.get_clock().now().to_msg();maps.publish(m)
                p=PoseWithCovarianceStamped();p.header.frame_id='map'
                try:
                    tr=frame_buffer.lookup_transform('map','base_footprint',Time())
                    stamp=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9
                    if not 0<=fixture.get_clock().now().nanoseconds*1e-9-stamp<=.5:return
                    p.header.stamp=tr.header.stamp
                    p.pose.pose.position.x=tr.transform.translation.x;p.pose.pose.position.y=tr.transform.translation.y
                    p.pose.pose.orientation=tr.transform.rotation
                except TransformException:
                    # One assigned initial calibration is permitted. Never
                    # refresh that old pose as new localization when TF fails.
                    if initial_pose_published:return
                    p.header.stamp=fixture.get_clock().now().to_msg()
                    p.pose.pose.position.x,p.pose.pose.position.y=start[:2];set_yaw(p.pose.pose.orientation,start[2])
                p.pose.covariance[0]=p.pose.covariance[7]=.000025;p.pose.covariance[35]=.0001
                pose_pub.publish(p);initial_pose_published=True
            fixture.create_timer(.5,known_sensors);known_sensors()
            gate=VelocityGate(parameter_overrides=[Parameter(k,value=v) for k,v in
                dict(allow_motion=True,mapping_mode=mapping_phase,**limits,stop_distance=DEFAULT_STOP_DISTANCE,
                     footprint_radius=geometry['footprint_radius'],reverse_radius=geometry['footprint_radius']).items()])
            nodes.append(gate)
            if args.task!='navigate':
                mapper=Mapper(parameter_overrides=[Parameter(k,value=v) for k,v in
                    dict(allow_motion=True,map_directory=str(directory/'maps')).items()])
                nodes.append(mapper)
        console=Console(parameter_overrides=[Parameter(k,value=v) for k,v in
            dict(port=0,phase=phase,allow_motion=True,session_directory=str(directory/'console'),
                 map_directory=str(directory/'maps')).items()]);nodes.append(console)
        recorder_node=Node('navigation_benchmark_recorder');nodes.append(recorder_node)
        recorder=NavigationRecorder(recorder_node,directory)
        def observe(source, linear, angular):
            if motion_started is not None and motion_finished is None:
                metrics.observe(source,time.monotonic(),linear,angular)
        fixture.create_subscription(Twist,'/project/nav_cmd_vel',lambda msg:observe('raw',msg.linear.x,msg.angular.z),10)
        fixture.create_subscription(Twist,'/cmd_vel',lambda msg:observe('final',msg.linear.x,msg.angular.z),10)
        fixture.create_subscription(Odometry,'/odom',lambda msg:observe('odometry',msg.twist.twist.linear.x,msg.twist.twist.angular.z),qos_profile_sensor_data)
        executor=MultiThreadedExecutor(num_threads=8)
        for n in nodes:executor.add_node(n)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        base=f'http://127.0.0.1:{console.p["port"]}/api/'
        def get():
            with urlopen(base+'state',timeout=5) as response:return json.load(response)
        def post(endpoint,body):
            s=get();r=Request(base+endpoint,json.dumps(body).encode(),{'Content-Type':'application/json',
                'X-Project-Token':s['token'],'X-Control-Generation':str(s['control_generation'])},method='POST')
            with urlopen(r,timeout=20) as response:d=json.load(response)
            if d.get('success') is False:raise RuntimeError(str(d))
            return d
        deadline=time.monotonic()+50
        lifecycle=fixture.create_client(GetState,'/bt_navigator/get_state',callback_group=service_group)
        if not lifecycle.wait_for_service(timeout_sec=30):raise TimeoutError('Nav2 lifecycle service unavailable')
        while time.monotonic()<deadline:
            active_state=future_result(lifecycle.call_async(GetState.Request()),5).current_state.id==3
            s=get()
            if active_state and (args.task=='navigate' or s.get('mission',{}).get('ready')):break
            if processes[0].poll() is not None:raise RuntimeError('Navigation launch exited; inspect launch.log')
            time.sleep(.2)
        else:raise TimeoutError('Mapping/TF/costmap readiness timeout')
        readiness_clients=[]
        for action in ('/navigate_to_pose','/project/nav2_navigate_to_pose','/compute_path_to_pose','/project/nav2_compute_path_to_pose'):
            kind=NavigateToPose if 'navigate' in action else ComputePathToPose
            probe=ActionClient(fixture,kind,action,callback_group=service_group);readiness_clients.append(probe)
            if not probe.wait_for_server(timeout_sec=20):raise TimeoutError(f'{action} server not ready')
        time.sleep(.5)
        graph_deadline=time.monotonic()+12.
        record['action_graph_history']=[]
        action_names=('/navigate_to_pose','/project/nav2_navigate_to_pose','/compute_path_to_pose','/project/nav2_compute_path_to_pose')
        while True:
            graph_snapshot=action_graph_snapshot(fixture,action_names)
            record['action_graph']=graph_snapshot
            record['action_graph_history'].append(graph_snapshot)
            complete=all(graph['server_count']==1 for graph in graph_snapshot.values())
            owned=any(info['node']=='/project_navigation_supervisor' for info in graph_snapshot['/navigate_to_pose']['servers'])
            if complete and owned:break
            if time.monotonic()>=graph_deadline:
                raise RuntimeError('Action graph did not converge to one server per public/private action; inspect action_graph_history')
            time.sleep(.25)
        reader=fixture.create_client(GetParameters,'/controller_server/get_parameters',callback_group=service_group)
        speed_names=(['max_vel_x','max_speed_xy','max_vel_theta'] if args.controller=='dwb' else
            ['desired_linear_vel','rotate_to_heading_angular_vel']+
            (['max_linear_vel','max_angular_vel'] if args.controller=='rpp' else []))
        diagnostic_names=[] if args.controller=='dwb' else ['max_angular_accel','lookahead_dist',
            'rotate_to_heading_min_angle','regulated_linear_scaling_min_speed',
            'min_approach_linear_velocity','use_collision_detection']
        names=['FollowPath.plugin']+['FollowPath.'+name for name in speed_names+diagnostic_names]
        reply=future_result(reader.call_async(GetParameters.Request(names=names)),10)
        record['controller_readback']={}
        for key,value in zip(names,reply.values):
            record['controller_readback'][key]={1:value.bool_value,2:value.integer_value,3:value.double_value,4:value.string_value}.get(value.type)
        expected_follow=configuration['controller_server']['ros__parameters']['FollowPath']
        if any(record['controller_readback'].get('FollowPath.'+key)!=expected_follow[key] for key in speed_names):
            raise RuntimeError('Controller speed parameters did not match the requested profile')
        # Mapping launch owns its gate. Set and read it explicitly so a fast
        # controller is never benchmarked through the original final cap.
        gate_setter=fixture.create_client(SetParametersAtomically,'/project_velocity_gate/set_parameters_atomically',callback_group=service_group)
        gate_reader=fixture.create_client(GetParameters,'/project_velocity_gate/get_parameters',callback_group=service_group)
        if not gate_setter.wait_for_service(timeout_sec=10) or not gate_reader.wait_for_service(timeout_sec=10):
            raise RuntimeError('Final velocity gate parameter services unavailable')
        update=SetParametersAtomically.Request(parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in limits.items()])
        applied=future_result(gate_setter.call_async(update),10).result
        if not applied.successful:raise RuntimeError('Final speed cap rejected: '+applied.reason)
        reply=future_result(gate_reader.call_async(GetParameters.Request(names=list(limits))),10)
        record['gate_speed_readback']={key:value.double_value for key,value in zip(limits,reply.values)}
        if record['gate_speed_readback']!=limits:raise RuntimeError('Final speed caps did not match the requested profile')
        gate_names=['mapping_mode','stop_distance','rotation_clearance','footprint_radius','reverse_radius','enforce_swept_map','swept_horizon']
        reply=future_result(gate_reader.call_async(GetParameters.Request(names=gate_names)),10)
        record['gate_safety_readback']={key:{1:value.bool_value,3:value.double_value,4:value.string_value}.get(value.type) for key,value in zip(gate_names,reply.values)}
        if record['gate_safety_readback'].get('mapping_mode')!=mapping_phase:raise RuntimeError('Final gate phase did not match navigation phase')
        gate_deadline=time.monotonic()+5.
        gate_topics=[]
        while time.monotonic()<gate_deadline:
            gate_topics=[topic for topic in ('/scan_fixed','/scan_navigation')
                if any(entry.node_name=='project_velocity_gate' for entry in fixture.get_subscriptions_info_by_topic(topic))]
            if gate_topics:break
            time.sleep(.05)
        record['gate_scan_endpoints']=gate_topics
        if gate_topics!=['/scan_fixed']:raise RuntimeError('Final gate did not use the original shared scan')
        scan_reader=fixture.create_client(GetParameters,'/project_scan_resampler/get_parameters',callback_group=service_group)
        if not scan_reader.wait_for_service(timeout_sec=5.):raise RuntimeError('Original scan producer readback unavailable')
        reply=future_result(scan_reader.call_async(GetParameters.Request(names=['input_topic','output_topic','self_mask_radius'])),5)
        scan_values=reply.values
        record['scan_filter_readback']=dict(input_topic=scan_values[0].string_value,output_topic=scan_values[1].string_value,self_mask_radius=scan_values[2].double_value)
        if (record['scan_filter_readback']['input_topic']!='/scan' or record['scan_filter_readback']['output_topic']!='/scan_fixed'
                or not math.isclose(record['scan_filter_readback']['self_mask_radius'],args.scan_mask_radius,abs_tol=1e-9)):
            raise RuntimeError('Original shared scan parameters differ from the requested cutoff')
        supervisor_reader=fixture.create_client(GetParameters,'/project_navigation_supervisor/get_parameters',callback_group=service_group)
        if not supervisor_reader.wait_for_service(timeout_sec=10):raise RuntimeError('Navigation supervisor readback unavailable')
        reply=future_result(supervisor_reader.call_async(GetParameters.Request(names=['goal_timeout','allow_reverse_recovery'])),10)
        record['navigation_supervisor_readback']=dict(goal_timeout=reply.values[0].double_value,
            allow_reverse_recovery=reply.values[1].bool_value)
        if record['navigation_supervisor_readback']['goal_timeout']!=(60. if mapping_phase else 75.):
            raise RuntimeError('Navigation supervisor timeout did not match the production phase default')
        if record['navigation_supervisor_readback']['allow_reverse_recovery']!=(args.reverse_recovery=='on'):
            raise RuntimeError('Navigation reverse-recovery permission did not match its requested setting')
        record['costmap_safety_readback']={}
        for name in ('local_costmap','global_costmap'):
            parameter_reader=fixture.create_client(GetParameters,f'/{name}/{name}/get_parameters',callback_group=service_group)
            costmap_reader=fixture.create_client(GetCostmap,f'/{name}/get_costmap',callback_group=service_group)
            if not parameter_reader.wait_for_service(timeout_sec=10) or not costmap_reader.wait_for_service(timeout_sec=10):
                raise RuntimeError(f'{name} safety readback service unavailable')
            names=['robot_radius','footprint_padding','resolution','obstacle_layer.scan.topic']
            response=future_result(parameter_reader.call_async(GetParameters.Request(names=names)),10)
            actual={key:{3:value.double_value,4:value.string_value}.get(value.type) for key,value in zip(names,response.values)}
            if actual.get('robot_radius')!=NAVIGATION_RADIUS or actual.get('footprint_padding')!=geometry['padding']:
                raise RuntimeError(f'{name} geometry did not match the production planning envelope')
            if actual.get('obstacle_layer.scan.topic')!='/scan_fixed':
                raise RuntimeError(f'{name} did not consume the original shared scan')
            costmap=future_result(costmap_reader.call_async(GetCostmap.Request()),10).map
            record['costmap_safety_readback'][name]=dict(parameters=actual,actual_resolution=costmap.metadata.resolution,
                frame=costmap.header.frame_id,size=[costmap.metadata.size_x,costmap.metadata.size_y])
        code_files=[ROOT/'scripts/simulation_robot.py',ROOT/'scripts/simulation_motion.py',
            *[ROOT/'src/ee3033_project/ee3033_project'/name for name in ['navigation_supervisor.py','navigation_supervision.py','path_planner.py','path_validation.py','velocity_gate.py','motion_safety.py','mapper.py','mission.py','frontiers.py','waypoint.py','navigation_costmap.py','trial.py','gap.py','robot_geometry.py']],
            ROOT/'src/ee3033_project/config/slam.yaml',ROOT/'src/ee3033_project/launch/navigation.launch.py',
            ROOT/'src/ee3033_navigation/src/regulated_path_follower.cpp']
        code_files += [ROOT/'scripts/navigation_benchmark.py',ROOT/'scripts/navigation_motion_metrics.py',
            ROOT/'src/ee3033_project/ee3033_project/navigation_map.py',
            ROOT/'src/ee3033_project/ee3033_project/scan_resampler.py',
            ROOT/'src/ee3033_project/ee3033_project/scan_core.py']
        code_files += [ROOT/'src/ee3033_navigation/include/ee3033_navigation/corner_tracking.hpp',
            ROOT/'src/ee3033_navigation/include/ee3033_navigation/regulated_path_follower.hpp',
            ROOT/'src/ee3033_navigation/include/ee3033_navigation/goal_heading_trigger.hpp',
            ROOT/'src/ee3033_navigation/include/ee3033_navigation/goal_heading_view.hpp',
            ROOT/'src/ee3033_navigation/include/ee3033_navigation/raw_cost_clearance.hpp',
            ROOT/'scripts/simulation_scenarios.py']
        record['source_sha256']={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in code_files if path.is_file()}
        if args.controller=='rpp':
            from ament_index_python.packages import get_package_prefix
            library=Path(get_package_prefix('ee3033_navigation'))/'lib/libee3033_regulated_path_follower.so'
            record['controller_library_sha256']=hashlib.sha256(library.read_bytes()).hexdigest()
        if args.map_mode=='slam':
            tr=frame_buffer.lookup_transform('map','odom_combined',Time())
            q=tr.transform.rotation;a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            goal=[tr.transform.translation.x+math.cos(a)*world_goal[0]-math.sin(a)*world_goal[1],
                  tr.transform.translation.y+math.sin(a)*world_goal[0]+math.cos(a)*world_goal[1],world_goal[2]+a]
            record['navigation_goal']=goal
            record['dispatch_map_from_odom']={'translation':[tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z],
                'rotation_xyzw':[q.x,q.y,q.z,q.w]}
        if args.task=='navigate':
            client=ActionClient(fixture,NavigateToPose,'/navigate_to_pose',callback_group=service_group)
            if not client.wait_for_server(timeout_sec=10):raise TimeoutError('NavigateToPose unavailable')
            drive=fixture.create_publisher(Bool,'/project/drive_enabled',LATCHED)
            active=fixture.create_publisher(Bool,'/project/mapping_active',LATCHED)
            heartbeat=fixture.create_timer(.1,lambda:(drive.publish(Bool(data=True)),active.publish(Bool(data=mapping_phase))))
            p=NavigateToPose.Goal();p.pose=PoseStamped();p.pose.header.frame_id='map';p.pose.header.stamp=fixture.get_clock().now().to_msg()
            p.pose.pose.position.x,p.pose.pose.position.y=goal[:2];set_yaw(p.pose.pose.orientation,goal[2])
            if args.goal_kind!='standard':
                from ament_index_python.packages import get_package_share_directory
                p.behavior_tree=str(Path(get_package_share_directory('ee3033_project'))/'behavior_trees'/f'navigate_{args.goal_kind}.xml')
            record['goal_acceptance_attempts']=[]
            acceptance_deadline=time.monotonic()+3.
            while True:
                handle=future_result(client.send_goal_async(p))
                status=get().get('navigation',{})
                record['goal_acceptance_attempts'].append(dict(accepted=handle.accepted,stamp=time.monotonic(),navigation=status))
                if handle.accepted:break
                # External DDS discovery can complete before the supervisor's
                # own private Nav2 action client discovers that same server.
                # Retry only a rejected, still-idle startup request; never
                # retry an accepted goal, busy owner or a navigation lock.
                if status.get('stage')!='IDLE' or status.get('locked') or time.monotonic()>=acceptance_deadline:
                    raise RuntimeError('Simulation goal rejected before motion: '+json.dumps(status))
                time.sleep(.25)
            record['accepted']=True
            motion_started=time.monotonic()
            done=handle.get_result_async()
            end=time.monotonic()+args.timeout
            while not done.done() and time.monotonic()<end:time.sleep(.1)
            if not done.done():raise TimeoutError('Navigation action exceeded benchmark deadline')
            answer=done.result();record['action_status']=answer.status
            success=answer.status==4
            motion_finished=time.monotonic()
            fixture.destroy_timer(heartbeat);drive.publish(Bool(data=False));active.publish(Bool(data=False))
        else:
            motion_started=time.monotonic()
            post('start',{}) if args.task=='automap' else post('map_waypoint',dict(zip(('x','y','yaw'),goal)))
            end=time.monotonic()+args.timeout
            while time.monotonic()<end:
                s=get();state=s['mission']['state']
                if state in ('FAILED','STOPPED','MAPPED','MAPPED_PARTIAL','WAYPOINT_REACHED'):break
                time.sleep(.25)
            else:raise TimeoutError('Mapper workflow exceeded benchmark deadline')
            record['mapper']=s['mission']
            success=(state=='MAPPED' and s['mission'].get('return_verified') is True) if args.task=='automap' else state=='WAYPOINT_REACHED'
            motion_finished=time.monotonic()
        with urlopen(base+'map',timeout=5) as response:final_map=json.load(response)
        if final_map:
            tr=frame_buffer.lookup_transform('map','odom_combined',Time());q=tr.transform.rotation
            final_transform={'translation':[tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z],
                'rotation_xyzw':[q.x,q.y,q.z,q.w]}
            record['final_map_from_world']=final_transform
            # A supplied known map does not test SLAM coverage. In narrow
            # corridors, the independent fixed5cm coverage sampler may have
            # no points even though the separately refined reachability oracle
            # proves a path; never let this irrelevant metric hide an action
            # failure or replace its physical endpoint diagnostics.
            if args.map_mode=='slam' or args.task=='automap':
                record['physical_map_coverage']=observed_free_coverage(scene,final_map,final_transform,start=world_start,radius=geometry['footprint_radius'])
        record['physical_final_pose']=list(robot.pose)
        record['final_distance']=math.dist(robot.pose[:2],(world_start if args.task=='automap' else world_goal)[:2])
        record['final_heading_error']=abs(math.atan2(math.sin(robot.pose[2]-world_goal[2]),math.cos(robot.pose[2]-world_goal[2])))
        record['collision_rejections']=robot.collision_rejections
        # A physically reachable positive case must actually finish. Negative
        # cases must stop/refuse safely; they are a distinct outcome category.
        record['outcome']='arrived' if success else 'safe_unreachable' if not oracle and robot.collision_rejections==0 else 'navigation_failure'
        record['passed']=bool(success and oracle and robot.collision_rejections==0)
        record['safe_negative']=bool(not oracle and not success and robot.collision_rejections==0)
        record['expected_safe_stop']=args.motion=='frozen'
        record['safe_stop']=bool(args.motion=='frozen' and not success and robot.collision_rejections==0
            and math.dist(robot.pose[:2],world_start[:2])<.005
            and all(abs(v)<.005 for v in robot.actual_velocity))
        if record['safe_stop']:record['outcome']='safe_frozen_stop'
        if success:
            distance_limit,heading_limit=arrival_limits(args.task,args.goal_kind)
            record['arrival_limits']=dict(distance=distance_limit,heading=heading_limit)
            record['passed'] &= record['final_distance']<=distance_limit
            if heading_limit is not None:record['passed'] &= record['final_heading_error']<=heading_limit
            if args.task=='automap':record['passed'] &= record.get('physical_map_coverage',{}).get('mapped_free_fraction',0)>=.98
    except KeyboardInterrupt:
        record['error']='Benchmark interrupted by outer watchdog or operator';record['outcome']='test_interrupted'
    except Exception as exc:
        record['error']=str(exc);record['outcome']='test_failure'
    finally:
        motion_finished=motion_finished or time.monotonic()
        if motion_started is not None:
            record['motion_seconds']=motion_finished-motion_started
        if console and args.task!='navigate':
            try:
                # Stop the real owner services; do not create a second writer.
                console.command('/api/stop',{})
            except Exception:pass
        if handle:
            try:future_result(handle.cancel_goal_async(),3)
            except Exception:pass
        if nodes:
            try:nodes[0].pause_commands()
            except Exception:pass
        if executor:
            executor.shutdown(timeout_sec=3);thread.join(timeout=3)
        if metrics is not None:
            record['motion_metrics']=metrics.summary(motion_finished)
            caps_ok=record['motion_metrics']['final_caps_respected'] and record['motion_metrics']['complete']
            for key in ('passed','safe_negative','safe_stop'):
                if key in record:record[key]=bool(record[key] and caps_ok)
            if case.get('expected_turn_sign'):
                signed=record['motion_metrics']['streams'].get('odometry',{}).get('signed_angular_integral',0.)
                record['expected_turn_sign']=case['expected_turn_sign']
                record['turn_direction_verified']=bool(signed*case['expected_turn_sign']>=math.pi-.25)
                record['passed']=bool(record.get('passed') and record['turn_direction_verified'])
        if recorder:recorder.close()
        for n in reversed(nodes):n.destroy_node()
        for process in processes:
            try:
                os.killpg(process.pid,signal.SIGINT);process.wait(timeout=12)
            except (ProcessLookupError,subprocess.TimeoutExpired):
                try:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
                except Exception:pass
        if rclpy.ok():rclpy.shutdown()
        record['wall_seconds']=time.monotonic()-started
        write_json(directory/'result.json',record)
    return record


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepare',action='store_true');p.add_argument('--matrix',action='store_true')
    p.add_argument('--aggregate',nargs='+',help='Combine final worker matrix.json files; incomplete/duplicate/mixed-version sets cannot pass')
    p.add_argument('--full-matrix',action='store_true',help='Run all three modes on every repetition (180 random runs)')
    p.add_argument('--shard',help='Partition whole cases across workers, e.g. 0/3; all modes remain balanced per worker')
    p.add_argument('--stop-file',help='Cooperatively interrupt the active case and stop scheduling when this host-bind file exists')
    p.add_argument('--fixed',action='store_true',help='Repeat each recorded fixed case five times')
    p.add_argument('--geometry-profile',choices=['measured','legacy'],default='measured')
    p.add_argument('--case');p.add_argument('--case-ids',help='Comma-separated frozen fixed case IDs; e.g. legacy95,seed102_live,successive_right_angles,tight_passage')
    p.add_argument('--controller',choices=['rpp','dwb','rpp-native'],default='rpp')
    p.add_argument('--speed-profile',choices=list(SPEED_PROFILES),default='original',help='Synchronize controller and final gate: original .12/.40 or fast .16/.60')
    p.add_argument('--goal-kind',choices=['standard','approach','home'],default='standard',help='Exercise the actual short-approach or position-only home behavior tree with navigate')
    p.add_argument('--navigation-phase',choices=['mapping','mission'],help='Default: mission for known-map navigate, mapping for automatic mapping/waypoints; preserves each production timeout/gate mode')
    p.add_argument('--reverse-recovery',choices=['on','off'],default='on',help='Explicit operator recovery permission; verified from the live simulated supervisor')
    p.add_argument('--scan-mask-radius',type=float,default=.14,help='Explicit near-return exclusion radius, recorded separately from physical geometry (0..0.20 m)')
    p.add_argument('--nav-config',help='Explicit diagnostic configuration; recorded by hash')
    p.add_argument('--known-map-resolution',type=float,help='Physical known-map raster resolution; defaults to production costmap resolution, never resamples captured maps')
    p.add_argument('--map-mode',choices=['known','slam'],default='known')
    p.add_argument('--task',choices=['navigate','waypoint','automap'],default='navigate')
    p.add_argument('--start',choices=['auto','origin','stalled'],default='auto',help='auto reproduces a recorded stalled pose when present, otherwise the assigned start');p.add_argument('--captured-map',action='store_true')
    p.add_argument('--motion',choices=['ideal','delayed_deadband','frozen'],default='ideal')
    p.add_argument('--texture',default='/tmp/ee3033_sim/assets/cup.png')
    p.add_argument('--output',default='/tmp/navigation_benchmark');p.add_argument('--timeout',type=float,default=240.)
    p.add_argument('--repeats',type=int,default=3)
    args=p.parse_args()
    if not math.isfinite(args.scan_mask_radius) or not 0<=args.scan_mask_radius<=.20:p.error('--scan-mask-radius must be within0..0.20 m')
    if args.goal_kind!='standard' and args.task!='navigate':p.error('--goal-kind requires --task navigate')
    if args.goal_kind!='standard' and (args.matrix or args.full_matrix):p.error('--goal-kind requires a single case or fixed navigate-only suite')
    if args.navigation_phase=='mission' and (args.matrix or args.full_matrix):p.error('Mission phase requires a known-map navigate-only case or fixed suite')
    if not(args.matrix or args.full_matrix):
        try:select_navigation_phase(args.task,args.map_mode,args.navigation_phase)
        except ValueError as error:p.error(str(error))
    # Set before importing any production node or geometry defaults. Every
    # separately launched ROS process inherits the same explicit profile.
    os.environ['EE3033_ROBOT_GEOMETRY']=args.geometry_profile
    if args.prepare:print(json.dumps(prepare_cases(),indent=2));return
    if args.aggregate:
        combined=aggregate_matrices([json.loads(Path(p).read_text()) for p in args.aggregate])
        write_json(Path(args.output)/'acceptance_matrix.json',combined)
        print(json.dumps({k:v for k,v in combined.items() if k!='rows'},indent=2))
        raise SystemExit(0 if combined['passed'] else 1)
    if args.matrix or args.fixed:
        cases=([args.case] if args.case else [str(p) for p in sorted(CASES.glob('*.json')) if not p.name.startswith('random_') and p.stem!='narrow_doorway']) if args.fixed else prepare_cases()
        if args.case_ids:
            wanted=set(args.case_ids.split(','));cases=[case for case in cases if Path(case).stem in wanted]
            if {Path(case).stem for case in cases}!=wanted:p.error('Unknown case ID in --case-ids')
        rows=[];output=Path(args.output);modes=[('known','navigate'),('slam','waypoint'),('slam','automap')]
        repeats=5 if args.fixed else args.repeats
        selected_cases,shard=partition_cases(cases,args.shard)
        full_expected=len(cases)*repeats*(3 if args.full_matrix else 1)
        expected=len(selected_cases)*repeats*(3 if args.full_matrix else 1)
        if not expected:p.error('This shard has no cases')
        for case_index,case in selected_cases:
            for repeat in range(repeats):
                selected=modes if args.full_matrix else [(args.map_mode,args.task)] if args.fixed else [modes[repeat%3]]
                for mode,task in selected:
                    path=output/Path(case).stem/f'{repeat}_{mode}_{task}'
                    cmd=[sys.executable,__file__,'--case',case,'--controller',args.controller,'--map-mode',mode,'--task',task,
                        '--motion',args.motion,'--speed-profile',args.speed_profile,'--goal-kind',args.goal_kind,
                        '--scan-mask-radius',str(args.scan_mask_radius),
                        '--geometry-profile',args.geometry_profile,'--texture',args.texture,'--output',str(path),'--timeout',str(args.timeout)]
                    if args.nav_config:cmd+=['--nav-config',args.nav_config]
                    if args.navigation_phase:cmd+=['--navigation-phase',args.navigation_phase]
                    cmd+=['--reverse-recovery',args.reverse_recovery]
                    if args.known_map_resolution:cmd+=['--known-map-resolution',str(args.known_map_resolution)]
                    if args.captured_map:cmd.append('--captured-map')
                    cmd+=['--start',args.start]
                    base_domain=int(os.environ.get('ROS_DOMAIN_ID','122'))
                    pool=[d for d in range(base_domain,min(base_domain+max(20,full_expected),233)) if d not in (0,6)]
                    if len(pool)<20:raise RuntimeError('Matrix needs at least 20 private DDS domains; choose a base domain <=213')
                    global_run_index=case_index*repeats*(3 if args.full_matrix else 1)+repeat*(3 if args.full_matrix else 1)+(modes.index((mode,task)) if args.full_matrix else 0)
                    child_env=dict(os.environ,ROS_DOMAIN_ID=str(pool[global_run_index%len(pool)]))
                    if args.stop_file and Path(args.stop_file).exists():
                        write_json(output/'matrix.json',dict(rows=rows,passed=False,expected_runs=expected,full_matrix_expected_runs=full_expected,shard=list(shard),stop_reason='Coordinator stop file'))
                        raise SystemExit(2)
                    child=subprocess.Popen(cmd,start_new_session=True,env=child_env)
                    end=time.monotonic()+args.timeout+120;interrupted=None
                    while child.poll() is None:
                        if args.stop_file and Path(args.stop_file).exists():interrupted='coordinator_stop'
                        elif time.monotonic()>=end:interrupted='outer_timeout'
                        if interrupted:
                            child.send_signal(signal.SIGINT)
                            try:child.wait(timeout=35)
                            except subprocess.TimeoutExpired:
                                child.kill();child.wait()
                                raise RuntimeError('Interrupted test could not cleanly stop; refusing a contaminated next DDS run')
                            break
                        time.sleep(.25)
                    exitcode=interrupted or child.returncode
                    result=path/'result.json'
                    row=json.loads(result.read_text()) if result.exists() else dict(case=case,mode=mode,task=task,passed=False,error=f'Runner exit {exitcode}')
                    row.update(repetition=repeat,case_index=case_index,shard=list(shard),artifact_directory=str(path))
                    rows.append(row)
                    write_json(output/'matrix.json',dict(rows=rows,passed=len(rows)==expected and all(r.get('passed') or r.get('safe_negative') or r.get('safe_stop') for r in rows),expected_runs=expected,
                        full_matrix_expected_runs=full_expected,shard=list(shard),
                        positive_arrivals=sum(bool(r.get('passed')) for r in rows),safe_negative_cases=sum(bool(r.get('safe_negative')) for r in rows),
                        stop_reason=interrupted))
                    if interrupted=='coordinator_stop':raise SystemExit(2)
        raise SystemExit(0 if all(r.get('passed') or r.get('safe_negative') or r.get('safe_stop') for r in rows) else 1)
    if not args.case:p.error('--case is required outside --prepare/--matrix')
    r=run_case(args);print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] or r.get('safe_negative') or r.get('safe_stop') else 1)


if __name__=='__main__':main()
