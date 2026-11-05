#!/usr/bin/env python3
"""Exercise the real simulation API, detector and navigation for one A/B run.

Run mapping once; restart preview_search_comparison.py with each policy before
running mission against that same saved map. The preview restart resets the
physical simulated world to its saved start. Never use a physical endpoint.
"""
import argparse
import gzip
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
from pathlib import Path
import time
import urllib.error
import urllib.request


def evaluate_target_position(before,mission,scene):
    """Evaluate after completion; simulator truth is never sent to algorithms."""
    map_pose=before.get('pose')
    world_pose=before.get('simulation_motion',{}).get('ground_truth')
    target=mission.get('target')
    if not map_pose or not world_pose:
        return dict(evaluation_only=True,position_error_m=None,error='Missing recorded map/world pose alignment')
    yaw=(map_pose[2]-world_pose[2]+math.pi)%(2*math.pi)-math.pi
    c,s=math.cos(yaw),math.sin(yaw)
    dx,dy=scene['target'][0]-world_pose[0],scene['target'][1]-world_pose[1]
    truth=[map_pose[0]+c*dx-s*dy,map_pose[1]+s*dx+c*dy]
    translation=[map_pose[0]-c*world_pose[0]+s*world_pose[1],
                 map_pose[1]-s*world_pose[0]-c*world_pose[1]]
    reported=[target['x'],target['y']] if target else None
    return dict(evaluation_only=True,
        alignment_source='Pre-Start estimated map-to-base pose paired with stationary simulator world pose',
        initial_map_pose=map_pose,initial_world_pose=world_pose,
        estimated_map_from_world=translation+[yaw],frozen_target_world=scene['target'],
        estimated_truth_map=truth,reported_target_map=reported,
        position_error_m=math.dist(reported,truth) if reported else None,
        reported_sigma_m=target.get('sigma') if target else None,
        limitation='Simulation-only error with estimated frame alignment; preferred sigma is not an accuracy validation')


def verify_actual_costmap_resolutions(maps,expected_resolutions):
    for name in ('global_costmap','local_costmap'):
        metadata=maps.get(name) or {}
        value=metadata.get('resolution')
        if (isinstance(value,bool) or not isinstance(value,(int,float))
                or not math.isfinite(value) or not math.isclose(value,expected_resolutions[name],abs_tol=1e-8)
                or not metadata.get('size_x') or not metadata.get('size_y')):
            raise RuntimeError('Actual '+name+' grid does not match the reviewed navigation resolution')
    return True


def baseline_costmap_resolutions(source_resolution, local_resolution):
    """Global static map follows baseline path_search_grid; local remains configured."""
    import numpy as np
    from ee3033_project.path_validation import path_search_grid
    _, global_resolution, _ = path_search_grid(np.zeros((1,1),dtype=int),source_resolution)
    return dict(global_costmap=global_resolution,local_costmap=local_resolution)


def verify_shared_scan_contract(producer,consumers,graph,scan_filter,perception):
    required={'/project_velocity_gate','/local_costmap/local_costmap','/global_costmap/global_costmap'}
    fixed=graph.get('/scan_fixed',{});fixed_subs=set(fixed.get('subscriptions',[]))
    unused_subs=set(graph.get('/scan_navigation',{}).get('subscriptions',[]))
    if producer.get('input_topic')!='/scan' or producer.get('output_topic')!='/scan_fixed':
        raise RuntimeError('Original shared scan producer routing differs')
    for cutoff in (producer.get('self_mask_radius'),scan_filter.get('value'),scan_filter.get('live_value')):
        if isinstance(cutoff,bool) or not isinstance(cutoff,(int,float)) or not math.isclose(cutoff,.20,abs_tol=1e-9):
            raise RuntimeError('Original shared scan cutoff must match the saved20cm setting')
    if scan_filter.get('available') is not True or scan_filter.get('synchronized') is not True or scan_filter.get('fault'):
        raise RuntimeError('Original scan filter live parameter readback is unavailable or mismatched')
    if any(consumers.get(name)!='/scan_fixed' for name in required) or not required.issubset(fixed_subs):
        raise RuntimeError('All navigation and safety consumers must use the original shared scan')
    if required&unused_subs or '/project_perception' in unused_subs:
        raise RuntimeError('An unused scan is connected to a control/perception consumer')
    if '/project_scan_resampler' not in fixed.get('publishers',[]) or '/project_scan_resampler' not in graph.get('/scan',{}).get('subscriptions',[]):
        raise RuntimeError('Actual original scan producer endpoints are unverified')
    if not {'/amcl','/project_mission'}.issubset(fixed_subs):
        raise RuntimeError('Localization or Mission original scan input changed')
    mode=perception.get('localization_method')
    if mode not in ('lidar','ground_plane','hybrid','bottom_inverse') or perception.get('scan_topic')!='/scan_fixed':
        raise RuntimeError('Perception localization method or configured input is unverified')
    if (mode=='lidar')!=('/project_perception' in fixed_subs):
        raise RuntimeError('Perception scan subscription differs from its effective localization method')
    return dict(verified=True,mode='baseline_shared_filtered',control_topic='/scan_fixed',
        mapping_radius_m=.20,navigation_radius_m=.20,actual_consumers=sorted(required))


class Driver:
    def __init__(self,args):
        self.args=args
        self.directory=Path(args.output)
        self.directory.mkdir(parents=True,exist_ok=True)
        self.stream=gzip.open(self.directory/'trace.jsonl.gz','wt')

    def get(self,name='state'):
        with urllib.request.urlopen(self.args.url.rstrip('/')+'/api/'+name,timeout=30.) as response:
            return json.load(response)

    def save(self,name,data):
        if isinstance(data,dict):data={k:v for k,v in data.items() if k not in ('token','control_generation')}
        (self.directory/name).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')

    def post(self,name,data):
        state=self.get()
        if state.get('simulated') is not True or state.get('preview_backend')!='main-ros':
            raise RuntimeError('Refusing a non-native-simulation endpoint')
        request=urllib.request.Request(self.args.url.rstrip('/')+'/api/'+name,
            data=json.dumps(data).encode(),headers={'Content-Type':'application/json',
            'Origin':self.args.origin,'X-Project-Token':state['token'],
            'X-Control-Generation':str(state['control_generation'])})
        try:
            with urllib.request.urlopen(request,timeout=90.) as response:result=json.load(response)
        except urllib.error.HTTPError as exc:raise RuntimeError(exc.read().decode()) from exc
        if result.get('success') is not True:raise RuntimeError(str(result))
        print(name,result.get('message'),flush=True)
        return result

    def wait(self,predicate,timeout,label,allow_failure=False):
        deadline=time.monotonic()+timeout
        previous=None
        while time.monotonic()<deadline:
            state=self.get()
            if state.get('simulated') is not True or state.get('preview_backend')!='main-ros':
                raise RuntimeError('Refusing a non-native-simulation endpoint')
            mission=state.get('mission',{})
            observer=getattr(self,'timing_process',None)
            if label=='mission' and observer is not None and observer.poll() is not None and not getattr(self,'instrumentation_failed',False):
                self.instrumentation_failed=True
                self.save('observer-exited-during-task.json',dict(returncode=observer.returncode,mission=mission))
                if mission.get('state') not in ('COMPLETE','FAILED','STOPPED','NOT_FOUND','FOUND_UNLOCALIZED','SEARCH_INCOMPLETE'):
                    self.post('stop',{})
            status=(mission.get('state'),mission.get('reason'))
            record=dict(wall=time.time(),phase=state.get('phase'),mission=mission,
                pose=state.get('pose'),navigation=state.get('navigation'),
                velocity_gate=state.get('velocity_gate'),simulation_motion=state.get('simulation_motion'),
                health=state.get('health'),perception_details=state.get('perception_details'),
                target_estimate=state.get('target_estimate'))
            self.stream.write(json.dumps(record,separators=(',',':'))+'\n')
            self.stream.flush()
            if status!=previous:
                print(label,status,'coverage',mission.get('observed_fraction'),flush=True)
                previous=status
            if predicate(state):return state
            if state.get('control_fault') or state.get('workflow_fault'):
                self.save(label+'-fault.json',state)
                raise RuntimeError(str(state.get('control_fault') or state.get('workflow_fault')))
            if not allow_failure and mission.get('state') in ('STOPPED','FAILED'):
                self.save(label+'-failure.json',state)
                raise RuntimeError(str(status))
            if label=='mission-ready':self.resend_missing_initial_amcl(state,deadline)
            time.sleep(.5)
        self.save(label+'-timeout.json',self.get())
        raise TimeoutError(label)

    def resend_missing_initial_amcl(self,state,deadline):
        setup=getattr(self,'initial_pose_setup',None)
        mission=state.get('mission') or {}
        if (not setup or setup['resent'] or time.monotonic()-setup['began']<20.
                or mission.get('state')!='WAITING' or mission.get('ready')
                or (mission.get('started_stamp') or 0.)!=0.
                or not str(mission.get('readiness_reason','')).startswith('No AMCL pose')):
            return
        # The first stationary AMCL publication can precede discovery of its
        # volatile Mission subscriber. Resend the SAME assigned setup pose once;
        # never change readiness, task state or a pose after Mission Start.
        setup['resent']=True
        pose=setup['pose']
        response=self.post('initial_pose',dict(x=pose[0],y=pose[1],yaw=pose[2]))
        self.save('prestart-initial-pose-resend.json',dict(pre_start_only=True,
            reason=mission['readiness_reason'],same_assigned_pose=pose,
            elapsed_setup_seconds=time.monotonic()-setup['began'],
            remaining_original_deadline_seconds=max(0.,deadline-time.monotonic()),response=response))

    def labels(self):
        deadline=time.monotonic()+90.
        while time.monotonic()<deadline:
            labels=self.get('labels')
            self.save('model-labels.json',labels)
            if labels.get('ready') and 'bottle' in labels.get('labels',[]):return
            print('labels',labels.get('error'),flush=True)
            time.sleep(1.)
        raise RuntimeError('Configured bottle detector labels did not become ready')

    def settings(self):
        self.post('trial',dict(max_linear=.16,max_angular=.6,goal_timeout=75.))
        self.post('gap',dict(enabled=False,audacity='bold'))
        self.post('recovery',dict(enabled=False))
        recovery=self.get('recovery')
        if recovery.get('value') is not False or recovery.get('live_value') is not False or not recovery.get('synchronized'):
            raise RuntimeError('Recovery reverse must be OFF and match live readback')
        self.save('recovery-readback.json',recovery)
        self.save('trial-readback.json',self.get('trial'))
        scan=self.get('scan_filter')
        if not math.isclose(scan['value'],.20,abs_tol=1e-8) or not math.isclose(scan.get('live_value') or 0.,.20,abs_tol=1e-8):
            raise RuntimeError('The effective near-return cutoff must remain 0.20 m')
        self.save('scan-filter-readback.json',scan)
        self.save('slam-readback.json',self.get('slam_settings'))

    def run(self):
        self.wait(lambda s:True,30.,'console',allow_failure=True)
        self.save('initial-state.json',self.get())
        self.labels()
        self.post('scan_filter',dict(radius=.20))
        self.post('recovery',dict(enabled=False))
        if self.args.phase=='mapping':
            self.post('new_trial',{})
            self.wait(lambda s:s['mission'].get('ready') and s['mission'].get('state')=='WAITING',90.,'mapping-ready')
            self.settings()
            self.save('before-mapping.json',self.get())
            self.post('start',{})
            state=self.wait(lambda s:s['mission'].get('state') in ('MAPPED','MAPPED_PARTIAL'),750.,'mapping')
            self.save('mapped-state.json',state)
            self.save('mapped-map.json',self.get('map'))
            entry=next(e for e in self.get('maps')['maps'] if e['path']==state['saved_map'])
            reviewed=self.post('review_map',dict(map_id=entry['id']))
            entry=reviewed.get('map',entry)
            summary=dict(map=entry,initial_pose=state['mission']['home'],
                scene=state['simulation_scenario'],scan_mask_radius=.20,
                mapping_return_verified=state['mission'].get('return_verified'),mapping_state=state['mission']['state'])
            self.save('mapping-summary.json',summary)
            return summary
        mapping=json.loads(Path(self.args.mapping_summary).read_text())
        initial=self.get()
        actual=initial.get('simulation_motion',{}).get('ground_truth')
        expected=mapping['scene']['start']
        if actual is None or math.dist(actual[:2],expected[:2])>.005:
            raise RuntimeError('Restart the preview so both policies begin at the same physical simulated start')
        scene=initial['simulation_scenario']
        for key in ('walls','target','start','camera'):
            if scene.get(key)!=mapping['scene'].get(key):raise RuntimeError('Paired physical world changed: '+key)
        entry=next(e for e in self.get('maps')['maps'] if e['path']==mapping['map']['path'])
        if not entry['reviewed']:raise RuntimeError('Paired map must already be reviewed')
        self.save('loaded-map-catalogue-entry.json',entry)
        mapping['map']=entry
        receipt=Path(self.args.mapping_summary)
        temporary=receipt.with_name(receipt.name+'.refreshing')
        temporary.write_text(json.dumps(mapping,indent=2)+'\n')
        os.replace(temporary,receipt)
        self.post('stage',dict(phase='mission',map_id=entry['id'],target_class='bottle',
            localization_method='hybrid',preferred_target_sigma=.3,return_tolerance=.2,
            enable_ab_maneuver=False,acquisition_seconds=90.,bottle_min_range=.35,bottle_max_range=.95))
        self.wait(lambda s:s.get('health',{}).get('Map',{}).get('ok') and s.get('health',{}).get('Mission',{}).get('ok') and s['mission'].get('stamp') and s['mission'].get('state')=='WAITING',90.,'mission-loaded')
        self.wait(lambda s:s.get('health',{}).get('YOLO',{}).get('ok') and bool(s.get('mission',{}).get('camera_model')),120.,'camera-calibration-ready')
        pose=mapping['initial_pose']
        self.post('initial_pose',dict(x=pose[0],y=pose[1],yaw=pose[2]))
        self.initial_pose_setup=dict(pose=list(pose),began=time.monotonic(),resent=False)
        self.wait(lambda s:s['mission'].get('ready'),120.,'mission-ready')
        self.settings()
        self.verify_policy()
        self.save('runtime-fingerprints.json',self.runtime_fingerprints(entry))
        self.save('before-search.json',self.get())
        timing=self.start_timing_logger()
        terminal=('COMPLETE','NOT_FOUND','FOUND_UNLOCALIZED','SEARCH_INCOMPLETE','FAILED','STOPPED')
        budget_exceeded=False
        try:
            prestart=self.get()
            frame=prestart.get('perception_details') or {}
            if (not prestart['mission'].get('ready') or frame.get('stale') is not False
                    or not isinstance(frame.get('age'),(int,float)) or not 0<=frame['age']<=1.):
                self.save('prestart-freshness-failure.json',prestart)
                raise RuntimeError('Fresh processed perception frames and Mission readiness are required before Start')
            self.save('before-search.json',prestart)
            self.post('start',{})
            try:
                state=self.wait(lambda s:s['mission'].get('state') in terminal,self.args.task_timeout,'mission',allow_failure=True)
            except TimeoutError:
                state=self.get()
                if state['mission'].get('state') in terminal:
                    budget_exceeded=state['mission'].get('elapsed_seconds',math.inf)>self.args.task_timeout
                else:
                    budget_exceeded=True
                    self.save('task-budget-before-stop.json',state)
                    self.post('stop',{})
                    state=self.wait(lambda s:s['mission'].get('state') in terminal,20.,'test-budget-stop',allow_failure=True)
        except Exception:
            self.stop_owned_task_on_error()
            raise
        finally:
            timing_status=self.stop_timing_logger(timing)
        self.save('final-state.json',state)
        self.save('result.json',self.get('result'))
        mission=state['mission']
        position=evaluate_target_position(json.loads((self.directory/'before-search.json').read_text()),mission,scene)
        self.save('target-position-evaluation.json',position)
        workflow_passed=bool(not budget_exceeded and mission.get('state')=='COMPLETE' and mission.get('return_verified') and mission.get('target_reported') and mission.get('target'))
        summary=dict(workflow_passed=workflow_passed,timing_evidence_valid=timing_status['valid'],test_instrumentation_failed=getattr(self,'instrumentation_failed',False),test_budget_seconds=self.args.task_timeout,test_budget_exceeded=budget_exceeded,position_evaluation=position,evidence='Real YOLO, production target estimation and native Nav2; simulated sensors/chassis; no physical validation',
            strategy=self.args.strategy,map=entry,task_seconds=mission.get('elapsed_seconds'),
            timing='Mission Start through localization/report, return and verified stop; mapping excluded',
            state=mission.get('state'),target=mission.get('target'),
            displayed_target=state.get('target_estimate'),return_verified=mission.get('return_verified'),
            target_reported=mission.get('target_reported'),scene=scene,
            passed=bool(workflow_passed and timing_status['valid']))
        self.save('summary.json',summary)
        print(json.dumps(summary),flush=True)
        return summary

    def timing_required_streams(self):
        required=('odom','tf:odom_combined->base_footprint','scan_fixed','mission_status','target_detections','cmd_vel')
        return required+('scan_navigation',) if getattr(self,'navigation_scan_required',True) else required

    def start_timing_logger(self):
        helper=Path(__file__).resolve().parent/'passive_simulation_timing.py'
        ready=self.directory/'timing-ready.json'
        ready.unlink(missing_ok=True)
        log=(self.directory/'timing-logger.log').open('w')
        command=[sys.executable,str(helper),'--output',str(self.directory/'raw-timing.jsonl.gz'),
            '--ready-file',str(ready),'--duration','1250']
        if getattr(self,'navigation_scan_required',True):command.append('--require-navigation-scan')
        if getattr(self.args,'navigation_geometry',False):command.append('--navigation-geometry')
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+15.
        while time.monotonic()<deadline:
            if ready.exists() and process.poll() is None:
                status=json.loads(ready.read_text())
                if status.get('ready'):
                    before=status;smoke_end=time.monotonic()+15.
                    while time.monotonic()<smoke_end:
                        if process.poll() is not None:break
                        status=json.loads(ready.read_text())
                        if not status.get('ready') or time.time()-status.get('updated_wall_seconds',0)>2.:break
                        time.sleep(.25)
                    else:
                        after=json.loads(ready.read_text())
                        growing=all(after['streams'][key]['samples']>before['streams'][key]['samples']
                                    for key in self.timing_required_streams() if key!='cmd_vel')
                        if growing:
                            self.save('timing-prestart-smoke.json',dict(passed=True,seconds=15.,before=before,after=after))
                            self.timing_process=process
                            return process,log
                    break
            if process.poll() is not None:break
            time.sleep(.1)
        self.stop_timing_logger((process,log))
        raise RuntimeError('Passive timing recorder did not become ready; mission was not started')

    def stop_timing_logger(self,timing):
        process,log=timing
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:process.wait(timeout=3.)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3.)
        log.close()
        summary_path=self.directory/'raw-timing.jsonl.gz.summary.json'
        summary=json.loads(summary_path.read_text()) if summary_path.is_file() else {}
        required=self.timing_required_streams()
        streams=summary.get('streams') or {}
        valid=bool(process.returncode==0 and summary and summary.get('error') is None
                   and all(streams.get(key,{}).get('samples',0)>0 for key in required)
                   and not getattr(self,'instrumentation_failed',False))
        result=dict(valid=valid,returncode=process.returncode,error=summary.get('error'),
                    required_streams={key:streams.get(key,{}).get('samples',0) for key in required})
        self.save('timing-recorder-status.json',result)
        return result

    def stop_owned_task_on_error(self):
        try:
            state=self.get()
            if state.get('simulated') is not True or state.get('preview_backend')!='main-ros':return
            if state['mission'].get('state') not in ('COMPLETE','FAILED','STOPPED','NOT_FOUND','FOUND_UNLOCALIZED','SEARCH_INCOMPLETE','WAITING'):
                self.post('stop',{})
                state=self.wait(lambda s:s['mission'].get('state') in ('STOPPED','FAILED','COMPLETE'),20.,'driver-error-stop',allow_failure=True)
            self.save('driver-error-final-state.json',state)
        except Exception as error:
            self.save('driver-error-stop-failure.json',dict(error=str(error)))
            if Path('/.dockerenv').exists() and os.environ.get('ROS_DOMAIN_ID')=='174':
                for process in Path('/proc').iterdir():
                    if not process.name.isdigit():continue
                    try:
                        argv=(process/'cmdline').read_bytes().split(b'\0')
                        env=(process/'environ').read_bytes().split(b'\0')
                    except OSError:continue
                    if len(argv)>1 and argv[1]==b'scripts/preview_search_comparison.py' and b'ROS_DOMAIN_ID=174' in env:
                        os.kill(int(process.name),signal.SIGINT)

    def runtime_fingerprints(self,entry):
        root=Path(__file__).resolve().parents[1]
        files=[root/'src/ee3033_project/ee3033_project'/name for name in
               ('perception.py','target_filter.py','mission.py','coverage.py','bottom_range.py','ros_utils.py','scan_resampler.py','velocity_gate.py','console_scan.py','path_planner.py','navigation_map.py')]
        files += [root/'scripts'/name for name in ('simulation_camera.py','simulation_geometry.py','simulation_robot.py','actual_yolo_search_comparison.py','passive_simulation_timing.py')]
        files += sorted((root/'src/ee3033_project/web').iterdir())
        files += [root/'src/ee3033_project/config/project.yaml']
        files += sorted((root/'src/ee3033_project/config').glob('navigation*.yaml'))
        files += [Path(entry['path']),Path(entry['path']).with_suffix('.pgm'),Path('/tmp/ee3033_sim/assets/yolo26n.pt')]
        served_ui={}
        for url,name in (('/', 'index.html'),('/app.js','app.js'),('/style.css','style.css')):
            with urllib.request.urlopen(self.args.url.rstrip('/')+url,timeout=30.) as response:body=response.read()
            expected=(root/'src/ee3033_project/web'/name).read_bytes()
            if body!=expected:raise RuntimeError('Served UI does not match source: '+name)
            served_ui[name]=hashlib.sha256(body).hexdigest()
        libraries=[]
        for process in Path('/proc').iterdir():
            if not process.name.isdigit():continue
            try:
                args=(process/'cmdline').read_bytes()
                environment=(process/'environ').read_bytes()
                if b'/nav2_controller/controller_server' not in args or b'ROS_DOMAIN_ID=174' not in environment:continue
                for row in (process/'maps').read_text().splitlines():
                    if 'libee3033_regulated_path_follower.so' not in row:continue
                    path=Path(row.split()[-1])
                    if str(path) not in libraries:libraries.append(str(path));files.append(path)
            except (OSError,PermissionError):continue
        if not libraries:raise RuntimeError('Could not verify the loaded isolated controller library')
        fingerprints={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in files}
        if self.args.controller_sha256 and any(fingerprints[path]!=self.args.controller_sha256 for path in libraries):
            raise RuntimeError('Loaded controller library differs from the accepted native build')
        return dict(controller_libraries=libraries,served_ui_sha256=served_ui,
            sha256=fingerprints,
            optical_model='Measured original unrectified K/D with physical distortion and image-aspect target',
            physical_target=dict(height_m=.18,texture_sha256=hashlib.sha256(Path('/tmp/ee3033_sim/assets/bottle.png').read_bytes()).hexdigest()))

    def verify_policy(self):
        if os.environ.get('ROS_DOMAIN_ID')!='174' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise RuntimeError('Policy readback is restricted to isolated domain 174')
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rcl_interfaces.msg import Parameter as ParameterMessage
        from rcl_interfaces.srv import GetParameters
        import yaml
        from nav2_msgs.srv import GetCostmap
        rclpy.init()
        node=Node('actual_yolo_comparison_readback',enable_rosout=False,start_parameter_services=False)
        try:
            client=node.create_client(GetParameters,'/project_mission/get_parameters')
            if not client.wait_for_service(timeout_sec=5.):raise RuntimeError('Mission parameter service unavailable')
            future=client.call_async(GetParameters.Request(names=['search_strategy','search_linear_speed','search_angular_speed']))
            rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
            if not future.done() or not future.result():raise RuntimeError('Mission strategy readback failed')
            values=future.result().values
            readback=dict(search_strategy=values[0].string_value,search_linear_speed=values[1].double_value,
                          search_angular_speed=values[2].double_value)
            self.save('policy-readback.json',readback)
            if readback['search_strategy']!=self.args.strategy:raise RuntimeError('Effective search strategy differs from requested comparison')
            def parameters(name,names):
                client=node.create_client(GetParameters,name+'/get_parameters')
                if not client.wait_for_service(timeout_sec=5.):raise RuntimeError('Scan consumer parameter service unavailable: '+name)
                future=client.call_async(GetParameters.Request(names=names))
                rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
                response=future.result() if future.done() else None
                if response is None or len(response.values)!=len(names):raise RuntimeError('Incomplete scan parameter readback: '+name)
                return {key:Parameter.from_parameter_msg(ParameterMessage(name=key,value=value)).value
                        for key,value in zip(names,response.values)}
            producer=parameters('/project_scan_resampler',['input_topic','output_topic','self_mask_radius'])
            consumer_params={name:parameters(name,[key]+(['resolution'] if name.endswith('/local_costmap') or name.endswith('/global_costmap') else [])) for name,key in (
                ('/local_costmap/local_costmap','obstacle_layer.scan.topic'),
                ('/global_costmap/global_costmap','obstacle_layer.scan.topic'))}
            consumers={name:values['obstacle_layer.scan.topic'] for name,values in consumer_params.items()}
            resolutions={name:values['resolution'] for name,values in consumer_params.items() if 'resolution' in values}
            if len(resolutions)!=2 or any(not isinstance(value,(int,float)) or not math.isclose(value,.0125,abs_tol=1e-9) for value in resolutions.values()):
                self.save('costmap-resolution-mismatch.json',resolutions)
                raise RuntimeError('Declared costmap parameters must match the retained12.5mm configuration')
            actual_costmaps={}
            for name in ('global_costmap','local_costmap'):
                client=node.create_client(GetCostmap,'/'+name+'/get_costmap')
                if not client.wait_for_service(timeout_sec=5.):raise RuntimeError('Actual costmap service unavailable: '+name)
                future=client.call_async(GetCostmap.Request())
                rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
                response=future.result() if future.done() else None
                if response is None:raise RuntimeError('Actual costmap grid readback failed: '+name)
                grid=response.map;metadata=grid.metadata
                actual_costmaps[name]=dict(frame=grid.header.frame_id,resolution=metadata.resolution,
                    size_x=metadata.size_x,size_y=metadata.size_y,
                    origin=[metadata.origin.position.x,metadata.origin.position.y],
                    origin_quaternion=[metadata.origin.orientation.x,metadata.origin.orientation.y,
                                       metadata.origin.orientation.z,metadata.origin.orientation.w])
            self.save('actual-costmap-metadata.json',actual_costmaps)
            map_entry=json.loads(Path(self.args.mapping_summary).read_text())['map']
            source_resolution=float(yaml.safe_load(Path(map_entry['path']).read_text())['resolution'])
            expected_resolutions=baseline_costmap_resolutions(source_resolution,resolutions['/local_costmap/local_costmap'])
            verify_actual_costmap_resolutions(actual_costmaps,expected_resolutions)

            perception_parameters=parameters('/project_perception',['localization_method','scan_topic'])
            effective_perception=self.get().get('perception') or {}
            perception=dict(configured_localization_method=perception_parameters['localization_method'],
                localization_method=effective_perception.get('localization_method'),scan_topic=perception_parameters['scan_topic'])
            scan_filter=self.get('scan_filter')
            control_topic='/scan_fixed'
            graph={}
            deadline=time.monotonic()+10.
            while time.monotonic()<deadline:
                graph={topic:dict(publishers=sorted({entry.node_namespace.rstrip('/')+'/'+entry.node_name
                    for entry in node.get_publishers_info_by_topic(topic)}),subscriptions=sorted({entry.node_namespace.rstrip('/')+'/'+entry.node_name
                    for entry in node.get_subscriptions_info_by_topic(topic)})) for topic in ('/scan','/scan_fixed','/scan_navigation')}
                complete=(
                    {'/project_velocity_gate','/local_costmap/local_costmap','/global_costmap/global_costmap'}.issubset(graph[control_topic]['subscriptions'])
                    and '/project_scan_resampler' in graph[control_topic]['publishers']
                    and '/project_scan_resampler' in graph['/scan_fixed']['publishers']
                    and '/project_scan_resampler' in graph['/scan']['subscriptions']
                    and {'/amcl','/project_mission'}.issubset(graph['/scan_fixed']['subscriptions'])
                    and (perception['localization_method']!='lidar' or '/project_perception' in graph['/scan_fixed']['subscriptions']))
                if complete:break
                rclpy.spin_once(node,timeout_sec=.1)
            scan_filter=self.get('scan_filter')
            # Baseline gate hardcodes this subscription and has no scan_topic parameter.
            gate_topics=[topic for topic,endpoints in graph.items() if '/project_velocity_gate' in endpoints['subscriptions']]
            consumers['/project_velocity_gate']=gate_topics[0] if gate_topics==['/scan_fixed'] else None
            receipt=dict(producer=producer,consumer_parameters=consumer_params,costmap_resolutions=resolutions,actual_costmap_metadata=actual_costmaps,expected_actual_resolutions=expected_resolutions,perception=perception,graph=graph,scan_filter=scan_filter)
            self.save('scan-scope-readback.json',receipt)
            receipt['validation']=verify_shared_scan_contract(producer,consumers,graph,scan_filter,perception)
            self.navigation_scan_required=control_topic=='/scan_navigation'
            self.save('scan-scope-readback.json',receipt)

        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase',choices=('mapping','mission'),required=True)
    parser.add_argument('--strategy',choices=('adaptive','greedy'),default='adaptive')
    parser.add_argument('--mapping-summary')
    parser.add_argument('--controller-sha256',help='Required accepted binary hash when checking a final candidate')
    parser.add_argument('--task-timeout',type=float,default=360.,help='Explicit test budget after Mission Start; expiry saves evidence and Stops this simulation mission')
    parser.add_argument('--navigation-geometry',action='store_true',help='Enable passive diagnostic scan/pose/plan/arc evidence; no algorithm changes')
    parser.add_argument('--output',required=True)
    parser.add_argument('--url',default='http://127.0.0.1:8790')
    parser.add_argument('--origin',default='http://127.0.0.1:8793')
    args=parser.parse_args()
    if args.phase=='mission' and not args.mapping_summary:parser.error('mission requires --mapping-summary')
    if not math.isfinite(args.task_timeout) or args.task_timeout<=0:parser.error('task-timeout must be positive and finite')
    driver=Driver(args)
    try:
        result=driver.run()
    finally:
        driver.stream.close()
    raise SystemExit(0 if args.phase=='mapping' or result.get('passed') else 1)
