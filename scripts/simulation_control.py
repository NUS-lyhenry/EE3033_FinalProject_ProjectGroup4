"""Scene lifecycle for the hardware-free preview, never imported by real Console."""
import copy
import json
import math
from pathlib import Path
import secrets
import time

from ee3033_project.assessment import ACTIVE
from simulation_scenarios import default_scenario,generate_scenario,validate_scenario,with_target,random_target


class SimulationScenarioControl:
    def attach_simulation(self,robot,root):
        self.simulation_robot=robot
        self.simulation_scenario_root=Path(root)/'scenarios'
        self.simulation_scenario_root.mkdir(parents=True,exist_ok=True)
        path=self.simulation_scenario_root/'current.json'
        stored=json.loads(path.read_text()) if path.is_file() else default_scenario()
        scene=validate_scenario(stored)
        self.simulation_scenario=dict(scene,revision=int(stored.get('revision',0)),
            placement=stored.get('placement',dict(mode='default',seed=None)))
        robot.pause_commands()
        robot.apply_scenario(scene,reset_pose=True)
        robot.resume_commands()

    def simulation_edit_reason(self):
        robot=getattr(self,'simulation_robot',None)
        if robot is None:return 'Simulation world is preparing'
        if self.state.get('state') in ACTIVE:return 'Stop the current run before changing the physical world'
        if self.stage is None:return 'Scenario controls require the managed simulation preview'
        if self.stage.loading:return 'Wait for the stage transition to finish'
        if self.p['phase']=='calibration':return 'Leave camera setup before changing the simulation world'
        if not robot.stationary():return 'Wait for the simulated robot to stop'
        return None

    def simulation_snapshot(self):
        reason=self.simulation_edit_reason()
        return dict(simulation_scenario=copy.deepcopy(getattr(self,'simulation_scenario',default_scenario())),
            simulation_can_apply=reason is None,simulation_scenario_reason=reason)

    def scenario_get(self):
        data=self.simulation_snapshot()
        return dict(simulated=True,scenario=data['simulation_scenario'],
            can_apply=data['simulation_can_apply'],reason=data['simulation_scenario_reason'])

    @staticmethod
    def scenario_seed(value):
        if value is None:return secrets.randbelow(2**32)
        if isinstance(value,bool) or not isinstance(value,int) or not 0<=value<2**32:
            raise ValueError('Seed must be an integer from 0 to 4294967295')
        return value

    def scenario_command(self,body,expected_generation=None):
        if not isinstance(body,dict):raise ValueError('Scenario request must be an object')
        reason=self.simulation_edit_reason()
        if reason:raise ValueError(reason)
        self.require_idle();self.require_camera_idle()
        action=body.get('action')
        current=copy.deepcopy(self.simulation_scenario)
        scene=validate_scenario(current)
        seed=None
        if action=='random_map':
            seed=self.scenario_seed(body.get('seed'));scene=generate_scenario(seed)
            placement=dict(mode='generated',seed=seed)
        elif action=='random_target':
            seed=self.scenario_seed(body.get('seed'));scene=random_target(scene,seed)
            placement=dict(mode='random',seed=seed)
        elif action=='set_target':
            scene=with_target(scene,[body['x'],body['y']])
            placement=dict(mode='manual',position=list(scene['target']))
        else:raise ValueError('Choose random_map, random_target or set_target')
        if expected_generation is not None and expected_generation!=self.control_generation:
            raise ValueError('Control context changed; refresh before changing the scenario')
        if self.stage.process is not None and self.stage.process.poll() is None:
            self.require_zero_command()
        # Target-only changes do not change the SLAM world or odometry. If a
        # mission stack exists, restart it to discard all old target evidence.
        reset_world=action=='random_map'
        restart=reset_world or self.p['phase']=='mission'
        selected=copy.deepcopy(self.selected_map) if not reset_world else None
        if restart and not reset_world and not selected:
            raise ValueError('No loaded map is available for the next simulated mission')
        options=copy.deepcopy(self.mission_options)
        pose_hint=self.scenario_pose_hint() if not reset_world and self.p['phase']=='mission' else None
        generation=self.control_generation
        robot=self.simulation_robot
        self.stage.cancel.clear();self.stage.loading=True
        robot.pause_commands()
        transition_failed=False
        try:
            if restart:
                self.stage.stop()
                deadline=time.monotonic()+4.
                while self.count_publishers('/cmd_vel') or self.count_publishers('/project/mission_status'):
                    if time.monotonic()>deadline:raise RuntimeError('Previous simulated stage publishers are still present')
                    time.sleep(.05)
            if self.stage.cancel.is_set() or self.control_generation!=generation:
                return dict(success=True,message='Scenario change canceled by Stop; world unchanged',scenario=current)
            robot.apply_scenario(scene,reset_pose=reset_world)
            self.simulation_scenario=dict(scene,revision=current.get('revision',0)+1,placement=placement)
            self.persist_simulation_scenario()
            if not restart:
                # Mapping never renders the target, so its sensor evidence and
                # current SLAM map remain valid when only target placement moves.
                with self.lock:
                    self.control_generation+=1
                    self.target_estimate=None;self.result_data=None
                return dict(success=True,message='Target placed in the physical simulation world; current map retained',
                    scenario=copy.deepcopy(self.simulation_scenario))
            phase='mapping' if reset_world else 'mission'
            self.record.persist()
            if reset_world:self.record.reset()
            elif selected:self.record.repeat_search(selected['id'])
            else:raise RuntimeError('No loaded map is available for the next simulated mission')
            with self.lock:
                self.control_generation+=1;generation=self.control_generation
                self.status_after=self.get_clock().now().nanoseconds*1e-9
                self.retry_after_stamp=-math.inf;self.retry_stop_generation=None
                self.p['phase']=phase
                self.state=dict(state='WAITING',reason='Preparing the new simulation '+phase+' stage',home=None,target=None,ready=False)
                self.map_data=None;self.map_revision+=1;self.route=[];self.path=[]
                self.saved_map=None;self.last_registered_map=None
                self.amcl_covariance=None;self.initial_pose_sent=False
                self.perception_data=None;self.result_data=None;self.target_estimate=None;self.gate_status=None
                self.jpeg.clear();self.jpeg_time.clear()
                self.selected_map=selected;self.mission_options=options
                self.trial_fault=self.gap_fault=self.workflow_fault=None;self.gap_reapply_required=False
                for key in ('Mission','Map','AMCL','YOLO','Lidar','Odometry'):self.freshness.pop(key,None)
            self.tf.clear()
            if self.stage.cancel.is_set() or self.control_generation!=generation:
                return dict(success=True,message='Scenario applied; Stop prevented starting a new stage',scenario=self.simulation_scenario)
            self.stage.map_partial=bool(selected and selected.get('quality')!='mapped')
            if not self.stage.launch(phase,selected['path'] if selected else None,options,self.p['map_directory']):
                return dict(success=True,message='Scenario applied; new stage remains stopped',scenario=self.simulation_scenario)
            if self.stage.cancel.is_set() or self.control_generation!=generation:
                self.stage.stop()
                with self.lock:
                    self.state=dict(state='STOPPED',reason='Stop prevented a late simulation stage launch',home=None,target=None,ready=False)
                return dict(success=True,message='Scenario applied; late stage stopped after Stop',scenario=self.simulation_scenario)
            if phase=='mapping':self.record.start('mapping')
            self.stage.loading=False
            seeded=False
            if phase=='mission' and pose_hint:
                seeded=self.seed_simulation_pose(pose_hint,generation)
            message=('New physical world loaded; fresh mapping is waiting for Start' if reset_world else
                'Target placed; same map reloaded with clean target evidence'+
                (' and the preserved robot pose' if seeded else '; set the current robot pose before searching'))
            return dict(success=True,message=message,scenario=copy.deepcopy(self.simulation_scenario))
        except Exception as exc:
            transition_failed=True
            self.workflow_fault='Simulation scene transition failed; no automatic restart: '+str(exc)
            raise
        finally:
            if not transition_failed:robot.resume_commands()
            self.stage.loading=False

    def scenario_pose_hint(self):
        if not self.state.get('ready'):return None
        try:
            from rclpy.time import Time
            from ee3033_project.core import quaternion_yaw,stamp_seconds
            tr=self.tf.lookup_transform('map','base_footprint',Time())
            age=self.get_clock().now().nanoseconds*1e-9-stamp_seconds(tr.header.stamp)
            if not -.5<=age<=1.:return None
            p,q=tr.transform.translation,tr.transform.rotation
            return dict(x=p.x,y=p.y,yaw=quaternion_yaw(q))
        except Exception:return None

    def seed_simulation_pose(self,pose,generation):
        deadline=time.monotonic()+12.
        while time.monotonic()<deadline:
            if self.stage.cancel.is_set() or self.control_generation!=generation:return False
            if (self.state.get('state')=='WAITING' and self.map_data and
                    self.initial_pub.get_subscription_count()>0 and
                    time.monotonic()-self.freshness.get('Mission',-math.inf)<2.):
                self.command('/api/initial_pose',pose,expected_generation=generation)
                return True
            time.sleep(.05)
        return False

    def persist_simulation_scenario(self):
        from ee3033_project.assessment import atomic_json
        data=copy.deepcopy(self.simulation_scenario)
        atomic_json(self.simulation_scenario_root/'current.json',data)
        atomic_json(self.simulation_scenario_root/('revision_'+str(data['revision'])+'.json'),data)
