import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest
from ee3033_project.assessment import TrialRecord

scripts=Path(__file__).parents[1]/'scripts'
sys.path.insert(0,str(scripts))
from simulation_control import SimulationScenarioControl
from simulation_scenarios import default_scenario,validate_scenario
from simulation_camera import default_camera_settings,validate_camera_settings


class Robot:
    def __init__(self):
        self.pose=[.7,-.5,.2];self.commands=True;self.calls=[];self.moving=False
        self.camera=default_camera_settings()
    def stationary(self):return not self.moving
    def pause_commands(self):self.commands=False;self.calls.append('pause')
    def resume_commands(self):self.commands=True;self.calls.append('resume')
    def apply_scenario(self,scene,reset_pose=False):
        assert not self.commands
        self.scene=validate_scenario(scene);self.calls.append(('apply',reset_pose))
        if reset_pose:self.pose=list(scene['start'])
    def camera_settings(self):return dict(self.camera)
    def apply_camera(self,settings):
        assert not self.commands
        self.camera=validate_camera_settings(settings);self.calls.append('apply_camera')
        return dict(self.camera)


class Stage:
    def __init__(self):
        self.loading=False;self.cancel=threading.Event();self.calls=[]
        self.process=NS(poll=lambda:None);self.after_stop=None
    def stop(self):
        self.calls.append('stop');self.process=None
        if self.after_stop:self.after_stop()
    def launch(self,phase,path,options,map_directory):
        self.calls.append(('launch',phase,path))
        self.process=NS(poll=lambda:None)
        return not self.cancel.is_set()


class Console(SimulationScenarioControl):
    def __init__(self,root,phase='mapping'):
        self.lock=threading.Lock();self.stage=Stage();self.p=dict(phase=phase,map_directory=str(root/'maps'))
        self.state=dict(state='WAITING',ready=True,home=None,target=None)
        self.control_generation=0;self.selected_map=None;self.mission_options={'target_class':'cup'}
        self.record=TrialRecord(root/'records',boot_id='fixture');self.record.start('mapping')
        self.map_data={'old_map':True};self.map_revision=4;self.route=[1];self.path=[2]
        self.saved_map='oldmap';self.last_registered_map='oldmap';self.amcl_covariance=[1]
        self.initial_pose_sent=True;self.perception_data={'old':True};self.result_data={'old':True}
        self.target_estimate={'old':True};self.gate_status={'old':True};self.jpeg={'old':b'jpeg'};self.jpeg_time={'old':1.}
        self.freshness=dict(Mission=1.,Map=1.,AMCL=1.,YOLO=1.,Lidar=1.,Odometry=1.)
        self.tf=NS(clear=lambda:setattr(self,'tf_cleared',True));self.tf_cleared=False
        self.get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000))
        self.require_idle=lambda:None;self.require_camera_idle=lambda:None;self.require_zero_command=lambda:None
        self.count_publishers=lambda topic:0;self.scenario_pose_hint=lambda:None
        self.attach_simulation(Robot(),root)


def test_random_map_replaces_the_world_resets_odometry_and_starts_only_fresh_mapping(tmp_path):
    c=Console(tmp_path);old_id=c.record.snapshot()['id'];c.simulation_robot.pose=[.6,.5,1.]
    result=c.scenario_command(dict(action='random_map',seed=42),expected_generation=0)
    assert result['success'] and result['scenario']['seed']==42 and result['scenario']['revision']==1
    assert c.simulation_robot.pose==result['scenario']['start']
    assert c.stage.calls==['stop',('launch','mapping',None)]
    assert c.state['state']=='WAITING' and c.state['target'] is None and c.p['phase']=='mapping'
    assert c.map_data is None and c.target_estimate is None and c.route==c.path==[]
    assert c.selected_map is None and c.saved_map is None and c.tf_cleared
    assert c.record.snapshot()['id']!=old_id and (tmp_path/'records'/old_id/'assessment.json').exists()
    assert c.record.snapshot()['includes_mapping'] and c.record.snapshot()['maps']==[]
    assert c.simulation_robot.commands and not c.stage.loading
    assert json.loads((tmp_path/'scenarios/revision_1.json').read_text())['seed']==42


@pytest.mark.parametrize('state',['MAPPING','SEARCHING','LOCALIZING','ACQUIRING','RECOVERING','RETURNING','VERIFYING_HOME'])
def test_scene_changes_cannot_interrupt_an_active_user_run(tmp_path,state):
    c=Console(tmp_path);c.state['state']=state;c.simulation_robot.calls.clear()
    before=copy.deepcopy(c.simulation_scenario)
    with pytest.raises(ValueError,match='Stop'):c.scenario_command(dict(action='random_map',seed=4))
    assert c.simulation_scenario==before and c.stage.calls==[] and c.simulation_robot.calls==[]


def test_target_only_mapping_preserves_map_and_current_physical_pose(tmp_path):
    c=Console(tmp_path);c.simulation_robot.pose=[.6,-.4,.8]
    old_walls=copy.deepcopy(c.simulation_scenario['walls']);old_trial=c.record.snapshot()['id']
    result=c.scenario_command(dict(action='set_target',x=-.7,y=.7))
    assert result['scenario']['target']==[-.7,.7,0.] and result['scenario']['walls']==old_walls
    assert c.simulation_robot.pose==[.6,-.4,.8] and c.stage.calls==[]
    assert c.map_data=={'old_map':True} and c.record.snapshot()['id']==old_trial
    assert c.target_estimate is None and c.simulation_scenario['placement']['mode']=='manual'


def test_target_only_mission_reloads_same_map_with_new_attempt_and_no_old_mmse(tmp_path):
    c=Console(tmp_path,phase='mission');c.state.update(state='COMPLETE',target={'x':999.})
    c.selected_map=dict(id='saved/map.yaml',path='/fixture/saved/map.yaml',quality='mapped')
    before_pose=list(c.simulation_robot.pose);old_trial=c.record.snapshot()['id']
    result=c.scenario_command(dict(action='random_target',seed=87))
    assert result['success'] and c.simulation_robot.pose==before_pose
    assert c.stage.calls==['stop',('launch','mission','/fixture/saved/map.yaml')]
    assert c.state['state']=='WAITING' and c.state['target'] is None
    assert c.target_estimate is None and c.result_data is None and not c.initial_pose_sent
    assert c.record.snapshot()['includes_mapping'] is False
    assert c.record.snapshot()['reused_map_from_trial']==old_trial
    assert c.record.snapshot()['maps']==['saved/map.yaml']
    assert c.simulation_scenario['placement']==dict(mode='random',seed=87)


def test_stop_race_prevents_world_change_and_successor_launch(tmp_path):
    c=Console(tmp_path);before=copy.deepcopy(c.simulation_scenario)
    def stop():c.stage.cancel.set();c.control_generation+=1
    c.stage.after_stop=stop
    result=c.scenario_command(dict(action='random_map',seed=9))
    assert result['success'] and 'canceled' in result['message']
    assert c.simulation_scenario==before and c.stage.calls==['stop']
    assert c.simulation_robot.commands and not c.stage.loading


def test_stale_control_generation_does_not_change_world_or_stop_stage(tmp_path):
    c=Console(tmp_path)
    with pytest.raises(ValueError,match='Control context'):c.scenario_command(dict(action='random_map',seed=9),expected_generation=2)
    assert not c.stage.calls and c.simulation_scenario['revision']==0


def test_stop_during_late_launch_reaps_the_successor_and_does_not_claim_ready(tmp_path):
    c=Console(tmp_path);original=c.stage.launch
    def late(*args):
        original(*args);c.stage.cancel.set();c.control_generation+=1
        return True
    c.stage.launch=late
    result=c.scenario_command(dict(action='random_map',seed=8))
    assert result['success'] and 'late stage stopped' in result['message']
    assert c.stage.calls==['stop',('launch','mapping',None),'stop']
    assert c.state['state']=='STOPPED' and not c.state['ready']


def test_stage_shutdown_failure_does_not_reenable_old_simulated_commands(tmp_path):
    c=Console(tmp_path)
    def fail():raise RuntimeError('synthetic shutdown failure')
    c.stage.stop=fail
    with pytest.raises(RuntimeError,match='shutdown failure'):
        c.scenario_command(dict(action='random_map',seed=8))
    assert not c.simulation_robot.commands
    assert c.simulation_scenario['revision']==0 and c.workflow_fault


def test_scene_revisions_are_persisted_without_destroying_prior_worlds(tmp_path):
    c=Console(tmp_path)
    c.scenario_command(dict(action='random_map',seed=1))
    first=(tmp_path/'scenarios/revision_1.json').read_bytes()
    c.scenario_command(dict(action='random_map',seed=2))
    assert (tmp_path/'scenarios/revision_1.json').read_bytes()==first
    assert json.loads((tmp_path/'scenarios/current.json').read_text())['seed']==2


def test_raycast_and_render_use_the_same_explicit_world_instead_of_captured_defaults():
    spec=importlib.util.spec_from_file_location('scenario_geometry',scripts/'simulation_geometry.py')
    world=importlib.util.module_from_spec(spec);spec.loader.exec_module(world)
    walls=np.array([[[-2.,-2.],[2.,-2.]],[[2.,-2.],[2.,2.]],[[2.,2.],[-2.,2.]],[[-2.,2.],[-2.,-2.]]])
    assert world.raycast([0.,0.],[0.],walls)[0]==pytest.approx(2.)
    assert world.clearance([0.,0.],walls)==pytest.approx(2.)
    world.WALLS=walls
    assert world.raycast([0.,0.],[0.])[0]==pytest.approx(2.)
    texture=np.full((80,80,3),(20,220,30),np.uint8);pose=[.95,.1,math.pi/2]
    visible=world.render_camera(pose,texture,True,walls,[.95,.85,0.])
    absent=world.render_camera(pose,texture,False,walls,[.95,.85,0.])
    moved=world.render_camera(pose,texture,True,walls,[-.7,.85,0.])
    assert not np.array_equal(visible,absent) and np.array_equal(moved,absent)


def test_physical_camera_mapping_updates_keep_map_pose_and_use_tf_in_estimator(tmp_path):
    c=Console(tmp_path);pose=list(c.simulation_robot.pose);before_map=copy.deepcopy(c.map_data)
    settings=dict(default_camera_settings(),camera_height=.31,camera_pitch=math.radians(28.6),
        camera_yaw=.1,camera_x=.07,camera_y=.01,horizontal_fov=1.25,camera_min_range=.6,camera_max_range=1.2)
    result=c.camera_command(settings,expected_generation=0)
    assert result['success'] and result['camera']['camera_height']==.31
    assert result['camera']['camera_pitch']==settings['camera_pitch'] and result['camera']['revision']==1
    assert c.stage.calls==[] and c.map_data==before_map and c.simulation_robot.pose==pose
    saved=json.loads(Path(c.p['vision_config']).read_text())['project_perception']['ros__parameters']
    assert saved['camera_height']==saved['horizontal_fov']==0. and saved['use_pitch_override'] is False
    assert saved['camera_min_range']==.6 and saved['camera_max_range']==1.2
    scene=json.loads((tmp_path/'scenarios/current.json').read_text())
    assert scene['camera']['camera_height']==.31
    assert c.simulation_snapshot()['simulation_camera']['camera_height']==.31


def test_physical_camera_change_restarts_mission_to_clear_old_target_evidence(tmp_path):
    c=Console(tmp_path,phase='mission');c.state.update(state='COMPLETE',target={'x':999.})
    c.selected_map=dict(id='saved/map.yaml',path='/fixture/saved/map.yaml',quality='mapped')
    result=c.camera_command(dict(default_camera_settings(),camera_height=.31))
    assert result['success'] and c.stage.calls==['stop',('launch','mission','/fixture/saved/map.yaml')]
    assert c.state['target'] is None and c.target_estimate is None and c.result_data is None
    assert c.record.snapshot()['includes_mapping'] is False and not c.initial_pose_sent


def test_saved_physical_camera_is_restored_and_not_reset_by_random_map(tmp_path):
    c=Console(tmp_path)
    c.camera_command(dict(default_camera_settings(),camera_height=.31,camera_pitch=.49))
    c.scenario_command(dict(action='random_map',seed=8))
    restored=Console(tmp_path)
    assert restored.camera_get()['camera']['camera_height']==.31
    assert restored.camera_get()['camera']['camera_pitch']==.49
    assert restored.vision_settings['camera_height']==0.


def test_active_physical_camera_change_is_rejected_before_world_or_stage_changes(tmp_path):
    c=Console(tmp_path);c.state['state']='SEARCHING';c.simulation_robot.calls.clear()
    with pytest.raises(ValueError,match='Stop'):
        c.camera_command(dict(default_camera_settings(),camera_height=.31))
    assert c.stage.calls==[] and c.simulation_robot.calls==[]
    assert c.camera_get()['camera']['camera_height']==.22
