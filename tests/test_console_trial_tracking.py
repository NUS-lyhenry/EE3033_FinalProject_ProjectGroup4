"""Exercise production Console transactions without ROS, sensors or motion."""
import ast
import copy
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import pytest
from ee3033_project.assessment import atomic_json
from ee3033_project.trial import LIMITS, TRIAL_DEFAULTS, parameter_plan, validate_trial


def console(tmp_path):
    path=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/console.py'
    owner=next(c for c in ast.parse(path.read_text()).body
               if isinstance(c,ast.ClassDef) and c.name=='Console')
    methods=[m for m in owner.body if isinstance(m,ast.FunctionDef)
             and m.name in ('trial_settings','trial_matches')]
    cls=ast.ClassDef(name='Transaction',bases=[],keywords=[],body=methods,decorator_list=[])
    scope=dict(math=math,time=time,validate_trial=validate_trial,
               parameter_plan=parameter_plan,LIMITS=LIMITS,TRIAL_DEFAULTS=TRIAL_DEFAULTS,Twist=lambda:None)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(path),'exec'),scope)
    c=scope['Transaction']();c.p={'phase':'mapping','navigation_profile':'original'};c.lock=threading.Lock()
    c.state={'state':'WAITING'};c.freshness={'Mission':time.monotonic()}
    c.manual_pub=SimpleNamespace(publish=lambda message:None);c.trial_fault=None
    c.nodes=parameter_plan(dict(max_linear=.12,max_angular=.4,goal_timeout=60.))
    c.nodes['project_velocity_gate'].update(stop_distance=.12,rotation_clearance=.14)
    c.read_trial_nodes=lambda:copy.deepcopy(c.nodes)
    c.writes=[]
    def write(name,entries):
        c.writes.append((name,copy.deepcopy(entries)))
        c.nodes[name].update(entries)
    c.write_trial_node=write
    c.navigation_settings_path=tmp_path/'navigation_settings.json'
    c.navigation_settings={}
    workflow=path.with_name('console_workflow.py')
    owner=next(c for c in ast.parse(workflow.read_text()).body if isinstance(c,ast.ClassDef))
    save=next(m for m in owner.body if isinstance(m,ast.FunctionDef) and m.name=='save_navigation_settings')
    namespace={'atomic_json':atomic_json}
    exec(compile(ast.Module(body=[save],type_ignores=[]),str(workflow),'exec'),namespace)
    c.save_navigation_settings=namespace['save_navigation_settings'].__get__(c)
    return c


def test_tracking_readback_and_old_speed_clients_preserve_selected_values(tmp_path):
    c=console(tmp_path);assert c.trial_settings()['values']['alignment_angle']==.35
    chosen=dict(max_linear=.12,max_angular=.4,goal_timeout=60.,lookahead_dist=.08,corner_lookahead_dist=.12,alignment_angle=.10)
    result=c.trial_settings(chosen);assert result['synchronized']
    assert result['values']==chosen
    speed=c.trial_settings(dict(max_linear=.16,max_angular=.6,goal_timeout=75.))
    assert speed['values']['lookahead_dist']==.08
    assert speed['values']['corner_lookahead_dist']==.12
    assert speed['values']['alignment_angle']==.10
    assert speed['values']['max_linear']==.16
    assert json.loads(c.navigation_settings_path.read_text())['trial']==speed['values']
    assert speed['defaults']==TRIAL_DEFAULTS
    live=c.nodes['controller_server']
    assert all(live['FollowPath.'+key]==.08 for key in ('lookahead_dist','min_lookahead_dist','max_lookahead_dist'))


def test_failed_later_node_rolls_back_tracking_and_speed_together(tmp_path):
    c=console(tmp_path);before=copy.deepcopy(c.nodes);write=c.write_trial_node;failed=[False]
    def reject_once(name,entries):
        if name=='project_navigation_supervisor' and not failed[0]:
            failed[0]=True;raise RuntimeError('Synthetic parameter rejection')
        write(name,entries)
    c.write_trial_node=reject_once
    with pytest.raises(RuntimeError,match='original values restored'):
        c.trial_settings(dict(max_linear=.16,max_angular=.6,goal_timeout=75.,lookahead_dist=.08,corner_lookahead_dist=.12,alignment_angle=.10))
    assert c.nodes==before and c.trial_fault is None
    assert c.trial_settings()['synchronized']


@pytest.mark.parametrize('phase,state',[('mapping','MAPPING'),('mission','SEARCHING'),('mission','COMPLETE')])
def test_tracking_is_locked_during_navigation(tmp_path,phase,state):
    c=console(tmp_path);c.p['phase']=phase;c.state['state']=state
    with pytest.raises(ValueError):
        c.trial_settings(dict(max_linear=.12,max_angular=.4,goal_timeout=60.,alignment_angle=.20))
    assert c.writes==[]


def test_storage_failure_rolls_back_live_settings_and_preserves_previous_file(tmp_path):
    c=console(tmp_path)
    chosen=dict(max_linear=.16,max_angular=.6,goal_timeout=80.,lookahead_dist=.09,corner_lookahead_dist=.07,alignment_angle=.31)
    c.trial_settings(chosen)
    before=c.navigation_settings_path.read_bytes()
    def fail(section, values):
        raise OSError('Disk is full')
    c.save_navigation_settings=fail
    with pytest.raises(RuntimeError,match='original values restored'):
        c.trial_settings(dict(chosen,alignment_angle=.25))
    assert c.trial_settings()['values']==chosen
    assert c.navigation_settings_path.read_bytes()==before
