"""Next-map settings persist without dynamic writes or fabricated readback."""
import ast
import json
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import pytest

from ee3033_project.assessment import ACTIVE, StageProcess, atomic_json
from ee3033_project.mapping_settings import DEFAULT_SLAM_SETTINGS, SLAM_NATIVE_NAMES, same_slam_settings, validate_slam_settings


def console(tmp_path):
    path=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/console_slam_settings.py'
    cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef))
    scope=dict(json=json,Path=Path,atomic_json=atomic_json,DEFAULT_SLAM_SETTINGS=DEFAULT_SLAM_SETTINGS,
        same_slam_settings=same_slam_settings,validate_slam_settings=validate_slam_settings,SLAM_NATIVE_NAMES=SLAM_NATIVE_NAMES,
        GetParameters=NS(Request=lambda **kw:NS(**kw)),Parameter=NS(from_parameter_msg=lambda msg:msg),
        ParameterMessage=lambda **kw:NS(**kw))
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),scope)
    node=scope['SlamSettings']();node.p=dict(phase='mapping',session_directory=str(tmp_path))
    node.lock=threading.Lock();node.control_generation=3;node.state=dict(state='WAITING')
    node.stage=NS(process=NS(pid=123,poll=lambda:None),loading=False)
    node.scan_filter_editable=lambda:node.state['state'] not in ACTIVE and not node.stage.loading
    node.parameter_clients={('slam_toolbox','get_parameters'):NS(wait_for_service=lambda **kw:True)}
    node.parameter_request=lambda *args:NS(values=[.15,2])
    node.init_slam_settings();return node


@pytest.mark.parametrize('threshold',[True,float('nan'),float('inf'),-.1,0.,1.,'0.1',None])
def test_threshold_validation_rejects_invalid_values(threshold):
    with pytest.raises(ValueError):validate_slam_settings(dict(occupancy_threshold=threshold,minimum_pass_through=2))


@pytest.mark.parametrize('passes',[True,2.,-1,101,'2',None])
def test_pass_count_requires_bounded_integer(passes):
    with pytest.raises(ValueError):validate_slam_settings(dict(occupancy_threshold=.1,minimum_pass_through=passes))


def test_defaults_and_boundaries_are_valid():
    assert validate_slam_settings(DEFAULT_SLAM_SETTINGS)==dict(occupancy_threshold=.15,minimum_pass_through=2)
    for threshold,passes in [(.01,0),(.99,100)]:
        assert validate_slam_settings(dict(occupancy_threshold=threshold,minimum_pass_through=passes))


def test_save_persists_next_launch_and_does_not_write_live_parameters(tmp_path):
    node=console(tmp_path);before=node.slam_settings()
    assert before['synchronized'] and before['live']==DEFAULT_SLAM_SETTINGS
    node.parameter_request=lambda *args:(_ for _ in ()).throw(AssertionError('Saving must not issue any ROS request'))
    selected=dict(occupancy_threshold=.15,minimum_pass_through=4)
    result=node.slam_settings(selected,3)
    assert result['selected']==selected and result['live']==DEFAULT_SLAM_SETTINGS
    assert result['requires_new_map'] and not result['synchronized']
    assert node.stage.slam_settings==selected
    assert json.loads(node.slam_settings_path.read_text())==selected
    reopened=console(tmp_path)
    assert reopened.slam_settings_snapshot()['selected']==selected


@pytest.mark.parametrize('state',['MAPPING','SAVING','SEARCHING','RETURNING','CANCELING','RESETTING'])
def test_active_run_cannot_change_saved_settings(tmp_path,state):
    node=console(tmp_path);node.state['state']=state
    with pytest.raises(ValueError,match='Stop'):node.slam_settings(dict(occupancy_threshold=.2,minimum_pass_through=3),3)
    assert not node.slam_settings_path.exists()


def test_stop_generation_and_transition_cannot_persist(tmp_path):
    node=console(tmp_path)
    with pytest.raises(ValueError,match='Stop superseded'):node.slam_settings(DEFAULT_SLAM_SETTINGS,2)
    node.stage.loading=True
    with pytest.raises(ValueError,match='Stop'):node.slam_settings(DEFAULT_SLAM_SETTINGS,3)
    assert not node.slam_settings_path.exists()


def test_missing_or_invalid_readback_never_means_effective(tmp_path):
    node=console(tmp_path);assert node.slam_settings()['available']
    node.parameter_clients[('slam_toolbox','get_parameters')].wait_for_service=lambda **kw:False
    result=node.slam_settings()
    assert result['live'] is None and result['synchronized'] is None and result['requires_new_map']
    node.parameter_clients[('slam_toolbox','get_parameters')].wait_for_service=lambda **kw:True
    node.parameter_request=lambda *args:NS(values=[True,2])
    result=node.slam_settings()
    assert not result['available'] and result['readback_error']


def test_readback_requests_humble_native_parameter_name(tmp_path):
    node=console(tmp_path);requests=[]
    def request(name,operation,body):
        requests.append(body.names);return NS(values=[.1,2])
    node.parameter_request=request
    assert node.slam_settings()['available']
    assert requests==[['occupancy_threshold','min_pass_through']]


def test_generation_stage_and_phase_changes_invalidate_cached_live_readback(tmp_path):
    node=console(tmp_path);assert node.slam_settings()['available']
    node.control_generation+=1;assert not node.slam_settings_snapshot()['available']
    assert node.slam_settings()['available']
    node.stage.process.pid=999;assert not node.slam_settings_snapshot()['available']
    node.p['phase']='mission';assert not node.slam_settings()['available']


def test_stage_launch_passes_saved_mapping_values_only_to_mapping(tmp_path):
    calls=[]
    def factory(args,**kwargs):calls.append(args);return NS(poll=lambda:None)
    manager=StageProcess(tmp_path,popen=factory)
    manager.slam_settings=dict(occupancy_threshold=.2,minimum_pass_through=7)
    manager.launch('mapping',map_directory=tmp_path)
    assert 'occupancy_threshold:=0.2' in calls[0] and 'minimum_pass_through:=7' in calls[0]
    manager.process=None
    manager.launch('calibration',map_directory=tmp_path)
    assert not any('occupancy_threshold:=' in arg or 'minimum_pass_through:=' in arg for arg in calls[1])
