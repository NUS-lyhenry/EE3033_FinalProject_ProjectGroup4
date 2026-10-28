"""Retry policy and late-message isolation, exercising production callbacks."""
import ast
from collections import deque
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS

import pytest
from ee3033_project.assessment import ACTIVE, TrialRecord, mission_options, require_preserved_unknown

ROOT=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project'


def callback(file,name):
    tree=ast.parse((ROOT/file).read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
    namespace=dict(json=json,math=math,time=time,ACTIVE=ACTIVE,mission_options=mission_options,
                   Twist=NS,Trigger=NS(Request=NS),stamp_seconds=lambda s:s.sec+s.nanosec*1e-9,
                   require_preserved_unknown=require_preserved_unknown)
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(ROOT/file),'exec'),namespace)
    return namespace[name]


command=callback('console_workflow.py','workflow_command')
on_status=callback('console.py','on_status')
on_estimate=callback('console.py','on_target_estimate')
on_result=callback('console_workflow.py','on_result')
console_command=callback('console.py','command')


def console(tmp_path):
    record=TrialRecord(tmp_path,boot_id='retry-test')
    record.start('mapping');record.add_map('room/map.yaml')
    record.status('mission',dict(state='COMPLETE',target={'x':1},return_verified=True))
    c=NS(record=record,p=dict(phase='mission',allow_motion=True),
        state=dict(state='COMPLETE',ready=True,started_stamp=10.,home=[0,0,0],target={'x':1}),
        selected_map={'id':'room/map.yaml'},stage=NS(process=object(),loading=False),
        freshness={'Mission':time.monotonic()},initial_pose_sent=True,
        trial_fault=None,gap_fault=None,workflow_fault=None,scan_filter_fault=None,retry_after_stamp=-math.inf,
        result_data={'old':True},target_estimate={'x':1},route=[1],path=[1],events=deque(),
        lock=threading.RLock(),control_generation=2,status_after=0.,map_revision=1,
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=20_000_000_000)))
    c.require_idle=lambda: (_ for _ in ()).throw(ValueError('Active')) if c.state['state'] in ACTIVE or c.stage.loading else None
    c.require_zero_command=lambda:None
    c.workflow_status=lambda status:c.record.status('mission',status)
    def start(path,body,**kwargs):
        assert path=='/api/start' and body=={}
        c.record.start('mission')
        return dict(success=True)
    c.command=start
    return c


def test_repeat_preserves_old_record_and_starts_mission_only_timer(tmp_path):
    c=console(tmp_path);old_id=c.record.data['id']
    old=(tmp_path/old_id/'assessment.json').read_bytes()
    result=command(c,'/api/retry_search',{})
    assert result['success']
    current=c.record.snapshot()
    assert current['id']!=old_id and current['maps']==['room/map.yaml']
    assert current['reused_map_from_trial']==old_id
    assert not current['includes_mapping'] and not current['timing_valid']
    assert current['running'] and current['result'] is None
    assert (tmp_path/old_id/'assessment.json').read_bytes()==old
    assert c.result_data is c.target_estimate is None
    assert c.route==c.path==[] and c.initial_pose_sent
    assert c.control_generation==3


@pytest.mark.parametrize('state',['COMPLETE','NOT_FOUND','FOUND_UNLOCALIZED','SEARCH_INCOMPLETE','STOPPED','FAILED'])
def test_all_terminal_search_states_can_retry(tmp_path,state):
    c=console(tmp_path);c.state['state']=state
    assert command(c,'/api/retry_search',{})['success']


@pytest.mark.parametrize('change',[
    lambda c:c.state.update(state='RECOVERING'),
    lambda c:setattr(c.stage,'loading',True),
    lambda c:c.state.update(ready=False),
    lambda c:c.freshness.update(Mission=-math.inf),
    lambda c:c.p.update(allow_motion=False),
    lambda c:setattr(c,'initial_pose_sent',False),
    lambda c:setattr(c,'workflow_fault','Robot unavailable'),
])
def test_retry_rejects_unready_or_active_without_resetting_record(tmp_path,change):
    c=console(tmp_path);old=c.record.data['id'];change(c)
    with pytest.raises(ValueError):command(c,'/api/retry_search',{})
    assert c.record.data['id']==old


def test_explicit_service_rejection_restores_previous_attempt(tmp_path):
    c=console(tmp_path);previous=json.loads(json.dumps(c.record.data))
    c.command=lambda *a,**k:dict(success=False,message='Mission not ready')
    assert not command(c,'/api/retry_search',{})['success']
    assert c.record.data==previous and c.retry_after_stamp==-math.inf
    assert c.result_data=={'old':True}


def test_late_status_result_and_estimate_cannot_contaminate_repeat(tmp_path):
    c=console(tmp_path);command(c,'/api/retry_search',{})
    before=c.record.snapshot()['id']
    old=dict(phase='mission',state='COMPLETE',started_stamp=10.,stamp=21.,
        reason='Old terminal heartbeat',target={'x':1},return_verified=True)
    on_status(c,NS(data=json.dumps(old)))
    on_result(c,NS(data=json.dumps(old)))
    estimate=dict(method='map-constrained MMSE',frame_id='map',mission_stamp=10.,
        stamp=21.,x=1,y=2,sigma=.1)
    on_estimate(c,NS(data=json.dumps(estimate)))
    assert c.result_data is c.target_estimate is None
    assert c.record.snapshot()['id']==before and c.record.snapshot()['running']
    new=dict(old,state='SEARCHING',started_stamp=20.1,target=None,return_verified=False)
    on_status(c,NS(data=json.dumps(new)))
    assert c.state['state']=='SEARCHING'
    on_estimate(c,NS(data=json.dumps(dict(estimate,mission_stamp=20.1))))
    assert c.target_estimate['mission_stamp']==20.1


@pytest.mark.parametrize('finished,interrupted,includes_mapping',[
    (True,False,True),(False,False,False),(False,True,True),
])
def test_explicit_saved_map_load_prepares_mission_without_rebuilding_or_starting(tmp_path,finished,interrupted,includes_mapping):
    c=console(tmp_path);old_id=c.record.data['id']
    c.record.data.update(finished=1. if finished else None,interrupted=interrupted,
                         includes_mapping=includes_mapping)
    c.p.update(phase='mapping',map_directory=str(tmp_path),calibration_directory='/profiles',
               calibration_port=8780,port=8765)
    c.state.update(state='WAITING');c.stage.cancel=threading.Event()
    entry=dict(id='room/map.yaml',path='/maps/room/map.yaml',reviewed=True,quality='mapped')
    c.catalog=NS(entry=lambda _:entry)
    c.model_labels=NS(require=lambda _:None)
    c.require_camera_idle=lambda:None;c.refresh_camera_profile=lambda:None
    c.manual_pub=NS(publish=lambda _:None)
    c.count_publishers=lambda _:0
    launches=[]
    c.stage.stop=lambda:setattr(c.stage,'process',None)
    c.stage.launch=lambda *a:launches.append(a) or True
    c.command=lambda *a:pytest.fail('Loading a saved map must not start motion')
    result=command(c,'/api/stage',dict(phase='mission',map_id=entry['id']))
    assert result['success'] and 'search-only' in result['message']
    assert launches[0][:2]==('mission',entry['path'])
    assert c.selected_map==entry and c.state['state']=='WAITING'
    assert not c.initial_pose_sent
    record=c.record.snapshot()
    assert record['maps']==[entry['id']] and not record['includes_mapping']
    assert record['started'] is None and record['reused_map_from_trial']==old_id
    assert (tmp_path/old_id/'assessment.json').is_file()


def test_known_bad_unknown_encoding_is_rejected_before_stopping_previous_stage(tmp_path):
    image=tmp_path/'map.pgm';image.write_bytes(b'P5\n3 1\n255\n'+bytes([0,205,254]))
    path=tmp_path/'map.yaml'
    path.write_text('image: map.pgm\nmode: trinary\nnegate: 0\nfree_thresh: 0.25\noccupied_thresh: 0.65\n')
    c=console(tmp_path/'records');old=c.record.data['id'];c.p['phase']='mapping'
    c.model_labels=NS(require=lambda _:None);c.require_camera_idle=lambda:None
    c.catalog=NS(entry=lambda _:dict(id='room/map.yaml',path=str(path),reviewed=True))
    c.stage.stop=lambda:pytest.fail('Bad map must be rejected before stopping old stage')
    with pytest.raises(ValueError,match='unknown cells as free'):
        command(c,'/api/stage',dict(phase='mission',map_id='room/map.yaml'))
    assert c.record.data['id']==old
    path.write_text(path.read_text().replace('0.25','0.196'))
    require_preserved_unknown(path)


@pytest.mark.parametrize('name,content',[
    ('map.png',b'not a PGM fixture'),('map.pgm',b'placeholder'),
    ('map.pgm',b'P5\n3 1\n255\n'+bytes([0,205,253])),
])
def test_legacy_guard_does_not_reinterpret_noncanonical_images(tmp_path,name,content):
    (tmp_path/name).write_bytes(content)
    path=tmp_path/'map.yaml'
    path.write_text(f'image: {name}\nnegate: 0\nfree_thresh: 0.25\noccupied_thresh: 0.65\n')
    require_preserved_unknown(path)


@pytest.mark.parametrize('stop_during',['service_wait','response'])
def test_stop_wins_over_start_wait_and_late_success(tmp_path,stop_during):
    c=console(tmp_path);c.record.repeat_search('room/map.yaml')
    c.workflow_command=lambda *a:None
    calls=[]
    def stop():
        c.control_generation+=1;c.state['state']='STOPPED'
    def available(**kw):
        if stop_during=='service_wait':stop()
        return True
    def request(_):
        calls.append('start')
        if stop_during=='response':stop()
        return NS(done=lambda:True,result=lambda:NS(success=True,message='Started'))
    def stop_request(_):
        calls.append('stop')
        return NS(done=lambda:True,result=lambda:NS(success=True,message='Stopped'))
    c.command_clients={'start':NS(wait_for_service=available,call_async=request),
                       'stop':NS(wait_for_service=lambda **kw:True,call_async=stop_request)}
    result=console_command(c,'/api/start',{})
    assert result['canceled_by_stop'] and not result['success']
    assert c.record.data['started'] is None and c.state['state']=='STOPPED'
    assert calls==([] if stop_during=='service_wait' else ['start','stop'])


def test_stop_during_retry_keeps_stop_generation_and_does_not_restore_old_running_attempt(tmp_path):
    c=console(tmp_path);old=c.record.data['id'];c.record.data['finished']=None
    c.events.append('Stop event')
    def interrupted(*a,**kw):
        c.control_generation+=1;c.state['state']='STOPPED'
        return dict(success=False,canceled_by_stop=True,message='Start canceled by Stop')
    c.command=interrupted
    result=command(c,'/api/retry_search',{})
    assert result['canceled_by_stop'] and c.control_generation==3
    assert c.record.data['id'] is None and c.record.data['reused_map_from_trial']==old
    assert c.state['state']=='STOPPED' and list(c.events)==['Stop event']
