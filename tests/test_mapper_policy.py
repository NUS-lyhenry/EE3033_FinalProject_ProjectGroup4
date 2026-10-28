"""Actual mapper callbacks loaded without ROS imports for focused regressions."""
import ast
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace as NS
import pytest
from ee3033_project.stationary import StationaryWindow

source=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/mapper.py'
tree=ast.parse(source.read_text())
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Mapper')
namespace=dict(Node=object,math=math,time=time,json=json)
exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),namespace)
Mapper=namespace['Mapper']


@pytest.mark.parametrize('goal,count,accepted',[
    ('01'*16,1,True),('02'*16,1,False),('01'*16,3,False),('01'*16,True,False)])
def test_recovery_status_only_belongs_to_current_accepted_goal(goal,count,accepted):
    mapper=Mapper.__new__(Mapper)
    mapper.state='MAPPING';mapper.ACTIVE={'MAPPING'}
    mapper.get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000))
    mapper.nav_handle=NS(goal_id=NS(uuid=bytes.fromhex('01'*16)))
    mapper.navigation_recoveries=[];mapper.seen_recoveries=set()
    msg=NS(data=json.dumps(dict(goal_id=goal,recovery_count=count,stamp=100.,reason='blocked')))
    mapper.on_navigation(msg);mapper.on_navigation(msg)
    assert len(mapper.navigation_recoveries)==int(accepted)


def test_costmap_rejects_a_candidate_that_changed_after_worker_selection():
    mapper=Mapper.__new__(Mapper)
    mapper.pending=False;mapper.nav_handle=None
    mapper.navigation_map=NS(fresh=lambda:True,allowed=lambda *a:False)
    results=[];mapper.done=results.append
    mapper.nav=NS(send_goal_async=lambda *a:pytest.fail('Unsafe goal dispatched'))
    mapper.send((1.,2.,.3))
    assert mapper.goal==(1.,2.,.3) and results==[False]


@pytest.mark.parametrize('fresh',[False,True])
def test_only_a_fresh_costmap_can_enable_physical_goal_selection(fresh):
    mapper=Mapper.__new__(Mapper)
    mask=object();calls=[]
    mapper.navigation_map=NS(fresh=lambda now:fresh,
        mask=lambda *args:(calls.append(args) or mask))
    result=mapper.selection_costmap_mask((20,30),.0125,(0.,0.,0.),42.)
    assert result is (mask if fresh else None)
    assert len(calls)==int(fresh)
    if not fresh:assert 'fresh Nav2 costmap' in mapper.reason


@pytest.mark.parametrize('state',['MAPPING','RETURNING'])
def test_costmap_expiry_after_selection_stops_before_goal_or_home_dispatch(state):
    mapper=Mapper.__new__(Mapper);mapper.state=state
    mapper.pending=False;mapper.nav_handle=None
    mapper.navigation_map=NS(fresh=lambda:False,
        allowed=lambda *a:pytest.fail('Stale map used for an endpoint check'))
    failures=[];mapper.fail=failures.append
    mapper.nav=NS(send_goal_async=lambda *a:pytest.fail('Stale goal dispatched'))
    mapper.send((1.,2.,.3))
    assert failures==['Fresh Nav2 costmap unavailable before mapping navigation dispatch']


@pytest.mark.parametrize('state,tree_name',[
    ('MAPPING',''),('RETURNING','navigate_home.xml'),
])
def test_only_mapping_home_dispatch_selects_position_only_navigation_tree(monkeypatch,state,tree_name):
    monkeypatch.setitem(namespace,'Path',Path)
    monkeypatch.setitem(namespace,'get_package_share_directory',lambda name:'/tmp/project-share')
    monkeypatch.setitem(namespace,'NavigateToPose',NS(Goal=lambda:NS(behavior_tree='')))
    mapper=Mapper.__new__(Mapper)
    mapper.state=state;mapper.pending=False;mapper.nav_handle=None;mapper.goal_serial=4
    mapper.navigation_map=NS(fresh=lambda:True,allowed=lambda *a:True)
    mapper.make_pose=lambda pose:pose
    sent=[]
    mapper.nav=NS(send_goal_async=lambda goal:(sent.append(goal) or NS(add_done_callback=lambda cb:None)))
    pose=(1.,2.,.3)
    mapper.send(pose)
    assert len(sent)==1 and sent[0].pose==pose
    assert sent[0].behavior_tree==(
        str(Path('/tmp/project-share/behavior_trees')/tree_name) if tree_name else '')
    assert mapper.pending and mapper.goal_serial==5 and mapper.goal==pose


def test_late_mapping_result_cannot_clear_a_newer_handle_or_resume_search():
    mapper=Mapper.__new__(Mapper)
    mapper.goal_serial=7;handle=object();mapper.nav_handle=handle
    mapper.done=lambda *a:pytest.fail('Old terminal result processed')
    mapper.result(None,6)
    assert mapper.nav_handle is handle


def test_late_mapping_acceptance_is_canceled_without_replacing_newer_handle():
    mapper=Mapper.__new__(Mapper)
    mapper.goal_serial=7;handle=object();mapper.nav_handle=handle
    canceled=[]
    old=NS(accepted=True,cancel_goal_async=lambda:canceled.append(True))
    mapper.accepted(NS(result=lambda:old),6)
    assert canceled==[True] and mapper.nav_handle is handle


def test_home_navigation_failure_preserves_map_without_claiming_return():
    mapper=Mapper.__new__(Mapper)
    mapper.state='RETURNING';mapper.home=(0.,0.,0.);mapper.pose=lambda:(1.,1.,0.)
    mapper.partial=False;mapper.return_verified=False;mapper.p={'map_directory':'unused'}
    mapper.worker=NS(submit=lambda *args:object())
    namespace['save_new_map']=lambda *a:None
    mapper.done(False)
    assert mapper.partial and not mapper.return_verified and mapper.state=='SAVING'
    assert 'failed' in mapper.return_reason.lower()


def test_nonzero_or_stale_final_command_cannot_accumulate_mapping_home_hold():
    mapper=Mapper.__new__(Mapper)
    mapper.stop_window=StationaryWindow(seconds=.1)
    mapper.command_zero=True;mapper.command_time=0.
    msg=NS(linear=NS(x=.01,y=0.,z=0.),angular=NS(x=0.,y=0.,z=0.))
    mapper.stop_window.add(1.,0.,0.,0.,0.,0.,'odom')
    mapper.on_command(msg)
    assert not mapper.command_zero and not mapper.stop_window.samples


def test_cancellation_finishes_stopped_without_dispatching_a_successor():
    mapper=Mapper.__new__(Mapper)
    mapper.state='CANCELING'
    mapper.send=lambda _:pytest.fail('Cancellation must not start recovery or exploration')
    mapper.done(False)
    assert mapper.state=='STOPPED' and mapper.finished_at>0


@pytest.mark.parametrize('state',['MAPPING','RETURNING','TURNING'])
def test_navigation_timeout_uses_first_revision_failure_policy(state):
    mapper=Mapper.__new__(Mapper)
    now=time.monotonic()
    mapper.state=state;mapper.nav_handle=object();mapper.pending=False
    mapper.p={'goal_timeout':60.,'clearance':.14}
    mapper.get_parameter=lambda key:NS(value=mapper.p[key])
    mapper.navigation_map=NS(poll=lambda _:None)
    mapper.auto_pub=mapper.drive_pub=NS(publish=lambda _:None)
    namespace['Bool']=lambda **kw:kw
    mapper.error=lambda:None
    mapper.goal_time=now-61
    failures=[]
    def fail(reason):
        failures.append(reason)
        raise TimeoutError(reason)
    mapper.fail=fail
    mapper.send=lambda _:pytest.fail('Timeout must not start another navigation goal')
    with pytest.raises(TimeoutError):mapper.tick()
    assert failures==['Mapping navigation time limit exceeded']
