"""Actual mapper callbacks loaded without ROS imports for focused regressions."""
import ast
import math
from pathlib import Path
import time
from types import SimpleNamespace as NS
import pytest
from ee3033_project.stationary import StationaryWindow

source=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/mapper.py'
tree=ast.parse(source.read_text())
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Mapper')
namespace=dict(Node=object,math=math,time=time)
exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),namespace)
Mapper=namespace['Mapper']


def test_costmap_rejects_a_candidate_that_changed_after_worker_selection():
    mapper=Mapper.__new__(Mapper)
    mapper.pending=False;mapper.nav_handle=None
    mapper.navigation_map=NS(allowed=lambda *a:False)
    results=[];mapper.done=results.append
    mapper.nav=NS(send_goal_async=lambda *a:pytest.fail('Unsafe goal dispatched'))
    mapper.send((1.,2.,.3))
    assert mapper.goal==(1.,2.,.3) and results==[False]


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
    mapper.escape_active=False
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


def test_escape_is_started_only_after_stalled_frontier_has_finished_canceling():
    mapper=Mapper.__new__(Mapper)
    mapper.state='CANCELING';mapper.resume_after_cancel=True;mapper.return_escape_pending=False
    mapper.escape_active=False;mapper.escape_pending=True;mapper.escape_target=(1.,2.,0.)
    calls=[]
    mapper.finish_frontier=lambda ok:calls.append(('frontier',ok))
    mapper.start_mapping_escape=lambda target:calls.append(('escape',mapper.state,target))
    mapper.done(False)
    assert calls==[('frontier',False),('escape','MAPPING',(1.,2.,0.))]
    assert not mapper.escape_pending


def test_stalled_escape_is_not_recorded_as_a_failed_frontier_or_retried():
    mapper=Mapper.__new__(Mapper)
    mapper.state='CANCELING';mapper.resume_after_cancel=True;mapper.return_escape_pending=False
    mapper.escape_active=True;mapper.escape_pending=False;mapper.goal=(1.,2.,0.)
    mapper.finish_frontier=lambda _:pytest.fail('Escape goal recorded as frontier')
    mapper.start_mapping_escape=lambda:pytest.fail('Stalled escape immediately retried')
    mapper.done(False)
    assert mapper.state=='MAPPING' and not mapper.escape_active and mapper.goal is None


def test_escape_is_bounded_and_requires_a_fresh_navigation_map():
    mapper=Mapper.__new__(Mapper)
    mapper.mode='automatic';mapper.map=object();mapper.pose=lambda:(0.,0.,0.)
    mapper.escape_attempts=6
    mapper.navigation_map=NS(fresh=lambda:pytest.fail('Exceeded recovery budget'))
    mapper.start_mapping_escape()
    mapper.escape_attempts=0
    mapper.navigation_map=NS(fresh=lambda:False)
    mapper.send=lambda _:pytest.fail('Stale map must not dispatch a goal')
    mapper.start_mapping_escape()


def test_user_stop_while_cancellation_is_pending_never_starts_recovery():
    mapper=Mapper.__new__(Mapper)
    mapper.state='CANCELING';mapper.resume_after_cancel=False;mapper.save_after_cancel=False
    mapper.return_escape_pending=False
    mapper.escape_active=False;mapper.escape_pending=False
    mapper.start_mapping_escape=lambda:pytest.fail('User Stop must prevent recovery')
    mapper.done(False)
    assert mapper.state=='STOPPED'


def test_return_recovery_waits_for_cancellation_and_retries_home_after_success():
    mapper=Mapper.__new__(Mapper)
    mapper.state='CANCELING';mapper.return_escape_pending=True;mapper.home=(0.,0.,0.)
    calls=[]
    mapper.start_mapping_escape=lambda home:calls.append(('escape',home)) or True
    mapper.done(False)
    assert mapper.state=='RETURNING' and calls==[('escape',mapper.home)]
    mapper.escape_active=True;mapper.goal=(.5,.5,0.)
    mapper.send=lambda goal:calls.append(('home',goal))
    mapper.done(True)
    assert not mapper.escape_active and calls[-1]==('home',mapper.home)


def test_return_recovery_without_a_safe_goal_saves_partial_map():
    mapper=Mapper.__new__(Mapper)
    mapper.state='CANCELING';mapper.return_escape_pending=True;mapper.home=(0.,0.,0.)
    mapper.start_mapping_escape=lambda _:False
    saved=[];mapper.save_partial=lambda:saved.append(True)
    mapper.done(False)
    assert saved==[True] and 'no safe recovery' in mapper.return_reason
