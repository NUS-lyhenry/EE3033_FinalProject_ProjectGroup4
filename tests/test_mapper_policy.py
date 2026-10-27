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
