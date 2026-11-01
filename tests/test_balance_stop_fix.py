"""Small balance motion is accepted; unsafe motion and raw locks remain closed."""
import ast
import json
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import pytest

from test_navigation_review import load_supervisor,action_fixture,outer_fixture
from test_navigation_recovery import mission_callbacks


class Clock:
    def __init__(self,mode):self.t=10.;self.mode=mode;self.node=None
    def monotonic(self):return self.t
    def sleep(self,seconds):
        self.t+=.02;n=self.node;t=self.t-10.
        if self.mode=='balance':
            x=.006*math.sin(2*math.pi*t);yaw=.02*math.sin(2*math.pi*t)+.0025*t
            vx=.006*2*math.pi*math.cos(2*math.pi*t);wz=.02*2*math.pi*math.cos(2*math.pi*t)+.0025
        else:
            x=.08*t if self.mode=='creep' else 0.
            yaw=.12*t if self.mode=='turn' else 0.
            vx=wz=0.  # Pose change must catch falsely reported zero twist.
        n.odom=(x,0.,yaw,vx,wz);n.odom_stamp=self.t;n.odom_time=self.t
        if self.mode!='stale_command':n.command_time=self.t


def fixture(mode):
    clock=Clock(mode);cls=load_supervisor(clock);n=cls.__new__(cls);clock.node=n
    n.guard=threading.RLock();n.stop=threading.Event()
    n.p=dict(stationary_timeout=2.,stationary_seconds=.5,sensor_timeout=1.)
    n.status={};n.set_stage=lambda *a,**kw:None;n.interrupted=lambda b:None
    n.odom=(0.,0.,0.,0.,0.);n.odom_stamp=n.odom_time=n.command_time=clock.t
    n.odom_frame=('odom_combined','base_footprint');n.final_command=(0.,0.)
    if mode=='nonzero_command':n.final_command=(.01,0.)
    n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int((clock.t+.001)*1e9)))
    return n,clock


def test_bounded_balance_and_slow_yaw_drift_complete_short_position_hold():
    n,c=fixture('balance')
    assert n.stationary()
    assert c.t>=10.5-1e-8 and n.status['stationary_check']['verified']
    assert n.status['stationary_check']['heading_span']>.02
    assert n.status['stationary_check']['position_only']
    assert n.status['stationary_check']['limits']['position_span']==.02


@pytest.mark.parametrize('mode',['creep','stale_command','nonzero_command'])
def test_unbounded_motion_stale_or_nonzero_command_never_confirms_stop(mode):
    n,_=fixture(mode)
    assert not n.stationary()
    assert not n.status['stationary_check']['verified']


def test_yaw_changes_with_fixed_xy_are_not_a_standstill_lock_trigger():
    n,_=fixture('turn')
    assert n.stationary()
    assert n.status['stationary_check']['heading_span']>.02
    assert n.status['stationary_check']['position_only']


def test_unverified_bounded_stop_still_locks_before_goal_result():
    n=action_fixture();n.status={};n.raw_future=object();events=[]
    n.finish_raw=lambda:True;n.stationary=lambda **kw:False
    n.set_stage=lambda *a,**kw:None
    n.terminal(outer_fixture(events),4,'Navigation goal reached')
    assert events==['aborted'] and n.blocked and n.busy
    assert n.status['lock_reason']=='bounded_stop_unverified'


def test_confirmed_bounded_stop_can_succeed_without_permanent_lock():
    n=action_fixture();n.status={};n.raw_future=object();events=[]
    n.finish_raw=lambda:True;n.stationary=lambda **kw:True
    n.reached_goal=lambda request:True;n.set_stage=lambda *a,**kw:None
    n.terminal(outer_fixture(events),4,'Navigation goal reached')
    assert events==['succeeded'] and not n.blocked
    assert n.status['stop_confirmation']['bounded_stop_confirmed']


def mapper_fixture():
    path=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/mapper.py'
    cls=next(x for x in ast.parse(path.read_text()).body if isinstance(x,ast.ClassDef))
    env=dict(Node=object,json=json,math=math)
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),env)
    n=env['Mapper'].__new__(env['Mapper']);n.state='MAPPING';n.nav_handle=None
    n.navigation_lock_reason=None;n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000))
    n.failures=[]
    def fail(reason):n.state='FAILED';n.failures.append(reason)
    n.fail=fail
    return n


def message(**kw):
    return NS(data=json.dumps(dict(stamp=10.,goal_id='01'*16,locked=True,reason='Unsafe stop',**kw)))


def test_mapper_latches_global_lock_after_goal_handle_was_cleared():
    n=mapper_fixture();n.on_navigation(message())
    assert n.state=='FAILED' and len(n.failures)==1
    assert 'Unsafe stop' in n.error()
    n.send((0.,0.,0.))
    assert len(n.failures)==2  # No new navigation/path request was touched.
    n.on_navigation(message())
    assert len(n.failures)==2


@pytest.mark.parametrize('stamp,goal_id',[(5.,'01'*16),(10.,'invalid')])
def test_mapper_ignores_stale_or_malformed_lock_status(stamp,goal_id):
    n=mapper_fixture();n.on_navigation(NS(data=json.dumps(dict(stamp=stamp,goal_id=goal_id,locked=True,reason='Old'))))
    assert n.navigation_lock_reason is None and n.state=='MAPPING'


def test_mission_latches_global_lock_without_current_handle_and_cancels_planning():
    calls=[];n=NS(state='PLANNING',ACTIVE={'PLANNING','CANCELING'},goal_handle=None,
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)),
        plan_cancel=threading.Event(),navigation_status={},cancel=lambda *a:calls.append(a))
    mission_callbacks()['on_navigation_status'](n,message())
    assert n.plan_cancel.is_set() and calls and calls[0][0]=='FAILED'
    assert 'Unsafe stop' in n.navigation_lock_reason


def test_latched_navigation_fault_does_not_override_operator_stop():
    calls=[];n=NS(state='CANCELING',cancel_next='STOPPED',ACTIVE={'CANCELING'},goal_handle=None,
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)),
        plan_cancel=threading.Event(),navigation_status={},cancel=lambda *a:calls.append(a))
    mission_callbacks()['on_navigation_status'](n,message())
    assert n.plan_cancel.is_set() and not calls
