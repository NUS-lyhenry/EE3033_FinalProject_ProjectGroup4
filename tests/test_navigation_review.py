"""Deterministic review regressions for cancellation and actual stationary pose."""
import ast
from concurrent.futures import Future
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
import pytest

from ee3033_project.core import quaternion_yaw,stamp_seconds
from ee3033_project.stationary import StationaryWindow
from ee3033_project.navigation_supervision import wrap


ROOT=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project'


def load_supervisor(clock=time):
    source=ROOT/'navigation_supervisor.py'
    cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef))
    env=dict(Node=object,time=clock,threading=threading,math=math,wrap=wrap,StationaryWindow=StationaryWindow,
             rclpy=NS(ok=lambda:True),stamp_seconds=stamp_seconds,quaternion_yaw=quaternion_yaw,
             GoalStatus=NS(STATUS_SUCCEEDED=4,STATUS_CANCELED=5,STATUS_ABORTED=6),
             NavigateToPose=NS(Result=object))
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),env)
    return env['NavigationSupervisor']


def completed(value):
    future=Future();future.set_result(value);return future


@pytest.mark.parametrize('distance,yaw,arrived',[(.10,0.,False),(.026,0.,False),(.02,0.,True),(.02,.11,False)])
def test_short_approach_requires_actual_translation_before_arrival(distance,yaw,arrived):
    cls=load_supervisor();node=cls.__new__(cls)
    node.pose=lambda:(0.,0.,0.);node.p={'xy_tolerance':.12}
    q=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))
    request=NS(behavior_tree='navigate_approach.xml',
        pose=NS(pose=NS(position=NS(x=distance,y=0.),orientation=q)))
    assert node.reached_goal(request) is arrived


def action_fixture():
    cls=load_supervisor();supervisor=cls.__new__(cls)
    supervisor.guard=threading.RLock();supervisor.stop=threading.Event()
    supervisor.raw_future=supervisor.raw_handle=supervisor.raw_result=supervisor.raw_cancel=None
    supervisor.raw_canceled=False;supervisor.raw_abandoned=False;supervisor.raw_generation=0
    supervisor.raw_finish_deadline=None;supervisor.acceptance_event=threading.Event()
    supervisor.blocked=False;supervisor.busy=True;supervisor.outer=object()
    supervisor.p={'cancel_timeout':.01}
    supervisor.get_logger=lambda:NS(error=lambda _:None)
    return supervisor


def test_late_acceptance_after_unconfirmed_finish_is_irreversibly_canceled():
    supervisor=action_fixture();response=Future();terminal=Future();cancels=[]
    handle=NS(accepted=True,get_result_async=lambda:terminal,
              cancel_goal_async=lambda:(cancels.append(True) or completed(None)))
    supervisor.raw_future=response;response.add_done_callback(supervisor.raw_accepted)
    assert not supervisor.finish_raw() and supervisor.blocked
    response.set_result(handle)
    assert cancels==[True], 'An abandoned raw goal must be canceled even without owner Stop'


def test_missing_result_future_never_counts_as_confirmed_raw_terminal():
    supervisor=action_fixture();cancels=[]
    def unavailable():raise RuntimeError('Result service unavailable')
    handle=NS(accepted=True,get_result_async=unavailable,
              cancel_goal_async=lambda:(cancels.append(True) or completed(None)))
    supervisor.raw_future=completed(handle)
    supervisor.raw_accepted(supervisor.raw_future)
    assert not supervisor.finish_raw(), 'Accepted action without a result cannot authorize successor'
    assert supervisor.blocked and cancels==[True]


def test_failed_result_transport_never_counts_as_an_action_terminal():
    supervisor=action_fixture();terminal=Future();terminal.set_exception(RuntimeError('Transport lost'))
    handle=NS(accepted=True,get_result_async=lambda:terminal,cancel_goal_async=lambda:completed(None))
    supervisor.raw_future=completed(handle);supervisor.raw_accepted(supervisor.raw_future)
    assert not supervisor.finish_raw() and supervisor.blocked


def test_a_real_terminal_result_can_release_the_raw_slot():
    supervisor=action_fixture();terminal=completed(NS(status=5))
    handle=NS(accepted=True,get_result_async=lambda:terminal,cancel_goal_async=lambda:completed(None))
    supervisor.raw_future=completed(handle);supervisor.raw_accepted(supervisor.raw_future)
    assert supervisor.finish_raw()
    assert supervisor.raw_future is supervisor.raw_handle is supervisor.raw_result is None


def test_stop_prevents_dispatch_before_any_raw_action_is_sent():
    supervisor=action_fixture();supervisor.stop.set();sent=[]
    def send(request,feedback_callback=None):
        sent.append(request);return completed(NS(accepted=False))
    assert not supervisor.dispatch(NS(send_goal_async=send),object(),None)
    assert not sent


class Clock:
    def __init__(self,moving):
        self.t=10.;self.supervisor=None;self.moving=moving
        self.motion=None;self.fault=None
    def monotonic(self):return self.t
    def sleep(self,seconds):
        self.t+=.05
        n=self.supervisor
        yaw=.4*(self.t-10.) if self.moving else 0.
        n.odom=(0.,0.,yaw,0.,0.)
        if self.motion:n.odom=self.motion(self.t-10.)
        n.odom_stamp=self.t;n.odom_frame=('odom_combined','base_footprint')
        n.odom_time=n.command_time=self.t
        if self.fault=='missing':n.odom=None
        if self.fault=='stale_odom':n.odom_time=self.t-2.
        if self.fault=='stale_capture':n.odom_stamp=self.t-.3
        if self.fault=='future_capture':n.odom_stamp=self.t+.1
        if self.fault=='duplicate_capture':n.odom_stamp=10.
        if self.fault=='stale_command':n.command_time=self.t-.31
        if self.fault=='nonzero_command':n.final_command=(.001,0.)


def stationary_fixture(moving):
    clock=Clock(moving);cls=load_supervisor(clock);supervisor=cls.__new__(cls)
    clock.supervisor=supervisor
    supervisor.guard=threading.RLock();supervisor.stop=threading.Event()
    supervisor.p={'stationary_timeout':5.,'sensor_timeout':1.,'stationary_seconds':.06}
    supervisor.status={};supervisor.set_stage=lambda *args,**kw:None
    supervisor.interrupted=lambda budget:None
    supervisor.odom=(0.,0.,0.,0.,0.);supervisor.odom_time=supervisor.command_time=clock.t
    supervisor.odom_stamp=clock.t;supervisor.odom_frame=('odom_combined','base_footprint')
    supervisor.final_command=(0.,0.)
    supervisor.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int((clock.t+.01)*1e9)))
    return supervisor,clock


def test_zero_reported_twist_does_not_prove_stationary_when_actual_yaw_changes():
    supervisor,_=stationary_fixture(True)
    assert not supervisor.stationary(None), 'Changing odometry pose must prevent a false stop confirmation'


def test_stationary_window_has_at_least_two_seconds_even_with_short_configuration():
    supervisor,clock=stationary_fixture(False)
    assert supervisor.stationary(None)
    assert clock.t-10.>=2.


def test_bounded_balance_oscillation_passes_real_two_second_stop_window():
    supervisor,clock=stationary_fixture(False)
    # 2.6 cm total position span; mean |v| around .031 m/s. Instantaneous
    # balance correction exceeds .035 m/s, while the chassis does not creep.
    clock.motion=lambda t:(.013*math.sin(2*math.pi*.6*t),0.,
        .015*math.sin(2*math.pi*.4*t),.013*2*math.pi*.6*math.cos(2*math.pi*.6*t),
        .015*2*math.pi*.4*math.cos(2*math.pi*.4*t))
    assert supervisor.stationary()
    check=supervisor.status['stationary_check']
    assert check['duration']==pytest.approx(2.) and check['verified']
    assert .025<check['position_span']<.027
    assert .025<check['linear_mean']<.035
    assert check['final_command']==[0.,0.]


@pytest.mark.parametrize('motion,reason_code',[
    (lambda t:(.03*t,0.,0.,.03,0.),'stationary_position_span'),
    (lambda t:(0.,0.,.08*t,0.,.08),'stationary_heading_span'),
    (lambda t:(.03*t,0.,0.,0.,0.),'stationary_position_span'),
    (lambda t:(0.,0.,.08*t,0.,0.),'stationary_heading_span'),
])
def test_slow_creep_is_rejected_even_with_zero_commands_and_low_or_zero_twist(motion,reason_code):
    supervisor,clock=stationary_fixture(False);clock.motion=motion
    assert not supervisor.stationary()
    assert supervisor.status['stationary_check']['reason_code']==reason_code


@pytest.mark.parametrize('fault,reason_code',[
    ('missing','stationary_odom_missing'),('stale_odom','stationary_odom_stale'),
    ('stale_capture','stationary_odom_capture_stale'),('future_capture','stationary_odom_capture_stale'),
    ('duplicate_capture','stationary_odom_capture_stale'),('stale_command','stationary_command_stale'),
    ('nonzero_command','stationary_command_nonzero'),
])
def test_stop_requires_fresh_actual_pose_and_zero_final_command(fault,reason_code):
    supervisor,clock=stationary_fixture(False);clock.fault=fault
    assert not supervisor.stationary()
    check=supervisor.status['stationary_check']
    assert not check['verified'] and check['reason_code']==reason_code


def outer_fixture(events):
    return NS(request=object(),is_cancel_requested=False,
        succeed=lambda:events.append('succeeded'),abort=lambda:events.append('aborted'),
        canceled=lambda:events.append('canceled'))


def test_outer_terminal_is_delayed_until_raw_terminal_and_actual_stationary_hold():
    supervisor=action_fixture();supervisor.status={};supervisor.raw_future=object();events=[]
    supervisor.finish_raw=lambda:(events.append('raw_terminal') or True)
    supervisor.stationary=lambda **kw:(events.append('actual_stationary') or True)
    supervisor.reached_goal=lambda request:True
    supervisor.set_stage=lambda *args,**kw:None
    supervisor.terminal(outer_fixture(events),4,'Reached')
    assert events==['raw_terminal','actual_stationary','succeeded']


def test_unconfirmed_raw_terminal_locks_busy_before_notifying_client():
    supervisor=action_fixture();supervisor.status={};supervisor.raw_future=object();events=[]
    supervisor.finish_raw=lambda:False
    supervisor.stationary=lambda **kw:(_ for _ in ()).throw(AssertionError('Raw action still unconfirmed'))
    supervisor.set_stage=lambda *args,**kw:None
    supervisor.terminal(outer_fixture(events),4,'Reached')
    assert events==['aborted'] and supervisor.busy and supervisor.blocked


def test_failed_stationary_lock_reports_the_actual_check_reason():
    supervisor=action_fixture();supervisor.status={};supervisor.raw_future=object();events=[]
    supervisor.finish_raw=lambda:True
    def unsettled(**kw):
        supervisor.status['stationary_check']={'verified':False,'reason':'Position is still drifting'}
        return False
    supervisor.stationary=unsettled
    statuses=[];supervisor.set_stage=lambda *args,**kw:statuses.append(args)
    supervisor.terminal(outer_fixture(events),4,'Navigation goal reached')
    assert events==['aborted'] and supervisor.busy and supervisor.blocked
    assert 'Position is still drifting' in statuses[-1][1]
    assert 'navigation locked' in statuses[-1][1]


def test_settled_pose_must_still_satisfy_goal_tolerance_before_success():
    supervisor=action_fixture();supervisor.status={};supervisor.raw_future=object();events=[]
    supervisor.finish_raw=lambda:True
    supervisor.stationary=lambda **kw:True
    supervisor.reached_goal=lambda request:False
    statuses=[];supervisor.set_stage=lambda *args,**kw:statuses.append(args)
    supervisor.terminal(outer_fixture(events),4,'Reached')
    assert events==['aborted'] and 'settled' in statuses[-1][1]


def test_stale_acceptance_generation_cannot_replace_current_handle():
    supervisor=action_fixture();supervisor.raw_generation=2;present=object();supervisor.raw_handle=present
    canceled=[]
    handle=NS(accepted=True,cancel_goal_async=lambda:canceled.append(True))
    supervisor.raw_accepted(completed(handle),generation=1)
    assert supervisor.raw_handle is present and canceled==[True]


@pytest.mark.parametrize('reason', [
    'scan_stale','scan_unusable','scan_transform_unavailable','swept_map_missing',
    'swept_transform_stale','swept_pose_unavailable','reverse_localization_stale',
    'reverse_map_or_localization_missing','reverse_transform_stale','reverse_pose_unavailable',
    'Invalid localization covariance','Localization uncertainty too high for reverse',
])
def test_fresh_gate_heartbeat_cannot_authorize_recovery_with_failed_sensors(reason):
    supervisor=action_fixture();now=time.monotonic()
    supervisor.p.update(sensor_timeout=1.,gate_timeout=1.5,enable_grace=2.)
    supervisor.odom=(0.,0.,0.,0.,0.);supervisor.odom_time=supervisor.gate_time=now
    supervisor.gate={'reason':reason,'scan_age':.01};supervisor.pose=lambda:(0.,0.,0.)
    assert supervisor.navigation_health(now,now).startswith('Navigation sensors')


@pytest.mark.parametrize('reason', ['passing','forward_obstacle','rotation_obstacle',
    'swept_map_obstacle','Unknown space behind robot','Obstacle in reverse envelope'])
def test_ordinary_obstacles_are_not_misclassified_as_sensor_failure(reason):
    supervisor=action_fixture();now=time.monotonic()
    supervisor.p.update(sensor_timeout=1.,gate_timeout=1.5,enable_grace=2.)
    supervisor.odom=(0.,0.,0.,0.,0.);supervisor.odom_time=supervisor.gate_time=now
    supervisor.gate={'reason':reason,'scan_age':.01};supervisor.pose=lambda:(0.,0.,0.)
    assert supervisor.navigation_health(now,now) is None


@pytest.mark.parametrize('age',[None,float('nan'),2.])
def test_invalid_scan_age_is_rejected_even_if_last_gate_reason_was_passing(age):
    supervisor=action_fixture();now=time.monotonic()
    supervisor.p.update(sensor_timeout=1.,gate_timeout=1.5,enable_grace=2.)
    supervisor.odom=(0.,0.,0.,0.,0.);supervisor.odom_time=supervisor.gate_time=now
    supervisor.gate={'reason':'passing','scan_age':age};supervisor.pose=lambda:(0.,0.,0.)
    assert supervisor.navigation_health(now,now).startswith('Navigation sensors')


def load_planner():
    source=ROOT/'path_planner.py'
    cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='PathPlanner')
    result=lambda:NS(path=None,planning_time=NS(sec=0,nanosec=0))
    env=dict(Node=object,time=time,threading=threading,rclpy=NS(ok=lambda:True),
        GoalStatus=NS(STATUS_SUCCEEDED=4,STATUS_CANCELED=5,STATUS_ABORTED=6),
        ComputePathToPose=NS(Result=result))
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),env)
    return env['PathPlanner']


def planner_fixture():
    cls=load_planner();planner=cls.__new__(cls)
    planner.guard=threading.RLock();planner.p={'cancel_timeout':.01,'planning_timeout':5.}
    planner.blocked=False;planner.revision=1;planner.get_logger=lambda:NS(error=lambda _:None)
    context=NS(created=time.monotonic(),stop=threading.Event(),accepted=threading.Event(),
               future=None,raw=None,result=None,canceled=False,abandoned=False,finish_deadline=None)
    return planner,context


def test_late_planner_acceptance_after_lockout_is_canceled_without_owner_stop():
    planner,context=planner_fixture();canceled=[];context.future=Future()
    handle=NS(accepted=True,get_result_async=Future,
        cancel_goal_async=lambda:(canceled.append(True) or completed(None)))
    context.future.add_done_callback(lambda f:planner.raw_accepted(f,context))
    assert not planner.finish_raw(context) and planner.blocked
    context.future.set_result(handle)
    assert canceled==[True]


def test_planner_accepted_without_result_future_is_never_released_as_terminal():
    planner,context=planner_fixture();canceled=[]
    def unavailable():raise RuntimeError('Result service unavailable')
    handle=NS(accepted=True,get_result_async=unavailable,
        cancel_goal_async=lambda:(canceled.append(True) or completed(None)))
    context.future=completed(handle);planner.raw_accepted(context.future,context)
    assert not planner.finish_raw(context) and planner.blocked and canceled==[True]


def test_planner_result_transport_failure_does_not_confirm_action_terminal():
    planner,context=planner_fixture();terminal=Future();terminal.set_exception(RuntimeError('Connection lost'))
    canceled=[]
    handle=NS(accepted=True,get_result_async=lambda:terminal,
        cancel_goal_async=lambda:(canceled.append(True) or completed(None)))
    context.future=completed(handle);planner.raw_accepted(context.future,context)
    assert not planner.finish_raw(context) and planner.blocked and canceled==[True]


def planner_terminal_fixture():
    planner,context=planner_fixture();events=[];published=[]
    key=b'\x01'*16
    handle=NS(goal_id=NS(uuid=list(key)),is_cancel_requested=False,is_active=True,
        succeed=lambda:events.append('succeeded'),abort=lambda:events.append('aborted'),
        canceled=lambda:events.append('canceled'))
    planner.contexts={key:context};planner.publisher=NS(publish=published.append)
    planner.status=lambda *args,**kw:None
    planner.get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000))
    return planner,context,handle,events,published


def test_cancel_token_before_ros_handle_state_never_publishes_validated_route():
    planner,context,handle,events,published=planner_terminal_fixture()
    context.stop.set() # cancel callback consent, before rclpy updates handle state
    planner.terminal(handle,'Validated',path=object(),validated_revision=1,cost_stamp=100.)
    assert not published and events==['aborted']


def test_planning_deadline_is_checked_after_validation_before_publishing():
    planner,context,handle,events,published=planner_terminal_fixture()
    context.created=time.monotonic()-6.
    planner.terminal(handle,'Validated',path=object(),validated_revision=1,cost_stamp=100.)
    assert not published and events==['aborted']


def test_costmap_must_still_be_fresh_when_validated_route_is_published():
    planner,context,handle,events,published=planner_terminal_fixture()
    planner.terminal(handle,'Validated',path=object(),validated_revision=1,cost_stamp=96.)
    assert not published and events==['aborted']


def test_final_map_revision_check_is_atomic_with_route_publication():
    planner,context,handle,events,published=planner_terminal_fixture()
    planner.revision=2
    planner.terminal(handle,'Validated',path=object(),validated_revision=1,cost_stamp=100.)
    assert not published and events==['aborted']


def test_a_fresh_validated_route_can_still_be_published():
    planner,context,handle,events,published=planner_terminal_fixture();path=object()
    result=planner.terminal(handle,'Validated',path=path,validated_revision=1,cost_stamp=100.)
    assert published==[path] and events==['succeeded'] and result.path is path


def test_candidate_route_is_validated_and_returned_without_repainting_navigation():
    planner,context,handle,events,published=planner_terminal_fixture();path=object()
    context.publish_plan=False
    result=planner.terminal(handle,'Validated',path=path,validated_revision=1,cost_stamp=100.)
    assert not published and events==['succeeded'] and result.path is path


def test_candidate_route_still_rejects_changed_map():
    planner,context,handle,events,published=planner_terminal_fixture()
    context.publish_plan=False;planner.revision=2
    planner.terminal(handle,'Validated',path=object(),validated_revision=1,cost_stamp=100.)
    assert not published and events==['aborted']
