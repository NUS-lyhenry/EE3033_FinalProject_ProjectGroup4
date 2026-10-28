"""Execute production readiness/dispatch methods without a ROS installation."""
import ast
import math
import time
from pathlib import Path
from types import MethodType, SimpleNamespace as NS

ROOT=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project'


def methods(filename, classname):
    tree=ast.parse((ROOT/filename).read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==classname)
    scope=dict(math=math,time=time,NavigateToPose=NS(Goal=NS))
    exec(compile(ast.Module(body=[n for n in cls.body if isinstance(n,ast.FunctionDef)],
                           type_ignores=[]),str(ROOT/filename),'exec'),scope)
    return scope


COST=methods('navigation_costmap.py','NavigationCostmap')
MISSION=methods('mission.py','Mission')


def node(fresh=False):
    now=time.monotonic();sent=[];fail=[];finished=[]
    cost=NS(map=object() if fresh else None,stamp=now,client=NS(service_is_ready=lambda:True))
    cost.fresh=MethodType(COST['fresh'],cost)
    cost.allowed=lambda x,y:False
    n=NS(navigation_map=cost,pending_navigation=None,goal_pending=False,goal_handle=None,
        nav=NS(server_is_ready=lambda:True,send_goal_async=lambda goal:
               (sent.append(goal) or NS(add_done_callback=lambda cb:None))),
        heartbeat=lambda *args:None,publish_status=lambda:None,target_detected=False,
        goal_finished=lambda value:fail.append(value),finish=lambda *args:finished.append(args),
        map=object(),state='WAITING',ACTIVE={'PLANNING','SEARCHING','LOCALIZING'},
        scan_time=now,odom_time=now,perception_time=now,geometry_error=None,
        amcl=NS(pose=NS(covariance=[0.]*36)),pose=lambda:(0.,0.,0.),
        p=dict(sensor_timeout=3.,perception_timeout=3.,max_xy_variance=.2,
               max_yaw_variance=.2,return_heading_required=True),
        goal_serial=0,nav_progress=NS(reset=lambda:None),index=0,route=[1],
        set_state=lambda state,reason:None,failures=[],failed_options=set(),seen=set())
    for name in ('defer_for_costmap','retry_costmap_wait','send_goal','readiness_error'):
        setattr(n,name,MethodType(MISSION[name],n))
    n.cancel=lambda state,reason:finished.append((state,reason))
    return n,sent,fail,finished


def pose():return NS(pose=NS(position=NS(x=1.,y=1.)))


def test_advertised_service_without_snapshot_cannot_start_or_fail_a_viewpoint():
    n,sent,fail,_=node()
    assert 'costmap' in n.readiness_error()
    n.send_goal(pose(),'search')
    assert n.pending_navigation is not None
    assert not sent and not fail and not n.failures and not n.failed_options and not n.seen


def test_stale_snapshot_waits_then_sends_the_same_goal_after_refresh():
    n,sent,fail,_=node(True);n.navigation_map.stamp-=4
    goal=pose();n.send_goal(goal,'search')
    assert n.pending_navigation[1]=='send_goal' and not sent and not fail
    n.navigation_map.stamp=time.monotonic();n.navigation_map.allowed=lambda x,y:True
    n.retry_costmap_wait(time.monotonic())
    assert n.pending_navigation is None and sent[0].pose is goal and not fail


def test_wait_timeout_reports_navigation_data_failure_without_blocked_view():
    n,sent,fail,finished=node();n.send_goal(pose(),'search')
    n.retry_costmap_wait(n.pending_navigation[0]+3.1)
    assert finished and finished[0][0]=='FAILED'
    assert not sent and not fail and not n.failed_options


def test_fresh_blocked_point_still_fails_and_home_retains_existing_policy():
    n,sent,fail,_=node(True);n.send_goal(pose(),'search')
    assert fail==[False] and not sent
    n,sent,fail,_=node();n.send_goal(pose(),'home')
    assert len(sent)==1 and not fail and n.pending_navigation is None


def test_deferred_selection_does_not_submit_worker_or_mark_absence():
    n,_,_,_=node();n.plan_cancel=NS(is_set=lambda:False)
    n.dispatch_viewpoint=MethodType(MISSION['dispatch_viewpoint'],n)
    n.dispatch_viewpoint(None)
    assert n.pending_navigation[1:] == ('dispatch_viewpoint',(None,))
    assert not n.failed_options and not n.seen


def test_cancel_drops_deferred_work_before_return_or_stop():
    n,_,_,_=node();n.send_goal(pose(),'search')
    n.begin_return=lambda:None
    MISSION['cancel'](n,'RETURN','Stop searching')
    assert n.pending_navigation is None


def test_freshness_rejects_missing_stale_and_future_stamps():
    n,_,_,_=node(True)
    assert n.navigation_map.fresh(n.navigation_map.stamp+3.)
    assert not n.navigation_map.fresh(n.navigation_map.stamp+3.01)
    assert not n.navigation_map.fresh(n.navigation_map.stamp-.1)
    n.navigation_map.map=None
    assert not n.navigation_map.fresh()


def test_costmap_wait_is_not_localization_work_or_waypoint_travel():
    n,_,_,_=node();n.state='LOCALIZING'
    n.send_goal(pose(),'search')
    samples=[]
    n.acquisition=NS(started=1.,advance_clock=lambda *args:samples.append(args))
    n.nav_progress=NS(phase='translation');n.goal_translation=True
    MISSION['update_localization_clock'](n)
    assert samples[0][1:]==(False,False)
