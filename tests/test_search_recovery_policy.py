"""Production recovery state machine and cross-node evidence boundaries."""
import ast
import json
import math
import threading
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_mission_policy import Mission, mission, namespace
from ee3033_project.core import angle_difference
from ee3033_project.coverage import Viewpoint
from ee3033_project.target_acquisition import SingleTarget

namespace.update(threading=threading,angle_difference=angle_difference)


def owner():
    m=mission();m.initialize_search_recovery()
    m.p=dict(target_class='bottle',reacquisition_timeout=10.,localization_stall_timeout=10.,
        max_search_restarts=2,max_goals=120,filter_reset_timeout=3.,cancel_timeout=10.)
    m.state='LOCALIZING';m.target_detected=True;m.target_ever_detected=True
    m.started=20.;m.started_stamp=9.;m.home=(1.,2.,.3);m.index=5
    m.position=[0.,0.,0.];m.pose=lambda:tuple(m.position)
    m.get_clock=lambda:NS(now=lambda:NS(nanoseconds=110_000_000_000))
    m.return_budget_exhausted=lambda:False
    m.navigation_map=NS(allowed=lambda x,y:True)
    m.goal_pending=False;m.goal_handle=None;m.goal_serial=7
    m.pending_navigation=None;m.cancel_next=None
    m.current_option=4;m.search_attempt_active=True;m.failures=[]
    m.localization_reason=None;m.search_stop_reason=None
    m.planner=NS(options=[Viewpoint(float(i),0.,0.,frozenset({(i,i)})) for i in range(5)])
    m.seen={(0,0),(1,1),(2,2),(3,3)}
    m.attempted=set(range(5));m.failed_options={1,2};m.navigation_failed_options={1};m.visual_failed_options={2}
    m.deferred_options={2};m.visual_retry_counts={1:2,2:2}
    m.view_frames={10.};m.observation_stamps={10.};m.best_target={'old':True}
    m.current_target_estimate={'old':True};m.target_details={'old':True};m.local_sigmas=[(1,.2)]
    m.acquisition.started=50.;m.acquisition.locked=True
    m.set_state=lambda state,reason:(setattr(m,'state',state),setattr(m,'reason',reason))
    m.publish_status=lambda:None;m.persist=lambda:None;m.heartbeat=lambda *a:None
    m.last_usable_bearing=m.last_localization_progress=100.
    m.observation_history=[dict(pose=[i,0.,0.],cells=[[i,i]],headings=[0.],options=[i],stamp=float(i),epoch=0,invalidated=False) for i in range(3)]
    return m


def test_lost_timeout_requests_cancellation_at_ten_seconds():
    m=owner()
    assert not m.localization_watchdog(109.99)
    assert m.localization_watchdog(110.)
    assert m.cancellations==['RESEARCH']
    assert m.failed_localization_positions==[[0.,0.]]
    assert m.plan_cancel.is_set()


def test_stall_moves_once_then_researches_without_resetting_total_budget():
    m=owner();m.last_usable_bearing=111.
    m.acquisition.alternate_observation_goal=lambda *a,**kw:(.3,0.,0.)
    m.make_pose=lambda *a:a
    sent=[];m.send_goal=lambda pose,kind:sent.append((pose,kind))
    assert m.localization_watchdog(110.)
    assert sent==[((.3,0.,0.),'acquire')]
    assert m.stall_alternate_attempted and not m.cancellations
    m.last_usable_bearing=121.
    assert m.localization_watchdog(120.)
    assert m.cancellations==['RESEARCH']
    assert m.started==20. and m.home==(1.,2.,.3) and m.acquisition.started==50.


def test_unsafe_stall_alternate_researches_immediately():
    m=owner();m.last_usable_bearing=111.
    m.acquisition.alternate_observation_goal=lambda *a,**kw:None
    assert m.localization_watchdog(110.)
    assert m.cancellations==['RESEARCH']


def test_batch_reset_and_duplicate_highwater_do_not_restart_progress_clock(monkeypatch):
    m=owner();m.bottom_ranging=True
    monkeypatch.setattr(namespace['time'],'monotonic',lambda:101.)
    for count in (1,0,1,0,1):m.observe_localization_progress({'mmse':{'batch_frames':count}})
    assert m.last_localization_progress==101. and m.batch_highwater==1
    monkeypatch.setattr(namespace['time'],'monotonic',lambda:108.)
    m.observe_localization_progress({'mmse':{'batch_frames':1}})
    assert m.last_localization_progress==101.
    m.observe_localization_progress({'mmse':{'batch_frames':2}})
    assert m.last_localization_progress==108.


def test_research_reopens_last_two_stations_and_interrupted_option_only():
    m=owner();m.reset_for_research('test loss')
    assert m.state=='RESETTING' and m.search_epoch==1 and m.search_restarts==1
    assert m.seen=={(0,0),(3,3)}
    assert m.attempted=={0,1,3} and m.failed_options=={1}
    assert m.navigation_failed_options=={1} and not m.visual_failed_options
    assert not m.visual_retry_counts
    assert [h['invalidated'] for h in m.observation_history]==[False,True,True]
    assert m.index==6 and m.started==20. and m.started_stamp==9. and m.home==(1.,2.,.3)
    assert m.best_target is None and m.current_target_estimate is None
    assert not m.target_detected and m.target_ever_detected
    assert m.current_option is None and not m.view_frames and not m.observation_stamps


def test_research_after_planning_lock_does_not_double_count_old_viewpoint():
    m=owner();m.search_attempt_active=False
    m.reset_for_research('lock during planning')
    assert m.index==5
    assert 4 in m.attempted


def test_pending_navigation_must_end_before_reset_then_late_result_is_stale():
    m=owner();m.goal_pending=True;m.goal_pending_serial=7
    m.cancel=Mission.cancel.__get__(m)
    m.request_research('lost')
    assert m.state=='CANCELING' and m.cancel_next=='RESEARCH' and m.search_epoch==0
    m.goal_handle_serial=None
    m.goal_result(NS(result=lambda:NS(status=0)),7)
    assert m.state=='RESETTING' and m.search_epoch==1 and m.goal_serial==8
    m.goal_result(NS(result=lambda:NS(status=4)),7)
    assert m.state=='RESETTING' and m.search_epoch==1


def test_reset_refuses_pending_navigation():
    m=owner();m.goal_pending=True;fail=[];m.finish=lambda *a:fail.append(a)
    m.reset_for_research('lost')
    assert m.search_epoch==0 and fail[0][0]=='FAILED'


@pytest.mark.parametrize('limit', ['restart','return','attempts'])
def test_restart_respects_all_mission_limits(limit):
    m=owner();m.report_best_target=lambda:False
    if limit=='restart':m.search_restarts=2
    if limit=='return':m.return_budget_exhausted=lambda:True
    if limit=='attempts':m.index=119;m.search_attempt_active=True
    m.request_research('lost')
    assert m.cancellations==['RETURN']
    assert m.search_epoch==0


def test_new_epoch_rejects_old_posterior_and_details_before_caching():
    m=owner();m.reset_for_research('lost')
    m.state='LOCALIZING';m.current_target_estimate=None
    old=dict(search_epoch=0,mission_stamp=9.,stamp=110.,label='bottle',x=1.,y=1.,confidence=.9,frames=8)
    m.on_estimate(NS(data=json.dumps(old)));m.on_details(NS(data=json.dumps(old)))
    assert m.current_target_estimate is None and m.target_details is None
    current=dict(old,search_epoch=1,stamp=109.)
    m.on_estimate(NS(data=json.dumps(current)))
    assert m.current_target_estimate is None


def test_failed_zone_cannot_immediately_relock_and_old_exposures_cannot_follow_exit(monkeypatch):
    m=owner();m.reset_for_research('lost');m.state='SEARCHING'
    m.get_clock=lambda:NS(now=lambda:NS(nanoseconds=111_000_000_000))
    m.last_usable_bearing=None
    for stamp in (110.1,110.2):
        m.on_detections(NS(data=json.dumps(dict(detections=[dict(stamp=stamp,label='bottle',confidence=.9,bearing=0.,origin=[0,0],box=[300,100,340,300])]))))
    assert not m.target_detected and m.acquisition.candidate is None
    m.position[:]=[.3,0.,0.]
    m.on_detections(NS(data=json.dumps(dict(detections=[dict(stamp=110.8,label='bottle',confidence=.9,bearing=0.,box=[300,100,340,300])]))))
    assert m.detection_started_stamp==111. and not m.target_detected
    m.get_clock=lambda:NS(now=lambda:NS(nanoseconds=111_500_000_000))
    for stamp in (111.1,111.3):
        m.on_detections(NS(data=json.dumps(dict(detections=[dict(stamp=stamp,label='bottle',confidence=.9,bearing=0.,box=[300,100,340,300])]))))
    assert m.target_detected and m.cancellations[-1]=='ACQUIRE'


def test_observation_history_groups_station_headings_and_audits_new_coverage():
    m=owner();m.observation_history=[];m.observation_events=[];m.seen=set();m.current_option=0
    m.record_observation((0.,0.,0.),{(0,0)});m.seen.add((0,0))
    m.record_observation((.05,0.,1.),{(0,0),(0,1)});m.seen.add((0,1))
    assert len(m.observation_history)==1 and len(m.observation_history[0]['headings'])==2
    assert m.observation_events[-1]['actual_new']==1
    m.current_option=1;m.record_observation((1.,0.,0.),{(1,1)})
    m.current_option=0;m.record_observation((0.,0.,2.),{(0,0)})
    assert len(m.observation_history)==3


def load_filter():
    source=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/target_filter.py'
    cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef))
    cls.bases=[];cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name!='__init__']
    scope=dict(json=json,math=math,np=np,String=NS)
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),scope)
    f=scope['TargetFilter']()
    class Resettable:
        def __init__(self):self.resets=0;self.candidate=None;self.frozen=None
        def reset(self):self.resets+=1;self.candidate=None;self.frozen=None
        def freeze(self,x):self.frozen=x
    for attr in ('tracker','bottom_batch','intersection','association','diagnostics'):setattr(f,attr,Resettable())
    f.mission_stamp=9.;f.search_epoch=0;f.epoch_started_stamp=9.;f.armed_at=9.
    f.target_locked=True;f.last_frame_stamp=10.;f.geometry_conflict=True;f.publish=lambda:None
    return f


def test_filter_epoch_reset_clears_all_correlated_state_and_cannot_roll_back():
    f=load_filter();f.on_status(NS(data=json.dumps(dict(started_stamp=9.,search_epoch=1,epoch_started_stamp=110.,target_detected=False))))
    assert f.search_epoch==1 and f.armed_at==110. and not f.target_locked
    for attr in ('tracker','bottom_batch','intersection','association','diagnostics'):assert getattr(f,attr).resets==1
    f.on_status(NS(data=json.dumps(dict(started_stamp=9.,search_epoch=0,target_detected=True))))
    assert f.search_epoch==1 and not f.target_locked


def test_filter_lock_uses_mission_association_and_post_departure_cutoff():
    f=load_filter();f.search_epoch=1;f.target_locked=False;f.armed_at=110.
    seed=dict(stamp=112.,label='bottle',confidence=.9,bearing=0.,box=[300,100,340,300])
    f.on_status(NS(data=json.dumps(dict(started_stamp=9.,search_epoch=1,epoch_started_stamp=110.,detection_started_stamp=111.,target_detected=True,target_lock=seed))))
    assert f.armed_at==111. and f.association.candidate==seed
    assert f.association.locked and f.association.frames==2


def test_filter_old_result_cannot_freeze_new_epoch():
    f=load_filter();f.search_epoch=1
    target=dict(estimator='map-constrained MMSE',**{'class':'bottle'},observation_stamp=10.)
    f.on_result(NS(data=json.dumps(dict(started_stamp=9.,search_epoch=0,target=target))))
    assert f.tracker.frozen is None


def scan_owner():
    m=owner();m.target_detected=False;m.target=None;m.plan_cancel=threading.Event()
    m.station_origin=None;m.station_views=0;m.station_attempts=0;m.route=[]
    m.defer_for_costmap=lambda *a:False;m.next_viewpoint=lambda:None
    m.make_pose=lambda *a:a
    m.sent=[]
    def send(pose,kind):
        m.sent.append((pose,kind));m.goal_kind=kind;m.state='SEARCHING'
    m.send_goal=send
    m.get_parameter=lambda name:NS(value=.4)
    m.planner.station_key=lambda x,y:(round(x,3),round(y,3))
    return m


def test_failed_scan_still_consumes_heading_slot_and_uses_actual_xy(monkeypatch):
    clock=[100.];monkeypatch.setattr(namespace['time'],'monotonic',lambda:clock[0])
    m=scan_owner()
    m.planner.options=[Viewpoint(.05,0.,i*.1,frozenset({(i,i)})) for i in range(7)]
    for i in range(6):
        clock[0]=100.+i
        m.dispatch_viewpoint(i)
        assert m.sent[-1][0][:2]==(0.,0.) and m.sent[-1][1]=='search_scan'
        m.goal_finished(False)
    assert m.station_attempts==6 and m.station_views==0
    assert m.station_started==100.
    assert not m.search_option_allowed(6,m.pose())


def test_station_dwell_starts_on_first_arrival_and_never_resets_for_scans(monkeypatch):
    clock=[10.];monkeypatch.setattr(namespace['time'],'monotonic',lambda:clock[0])
    m=scan_owner();m.planner.options=[Viewpoint(1.,0.,0.,frozenset({(0,0)})),Viewpoint(1.,0.,.2,frozenset({(1,1)}))]
    m.dispatch_viewpoint(0)
    assert m.station_arrival_pending and m.sent[-1][1]=='search'
    clock[0]=40.;m.position[:]=[1.,0.,0.];m.goal_finished(True)
    assert m.station_started==40. and not m.station_arrival_pending
    clock[0]=42.;m.dispatch_viewpoint(1);m.goal_finished(True)
    assert m.station_started==40. and m.station_attempts==2


def test_all_departed_stations_remain_in_return_cost_history():
    m=scan_owner()
    m.planner.options=[Viewpoint(float(i),0.,0.,frozenset({(i,i)})) for i in range(5)]
    for i in range(5):
        m.position[:]=[float(i),0.,0.]
        m.dispatch_viewpoint(i)
    assert len(m.previous_stations)==4


def test_ever_detected_unlocalized_stays_found_after_current_lock_released(monkeypatch):
    from ee3033_project.coverage import returned_home
    namespace['returned_home']=returned_home
    monkeypatch.setattr(namespace['time'],'monotonic',lambda:100.)
    m=owner();m.state='VERIFYING';m.target_detected=False;m.target_ever_detected=True
    m.home=(0.,0.,0.);m.p.update(goal_timeout=75.,mission_timeout=900.,return_tolerance=.2,
        return_heading_required=False,verify_timeout=12.)
    m.get_parameter=lambda name:NS(value=m.p[name])
    m.navigation_map.poll=lambda now:None;m.update_localization_clock=lambda:None
    m.readiness_error=lambda **kw:None;m.zero_command_fresh=lambda:True
    m.stop_window=NS(evaluate=lambda stamp:dict(verified=True),reset=lambda:None)
    m.goal_started=99.;m.last_status=100.;m.goal_handle=None
    outcomes=[];m.finish=lambda state,reason:outcomes.append(state)
    m.tick()
    assert outcomes==['FOUND_UNLOCALIZED']
