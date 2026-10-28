"""Exercise the actual Mission callbacks without importing a ROS installation.

AST loading removes ROS imports only; production method bodies are unchanged.
Native transport tests remain in integration_visual_search.py.
"""
import ast
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
from ee3033_project.target_acquisition import SingleTarget
from ee3033_project.frame_journal import FrameJournal
from ee3033_project.target_estimate import map_signature
from ee3033_project.coverage import CoveragePlanner

source=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/mission.py'
cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='Mission')
namespace=dict(Node=object,math=math,time=time,np=np,json=json,FrameJournal=FrameJournal)
namespace.update(map_signature=map_signature,String=NS)
exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),namespace)
Mission=namespace['Mission']


def mission():
    m=Mission.__new__(Mission)
    m.state='SEARCHING';m.planner=object();m.started_stamp=9.;m.acquisition_retry_after=0.
    m.acquisition=SingleTarget();m.target_detected=False;m.target=None
    m.get_clock=lambda:NS(now=lambda:NS(nanoseconds=11_000_000_000))
    import threading
    m.plan_cancel=threading.Event();m.cancellations=[]
    m.cancel=lambda state,reason:m.cancellations.append(state)
    return m


def test_detection_without_any_geometry_locks_and_cancels_global_navigation():
    m=mission()
    for stamp in (10.,10.2):
        d=dict(stamp=stamp,label='cup',confidence=.8,box=[100,100,160,220])
        m.p={'target_class':'cup'}
        m.on_detections(NS(data=json.dumps(dict(detections=[d]))))
    assert m.target_detected and m.plan_cancel.is_set()
    assert m.cancellations==['ACQUIRE']
    assert m.detection_stamp==10.2


@pytest.mark.parametrize('reason',['Target lost','Localization timeout','Approach rejected','Expired MMSE'])
def test_locked_target_never_resumes_map_search(reason):
    m=mission();m.target_detected=True;m.state='LOCALIZING';m.report_best_target=lambda:False
    m.resume_search(reason)
    assert m.cancellations==['RETURN']
    assert m.localization_reason==reason


def test_late_selection_cannot_dispatch_after_lock_or_stop():
    m=mission();m.target_detected=True
    m.send_goal=lambda *a:pytest.fail('Late search goal was dispatched')
    m.dispatch_viewpoint(0)
    m.target_detected=False;m.plan_cancel.set()
    m.dispatch_viewpoint(0)


def test_late_navigation_result_cannot_clear_or_finish_newer_goal():
    m=mission();m.goal_serial=9;m.goal_handle=object();handle=m.goal_handle
    m.goal_handle_serial=9;m.goal_pending=True;m.goal_pending_serial=9
    m.state='CANCELING';m.after_cancel=lambda:pytest.fail('Old result completed a newer cancellation')
    m.goal_result(None,8)
    assert m.goal_handle is handle and m.goal_pending


def test_late_acceptance_is_cancelled_without_overwriting_newer_handle():
    m=mission();m.goal_serial=9;m.goal_handle=object();handle=m.goal_handle
    m.goal_handle_serial=9;m.goal_pending=True;m.goal_pending_serial=9
    cancelled=[];old=NS(accepted=True,cancel_goal_async=lambda:cancelled.append(True))
    m.goal_response(NS(result=lambda:old),8)
    assert cancelled==[True] and m.goal_handle is handle and m.goal_pending


def test_long_waypoint_journey_does_not_consume_localization_work_budget():
    a=SingleTarget();a.started=10.
    a.advance_clock(10.,working=True)
    a.advance_clock(14.,traveling=True)
    a.advance_clock(89.,working=True)
    assert a.active_seconds==4. and a.travel_seconds==75.
    assert not a.expired(89.)
    a.advance_clock(130.)
    assert a.active_seconds==45. and a.expired(130.)


def test_local_reacquisition_keeps_locked_candidate_after_old_three_second_cutoff():
    a=SingleTarget();a.started=1.;a.locked=True
    a.candidate=dict(stamp=10.,label='cup',confidence=.8,bearing=.5)
    a.active_seconds=4.
    first=a.recovery_goal((1.,1.,0.),14.)
    assert first==(1.,1.,.5)
    assert a.locked and not a.expired(14.)
    a.active_seconds=6.
    second=a.recovery_goal((1.,1.,.5),16.)
    assert second[:2]==first[:2] and second[2]>first[2]
    a.active_seconds=14.
    assert a.recovery_goal((1.,1.,.5),24.) is None
    assert a.locked  # bounded local scan ending never reopens global search


def test_exhausted_motion_and_sites_observe_until_absolute_deadline(monkeypatch):
    m=mission();m.state='LOCALIZING';m.target_detected=True
    m.p=dict(goal_timeout=75.,mission_timeout=900.,acquisition_seconds=45.,localize_wait_seconds=2.)
    m.get_parameter=lambda name:NS(value=m.p[name])
    m.navigation_map=NS(poll=lambda now:None,allowed=lambda *a:True)
    m.update_localization_clock=lambda:None;m.heartbeat=lambda *a:None
    m.publish_status=lambda:None;m.goal_handle=None;m.started=0.;m.last_status=0.
    m.return_budget_exhausted=lambda:False;m.readiness_error=lambda **k:None
    m.acquisition_wait=0.;m.acquisition_decision_after=0.
    m.current_target_estimate={};m.pose=lambda:(0.,0.,0.)
    m.planner=NS(camera_model=None)
    m.acquisition.started=1.;m.acquisition.timeout=45.;m.acquisition.steps=5
    m.acquisition.positions=[(0.,0.),(.4,0.),(.8,0.)]
    m.acquisition.candidate=dict(stamp=11.,bearing=0.,confidence=.8,label='cup',center_error=.5)
    m.acquisition.active_seconds=26.9
    monkeypatch.setattr(time,'monotonic',lambda:30.)
    m.tick()
    assert m.state=='LOCALIZING' and not m.cancellations
    assert 'continuing localization' in m.acquisition.phase
    # The absolute deadline includes travel; old step/site counts do not end it.
    m.report_best_target=lambda:False
    monkeypatch.setattr(time,'monotonic',lambda:46.)
    m.tick()
    assert m.cancellations==['RETURN']


def test_frame_rejection_evidence_is_persisted_and_old_missions_are_ignored(tmp_path):
    m=mission();m.state='LOCALIZING';m.run_dir=tmp_path;m.frame_journal=None
    m.latest_frame_diagnostic=None
    m.on_localization_diagnostics(NS(data=json.dumps(dict(
        mission_stamp=8.,stamp=10.,reason_codes=['center_error']))))
    assert not list(tmp_path.iterdir())
    event=dict(mission_stamp=9.,stamp=10.,reason_codes=['center_error'],
        selected=dict(center_error=.1,settled=True))
    m.on_localization_diagnostics(NS(data=json.dumps(event)))
    saved=json.loads((tmp_path/'localization_frames.jsonl').read_text())
    assert saved['reason_codes']==['center_error']
    assert saved['selected']['center_error']==.1
    assert m.frame_journal.summary()['written']==1


def test_frame_journal_reports_disk_cap_instead_of_silent_evidence_loss(tmp_path):
    journal=FrameJournal(tmp_path/'frames.jsonl',max_bytes=120)
    event=dict(stamp=1.,reason_codes=['unsettled'])
    assert journal.append(event)
    journal.append(dict(event,padding='x'*200))
    summary=journal.summary()
    assert summary['truncated'] and summary['written']==1 and summary['frames']==2
    assert summary['reason_counts']['unsettled']==2


def test_arrival_centering_counts_as_work_instead_of_waypoint_travel(monkeypatch):
    m=mission();m.state='ACQUIRING';m.acquisition.started=1.;m.goal_translation=True
    m.navigation_status={'stage':'TRACKING'}
    now=[1.];monkeypatch.setattr(time,'monotonic',lambda:now[0])
    m.update_localization_clock()
    now[0]=11.;m.navigation_status['stage']='FINAL_ALIGN';m.update_localization_clock()
    now[0]=16.;m.update_localization_clock()
    assert m.acquisition.travel_seconds==10.
    assert m.acquisition.active_seconds==5.


def test_default_approach_advances_ten_cm_without_dedicated_lateral_baseline():
    acquisition=SingleTarget()
    assert acquisition.enable_ab_maneuver is False
    acquisition.candidate=dict(stamp=10.,bearing=0.,origin=[0.,0.],label='cup',
        confidence=.8,projection_valid=False,bearing_valid=True)
    planner=CoveragePlanner(np.zeros((100,100)),.05,(-2.,-2.,0.))
    goal=acquisition.observation_goal(planner,(0.,0.,0.),force_baseline=True,allowed=lambda *a:True)
    assert goal==pytest.approx((.10,0.,0.))
    assert acquisition.last_goal_action=='approach'


def test_approach_shrinks_to_five_cm_or_holds_when_navigation_mask_refuses():
    acquisition=SingleTarget()
    acquisition.candidate=dict(stamp=10.,bearing=0.,origin=[0.,0.],label='cup',confidence=.8)
    planner=CoveragePlanner(np.zeros((100,100)),.05,(-2.,-2.,0.))
    goal=acquisition.observation_goal(planner,(0.,0.,0.),allowed=lambda x,y:x<=.055)
    assert goal==pytest.approx((.05,0.,0.))
    acquisition.visited.clear()
    assert acquisition.observation_goal(planner,(0.,0.,0.),allowed=lambda *a:None) is None


def test_short_approach_cannot_cross_unknown_with_any_part_of_footprint():
    grid=np.zeros((100,100));grid[:,43:]=-1
    planner=CoveragePlanner(grid,.05,(-2.,-2.,0.))
    acquisition=SingleTarget()
    acquisition.candidate=dict(stamp=10.,bearing=0.,origin=[0.,0.],label='cup',confidence=.8)
    assert acquisition.observation_goal(planner,(0.,0.,0.),allowed=lambda *a:True) is None


def test_one_absolute_localization_budget_includes_travel_and_recovery():
    acquisition=SingleTarget();acquisition.started=10.
    acquisition.active_seconds=0.;acquisition.travel_seconds=70.
    assert not acquisition.expired(99.999)
    assert acquisition.expired(100.)
    acquisition.turn_sites.clear();acquisition.positions.clear()
    assert acquisition.expired(101.)  # resetting motion history cannot extend it


def test_sensor_abort_is_stopped_with_a_reason_not_success(monkeypatch):
    m=mission();m.goal_serial=0;m.goal_handle=None
    m.heartbeat=lambda *a:None;m.persist=lambda:None
    m.result_pub=NS(publish=lambda msg:None)
    m.summary=lambda:dict(state=m.state)
    m.set_state=lambda state,reason:(setattr(m,'state',state),setattr(m,'reason',reason))
    m.finish('FAILED','Lidar stream is stale')
    assert m.state=='STOPPED' and m.target is None
    assert m.internal_error==m.termination_reason=='Lidar stream is stale'


def test_best_same_map_posterior_survives_worse_frames_and_temporary_loss():
    m=mission();m.state='LOCALIZING';m.target_detected=True
    m.planner=NS(grid=np.zeros((20,20)),resolution=.1,origin=(0.,0.,0.))
    m.p=dict(map_frame='map',target_class='cup',max_target_age=2.,max_target_variance=.0001)
    m.best_target=None;m.current_target_estimate=None;m.local_sigmas=[]
    m.readiness_error=lambda **k:None
    signature=map_signature(m.planner.grid,m.planner.resolution,m.planner.origin)
    def estimate(stamp,variance):
        return dict(method='map-constrained MMSE',stable=True,prior_conflict=False,frame_id='map',
            label='cup',mission_stamp=9.,stamp=stamp,x=.8,y=.5,sigma=math.sqrt(variance),
            variance=variance,covariance=[[variance,0.],[0.,variance]],map_signature=signature,
            position_confidence=.4,frames=4,estimate_id=str(stamp))
    m.on_estimate(NS(data=json.dumps(estimate(10.,.02))))
    m.on_estimate(NS(data=json.dumps(estimate(10.5,.08))))
    assert m.best_target['stamp']==10.
    reported=[];m.report_target=lambda data,quality:reported.append((data,quality))
    m.get_clock=lambda:NS(now=lambda:NS(nanoseconds=30_000_000_000))
    assert m.report_best_target()
    assert reported[0][0]['stamp']==10. and reported[0][1]=='approximate'
    m.planner.grid[0,0]=100
    assert not m.report_best_target()  # a changed coordinate/map context invalidates it
