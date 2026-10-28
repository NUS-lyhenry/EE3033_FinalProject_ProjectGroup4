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

source=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/mission.py'
cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='Mission')
namespace=dict(Node=object,math=math,time=time,np=np,json=json,FrameJournal=FrameJournal)
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


def test_exhausted_motion_and_sites_continue_observing_until_45_seconds(monkeypatch):
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
    m.acquisition.started=1.;m.acquisition.steps=5
    m.acquisition.positions=[(0.,0.),(.4,0.),(.8,0.)]
    m.acquisition.candidate=dict(stamp=11.,bearing=0.,confidence=.8,label='cup',center_error=.5)
    m.acquisition.active_seconds=26.9
    monkeypatch.setattr(time,'monotonic',lambda:30.)
    m.tick()
    assert m.state=='LOCALIZING' and not m.cancellations
    assert 'continuing localization' in m.acquisition.phase
    # Exactly the work budget, rather than five goals, ends refinement.
    m.report_best_target=lambda:False
    m.acquisition.active_seconds=45.
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
