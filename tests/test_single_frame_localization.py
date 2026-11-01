"""Single-exposure acceptance, honest uncertainty and finite localization evidence."""
import ast
import json
import math
import threading
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from ee3033_project.bottom_range import BottomCamera,CALIBRATION_K,CALIBRATION_D,single_observation
from ee3033_project.capture_covariance import capture_covariance
from ee3033_project.observation_motion import ObservationMotion
from ee3033_project.target_estimate import MapMMSE
from test_mission_policy import mission,namespace
from ee3033_project.core import angle_difference

namespace.update(angle_difference=angle_difference)


def estimate(stamp=10.,sigma=.35,centered=True):
    d=dict(stamp=stamp,origin=[0.,0.],capture_heading=0.,projection_valid=True,
        bearing_valid=True,bearing=.2,centered=centered,center_error_px=0. if centered else 80.,
        settled=False,range_m=.7,ranging_mode='calibrated')
    o=dict(stamp=stamp,source_frame=stamp,label='bottle',x=.7,y=.14,sigma=sigma,
        confidence=.9,method='bottom_inverse',quality_flags=['capture_motion_uncertainty'])
    t=MapMMSE();t.set_map(np.zeros((100,100),int),.02,(-1.,-1.,0.))
    measurement=single_observation(o,d,stamp+.1)
    assert measurement and t.replace_visual_observation(measurement,stamp+.1)
    return t,t.snapshot(stamp+.1)


def owner():
    m=mission();m.initialize_search_recovery();m.bottom_ranging=True
    m.p=dict(target_class='bottle',map_frame='map',max_target_age=2.,max_target_variance=.09)
    m.current_target_estimate=None;m.best_target=None;m.local_sigmas=[];m.view_frames=set()
    m.target_details=None;m.localization_reason=None;m.acquisition_wait=100.
    m.planner=NS(grid=np.zeros((100,100),int),resolution=.02,origin=(-1.,-1.,0.),camera_model=None)
    m.ros_now=10.1;m.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(m.ros_now*1e9)))
    m.readiness_error=lambda **kw:None;m.persist=lambda:None
    m.make_pose=lambda *a:a;m.pose=lambda:(.1,0.,0.)
    m.target_pub=NS(publish=lambda msg:None);m.events=[]
    m.result_pub=NS(publish=lambda msg:m.events.append(json.loads(msg.data)))
    m.get_logger=lambda:NS(info=lambda msg:None)
    return m


def test_one_moving_frame_is_reportable_without_faking_stability_or_erasing_sigma():
    t,e=estimate()
    assert e['source']=='bottom_inverse_single' and e['frames']==e['samples']==1
    assert e['stable'] is False and e['reportable'] is True
    assert e['sigma']>=.35 and len(t.samples)==1 and not t.bearings
    t.freeze(dict(e,stable=False))
    assert t.snapshot(13.)['stale'] and not t.snapshot(13.)['stable']
    assert t.snapshot(13.)['frames']==1


def test_centered_single_frame_reports_immediately_even_above_preferred_sigma():
    m=owner();_,e=estimate()
    m.on_estimate(NS(data=json.dumps(dict(e,mission_stamp=9.,search_epoch=0))))
    assert m.target_reported and m.target['frames']==1
    assert m.target['quality']=='approximate' and not m.target['measurement_stable']
    assert m.cancellations==['RETURN']
    assert [event['event'] for event in m.events]==['target_accepted','target_reported']


def test_cancel_delay_cannot_expire_an_accepted_frame_or_force_another_exposure():
    m=owner();_,e=estimate(centered=False)
    m.on_estimate(NS(data=json.dumps(dict(e,mission_stamp=9.,search_epoch=0))))
    assert m.target and not m.target_reported
    m.ros_now=14.;m.state='LOCALIZING'
    sent=[];m.send_goal=lambda pose,kind:sent.append((pose,kind))
    m.bottle_localization_tick(104.)
    assert len(sent)==1 and sent[0][1]=='acquire' and sent[0][0][:2]==(.1,0.)
    # Completion/cancellation of that sole turn finalizes the retained record.
    m.bottle_localization_tick(107.)
    assert m.target_reported and m.target['observation_stamp']==10.
    assert m.target['stale'] and m.target['age']==pytest.approx(4.)
    assert m.target['source_frames']==[10.] and m.cancellations[-1]=='RETURN'


def test_a_worse_centered_frame_finishes_centering_without_replacing_better_coordinates():
    m=owner();_,first=estimate(centered=False)
    m.on_estimate(NS(data=json.dumps(dict(first,mission_stamp=9.,search_epoch=0))))
    original=(m.target['x'],m.target['y'],m.target['sigma'])
    m.ros_now=10.3;m.state='LOCALIZING';_,worse=estimate(stamp=10.2,sigma=.50,centered=True)
    m.on_estimate(NS(data=json.dumps(dict(worse,mission_stamp=9.,search_epoch=0))))
    assert m.target_reported and (m.target['x'],m.target['y'],m.target['sigma'])==original
    assert m.target['observation_stamp']==10.


def test_stop_keeps_the_accepted_coordinate_and_never_starts_return():
    m=owner();_,e=estimate(centered=False)
    m.on_estimate(NS(data=json.dumps(dict(e,mission_stamp=9.,search_epoch=0))))
    m.state='STOPPED';m.bottle_localization_tick(104.)
    assert m.target and not m.target_reported and 'RETURN' not in m.cancellations


def odometry(end=20.):
    return [(float(t),.01*(t-10.),0.,.02*(t-10.),.08,.2,'odom') for t in np.arange(10.,end+.001,.1)]


def test_ten_second_old_covariance_is_propagated_with_real_continuous_motion():
    value,info=capture_covariance([(10.,(.01,.02))],20.,odometry=odometry())
    assert info['accepted'] and info['propagated'] and info['reason']=='propagated_odometry'
    assert value[0]==pytest.approx(.01+.1**2*.02+.01**2*10+.03**2*.1)
    assert value[1]==pytest.approx(.02+.02**2*10+.05**2*.2)


@pytest.mark.parametrize('change',['gap','frame','too_old','future','missing'])
def test_covariance_propagation_has_a_finite_evidence_boundary(change):
    history=[(10.,(.01,.02))];track=odometry(25.1);stamp=20.
    if change=='gap':track=[s for s in track if not 15.<s[0]<15.5]
    if change=='frame':track=[(*s[:6],'other') if s[0]>15. else s for s in track]
    if change=='too_old':stamp=25.01
    if change=='future':history=[(21.,(.01,.02))]
    if change=='missing':history=[]
    value,info=capture_covariance(history,stamp,odometry=track)
    assert value is None and not info['accepted']


def test_centering_is_looser_but_off_axis_error_is_not_erased():
    c=BottomCamera(640,480,CALIBRATION_K,CALIBRATION_D)
    good=[300.,100.,340.,258.]
    thirty=[330.,100.,370.,258.]
    assert c.view_requirements(thirty)['centered']
    assert c.project_box(thirty)[1]>c.project_box(good)[1]
    assert not c.view_requirements([333.,100.,373.,258.])['centered']


def test_motion_does_not_need_a_complete_stationary_window_to_supply_uncertainty():
    m=ObservationMotion();m.add(10.,0.,0.,0.,.08,.2,'odom')
    assert not m.evaluate(10.)['verified']
    evidence=m.uncertainty_at(10.,.7)
    assert evidence['verified'] and evidence['sigma']>0


def test_a_queued_old_amcl_cannot_rebuild_a_reset_generation():
    path=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/perception.py'
    cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef))
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('reset_covariance_evidence','on_pose')]
    scope=dict(math=math,stamp_seconds=lambda stamp:stamp)
    exec(compile(ast.Module(body=methods,type_ignores=[]),str(path),'exec'),scope)
    n=NS(lock=threading.Lock(),pose_history=deque([(9.,(.1,.1))]),covariance_odometry=deque(),
        covariance_generation=0,covariance_cutoff=0.,p={'map_frame':'map'},
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)))
    scope['reset_covariance_evidence'](n)
    cov=[0.]*36;cov[0]=cov[7]=cov[35]=.01
    scope['on_pose'](n,NS(header=NS(frame_id='map',stamp=9.9),pose=NS(covariance=cov)))
    assert not n.pose_history and n.covariance_generation==1
    scope['on_pose'](n,NS(header=NS(frame_id='map',stamp=10.1),pose=NS(covariance=cov)))
    assert len(n.pose_history)==1


def test_covariance_data_stall_never_dispatches_an_alternate_position():
    from test_search_recovery_policy import owner as recovery_owner
    m=recovery_owner();m.bottom_ranging=True;m.last_usable_bearing=111.
    m.acquisition.candidate={'projection_error':'Capture-time localization covariance missing or stale'}
    m.acquisition.alternate_observation_goal=lambda *a,**kw:pytest.fail('Data failure moved the robot')
    assert m.localization_watchdog(110.)
    assert m.cancellations==['RESEARCH']


def test_planning_single_frame_is_accepted_and_cancels_the_worker():
    m=owner();m.state='PLANNING';_,e=estimate()
    m.on_estimate(NS(data=json.dumps(dict(e,mission_stamp=9.,search_epoch=0))))
    assert m.target_reported and m.target['frames']==1
    assert m.plan_cancel.is_set() and m.cancellations==['RETURN']


def test_mission_and_filter_prefer_a_valid_projection_over_a_clipped_high_confidence_box():
    m=owner()
    common=dict(stamp=10.,label='bottle',bearing=0.,origin=[0.,0.],image_width=640,
        range_model='bottom_inverse_17pt_20261031',bearing_valid=True)
    bad=dict(common,confidence=.99,box=[0.,100.,40.,300.],projection_valid=False)
    good=dict(common,confidence=.8,box=[300.,100.,340.,300.],projection_valid=True)
    m.on_detections(NS(data=json.dumps(dict(detections=[bad,good]))))
    assert m.acquisition.candidate['box']==good['box']
    assert m.target_detected and m.acquisition.frames==1
