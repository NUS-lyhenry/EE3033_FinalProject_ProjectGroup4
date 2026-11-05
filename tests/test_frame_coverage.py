"""Real exposure footprints: no stop, waypoint, multi-frame or empty-FOV gate."""
import json
import math
from types import SimpleNamespace as NS
import numpy as np
import pytest
from ee3033_project.bottom_range import BottomCamera,CALIBRATION_K,CALIBRATION_D
from ee3033_project.frame_coverage import exposure_view
from ee3033_project.coverage import CoveragePlanner
from ee3033_project.vision_geometry import FloorCamera
from ee3033_project.core import angle_difference
from ee3033_project.stationary import StationaryWindow
from test_mission_policy import mission,namespace

namespace.update(exposure_view=exposure_view,FloorCamera=FloorCamera,angle_difference=angle_difference)


def test_camera_coverage_spacing_is_independent_of_viewpoint_spacing():
    grid = np.zeros((32,32), dtype=int)
    default = CoveragePlanner(grid,.05,(0.,0.,0.))
    denser = CoveragePlanner(grid,.05,(0.,0.,0.),spacing=.45,coverage_spacing=.10)
    assert default.spacing == .65 and default.target_stride == 3
    assert denser.spacing == .45 and denser.target_stride == 2


def frame(stamp=10.,pose=(1.23,1.27,0.),**overrides):
    value=dict(stamp=stamp,ready=True,target_class='bottle',geometry_ready=True,
        camera_model=BottomCamera(640,480,CALIBRATION_K,CALIBRATION_D).metadata(),
        capture_pose=dict(stamp=stamp,frame_id='map',x=pose[0],y=pose[1],yaw=pose[2]),
        observation_ready=False,motion_check={'verified':False,'reason':'normal balancing'},
        detected_candidates=1)
    value.update(overrides)
    return value


def owner():
    m=mission();m.initialize_search_recovery()
    m.state='SEARCHING';m.p=dict(map_frame='map',target_class='bottle',perception_timeout=3.)
    grid=np.zeros((65,65),dtype=int);grid[:,40]=100
    m.planner=CoveragePlanner(grid,.05,(0,0,0),camera_model=BottomCamera(640,480,CALIBRATION_K,CALIBRATION_D))
    m.planner.prepare(1.2,1.2)
    m.seen=set();m.current_option=0;m.observation_stamps=set();m.observe_after_stamp=9.
    m.geometry_fault_since=None;m.persist_observation_audit=lambda:None
    m.pose=lambda:pytest.fail('Coverage must never query callback-time currentPose')
    return m


@pytest.mark.parametrize('state',['SEARCHING','OBSERVING','ACQUIRING','LOCALIZING','PLANNING'])
def test_single_moving_balancing_exposure_with_target_credits_actual_fov(state):
    m=owner();m.state=state;d=frame()
    m.on_perception(NS(data=json.dumps(d)))
    camera=FloorCamera.from_metadata(d['camera_model']);pose=(1.23,1.27,0.)
    expected=m.planner.visible(*m.planner.world_to_cell(*pose[:2]),pose[2],camera_model=camera,position=pose[:2])
    assert m.seen==expected and m.seen
    assert all(c<40 for r,c in m.seen)  # No credit through the wall.
    assert m.observation_history[0]['pose']==list(pose)
    assert m.observation_events[0]['source']=='exposure'
    assert m.observation_events[0]['stamp']==10.
    assert not d['observation_ready'] and d['detected_candidates']==1


def test_two_exposures_union_only_their_actual_footprints_not_the_swept_gap():
    m=owner();m.capture_coverage(frame(stamp=10.,pose=(.6,.6,0.)))
    first=set(m.seen)
    d=frame(stamp=10.2,pose=(1.7,1.8,math.pi));m.capture_coverage(d)
    model=FloorCamera.from_metadata(d['camera_model'])
    second=m.planner.visible(*m.planner.world_to_cell(1.7,1.8),math.pi,camera_model=model,position=(1.7,1.8))
    assert m.seen==first|second
    assert len(m.observation_history)==2


@pytest.mark.parametrize('change',[
    {'stamp':8.9}, {'stamp':11.1}, {'ready':False}, {'capture_pose':None},
    {'capture_pose':dict(stamp=10.1,frame_id='map',x=1.,y=1.,yaw=0.)},
    {'capture_pose':dict(stamp=10.,frame_id='odom',x=1.,y=1.,yaw=0.)},
    {'capture_pose':dict(stamp=10.,frame_id='map',x=float('nan'),y=1.,yaw=0.)},
    {'search_epoch':1}, {'mission_stamp':8.}, {'camera_model':None},
])
def test_stale_future_invalid_pose_camera_and_wrong_epoch_never_credit(change):
    m=owner();assert not m.capture_coverage(frame(**change));assert not m.seen


def test_unknown_motion_is_not_replaced_by_zero_and_does_not_block_valid_exposure():
    m=owner();d=frame();d.pop('motion_check');d.pop('observation_ready')
    assert m.capture_coverage(d)
    assert 'motion_check' not in d and 'observation_ready' not in d


def test_duplicate_exposure_and_old_epoch_cannot_restore_invalidated_cells():
    m=owner();d=frame();assert m.capture_coverage(d)
    count=len(m.observation_events);assert not m.capture_coverage(d)
    assert len(m.observation_events)==count
    m.search_epoch=1;m.epoch_started_stamp=10.2;m.seen.clear()
    assert not m.capture_coverage(d) and not m.seen
    assert m.capture_coverage(frame(stamp=10.3,search_epoch=1))


def test_failed_navigation_preserves_independent_exposure_evidence():
    m=owner();m.capture_coverage(frame());seen=set(m.seen)
    m.goal_kind='search_scan';m.heartbeat=lambda *a:None;m.index=0;m.failures=[]
    m.failed_options=set();m.next_viewpoint=lambda:None
    m.goal_finished(False)
    assert m.seen==seen


def test_many_frames_at_one_heading_do_not_consume_six_scan_views():
    m=owner()
    for i in range(7):assert m.capture_coverage(frame(stamp=10.+i*.1))
    assert m.station_views==1
    assert m.observation_history[0]['exposure_count']==7


def test_return_hold_allows_five_cm_xy_without_twist_or_heading_stillness():
    hold=StationaryWindow.position_hold(.5,.05)
    for i in range(6):hold.add(i*.1,i*.008,0.,i*.2,.12,.7,('odom','base'))
    assert hold.evaluate(.5)['verified']
    narrow=StationaryWindow.position_hold(.5,.02)
    for row in hold.samples:narrow.add(*row)
    assert not narrow.evaluate(.5)['verified']


def test_research_revokes_real_two_station_footprints_and_old_frames_cannot_refill():
    from test_search_recovery_policy import owner as recovery_owner
    m=recovery_owner();m.planner=owner().planner;m.current_option=0
    m.observation_history=[];m.observation_events=[];m.seen=set()
    for stamp,pose in [(109.,(.6,.6,0.)),(109.1,(1.,1.,0.)),(109.2,(1.4,1.4,math.pi))]:
        assert m.capture_coverage(frame(stamp=stamp,pose=pose))
    before=set(m.seen)
    reopened=set().union(*({tuple(p) for p in h['cells']} for h in m.observation_history[-2:]))
    m.reset_for_research('target lost')
    assert m.seen==before-reopened
    assert [h['invalidated'] for h in m.observation_history]==[False,True,True]
    assert not m.capture_coverage(frame(stamp=109.2,pose=(1.4,1.4,math.pi)))
    assert m.seen==before-reopened
