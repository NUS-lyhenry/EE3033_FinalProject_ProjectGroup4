"""Empirical range boundaries, stationary batches and actual production callbacks."""
import ast
import json
import math
import threading
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
from ee3033_project.bottom_range import (BottomCamera,SettledBatch,single_observation,attitude_window,A,B,Y_MIN,Y_MAX,MODEL)
from ee3033_project.vision_geometry import FloorCamera
from ee3033_project.target_estimate import MapMMSE
from ee3033_project.target_acquisition import SingleTarget
from ee3033_project.attitude_geometry import AttitudeBuffer
from ee3033_project.bottom_range import CALIBRATION_K,CALIBRATION_D
from ee3033_project.capture_covariance import capture_covariance


def camera():
    return BottomCamera(640,480,[[450.,0,320],[0,450,240],[0,0,1]],[.2,-.1,0,0,0])


def box(distance=.7,offset=0): return [300+offset,100,340+offset,A/(distance-B)]


@pytest.mark.parametrize('y',[200.,300.,400.,Y_MAX])
def test_original_bottom_formula_and_axle_reference(y):
    c=camera();point,sigma,distance=c.project_box([300,100,340,y])
    assert distance==pytest.approx(A/y+B)
    assert point==pytest.approx([A/y+B,0,0])
    assert sigma>=.02
    assert c.translation.tolist()==[0,0,0]
    assert FloorCamera.from_metadata(c.metadata()).project_box([300,100,340,y])[0]==pytest.approx(point)


@pytest.mark.parametrize('y',[188.69,float('nan')])
def test_outside_range_never_extrapolates_or_clamps(y):
    with pytest.raises(ValueError):camera().project_box([300,100,340,y],coarse_max_range=100)
    assert camera().view_requirements([300,100,340,150])['range_hint_m'] is None


@pytest.mark.parametrize('offset',[-10,10])
def test_center_boundary(offset): camera().project_box(box(offset=offset))


@pytest.mark.parametrize('offset',[-150.,100.,150.])
def test_off_center_is_explicitly_less_certain_instead_of_a_hard_center_failure(offset):
    c=camera();point,sigma,distance=c.project_box(box(offset=offset))
    assert np.isfinite(point).all() and sigma>.15
    assert c.view_requirements(box(offset=offset))['centered'] is (abs(offset)<=32)


def test_no_inference_pixels_or_rectification():
    with pytest.raises(ValueError):BottomCamera(416,416,np.eye(3),[])
    c=camera();original=box();top_crop=original.copy();top_crop[1]=0
    assert c.project_box(top_crop)[2]==pytest.approx(c.project_box(original)[2])
    assert c.project_box(original)[2]==pytest.approx(.7)
    assert not c.view_requirements(top_crop)['reframe_required']
    for bad in ([0,100,30,300],[319,100,321,300],[300,475,340,479]):
        with pytest.raises(ValueError):c.project_box(bad)


def test_attitude_is_stability_only_without_an_old_zero():
    samples=[(10+i*.1,.12,.25+.001*(i%2)) for i in range(12)]
    result=attitude_window(samples,11.)
    assert result['valid'] and not result['compensation']
    assert result['fixed_pose_bias_checked'] is False
    for invalid in (samples[:3],[(10+i*.1,0.,i*.02) for i in range(12)]):
        with pytest.raises(ValueError):attitude_window(invalid,11.)
    with pytest.raises(ValueError):attitude_window(samples,12.)


def sample(stamp,x=.7,**changes):
    o=dict(stamp=stamp,x=x,y=.4,sigma=.03,confidence=.9,label='bottle',method='bottom_inverse')
    d=dict(stamp=stamp,settled=True,projection_valid=True,centered=True,
        attitude=dict(valid=True),origin=[0.,.4],capture_heading=0.)
    d.update(changes)
    return o,d


def add(batch,stamp,x=.7,**changes):return batch.add(*sample(stamp,x,**changes))


def test_eight_distinct_exposures_and_common_error_do_not_shrink():
    b=SettledBatch()
    for i in range(7): assert add(b,10+i*.1) is None
    assert add(b,10.6) is None
    group=add(b,10.7)
    assert len(group['source_frames'])==8
    assert group['sigma']>=.03
    for i in range(8,20):group=add(b,10+i*.1)
    assert group['sigma']>=.03
    assert len(group['source_frames'])==8


@pytest.mark.parametrize('change',[dict(settled=False),dict(projection_valid=False)])
def test_invalid_views_break_stationary_group(change):
    b=SettledBatch()
    for i in range(7):add(b,10+i*.1)
    assert add(b,10.7,**change) is None
    assert len(b.samples)==0
    assert add(b,10.8) is None


def test_site_changes_gaps_missing_frames_and_outliers():
    b=SettledBatch()
    for i in range(7):add(b,10+i*.1)
    assert add(b,10.7,origin=[.1,.4]) is None
    assert len(b.samples)==1
    b.add(None,None); assert len(b.samples)==0
    for i in range(8):result=add(b,12+i*.1,x=1. if i==7 else .7)
    assert result is None
    result=add(b,12.8)
    assert result['x']==pytest.approx(.7)
    assert 12.7 not in result['source_frames']
    assert add(b,14.5) is None and len(b.samples)==1


def test_batch_replaces_correlated_likelihood_and_preserves_freshness():
    b=SettledBatch();t=MapMMSE()
    t.set_map(np.zeros((100,100),int),.02,(0,0,0))
    for i in range(8):group=add(b,10+i*.1)
    assert t.replace_batch(group,10.7)
    r=t.snapshot(10.7)
    assert r['stable'] and r['samples']==8 and len(t.samples)==1
    assert r['source']=='bottom_inverse_batch' and r['sigma']>=.02
    assert not t.replace_batch(group,10.7)
    group=add(b,10.8)
    assert t.replace_batch(group,10.8) and len(t.samples)==1
    assert not t.snapshot(13.)['stable']
    invalid=dict(group,stamp=10.9,source_frames=[10.9]*8)
    assert not t.replace_batch(invalid,10.9)


def test_batch_keeps_map_conflict_rejection():
    b=SettledBatch();t=MapMMSE();t.set_map(np.zeros((10,10),int),.02,(0,0,0))
    for i in range(8):group=add(b,10+i*.1)
    accepted=t.replace_batch(group,10.7)
    assert not accepted or (t.snapshot(10.7)['prior_conflict'] and not t.snapshot(10.7)['stable'])


def test_bottle_width_cannot_force_unnecessary_approach():
    target=SingleTarget()
    for stamp in (10,10.1):
        target.update([dict(stamp=stamp,confidence=.9,label='bottle',box=[315,100,325,300],
            image_width=640,range_model=MODEL,bearing=0.,origin=[0,0],range_state='within_range',
            centered=True,range_m=.7)],stamp,0.)
    assert target.locked and not target.needs_closer_view()
    planner=NS(camera_model=camera())
    assert target.observation_goal(planner,(0.,0.,0.)) is None
    assert 'eight' in target.last_goal_reason


def test_production_perception_uses_capture_tf_without_optical_offset_or_old_pitch():
    path=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/perception.py'
    cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef))
    cls.bases=[];cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='locate']
    calls=[]
    def lookup(*args):calls.append(args);return None
    scope=dict(math=math,np=np,capture_covariance=capture_covariance,Time=NS(from_msg=lambda stamp:stamp),
        stamp_seconds=lambda stamp:stamp,transform_arrays=lambda tr:(np.array([[0.,-1,0],[1.,0,0],[0,0,1.]]),np.array([2.,3.,0.])))
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),scope)
    node=scope['Perception']();node.empirical=True;node.floor_camera=camera()
    node.p=dict(map_frame='map',base_frame='base_footprint',camera_x=99.,camera_pitch=99.)
    node.buffer=NS(lookup_transform=lookup);node.attitude_check=dict(span=[0.,0.])
    node.lock=threading.Lock();node.pose_history=[(9.9,(.001,.002)),(10.1,(99,99))]
    image=NS(header=NS(stamp=10.))
    point=node.locate(box(),image,None,[])
    assert point==pytest.approx([2.,3.7,0.])
    assert calls==[('map','base_footprint',10.)]
    assert node.projection_sigma<.1
    assert node.capture_pose_diagnostic['selected_pose_stamp']==9.9
    assert node.capture_pose_diagnostic['latest_pose_stamp']==10.1
    node.pose_history=[]
    with pytest.raises(ValueError,match='covariance'):node.locate(box(),image,None,[])
    assert node.capture_pose_diagnostic['reason']=='missing'


def test_actual_detector_callback_delivers_each_single_frame_without_settling_requirement():
    """Exercise message construction, association, batching and map acceptance.

    ROS transport and the YOLO network are mocked; production callback bodies
    run unchanged. This catches publication order and missing metadata errors.
    """
    from collections import deque
    from ee3033_project.capture_frame import CaptureFrameGate
    from ee3033_project.localization_diagnostics import DiagnosticHistory
    from ee3033_project.triangulation import BearingIntersection
    from ee3033_project.vision_geometry import camera_matrix
    class Publisher:
        def __init__(self): self.messages=[]
        def publish(self,msg): self.messages.append(msg)
    def pose_message():
        return NS(header=NS(stamp=NS()),pose=NS(pose=NS(position=NS(),orientation=NS()),covariance=[0.]*36))
    def load_class(name,filename,scope,skip=()):
        path=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project'/filename
        cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name==name)
        cls.bases=[];cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name not in ('__init__',*skip)]
        exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),scope)
        return scope[name]
    common=dict(math=math,np=np,single_observation=single_observation,capture_covariance=capture_covariance,json=json,time=__import__('time'),cv2=__import__('cv2'),
        Time=NS(from_msg=lambda s:s),String=NS,TransformException=LookupError,
        stamp_seconds=lambda s:s,transform_arrays=lambda tr:(np.eye(3),np.zeros(3)),
        camera_matrix=camera_matrix,BottomCamera=BottomCamera,attitude_window=attitude_window,CALIBRATION_K=CALIBRATION_K,CALIBRATION_D=CALIBRATION_D,
        angle_difference=lambda a,b:math.atan2(math.sin(a-b),math.cos(a-b)))
    Perception=load_class('Perception','perception.py',dict(common))
    node=Perception();node.empirical=True;node.lock=threading.Lock()
    node.p=dict(localization_method='bottom_inverse',target_class='bottle',confidence=.6,
        image_size=416,image_topic='/image_raw',max_image_age=1.5,
        pixel_sigma=2.,pixel_margin=8.,position_sigma=.1,max_report_sigma=.1,
        map_frame='map',base_frame='base_footprint',odom_frame='odom_combined',calibration_file='')
    node.camera=NS(width=640,height=480,header=NS(frame_id='optical'),k=CALIBRATION_K,d=CALIBRATION_D)
    node.latest_image=None;node.scans=[];node.capture_frame_gate=CaptureFrameGate()
    node.last_processed=None;node.quiet_since=None;node.motion_check=None
    node.last_odometry_age=0.;node.attitude_check={};node.pose_variance=(.0004,.0004)
    node.motion=NS(evaluate=lambda stamp:dict(verified=True,position_span=0.,heading_span=0.))
    node.buffer=NS(can_transform=lambda *a:True,lookup_transform=lambda *a:None)
    node.attitude_buffer=AttitudeBuffer()
    for stamp in np.arange(9.,13.,.1):node.attitude_buffer.add(float(stamp),.1,.2)
    node.bridge=NS(imgmsg_to_cv2=lambda *a,**k:np.zeros((480,640,3),np.uint8),
        cv2_to_imgmsg=lambda *a,**k:NS())
    tensor=NS(cpu=lambda:NS(numpy=lambda:np.array(box())))
    detection=NS(cls=NS(item=lambda:0),conf=NS(item=lambda:.9),xyxy=[tensor])
    result=NS(orig_shape=(480,640),names={0:'bottle'},boxes=[detection],plot=lambda:np.zeros((480,640,3),np.uint8))
    node.model=NS(predict=lambda *a,**k:[result]);node.calibration=None
    node.image_pub=Publisher();node.obs_pub=Publisher();node.status_pub=Publisher();node.detections_pub=Publisher()
    warnings=[];node.warn=warnings.append
    Filter=load_class('TargetFilter','target_filter.py',dict(common,
        PoseWithCovarianceStamped=pose_message,mmse_rejection_reason=lambda *a:'rejected'))
    f=Filter();f.p=dict(map_frame='map',target_class='bottle',max_observation_age=2.)
    f.tracker=MapMMSE();f.tracker.set_map(np.zeros((100,100),int),.02,(-1,-1,0.))
    f.bottom_batch=SettledBatch();f.active=True;f.target_locked=True;f.armed_at=9.
    f.mission_stamp=9.;f.last_frame_stamp=0.;f.geometry_conflict=False
    f.association=SingleTarget();f.association.started=1.;f.intersection=BearingIntersection()
    f.diagnostics=DiagnosticHistory();f.diagnostics_pub=Publisher();f.pose_pub=Publisher()
    f.details_pub=Publisher();f.estimate_pub=Publisher()
    for i in range(8):
        stamp=10.+i*.2
        node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int((stamp+.1)*1e9)))
        f.now=lambda:stamp+.1
        node.pose_history=[(stamp-.1,(.0004,.0004))]
        node.latest_image=NS(width=640,height=480,header=NS(frame_id='optical',stamp=stamp))
        node.process()
        frame=json.loads(node.obs_pub.messages[-1].data)
        assert len(frame['observations'])==1,(warnings,frame)
        assert frame['bearings'][0]['projection_valid'] is True
        assert frame['bearings'][0]['centered'] is True
        assert frame['bearings'][0]['range_m']==pytest.approx(.7)
        f.on_frame(node.obs_pub.messages[-1])
        assert len(f.pose_pub.messages)==i+1
    estimate=f.tracker.snapshot(stamp+.1)
    assert not estimate['stable'] and estimate['samples']==1 and estimate['reportable']
    assert estimate['source']=='bottom_inverse_single' and not f.tracker.bearings
    assert estimate['x']==pytest.approx(.7,abs=.01) and estimate['y']==pytest.approx(0.,abs=.01)
    assert not warnings
    # Moving detections still steer but cannot start another valid batch.
    node.motion=NS(evaluate=lambda stamp:dict(verified=False))
    stamp=11.6;node.latest_image.header.stamp=stamp;node.pose_history=[(stamp-.1,(.0004,.0004))]
    node.process();frame=json.loads(node.obs_pub.messages[-1].data)
    assert frame['observations'] and frame['bearings']
    assert not frame['bearings'][0]['settled']
    f.on_frame(node.obs_pub.messages[-1]);assert len(f.bottom_batch.samples)==0


@pytest.mark.parametrize('offset',[11.,30.,60.])
def test_off_axis_estimates_add_uncertainty_instead_of_demanding_perfect_center(offset):
    c=camera();point,sigma,distance=c.project_box(box(offset=offset))
    assert np.linalg.norm(point[:2])==pytest.approx(.7)
    assert sigma>c.project_box(box())[1]
    assert c.view_requirements(box(offset=offset))['centered'] is (abs(offset)<=32)


def test_centering_and_posture_grace_is_bounded():
    b=SettledBatch();result=None
    for i in range(8):
        assert add(b,10+i*.2,centered=False,attitude=dict(valid=False)) is None
    for i in range(8,16):result=add(b,10+i*.2,centered=False,attitude=dict(valid=False))
    assert result and result['distinct_frames']==8


def test_camera_range_default_95cm_and_maximum_100cm():
    c=camera()
    assert c.min_range==.35 and c.max_range==.95
    for distance in (.951,1.,1.1,2.):
        d=box(distance)
        assert not c.view_requirements(d)['search_eligible']
        with pytest.raises(ValueError):c.project_box(d)
    c.max_range=1.
    assert c.view_requirements(box(.99))['search_eligible']
    assert not c.view_requirements(box(1.01))['search_eligible']
    assert not c.contains_floor([[1.01,0]])[0]
    from ee3033_project.assessment import mission_options
    assert mission_options(dict(target_class='bottle'))['bottle_max_range']==.95
    for lo,hi in ((.34,.95),(.35,1.01),(.95,.9),(True,.95)):
        with pytest.raises(ValueError):mission_options(dict(target_class='bottle',bottle_min_range=lo,bottle_max_range=hi))


def test_bottom_edge_is_a_near_guess_without_inverse_extrapolation():
    c=camera();near=[300,200,340,480]
    view=c.view_requirements(near)
    assert view['ranging_mode']=='near_guess' and view['search_eligible']
    point,sigma,distance=c.project_box(near)
    assert .12<=distance<.35 and sigma>=.12
    assert distance!=pytest.approx(A/480+B)
    with pytest.raises(ValueError):c.project_box([300,0,340,0])


def test_near_guess_map_support_is_bounded_and_precision_has_a_large_floor():
    c=camera();b=SettledBatch();t=MapMMSE()
    t.set_map(np.zeros((150,150),int),.01,(-.5,-.5,0.))
    for i in range(8):
        o,d=sample(10+i*.1,x=.235)
        o.update(y=0.,sigma=.12,quality_flags=['near_field_guess'])
        d.update(origin=[0,0],ranging_mode='near_guess',camera_min_range=.35,bearing=0.)
        group=b.add(o,d)
    assert group['observation_quality']=='approximate'
    assert t.replace_batch(group,10.7)
    result=t.snapshot(10.7)
    assert .12<=math.hypot(result['x'],result['y'])<=.35
    assert result['sigma']>=.12 and result['near_guess']
    assert result['stable']



def test_out_of_range_yolo_does_not_lock_or_cancel_global_search():
    from test_mission_policy import mission
    m=mission();m.p=dict(target_class='bottle');m.view_frames=set()
    for stamp in (10.,10.2,10.4):
        d=dict(stamp=stamp,label='bottle',confidence=.95,box=box(1.2),
            search_eligible=False,range_state='too_far')
        m.on_detections(NS(data=json.dumps(dict(detections=[d]))))
    assert not m.target_detected and not m.cancellations
    assert not m.plan_cancel.is_set()



def test_near_guess_never_commands_an_approach_into_the_bottle():
    target=SingleTarget();target.candidate=dict(stamp=10.,confidence=.9,label='bottle',range_model=MODEL,
        ranging_mode='near_guess',range_state='too_near',centered=True,projection_valid=True,
        bearing=0.,origin=[0,0],range_m=.235)
    planner=NS(camera_model=camera())
    assert target.observation_goal(planner,(0,0,0)) is None
    assert 'near-field estimate' in target.last_goal_reason


def test_unsafe_alternate_view_reports_a_fresh_near_estimate_without_waiting_for_timeout(monkeypatch):
    from test_mission_policy import mission
    import time
    m=mission();m.state='LOCALIZING';m.target_detected=True
    m.p=dict(goal_timeout=75.,mission_timeout=900.,acquisition_seconds=45.,localize_wait_seconds=2.,max_target_age=2.)
    m.get_parameter=lambda name:NS(value=m.p[name])
    m.navigation_map=NS(poll=lambda now:None,allowed=lambda *a:False)
    m.update_localization_clock=lambda:None;m.heartbeat=lambda *a:None
    m.publish_status=lambda:None;m.goal_handle=None;m.started=0.;m.last_status=0.
    m.return_budget_exhausted=lambda:False;m.readiness_error=lambda **k:None
    m.acquisition_wait=0.;m.acquisition_decision_after=0.;m.pose=lambda:(0.,0.,0.)
    m.current_target_estimate=dict(source='bottom_inverse_batch',stable=True,stamp=11.,observation_quality='approximate')
    m.planner=NS(camera_model=camera());m.acquisition.started=1.;m.acquisition.timeout=45.
    m.acquisition.candidate=dict(stamp=11.,bearing=0.,origin=[0,0],confidence=.9,label='bottle',
        range_model=MODEL,ranging_mode='near_guess',centered=True,projection_valid=True,center_error=0.)
    m.acquisition.observation_goal=lambda *a,**k:None
    reports=[];m.report_target=lambda estimate,quality:reports.append((estimate,quality))
    monkeypatch.setattr(time,'monotonic',lambda:4.)
    m.tick()
    assert reports and reports[0][1]=='approximate'
    assert not m.cancellations



def test_first_near_batch_cannot_jump_to_distant_map_support():
    b=SettledBatch();t=MapMMSE();t.set_map(np.zeros((20,20),int),.01,(.6,0,0.))
    for i in range(8):
        o,d=sample(10+i*.1,x=.235)
        o.update(y=0.,sigma=.2,quality_flags=['near_field_guess'])
        d.update(origin=[0,0],ranging_mode='near_guess',camera_min_range=.35,bearing=0.)
        group=b.add(o,d)
    assert not t.replace_batch(group,10.7)
    assert t.snapshot(10.7) is None
