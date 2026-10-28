import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
from ee3033_project.localization_diagnostics import DiagnosticHistory, finite_json, mmse_rejection_reason
from ee3033_project.target_estimate import MapMMSE
from ee3033_project.target_acquisition import SingleTarget
from ee3033_project.triangulation import BearingIntersection


source=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/target_filter.py'
cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef))
method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='on_frame')
namespace=dict(json=json,math=math,np=np,String=NS,mmse_rejection_reason=mmse_rejection_reason)
exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
on_frame=namespace['on_frame']


def filter_node():
    tracker=MapMMSE();tracker.set_map(np.zeros((40,40)),.1,(0,0,0))
    events=[]
    n=NS(now=lambda:12.,mission_stamp=10.,target_locked=True,armed_at=10.,last_frame_stamp=0.,
        p=dict(max_observation_age=2.,target_class='cup'),tracker=tracker,
        association=SingleTarget(),intersection=BearingIntersection(),geometry_conflict=False,
        active=False,diagnostics=DiagnosticHistory(),publish=lambda:None,
        diagnostics_pub=NS(publish=lambda m:events.append(json.loads(m.data))))
    return n,events


def bearing(stamp,origin=(0.,0.),target=(1.,1.),**extra):
    return dict(stamp=stamp,label='cup',confidence=.9,box=[100,100,140,200],
        origin=list(origin),odom_origin=list(origin),map_odom=[0.,0.,0.],
        odom_bearing=math.atan2(target[1]-origin[1],target[0]-origin[0]),
        bearing=math.atan2(target[1]-origin[1],target[0]-origin[0]),bearing_sigma=.02,
        center_error=0.,position_sigma=.02,settled=True,motion_check={'verified':True},**extra)


def deliver(n,stamp,bearings=(),observations=(),**diagnostics):
    n.now=lambda:stamp+.1
    on_frame(n,NS(data=json.dumps(dict(stamp=stamp,observations=list(observations),bearings=list(bearings),
        diagnostics=dict(detections=list(bearings),motion_check={'verified':True},**diagnostics)))))


def test_history_is_bounded_and_jsonsafe_without_recursive_summaries():
    h=DiagnosticHistory()
    for i in range(70):h.add(dict(stamp=float(i),reason_codes=['unsettled'],frame={'age':math.inf,'bad':np.nan,'settled':True}))
    summary=h.summary()
    assert summary['frames']==70 and summary['retained']==32
    assert summary['reason_counts']['unsettled']==70
    assert summary['latest']['frame']==dict(age=None,bad=None,settled=True)
    json.dumps(summary,allow_nan=False)


@pytest.mark.parametrize('stamp,last,armed,code',[
    (9.,0.,10.,'frame_before_current_mission'),
    (11.,11.,10.,'frame_duplicate_or_reordered'),
])
def test_early_rejected_frame_still_publishes_one_mission_tagged_event(stamp,last,armed,code):
    n,events=filter_node();n.last_frame_stamp=last;n.armed_at=armed
    deliver(n,stamp)
    assert len(events)==1 and events[0]['mission_stamp']==10.
    assert code in events[0]['reason_codes'] and events[0]['stamp']==stamp


def test_stale_or_broken_frame_is_diagnosed_without_crashing_callback():
    n,events=filter_node()
    on_frame(n,NS(data=json.dumps(dict(stamp=5.,observations=[],bearings=[],diagnostics={'reason_codes':['perception_image_stale_or_future']}))))
    assert 'frame_stale_future_or_invalid' in events[-1]['reason_codes']
    on_frame(n,NS(data='[]'))
    assert 'localization_frame_processing_error' in events[-1]['reason_codes']
    assert events[-1]['mission_stamp']==10.


def test_perception_tf_and_crop_failures_survive_filter_event():
    n,events=filter_node()
    deliver(n,11.,reason_codes=['bearing_tf_error','bearing_horizontal_clipping'],error='Missing odom transform')
    assert set(['bearing_tf_error','bearing_horizontal_clipping','no_bearing_measurement'])<=set(events[-1]['reason_codes'])
    assert events[-1]['frame']['error']=='Missing odom transform'


def test_each_bearing_rejection_explains_motion_center_confidence_and_duplicate():
    t=BearingIntersection()
    for changes,code in [({'settled':False},'bearing_motion_unsettled'),
        ({'center_error':.2},'bearing_not_centered'),({'confidence':.5},'bearing_low_confidence')]:
        d=bearing(11.);d.update(changes);assert t.add(d) is None
        assert t.reason_code==code and not t.frame_accepted
    t.add(bearing(11.));t.add(bearing(11.))
    assert t.reason_code=='bearing_duplicate_or_reordered_frame'


def test_collecting_weak_crossing_and_success_events_keep_source_geometry():
    n,events=filter_node()
    for stamp in (11.,11.2,11.4,11.6):deliver(n,stamp,[bearing(stamp)])
    assert events[-1]['intersection']['reason_code']=='bearing_collecting_site_frames'
    assert events[-1]['progress']['sites'][0]['frames']==4
    for stamp in (13.,13.2,13.4,13.6):deliver(n,stamp,[bearing(stamp,(0.,1.))])
    assert events[-1]['intersection']['accepted']
    assert events[-1]['selected']['map_odom']==[0.,0.,0.]
    assert events[-1]['mmse']['updated'] and events[-1]['mmse']['stable']
    assert events[-1]['mmse']['source']=='bearing_intersection'
    weak=BearingIntersection()
    for origin,start in [((0.,0.),11.),((0.,.1),13.)]:
        for i in range(4):weak.add(bearing(start+i*.2,origin,(2.,0.)))
    assert weak.reason_code=='bearing_weak_crossing_angle'


def test_mmse_rejection_is_distinguished_from_unfinished_confirmation():
    m=MapMMSE();m.set_map(np.zeros((40,40)),.1,(0,0,0))
    d=dict(stamp=11.,x=1.,y=1.,sigma=.1,label='cup',confidence=.9)
    assert m.update(d,11.)
    assert not m.update(d,11.1)
    assert mmse_rejection_reason(m,d,11.1)=='mmse_duplicate_or_reordered_evidence'
    assert mmse_rejection_reason(m,dict(d,stamp=11.2,x=3.),11.2)=='mmse_innovation_rejected'


def test_bearing_only_map_posterior_precedes_natural_intersection_with_ab_maneuver_off():
    n,events=filter_node()
    # This flag belongs to the motion owner; it must never disable passive
    # geometry processing when two naturally visited sites provide bearings.
    n.p['enable_ab_maneuver']=False
    for stamp in (11.,11.2,11.4,11.6):deliver(n,stamp,[bearing(stamp)])
    first=n.tracker.snapshot(11.7)
    assert first['source']=='bearing_only' and first['stable']
    assert first['sigma']>.5 and first['samples']==4
    assert events[-1]['mmse']['source']=='bearing_only'
    for stamp in (13.,13.2,13.4,13.6):deliver(n,stamp,[bearing(stamp,(0.,1.))])
    result=n.tracker.snapshot(13.7)
    assert result['source']=='bearing_intersection' and result['stable']
    assert result['sigma']<first['sigma']
    assert len(n.tracker.samples)==1 and not n.tracker.bearings
    assert len(result['source_frames'])==8 and result['first_source_stamp']==11.
    assert result['last_source_stamp']==13.6
    assert result['position_confidence']>first['position_confidence']
    assert 0<=result['position_confidence']<=1 and result['confidence_radius_m']==.2


def test_same_frame_floor_and_ray_are_not_counted_twice_and_joint_factor_replaces_both():
    n,_=filter_node()
    for origin,start in [((0.,0.),11.),((0.,1.),13.)]:
        for i in range(4):
            stamp=start+i*.2
            floor=dict(stamp=stamp,source_frame=stamp,x=1.,y=1.,sigma=.2,
                label='cup',confidence=.9,box=[100,100,140,200])
            deliver(n,stamp,[bearing(stamp,origin)],[floor])
            if n.tracker.factor is None:
                assert not n.tracker.bearings
    result=n.tracker.snapshot(13.7)
    assert result['source']=='bearing_intersection'
    assert len(n.tracker.samples)==1 and not n.tracker.bearings
    before=result['covariance']
    floor.update(stamp=13.8,source_frame=13.8)
    deliver(n,13.8,[],[floor])
    assert n.tracker.snapshot(13.9)['covariance']==before


def test_invalid_capture_bearing_cannot_create_map_posterior_without_floor():
    n,events=filter_node()
    deliver(n,11.,[dict(bearing(11.),settled=False)])
    assert n.tracker.snapshot(11.1) is None
    assert 'bearing_motion_unsettled' in events[-1]['reason_codes']
