"""Unconfirmed display data never writes to the mission estimator."""
import json
import math
import pytest
from ee3033_project.visual_telemetry import VisualTelemetry


def detection(stamp=10.,**changes):
    d=dict(stamp=stamp,label='bottle',confidence=.8,box=[300,100,340,300],
        image_width=640,image_height=480,x=.7,y=.2,sigma=.18,
        settled=False,projection_valid=False,search_eligible=True,
        range_m=.7,ranging_mode='calibrated',center_error_px=0.,origin=[0,0],bearing=.2)
    d.update(changes);return dict(stamp=stamp,detections=[d])


def test_first_unsettled_frame_is_a_visible_preview_without_confirmation():
    t=VisualTelemetry();assert t.ingest(detection(),10.2,'bottle')
    p=t.displayed_preview(10.2)
    assert p['x']==.7 and p['y']==.2 and p['sigma']==.18
    assert p['samples']==1 and p['display_only'] and not p['stable'] and not p['confirmed']
    assert p['method']=='single-frame visual preview'


def test_missing_detection_keeps_last_coordinate_marked_stale_even_after_thirty_seconds():
    t=VisualTelemetry();t.ingest(detection(),10.2,'bottle')
    t.ingest(dict(stamp=10.5,detections=[]),10.6,'bottle')
    assert t.snapshot(10.6)['raw_count']==0
    assert t.displayed_preview(10.6)['stale']
    assert t.displayed_preview(100.)['x']==.7
    assert t.snapshot(100.)['last_seen_age']==90.


def test_no_map_transform_means_range_and_reason_without_a_fake_map_point():
    t=VisualTelemetry();f=detection()
    for key in ('x','y','sigma'):del f['detections'][0][key]
    f['detections'][0]['projection_error']='Capture-time transform unavailable'
    t.ingest(f,10.2,'bottle')
    assert t.displayed_preview(10.2) is None
    assert t.snapshot(10.2)['selected']['range_m']==.7


def test_out_of_range_boxes_can_show_direction_but_never_create_a_position():
    t=VisualTelemetry();t.ingest(detection(search_eligible=False,range_state='too_far'),10.1,'bottle')
    assert t.snapshot(10.1)['raw_count']==1 and t.snapshot(10.1)['in_range_count']==0
    assert t.displayed_preview(10.1) is None


@pytest.mark.parametrize('change',[dict(x=math.nan),dict(y=math.inf),dict(sigma=-1),dict(sigma=None)])
def test_invalid_coordinates_are_not_drawn(change):
    t=VisualTelemetry();t.ingest(detection(**change),10.1,'bottle')
    assert t.displayed_preview(10.1) is None


def test_reset_rejects_frames_from_before_map_pose_or_mission_change():
    t=VisualTelemetry();t.ingest(detection(),10.1,'bottle');t.reset(11.)
    assert t.displayed_preview(11.) is None
    assert not t.ingest(detection(stamp=10.9),11.1,'bottle')
    assert t.ingest(detection(stamp=11.1),11.2,'bottle')


def test_duplicates_wrong_classes_old_and_future_frames_do_not_refresh_a_preview():
    t=VisualTelemetry();t.ingest(detection(),10.1,'bottle')
    assert not t.ingest(detection(),10.5,'bottle')
    assert not t.ingest(detection(stamp=11.),10.5,'bottle')
    assert not t.ingest(detection(stamp=5.),10.5,'bottle')
    t.ingest(detection(stamp=10.5,label='cup'),10.6,'bottle')
    assert t.displayed_preview(10.6)['stamp']==10. and t.displayed_preview(10.6)['stale']


def test_large_uncertainty_is_retained_and_diagnostics_expose_batch_progress():
    t=VisualTelemetry();t.ingest(detection(sigma=1.4),10.1,'bottle')
    assert t.displayed_preview(10.1)['sigma']==1.4
    t.ingest_diagnostic(dict(stamp=10.,mmse=dict(batch_frames=3,required_frames=8,updated=False)),10.1)
    assert t.snapshot(10.1)['diagnostic']['mmse']['batch_frames']==3
    assert t.snapshot(14.)['diagnostic'] is None


def test_no_selected_point_and_raw_detection_counts_are_independent_of_fusion():
    t=VisualTelemetry();f=detection()
    f['detections']+=[dict(f['detections'][0],x=.9,confidence=.65)]
    assert t.ingest(f,10.1,'bottle')
    assert t.snapshot(10.1)['raw_count']==2 and t.displayed_preview(10.1)['x']==.7
    # No ROS publisher, navigation callback or estimator is referenced by this state.
    json.dumps(t.snapshot(10.1),allow_nan=False)



def test_locked_target_preview_does_not_jump_to_an_unassociated_stronger_box():
    t=VisualTelemetry();t.ingest(detection(),10.1,'bottle')
    t.ingest_diagnostic(dict(stamp=10.,target_locked=True,selected=dict(x=.7,y=.2)),10.1)
    t.ingest(detection(stamp=10.2,x=2.5,confidence=.99),10.3,'bottle')
    assert t.displayed_preview(10.3)['x']==.7 and t.displayed_preview(10.3)['stale']


def test_exceptional_nested_values_do_not_break_the_state_api_json():
    t=VisualTelemetry();t.ingest(detection(motion_check=dict(linear_mean=float('nan'))),10.1,'bottle')
    assert t.snapshot(10.1)['selected']['motion_check']['linear_mean'] is None
    json.dumps(t.snapshot(10.1),allow_nan=False)



def test_capture_time_bearing_and_valid_range_show_a_coarse_point_when_covariance_is_missing():
    t=VisualTelemetry();f=detection(bearing_valid=True,bearing=math.pi/2,origin=[2.,3.],
        projection_error='Capture-time localization covariance missing or stale')
    for key in ('x','y','sigma'):del f['detections'][0][key]
    t.ingest(f,10.1,'bottle')
    p=t.displayed_preview(10.1)
    assert p['x']==pytest.approx(2.) and p['y']==pytest.approx(3.7)
    assert p['sigma']>=.15 and p['display_only'] and not p['projection_valid']
    assert 'unverified_position_preview' in p['quality_flags']


def test_valid_moving_projection_keeps_backend_uncertainty_without_a_second_motion_floor():
    telemetry=VisualTelemetry()
    telemetry.ingest(detection(sigma=.04,projection_valid=True,settled=False,
        motion_check=dict(verified=False,samples=10,linear_mean=.08,angular_mean=.2)),10.1,'bottle')
    preview=telemetry.displayed_preview(10.1)
    assert preview['sigma']==pytest.approx(.04)
    assert preview['samples']==1 and preview['display_only'] and not preview['confirmed']
    assert not preview['stable']
