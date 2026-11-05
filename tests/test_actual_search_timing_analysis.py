"""Transport receipt time cannot masquerade as a source exposure/decision."""
from pathlib import Path
import math
import runpy
import pytest
analyze=runpy.run_path(str(Path(__file__).parents[1]/'scripts/analyze_actual_search_timing.py'))['analyze']


def row(stream,receipt,**values):
    return dict(event='sample',stream=stream,receipt_ros_seconds=receipt,**values)


def test_separate_raw_eligible_accepted_report_and_verified_end_metrics():
    events=[row('target_detections',101.2,payload=dict(stamp=101.,detections=[dict(label='bottle',search_eligible=False)])),
            row('target_detections',102.2,payload=dict(stamp=102.,detections=[dict(label='bottle',search_eligible=True)])),
            row('mission_status',102.5,payload=dict(started_stamp=100.,state='LOCALIZING',target_accepted_at=102.3)),
            row('mission_result',104.,payload=dict(started_stamp=100.,event='target_reported',target=dict(accepted_at=102.3))),
            row('mission_status',110.,payload=dict(started_stamp=100.,state='COMPLETE',target_reported=True))]
    events.extend(row('cmd_vel',stamp,linear_x=0.,angular_z=0.) for stamp in (109.5,109.8,110.))
    for stamp,x,yaw in ((100.1,0.,0.),(105.,1.,math.pi/2),(109.9,0.,0.)):
        events.append(row('tf:odom_combined->base_footprint',stamp,header_stamp_seconds=stamp,
            publisher_gid='one',translation=[x,0.,0.],rotation_xyzw=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)]))
    state=dict(mission=dict(started_stamp=100.,target_class='bottle',state='COMPLETE',elapsed_seconds=9.9,
        target_reported=True,return_verified=True,target=dict(x=.5,y=.2)),route=[[0.,0.],[0.,0.],[1.,0.],[0.,0.]])
    result=analyze(events,state)
    assert result['first_raw_detection_exposure_seconds']==1.
    assert result['first_explicitly_eligible_detection_exposure_seconds']==2.
    assert result['first_accepted_decision_seconds']==pytest.approx(2.3)
    assert result['first_report_event_received_seconds']==4.
    assert result['full_task_seconds']==9.9
    assert result['final_half_second']['all_recorded_commands_zero'] is True
    assert result['sampled_odom_frame_path_metres']==2.
    assert result['sampled_absolute_turn_radians']==pytest.approx(math.pi)
    assert result['route_diagnostics']['planned_station_revisits']==1


def test_missing_frames_and_report_events_stay_unmeasured():
    state=dict(mission=dict(started_stamp=100.,target_class='bottle',state='SEARCH_INCOMPLETE',elapsed_seconds=10.,
        target_reported=False,return_verified=True,target=None))
    result=analyze([],state)
    assert result['first_raw_detection_exposure_seconds'] is None
    assert result['first_report_event_received_seconds'] is None
    assert result['final_half_second']['command_samples']==0
