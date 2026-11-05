"""Passive timing summaries keep source freshness separate from delivery gaps."""
import importlib.util
from pathlib import Path


def stats():
    path=Path(__file__).resolve().parents[1]/'scripts/passive_simulation_timing.py'
    spec=importlib.util.spec_from_file_location('passive_simulation_timing',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.StreamStats()


def test_postdated_tf_is_preserved_and_gap_uses_monotonic_receipt_time():
    stream=stats()
    stream.add(100.,100.5,10.)
    stream.add(101.,100.8,11.4)
    result=stream.snapshot()
    assert result['samples']==2
    assert result['minimum_header_age_seconds']==-.5
    assert abs(result['maximum_header_age_seconds']-.2)<1e-8
    assert abs(result['maximum_receive_gap_seconds']-1.4)<1e-8
    assert result['backward_header_stamps']==0


def test_backward_source_stamps_and_empty_stream_are_reported():
    stream=stats();assert stream.snapshot()['minimum_header_age_seconds'] is None
    stream.add(100.,99.9,10.)
    stream.add(101.,99.8,10.1)
    assert stream.snapshot()['backward_header_stamps']==1


def test_headerless_commands_have_receive_gaps_without_fabricated_age():
    stream=stats();stream.add(100.,None,10.);stream.add(101.,None,10.2)
    result=stream.snapshot()
    assert result['samples']==2 and result['last_header_stamp_seconds'] is None
    assert result['minimum_header_age_seconds'] is None and result['maximum_header_age_seconds'] is None


def test_compact_mission_and_detection_fields_keep_latency_evidence():
    path=Path(__file__).resolve().parents[1]/'scripts/passive_simulation_timing.py'
    spec=importlib.util.spec_from_file_location('passive_simulation_timing',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    detection=module.compact_payload('target_detections',{'stamp':12.,'detections':[dict(label='bottle',confidence=.8,search_eligible=False,projection_valid=False,reason_codes=['near_field'],unused_large_field=[1]*100)]})
    assert detection['detections'][0]['search_eligible'] is False and 'unused_large_field' not in detection['detections'][0]
    event=module.compact_payload('mission_result',dict(event='target_reported',started_stamp=10.,target_reported=True,target=dict(x=1.,y=2.,accepted_at=14.,source_frames=[12.],unused=[1]*100)))
    assert 'stamp' not in event and event['target']['accepted_at']==14. and 'unused' not in event['target']
