"""Measured events stay distinct from planner visibility and accepted estimates."""
from pathlib import Path
import runpy

helpers = runpy.run_path(str(Path(__file__).parents[1]/'scripts/compare_search_strategies.py'))
summarize = helpers['summarize_events']


def event(topic, stamp, **message):
    return dict(topic='/project/'+topic, stamp=stamp, message=message)


def test_native_detection_lock_and_report_have_separate_timestamps():
    events = [event('mission_status', 100., phase='mission', started_stamp=100., target_class='bottle'),
              event('target_detections', 101., detections=[dict(label='cup', stamp=100.5),
                  dict(label='bottle', stamp=99.9), dict(label='bottle', stamp=100.8)]),
              event('mission_status', 102., phase='mission', started_stamp=100.,
                  target_detected=True, detection_stamp=101.7),
              event('mission_result', 103., started_stamp=100., event='target_accepted'),
              event('mission_result', 104., started_stamp=100., event='target_reported'),
              event('mission_status', 105., phase='mission', started_stamp=100.,
                  state='COMPLETE', target_reported=True, return_verified=True, observed_fraction=.8)]
    result = summarize(events)
    assert round(result['first_target_detection_seconds'], 6) == .8
    assert round(result['first_target_lock_seconds'], 6) == 1.7
    assert result['first_target_report_seconds'] == 4.
    assert result['return_verified'] is True


def test_absent_or_stale_detection_never_invents_detection_or_report():
    events = [event('mission_status', 100., phase='mission', started_stamp=100., target_class='bottle'),
              event('target_detections', 105., detections=[dict(label='bottle', stamp=101.)]),
              event('mission_status', 106., phase='mission', started_stamp=100., state='SEARCH_INCOMPLETE')]
    result = summarize(events)
    assert result['first_target_detection_seconds'] is None
    assert result['first_target_report_seconds'] is None
    assert result['final_state'] == 'SEARCH_INCOMPLETE'


def test_second_mission_does_not_supply_first_missions_result():
    events = [event('mission_status', 100., phase='mission', started_stamp=100., target_class='bottle'),
              event('mission_status', 200., phase='mission', started_stamp=200., target_class='bottle'),
              event('mission_result', 204., started_stamp=200., event='target_reported')]
    assert summarize(events)['first_target_report_seconds'] is None


def test_matrix_requires_all_paired_scene_and_target_cases():
    cases = helpers['matrix']()
    assert len(cases) == 24
    assert {c['status'] for c in cases} == {'not_run'}
    assert len({(c['scene'], c['target'], c['strategy']) for c in cases}) == 24


def test_total_task_uses_mission_completion_and_excludes_mapping():
    events = [event('mission_status', 50., phase='mapping', started_stamp=10., elapsed_seconds=40.),
              event('mission_status', 100., phase='mission', started_stamp=100., target_class='bottle'),
              event('mission_status', 112., phase='mission', started_stamp=100.,
                    state='COMPLETE', target_reported=True, target=dict(x=.5,y=.2),
                    return_verified=True, elapsed_seconds=12.)]
    result = summarize(events)
    assert result['task_seconds'] == 12.
    assert result['task_complete'] is True
    assert result['started_stamp'] == 100.
