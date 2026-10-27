import math
from ee3033_project.observation_motion import ObservationMotion
from ee3033_project.core import TargetTracker, Observation


def fill(window, start=10., speed=.04, drift=.01):
    for i in range(101):
        t=start+i*.02
        window.add(t, drift*math.sin(i*.25), 0., .01*math.sin(i*.2),
                   speed*(1 if i%2 else -1), .04, 'odom')


def test_balancing_oscillation_passes_but_sustained_navigation_does_not():
    window=ObservationMotion()
    fill(window)
    assert window.evaluate(12.)['verified']
    fill(window, start=13., speed=.12, drift=.01)
    assert not window.evaluate(15.)['verified']


def test_window_is_evaluated_at_capture_not_after_a_later_motion():
    window=ObservationMotion()
    fill(window)
    fill(window, start=12.02, speed=.15, drift=.1)
    assert window.evaluate(11.9)['verified']
    assert not window.evaluate(14.)['verified']
    assert not window.evaluate(20.)['verified']


def test_missing_and_reversed_odometry_cannot_approve_an_observation():
    window=ObservationMotion()
    fill(window)
    window.add(10., 0., 0., 0., 0., 0., 'odom')
    assert not window.evaluate(12.)['verified']


def test_deferred_visible_frames_preserve_but_never_count_toward_confirmation():
    tracker=TargetTracker(frames=4, max_gap=1.2)
    for i in range(3):
        assert tracker.update(Observation(10+i*.4, 1., 2., .9, 'bottle')) is None
        tracker.defer(10+i*.4+.2)
    assert len(tracker.samples)==3
    assert tracker.update(Observation(11.2, 1., 2., .9, 'bottle'))['frames']==4
    tracker.defer(14.)
    assert not tracker.samples
    tracker.update(Observation(15., 1., 2., .9, 'bottle'))
    tracker.update(None)  # A genuine detector miss still resets the track.
    assert not tracker.samples
