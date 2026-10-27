import math
import pytest
from ee3033_project.target_estimate import LiveTargetEstimate


def detection(stamp=10., x=1., sigma=1., label='bottle'):
    return dict(stamp=stamp, x=x, y=2., sigma=sigma, confidence=.8, label=label)


def test_high_uncertainty_is_displayed_and_precise_observations_get_more_weight():
    tracker=LiveTargetEstimate()
    tracker.update([detection(sigma=20.)],10.,'bottle')
    assert tracker.snapshot(10.)['sigma']==pytest.approx(20.)
    tracker=LiveTargetEstimate()
    tracker.update([detection(10.,x=0.,sigma=1.)],10.,'bottle')
    tracker.update([detection(10.01,x=4.,sigma=2.)],10.01,'bottle')
    result=tracker.snapshot(10.01)
    assert .79<result['x']<.82
    assert result['sigma']>=1.
    assert result['confirmed'] is False


def test_repeated_frames_do_not_count_and_correlated_frames_keep_uncertainty_floor():
    tracker=LiveTargetEstimate()
    for _ in range(10):tracker.update([detection()],10.,'bottle')
    assert tracker.snapshot(10.)['samples']==1
    for i in range(1,21):tracker.update([detection(10+i*.1)],10+i*.1,'bottle')
    assert tracker.snapshot(12.)['sigma']==pytest.approx(1.)


def test_rolling_window_stale_label_and_geometry_resets():
    tracker=LiveTargetEstimate()
    tracker.update([detection()],10.,'bottle')
    assert tracker.snapshot(13.)['stale']
    assert tracker.snapshot(41.) is None
    tracker.update([detection(20.,x=5.)],20.,'bottle')
    assert tracker.snapshot(20.)['x']==5.
    tracker.update([detection(20.1,label='cup',x=7.)],20.1,'cup')
    assert tracker.snapshot(20.1)['x']==7.
    tracker.reset()
    assert tracker.snapshot(20.1) is None


@pytest.mark.parametrize('overrides',[{'x':math.nan},{'sigma':0.},{'sigma':math.inf},{'stamp':3.},{'label':'cup'}])
def test_invalid_or_old_projections_do_not_fabricate_a_map_position(overrides):
    tracker=LiveTargetEstimate();d=detection();d.update(overrides)
    tracker.update([d],10.,'bottle')
    assert tracker.snapshot(10.) is None
