import pytest
from ee3033_project.capture_covariance import capture_covariance


def test_future_pose_is_diagnostic_only_and_latest_past_pose_is_selected():
    value, info = capture_covariance([(9.5, (.1, .2)), (10.1, (9., 9.)),
                                      (9.9, (.01, .02))], 10.)
    assert value == (.01, .02)
    assert info['selected_pose_stamp'] == 9.9
    assert info['pose_age_seconds'] == pytest.approx(.1)
    assert info['latest_pose_age_seconds'] == pytest.approx(-.1)


@pytest.mark.parametrize('history,reason', [([], 'missing'),
    ([(8.9, (.01, .01))], 'stale'), ([(10.1, (.01, .01))], 'future_only')])
def test_missing_stale_and_future_covariance_remain_rejected(history, reason):
    value, info = capture_covariance(history, 10.)
    assert value is None and not info['accepted']
    assert info['reason'] == reason


def test_sparse_stationary_amcl_does_not_gain_an_unverified_age_extension():
    history = [(10., (.01, .01))]
    assert capture_covariance(history, 11.)[0] == (.01, .01)
    value, info = capture_covariance(history, 11.001)
    assert value is None and info['reason'] == 'stale'
    assert info['latest_pose_age_seconds'] == pytest.approx(1.001)
