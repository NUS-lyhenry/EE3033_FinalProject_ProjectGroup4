"""Immediate search utility, with existing reachability and recovery limits."""
import math
import threading
import time
import pytest
from test_selected_path_probe import owner, v


def test_greedy_does_not_use_station_batches_or_local_preference():
    # A batch benefits from many local headings, but its next single exposure
    # is less useful than the remote heading.
    m = owner([v(1.05, 1.05, 0., cells=range(5)),
               v(1.05, 1.05, .1, cells=range(5, 10)),
               v(1.05, 1.05, .2, cells=range(10, 15)),
               v(1.65, 1.05, cells=range(20, 40))])
    m.path_nav.server_is_ready = lambda: False
    expected = m.planner.rank_viewpoints(1.05, 1.05, set(), yaw=0.,
                                         observation_seconds=.8)[0][1]
    assert expected == 3
    assert m.planner.select_station_view(1.05, 1.05, set(), yaw=0., strategy='greedy') == 0
    m.planner.select_station_view = lambda *a, **kw: pytest.fail('Greedy used station batch')
    assert m.select_viewpoint((1.05, 1.05, 0.)) == expected


def test_same_site_cost_is_direct_rotation_without_tiny_translation():
    m = owner([v(1.05, 1.15, -.05), v(1.35, 1.05)])
    score, index = m.planner.rank_viewpoints(1.05, 1.05, set(), yaw=0.,
                                             observation_seconds=.8)[0]
    assert index == 0
    assert score == pytest.approx(1 / (.8 + .05/.4 + 1.))


@pytest.mark.parametrize('budget', ['attempts', 'time'])
@pytest.mark.parametrize('strategy', ['greedy', 'adaptive'])
def test_exhausted_local_site_does_not_hide_remote_alternative(budget, strategy):
    m = owner([v(1.05, 1.05, cells=range(20)), v(1.65, 1.05, cells=range(20, 22))])
    m.p['search_strategy'] = strategy
    m.path_nav.server_is_ready = lambda: False
    m.station_attempts = 6 if budget == 'attempts' else 0
    m.station_started = time.monotonic() - (25. if budget == 'time' else 0.)
    assert m.select_viewpoint((1.05, 1.05, 0.)) == 1


def test_failed_localization_site_is_excluded_before_utility_comparison():
    m = owner([v(1.05, 1.05, cells=range(30)), v(1.65, 1.05, cells=range(30, 32))])
    m.failed_localization_positions = [(1.05, 1.05)]
    assert m.select_viewpoint((1.05, 1.05, 0.)) == 1


def test_greedy_retains_soft_revisit_cost_without_blacklisting():
    m = owner([v(1.65, .85), v(1.65, 1.25, cells=((1, 2),))])
    m.previous_stations = [m.planner.station_key(1.65, .85)]
    assert m.select_viewpoint((1.05, 1.05, 0.)) == 1
    m.seen = {(1, 2)}
    assert m.select_viewpoint((1.05, 1.05, 0.)) == 0


def test_predictions_do_not_credit_observed_cells_and_cancellation_stops_selection():
    m = owner([v(1.65, 1.05, cells=range(5))])
    m.select_viewpoint((1.05, 1.05, 0.))
    assert m.seen == set()
    m.plan_cancel = threading.Event()
    m.plan_cancel.set()
    with pytest.raises(RuntimeError, match='canceled'):
        m.select_viewpoint((1.05, 1.05, 0.))


def test_native_unreachable_heading_is_not_selected():
    m = owner([v(1.65, 1.05), v(2.25, 1.05, cells=((1, 2),))])
    ranked = m.planner.rank_viewpoints(1.05, 1.05, set(), yaw=0.,
                                      path_lengths={0: math.inf})
    assert [index for _, index in ranked] == [1]
