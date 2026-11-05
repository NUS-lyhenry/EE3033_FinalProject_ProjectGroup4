import time
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import pytest
from ee3033_project.coverage import CoveragePlanner, Viewpoint
from ee3033_project.navigation_supervision import navigation_budget


def planner(options):
    p = CoveragePlanner(np.zeros((30, 40), dtype=int), .1, (0, 0, 0), spacing=.65)
    p.options = tuple(options)
    return p


def view(x, yaw, cells, y=1.):
    return Viewpoint(x, y, yaw, frozenset(cells))


def choose(p, **kw):
    return p.select_station_view(1.05, 1.05, (), yaw=0., deadline=time.monotonic()+2., **kw)


def test_direct_rotation_does_not_charge_departure_heading_or_tiny_travel():
    p = planner([view(1.05, -.05, {(1, 1)}, y=1.10), view(1.35, 0., {(1, 2)})])
    assert choose(p) == 0


def test_station_batch_values_all_distinct_headings_before_leaving():
    p = planner([view(1.05, 0., {(1, 1)}), view(1.05, .3, {(1, 2)}),
                 view(1.65, 0., {(1, 3), (1, 4)})])
    assert choose(p) in (0, 1)
    assert p.select_station_view(1.05, 1.05, {(1, 1)}, excluded={0}, yaw=0.,
                                 deadline=time.monotonic()+2.) == 1


@pytest.mark.parametrize('budget', [dict(station_views=6), dict(station_seconds=25.)])
def test_dwell_budget_forces_new_station_when_available(budget):
    p = planner([view(1.05, 0., {(1, 1)}), view(1.65, 0., {(1, 2)})])
    assert choose(p, **budget) == 1


def test_recent_failed_localization_positions_are_excluded_only_locally():
    p = planner([view(1.05, 0., {(1, 1)}), view(1.35, 0., {(1, 2)})])
    assert choose(p, failed_positions=[(1.05, 1.05)]) == 1
    assert choose(p, failed_positions=[(1.05, 1.05), (1.35, 1.)]) is None


def test_recent_station_adds_cost_without_permanent_blacklist():
    p = planner([view(1.55, 0., {(1, 1)}, y=.85), view(1.55, 0., {(1, 2)}, y=1.25)])
    key = p.station_key(1.55, .85)
    assert choose(p, previous_stations=[key]) == 1
    # Return cost also applies to sites left earlier than the last two visits.
    assert choose(p, previous_stations=[key, (0, 0), (0, 1)]) == 1
    assert p.select_station_view(1.05, 1.05, {(1, 2)}, excluded={1},
        yaw=0., previous_stations=[key], deadline=time.monotonic()+2.) == 0


def test_station_selection_never_credits_predicted_observations():
    p = planner([view(1.05, 0., {(1, 1)})]); seen = set()
    assert p.select_station_view(1.05, 1.05, seen, yaw=0.) == 0
    assert seen == set()
    assert p.select_station_view(1.05, 1.05, seen, excluded={0}, yaw=0.) is None


def test_scan_is_one_twelve_second_attempt_without_reverse():
    b, reverse = navigation_budget(10., 75., 2, True, '/trees/navigate_search_scan.xml')
    assert b.deadline == 22. and not reverse
    assert b.next(11., True, allow_reverse=reverse) is None
    short, _ = navigation_budget(10., 4., 2, True, 'navigate_search_scan.xml')
    assert short.deadline == 14.
    ordinary, reverse = navigation_budget(10., 75., 2, True, 'navigate_supervised.xml')
    assert ordinary.deadline == 85. and reverse
    assert ordinary.next(11., True) == 'refresh'
    assert ordinary.next(12., True) == 'reverse'


def test_scan_tree_uses_existing_ordinary_goal_checker():
    path = Path(__file__).parents[1]/'src/ee3033_project/behavior_trees/navigate_search_scan.xml'
    tree = ET.parse(path)
    assert tree.find('.//FollowPath').attrib['goal_checker_id'] == 'goal_checker'
    assert tree.find('.//BackUp') is None


def test_cancellation_does_not_become_a_fallback_navigation_choice():
    import threading
    p = planner([view(1.05, 0., {(1, 1)})])
    cancel = threading.Event(); cancel.set()
    with pytest.raises(RuntimeError, match='canceled'):
        choose(p, cancel=cancel)


def test_first_scan_must_fit_remaining_dwell_including_frame_collection():
    p = planner([view(1.05, 0., {(1, 1)})])
    # Even an aligned view needs 0.8 seconds observation + one second overhead.
    assert choose(p, station_seconds=24.) is None
    assert choose(p, station_seconds=23.) == 0
    p.options = (view(1.05, 2., {(1, 1)}),)
    assert choose(p, station_seconds=20.) is None


@pytest.mark.parametrize('strategy, expected', [('greedy', 4), ('lookahead', 2), ('adaptive', 2)])
def test_station_selector_preserves_explicit_strategy_semantics(strategy, expected):
    from test_optimization_selection import coverage_tradeoff
    p = coverage_tradeoff()
    assert p.select_station_view(2.05, 2.05, set(), yaw=0., strategy=strategy,
                                 deadline=time.monotonic()+1.5) == expected
    if strategy == 'adaptive':
        assert p.select_station_view(2.05, 2.05, set(), yaw=0., strategy=strategy,
                                     adaptive_margin=1.) == 4


def test_deadline_fallback_still_excludes_failed_positions_and_exhausted_local_site(monkeypatch):
    p = planner([view(1.05, 0., {(1, 1)}), view(1.35, 0., {(1, 2)}),
                 view(1.65, 0., {(1, 3)})])
    original = p.travel_field
    calls = []
    def field(x, y, deadline=None, cancel=None):
        calls.append((x,y))
        if len(calls) > 1:
            raise TimeoutError('Beam exhausted')
        return original(x,y,deadline,cancel)
    monkeypatch.setattr(p, 'travel_field', field)
    # Local first heading cannot fit, and index 1 is in a failed-site radius.
    assert choose(p, station_seconds=24., failed_positions=[(1.35,1.)]) == 2


def test_native_rejection_is_honored_for_same_site_rotations():
    p = planner([view(1.05, 0., {(1, 1)}), view(1.35, 0., {(1, 2)})])
    assert choose(p, path_lengths={0:float('inf')}) == 1
