import math
import threading
import time
import numpy as np
import pytest
from ee3033_project.coverage import CoveragePlanner, Viewpoint, returned_home


def room():
    grid = np.zeros((40,50), dtype=int)
    grid[[0,-1],:] = 100
    grid[:,[0,-1]] = 100
    return grid


def test_route_has_clearance_and_covers_observable_cells():
    planner = CoveragePlanner(room(), 0.1, (0,0,0), spacing=0.8, headings=6)
    route = planner.plan(0.85,0.85)
    assert route and len(route) <= 120
    for v in route:
        assert planner.safe[planner.world_to_cell(v.x,v.y)]
    covered = set().union(*(v.visible for v in route))
    assert len(covered) >= 0.98*len(planner.coverable)
    assert planner.coverable <= planner.target_cells


def test_wall_and_unknown_space_occlude_camera_and_exclude_goals():
    grid = room()
    grid[:,25] = 100
    grid[10:15,5:10] = -1
    planner = CoveragePlanner(grid, 0.1, (0,0,0), spacing=0.8)
    route = planner.plan(0.85,0.85)
    assert all(v.x < 2.5 for v in route)
    assert all(c < 25 for _, c in planner.coverable)
    assert not any(10 <= r < 15 and 5 <= c < 10 for r,c in planner.target_cells)


def test_narrow_passage_excludes_unsafe_room():
    grid = room()
    grid[:,25] = 100
    grid[20,25] = 0
    planner = CoveragePlanner(grid,0.1,(0,0,0),clearance=0.22)
    assert all(v.x < 2.5 for v in planner.plan(0.85,0.85))


def test_world_cell_round_trip_with_rotated_negative_origin():
    planner = CoveragePlanner(room(),0.1,(-2.0,-3.0,math.pi/3))
    for cell in [(2,3),(20,30),(37,47)]:
        assert planner.world_to_cell(*planner.cell_to_world(*cell)) == cell


def test_invalid_start_and_candidate_bounds():
    planner = CoveragePlanner(room(),0.1,(0,0,0))
    with pytest.raises(ValueError):
        planner.plan(-1,0)
    with pytest.raises(ValueError):
        planner.plan(0.05,0.05)
    capped = CoveragePlanner(room(),0.1,(0,0,0), max_candidates=3,max_goals=1)
    assert len(capped.plan(0.85,0.85)) == 1
    assert capped.truncated


def test_return_ignores_heading_by_default_and_checks_distance():
    assert returned_home((1.1,1.1,3.14),(1,1,0),0.2)
    assert not returned_home((1.3,1,0),(1,1,0),0.2)


def test_viewpoint_does_not_see_behind_heading():
    planner = CoveragePlanner(room(),0.1,(0,0,0),fov=0.8)
    planner.plan(2.05,2.05)
    visible = planner.visible(20,20,0)
    assert visible
    assert all(c >= 20 for _,c in visible)


def test_offline_route_keeps_all_online_candidates_including_chosen_views():
    planner = CoveragePlanner(np.zeros((15,15)), .1, (0,0,0), spacing=10, headings=4)
    route = planner.plan(.75,.75)
    assert len(route) == 4
    assert len(planner.options) == 4
    assert all(v in planner.options for v in route)
    assert planner.next_viewpoint(.75,.75,set(),yaw=0) is not None
    assert set().union(*(v.visible for v in planner.options)) == planner.coverable


def test_online_selection_uses_actual_coverage_and_excludes_attempted_views():
    planner = CoveragePlanner(room(),.1,(0,0,0))
    first = Viewpoint(1.,1.,0.,frozenset({(1,1),(1,2)}))
    second = Viewpoint(1.,1.,0.,frozenset({(2,1)}))
    planner.options = (first, second)
    assert planner.next_viewpoint(1,1,set(),yaw=0) == 0
    assert planner.next_viewpoint(1,1,first.visible,yaw=0) == 1
    assert planner.next_viewpoint(1,1,set(),excluded={0},yaw=0) == 1
    assert planner.next_viewpoint(1,1,first.visible|second.visible,yaw=0) is None
    assert planner.next_viewpoint(1,1,set(),excluded={0,1},yaw=0) is None


def test_online_selection_accounts_for_rotation_and_travel_time():
    planner = CoveragePlanner(room(),.1,(0,0,0))
    cells = frozenset({(1,1)})
    planner.options = (Viewpoint(1,1,math.pi,cells), Viewpoint(1,1,0,cells))
    assert planner.next_viewpoint(1,1,set(),yaw=0) == 1
    assert planner.next_viewpoint(1,1,set(),yaw=math.pi) == 0
    planner.options = (Viewpoint(3,1,0,cells), Viewpoint(1.1,1,0,cells))
    assert planner.next_viewpoint(1,1,set(),yaw=0) == 1


def test_candidate_threshold_does_not_discard_views_or_claim_truncation():
    planner = CoveragePlanner(room(),.1,(0,0,0),spacing=.8,max_candidates=1)
    options = planner.prepare(.85,.85)
    assert len(options) > planner.headings
    assert planner.candidate_threshold_exceeded
    assert not planner.truncated
    assert planner.unobservable == planner.target_cells-planner.coverable


def test_candidate_preparation_stops_on_deadline_and_cancellation(monkeypatch):
    planner = CoveragePlanner(room(),.1,(0,0,0))
    with pytest.raises(TimeoutError):
        planner.prepare(.85,.85,deadline=time.monotonic()-1)
    cancel = threading.Event()
    cancel.set()
    with pytest.raises(RuntimeError,match='canceled'):
        planner.prepare(.85,.85,cancel=cancel)
    cancel.clear()
    original = planner.visible
    def stop_after_first_view(*args):
        cancel.set()
        return original(*args)
    monkeypatch.setattr(planner,'visible',stop_after_first_view)
    with pytest.raises(RuntimeError,match='canceled'):
        planner.prepare(.85,.85,cancel=cancel)
    assert planner.options == ()


def test_preparing_again_resets_prior_route_limit_state():
    planner = CoveragePlanner(room(),.1,(0,0,0),max_goals=1)
    planner.plan(.85,.85)
    assert planner.truncated
    planner.prepare(.85,.85)
    assert not planner.truncated


def test_excessive_candidate_count_fails_before_visibility_work(monkeypatch):
    planner = CoveragePlanner(room(),.1,(0,0,0),candidate_limit=2)
    monkeypatch.setattr(planner,'visible',lambda *args: pytest.fail('Candidate limit must run first'))
    with pytest.raises(ValueError,match='Too many search candidates'):
        planner.prepare(.85,.85)
    assert planner.options == ()
