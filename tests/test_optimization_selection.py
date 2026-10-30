"""Focused regression checks for lightweight navigation/search enhancements."""
import ast
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import numpy as np
import pytest

from ee3033_project.coverage import CoveragePlanner, Viewpoint
from ee3033_project.frontiers import frontier_goal
from ee3033_project.target_acquisition import SingleTarget
import ee3033_project.target_acquisition as acquisition


def test_frontier_cost_field_is_invoked_once(monkeypatch):
    grid=np.full((45,60), -1, dtype=int)
    grid[5:40,5:35]=0
    original=CoveragePlanner.travel_field
    starts=[]
    def tracked(self, x, y, *args, **kwargs):
        starts.append((x,y))
        return original(self,x,y,*args,**kwargs)
    monkeypatch.setattr(CoveragePlanner,'travel_field',tracked)
    goal,count=frontier_goal(grid,.05,(0,0,0),(.8,1.2,0.),clearance=.14)
    assert count>0 and goal is not None
    assert len(starts)==1


def test_adaptive_fallback_matches_greedy_when_gain_threshold_is_large():
    grid=np.zeros((24,24),dtype=int)
    planner=CoveragePlanner(grid,.1,(0,0,0))
    planner.options=(
        Viewpoint(1.0,1.0,0.,frozenset({(1,1),(1,2)})),
        Viewpoint(1.2,1.0,0.,frozenset({(2,1),(2,2)})),
        Viewpoint(1.0,1.2,0.,frozenset({(3,1),(3,2)})))
    greedy=planner.next_viewpoint(1.,1.,set(),yaw=0.)
    chosen=planner.lookahead(1.,1.,set(),yaw=0.,strategy='adaptive',adaptive_margin=1.)
    assert chosen==greedy


def test_bearing_baseline_ranks_parallax_not_first_valid_radius(monkeypatch):
    class Planner:
        camera_model=None
        safe=np.ones((100,100),dtype=bool)
        fov=1.35
        def world_to_cell(self,x,y):
            return int(y/.05),int(x/.05)
        def in_bounds(self,row,col):
            return 0<=row<100 and 0<=col<100
        def travel_field(self,x,y,deadline=None,cancel=None):
            return np.zeros_like(self.safe,dtype=float),{}
    def visible(_planner,goal,**kwargs):
        return SimpleNamespace(visible=True,reason='ok',support=1.,range_support=1.,camera_origin=goal[:2])
    monkeypatch.setattr(acquisition,'observation_visibility',visible)
    item=SingleTarget()
    item.candidate={'origin':(2.,2.),'bearing':0.,'stamp':1.}
    goal=item.bearing_baseline(Planner(),(2.,2.,0.))
    assert goal is not None
    # Larger safe lateral baseline gives stronger useful parallax than the
    # first .4 m candidate when travel cost is identical.
    assert abs(goal[1]-2.)>=.6-1e-8


def test_frontier_prefers_shorter_safe_route_over_nearer_goal_across_wall():
    grid = np.zeros((65, 80), dtype=int)
    grid[[0, -1], :] = 100
    grid[:, [0, -1]] = 100
    grid[:52, 39:41] = 100  # The only doorway is well below the robot.
    grid[12:17, 55:60] = -1
    grid[35:40, 12:17] = -1
    pose = (3.05, 1.45, 0.)
    goal, count = frontier_goal(grid, .1, (0, 0, 0), pose, clearance=.2)
    assert count == 2 and goal is not None
    assert goal[0] < 3.9
    # The equally sized right-hand frontier is nearer in Euclidean distance,
    # but its safe route must detour through the bottom opening.
    across_wall = (5.35, 1.25)
    assert math.dist(pose[:2], across_wall) < math.dist(pose[:2], goal[:2])
    planner = CoveragePlanner(grid, .1, (0, 0, 0), clearance=.2)
    field, _ = planner.travel_field(*pose[:2])
    assert field[planner.world_to_cell(*goal[:2])] < field[planner.world_to_cell(*across_wall)]


def coverage_tradeoff():
    """Fixed overlap/turning case where the best first action differs by horizon."""
    planner = CoveragePlanner(np.zeros((45, 45), dtype=int), .1, (0, 0, 0))
    planner.options = (
        Viewpoint(1.45, 1.05, 4.71, frozenset({4,6,8,14,15,16,20,23,24,26,28,29,30,32,33,36,37,41,45,46,47,48})),
        Viewpoint(1.05, 2.85, 0., frozenset({0,1,5,6,15,16,22,24,28,33,35,40,41,44,45,46,49})),
        Viewpoint(1.95, 2.25, 3.14, frozenset({4,5,7,12,18,24,27,29,30,31,47})),
        Viewpoint(.95, 1.55, 0., frozenset({1,6,13,17,20,29,31,32,36,44,45,48})),
        Viewpoint(1.85, 2.25, 4.71, frozenset({1,10,12,14,15,16,19,20,22,23,25,27,28,32,33,39,42,43,44,45,49})),
    )
    return planner


def test_adaptive_switches_only_when_route_gain_exceeds_margin():
    planner = coverage_tradeoff()
    pose = (2.05, 2.05, 0.)
    greedy = planner.next_viewpoint(*pose[:2], set(), yaw=pose[2], observation_seconds=.8)
    assert greedy == 4
    assert planner.lookahead(*pose[:2], set(), yaw=pose[2]) == 2
    assert planner.lookahead(*pose[:2], set(), yaw=pose[2], strategy='adaptive') == 2
    assert planner.lookahead(*pose[:2], set(), yaw=pose[2], strategy='adaptive', adaptive_margin=1.) == greedy


@pytest.mark.parametrize('strategy', ['lookahead', 'adaptive'])
def test_search_respects_observed_and_excluded_candidates(strategy):
    planner = coverage_tradeoff()
    all_cells = set().union(*(v.visible for v in planner.options))
    assert planner.lookahead(2.05, 2.05, all_cells, strategy=strategy) is None
    assert planner.lookahead(2.05, 2.05, set(), excluded=range(5), strategy=strategy) is None
    assert planner.lookahead(2.05, 2.05, set(), excluded={2, 4}, strategy=strategy) in {0, 1, 3}


def test_adaptive_deadline_during_beam_returns_greedy_not_partial_alternative(monkeypatch):
    planner = coverage_tradeoff()
    original = planner.travel_field
    calls = []
    def timed(x, y, deadline=None, cancel=None):
        calls.append((x, y))
        if len(calls) == 3:
            raise TimeoutError('Beam budget exhausted')
        return original(x, y, deadline, cancel)
    monkeypatch.setattr(planner, 'travel_field', timed)
    assert planner.lookahead(2.05, 2.05, set(), strategy='adaptive') == 4
    assert len(calls) == 3


def test_adaptive_preserves_native_path_rejection_on_deadline(monkeypatch):
    planner = coverage_tradeoff()
    original = planner.travel_field
    calls = []
    def timed(x, y, deadline=None, cancel=None):
        calls.append((x, y))
        if len(calls) > 1:
            raise TimeoutError('No beam budget left')
        return original(x, y, deadline, cancel)
    monkeypatch.setattr(planner, 'travel_field', timed)
    selected = planner.lookahead(2.05, 2.05, set(), strategy='adaptive', path_lengths={4: math.inf})
    assert selected is not None and selected != 4


def test_adaptive_cancellation_does_not_return_an_action(monkeypatch):
    planner = coverage_tradeoff()
    cancel = threading.Event()
    original = planner.rank_viewpoints
    def ranked(*args, **kwargs):
        result = original(*args, **kwargs)
        cancel.set()
        return result
    monkeypatch.setattr(planner, 'rank_viewpoints', ranked)
    with pytest.raises(RuntimeError, match='canceled'):
        planner.lookahead(2.05, 2.05, set(), strategy='adaptive', cancel=cancel)


@pytest.mark.parametrize('kwargs', [
    {'strategy': 'typo'}, {'adaptive_margin': -.01},
    {'adaptive_margin': 1.01}, {'adaptive_margin': math.nan},
])
def test_invalid_adaptive_settings_are_rejected(kwargs):
    with pytest.raises(ValueError):
        coverage_tradeoff().lookahead(2.05, 2.05, set(), **kwargs)


@pytest.mark.parametrize('strategy, expected', [('greedy', 4), ('lookahead', 2), ('adaptive', 2)])
def test_mission_dispatches_configured_search_strategy(strategy, expected):
    # Execute the production method with its ROS transport boundary stubbed.
    source = Path(__file__).parents[1] / 'src/ee3033_project/ee3033_project/mission.py'
    cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'select_viewpoint')
    namespace = dict(time=time, math=math)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    p = dict(search_strategy=strategy, adaptive_margin=.08, observe_seconds=3.,
             min_observe_seconds=.8, search_linear_speed=.12, search_angular_speed=.4)
    mission = SimpleNamespace(planner=coverage_tradeoff(), seen=set(), attempted=set(),
        failed_options=set(), p=p, plan_cancel=threading.Event(),
        path_nav=SimpleNamespace(server_is_ready=lambda: False),
        get_parameter=lambda name: SimpleNamespace(value=p[name]))
    assert namespace['select_viewpoint'](mission, (2.05, 2.05, 0.)) == expected


def baseline_planner():
    return CoveragePlanner(np.zeros((100, 100), dtype=int), .05, (0, 0, 0), clearance=.15)


def bearing_target():
    item = SingleTarget()
    item.candidate = {'origin': (2., 2.), 'bearing': 0., 'stamp': 1.}
    return item


def test_baseline_path_cost_can_outweigh_larger_parallax(monkeypatch):
    planner = baseline_planner()
    field = np.full(planner.grid.shape, 100.)
    field[planner.world_to_cell(2., 2.4)] = 0.
    monkeypatch.setattr(planner, 'travel_field', lambda *a, **kw: (field, {}))
    monkeypatch.setattr(acquisition, 'observation_visibility', lambda _p, goal, **kw:
        SimpleNamespace(visible=True, reason='ok', support=1., range_support=1., camera_origin=goal[:2]))
    goal = bearing_target().bearing_baseline(planner, (2., 2., 0.))
    assert goal[:2] == pytest.approx((2., 2.4))


@pytest.mark.parametrize('rejection', ['disconnected', 'invisible', 'disallowed', 'visited'])
def test_baseline_rejects_unusable_candidates_without_spending_a_site(monkeypatch, rejection):
    planner = baseline_planner()
    item = bearing_target()
    kwargs = {}
    if rejection == 'disconnected':
        monkeypatch.setattr(planner, 'travel_field', lambda *a, **kw: (np.full(planner.grid.shape, np.inf), {}))
    elif rejection == 'invisible':
        monkeypatch.setattr(acquisition, 'observation_visibility', lambda *a, **kw:
            SimpleNamespace(visible=False, reason='Blocked by wall'))
    elif rejection == 'disallowed':
        kwargs['allowed'] = lambda x, y: False
    else:
        item.visited = {(r, c) for r in range(100) for c in range(100)}
    before = item.visited.copy()
    assert item.bearing_baseline(planner, (2., 2., 0.), **kwargs) is None
    assert item.visited == before


def test_baseline_does_not_invent_missing_camera_origin():
    item = bearing_target()
    del item.candidate['origin']
    assert item.bearing_baseline(baseline_planner(), (2., 2., 0.)) is None
    assert 'camera origin' in item.last_goal_reason


def test_baseline_deadline_keeps_best_completed_visible_candidate(monkeypatch):
    planner = baseline_planner()
    item = bearing_target()
    now = [10.]
    calls = []
    monkeypatch.setattr(acquisition.time, 'monotonic', lambda: now[0])
    def visible(_p, goal, **kwargs):
        calls.append(goal)
        now[0] += .08
        return SimpleNamespace(visible=now[0] < kwargs['deadline'], reason='ok',
            support=1., range_support=1., camera_origin=goal[:2])
    monkeypatch.setattr(acquisition, 'observation_visibility', visible)
    goal = item.bearing_baseline(planner, (2., 2., 0.), deadline=10.2)
    assert len(calls) == 3
    assert goal in calls[:2]  # The third, unfinished check cannot authorize motion.
    assert planner.world_to_cell(*goal[:2]) in item.visited


@pytest.mark.parametrize('stop', ['deadline', 'cancel'])
def test_baseline_stops_during_visibility_without_selecting_earlier_option(monkeypatch, stop):
    planner = baseline_planner()
    item = bearing_target()
    cancel = threading.Event()
    now = [10.]
    monkeypatch.setattr(acquisition.time, 'monotonic', lambda: now[0])
    def visible(_p, goal, **kwargs):
        if stop == 'cancel':
            cancel.set()
        else:
            now[0] = 12.
        return SimpleNamespace(visible=True, reason='ok', support=1., range_support=1., camera_origin=goal[:2])
    monkeypatch.setattr(acquisition, 'observation_visibility', visible)
    assert item.bearing_baseline(planner, (2., 2., 0.), deadline=11., cancel=cancel) is None
    assert not item.visited
