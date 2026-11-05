"""Motion regressions that do not launch ROS or a physical/simulated robot."""
import copy
import json
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from navigation_motion_metrics import MotionMetrics, configure_speed, arrival_limits, select_navigation_phase
from navigation_benchmark_geometry import physically_reachable, geometry_profile
from simulation_scenarios import validate_scenario


@pytest.mark.parametrize('controller,filename', [
    ('rpp', 'navigation.yaml'), ('rpp-native', 'navigation_rpp_native.yaml'),
    ('dwb', 'navigation_dwb.yaml'),
])
@pytest.mark.parametrize('profile,linear,angular', [('original', .12, .4), ('fast', .16, .6)])
def test_speed_profiles_update_real_controller_caps_without_changing_safety(controller, filename, profile, linear, angular):
    config = yaml.safe_load((ROOT / 'src/ee3033_project/config' / filename).read_text())
    previous = copy.deepcopy(config)
    assert configure_speed(config, controller, profile) == dict(max_linear=linear, max_angular=angular)
    follow = config['controller_server']['ros__parameters']['FollowPath']
    if controller == 'dwb':
        assert [follow[key] for key in ('max_vel_x', 'max_speed_xy', 'max_vel_theta')] == [linear, linear, angular]
        assert follow['BaseObstacle.scale'] == previous['controller_server']['ros__parameters']['FollowPath']['BaseObstacle.scale']
    else:
        assert follow['desired_linear_vel'] == linear
        assert follow['rotate_to_heading_angular_vel'] == angular
        assert follow['use_collision_detection'] is True
        assert follow['max_angular_accel'] == .8
        if controller == 'rpp':
            assert follow['max_linear_vel'] == linear and follow['max_angular_vel'] == angular
    for name in ('local_costmap', 'global_costmap'):
        assert config[name] == previous[name]
    assert config['controller_server']['ros__parameters']['home_goal_checker'] == previous['controller_server']['ros__parameters']['home_goal_checker']


def test_actual_final_commands_reveal_residual_curvature_slowdown_below_deadband():
    metrics = MotionMetrics(dict(max_linear=.16, max_angular=.6))
    # A native .035 floor can still be uniformly scaled to .0175 by a turn cap.
    metrics.observe('raw', 0., .035, 1.2)
    metrics.observe('final', 0., .0175, .6)
    metrics.observe('odometry', 0., 0., .6)
    result = metrics.summary(.2)
    assert result['final_caps_respected']
    final = result['streams']['final']
    assert final['below_linear_deadband_seconds'] == pytest.approx(.2)
    assert final['linear_distance_integral'] == pytest.approx(.0035)
    odom = result['streams']['odometry']
    assert odom['translation_commanded_but_stationary_seconds'] == pytest.approx(.2)
    assert odom['mean_abs_linear'] == 0.
    assert result['streams']['raw']['cap_violations'] == 1


def test_cap_diagnostics_cannot_pass_without_final_samples_or_hide_an_excess():
    metrics = MotionMetrics(dict(max_linear=.12, max_angular=.4))
    assert not metrics.summary(0.)['final_caps_respected']
    metrics.observe('final', 0., .12, -.4)
    assert metrics.summary(.1)['final_caps_respected']
    assert not metrics.summary(.1)['complete']
    metrics.observe('final', .1, .121, .4)
    assert not metrics.summary(.2)['final_caps_respected']


def test_metrics_use_elapsed_time_and_exclude_stale_gaps():
    metrics = MotionMetrics(dict(max_linear=.12, max_angular=.4))
    metrics.observe('odometry', 0., .1, 0.)
    metrics.observe('odometry', .2, 0., .2)
    metrics.observe('odometry', .3, 0., 0.)
    metrics.observe('odometry', 10., 0., 0.)
    result = metrics.summary(10.1)['streams']['odometry']
    assert result['sampled_seconds'] == pytest.approx(.4)
    assert result['translation_seconds'] == pytest.approx(.2)
    assert result['rotation_only_seconds'] == pytest.approx(.1)
    assert result['stopped_seconds'] == pytest.approx(.1)
    assert result['mean_abs_linear'] == pytest.approx(.05)


def test_short_approach_and_home_acceptance_use_real_special_goal_tolerances():
    assert arrival_limits('navigate', 'approach') == (.025, .10)
    assert arrival_limits('navigate', 'home') == (.06, None)
    assert arrival_limits('waypoint', 'standard') == (.15, .15)


def test_motion_case_geometry_includes_short_move_and_measured_34cm_opening():
    folder = ROOT / 'validation/navigation_cases'
    radius = geometry_profile('measured')['footprint_radius']
    for name in ('motion_straight', 'motion_short_approach', 'motion_opening34'):
        case = json.loads((folder / (name + '.json')).read_text())
        assert validate_scenario(case['scenario'])
        assert physically_reachable(case['scenario'], case['goal'], radius=radius)
    short = json.loads((folder / 'motion_short_approach.json').read_text())
    assert .025 < short['goal'][0] < .3
    opening = json.loads((folder / 'motion_opening34.json').read_text())
    assert opening['door_width'] == .34
    assert opening['scenario']['walls'][5][0][1] - opening['scenario']['walls'][4][1][1] == .34


def test_six_free_5cm_cells_match_physical_30cm_opening_in_both_directions():
    import numpy as np
    folder = ROOT / 'validation/navigation_cases'
    radius = geometry_profile('measured')['footprint_radius']
    for name in ('motion_six_cells_forward', 'motion_six_cells_reverse'):
        case = json.loads((folder / (name + '.json')).read_text())
        data = case['map']
        assert data['resolution'] == .05 and case['door_width'] == .30
        assert case['scenario']['walls'][5][0][1] - case['scenario']['walls'][4][1][1] == .30
        assert physically_reachable(case['scenario'], case['goal'], start=case['start_pose'], radius=radius)
        grid = np.array(data['cells']).reshape(data['height'], data['width'])
        ys = data['origin'][1] + (np.arange(data['height']) + .5) * .05
        col = int((.7 - data['origin'][0]) / .05)
        inside = (ys > -1.) & (ys < 2.)
        free_rows = np.flatnonzero((grid[:, col] == 0) & inside)
        assert len(free_rows) == 6
        assert np.all(np.diff(free_rows) == 1)
        assert ys[free_rows[0]] - .025 == pytest.approx(-.15)
        assert ys[free_rows[-1]] + .025 == pytest.approx(.15)


def test_l90_corridor_has_six_free_cells_in_each_leg_and_full_footprint_route():
    import numpy as np
    from ee3033_project.motion_safety import polyline_map_clearance
    folder = ROOT / 'validation/navigation_cases'
    for name in ('motion_six_l90_forward', 'motion_six_l90_reverse'):
        case = json.loads((folder / (name + '.json')).read_text())
        data = case['map']
        grid = np.array(data['cells']).reshape(data['height'], data['width'])
        ox, oy, _ = data['origin']
        assert np.count_nonzero(grid[:, int((-.3-ox)/.05)] == 0) == 6
        assert np.count_nonzero(grid[int((.6-oy)/.05), :] == 0) == 6
        route = [[-.3, 0.], [.6, 0.], [.6, .9]]
        if name.endswith('reverse'):
            route.reverse()
        assert polyline_map_clearance(grid, .05, data['origin'], route, radius=.116801).allowed
        assert physically_reachable(case['scenario'], case['goal'], start=case['start_pose'], radius=.116801)


def test_in_place_180_cases_keep_both_yaw_signs_and_full_circle_clearance():
    import math
    import numpy as np
    from ee3033_project.motion_safety import swept_map_clearance
    for label, sign in [('positive', 1), ('negative', -1)]:
        case = json.loads((ROOT / 'validation/navigation_cases' / ('motion_six_turn180_' + label + '.json')).read_text())
        data = case['map']
        grid = np.array(data['cells']).reshape(data['height'], data['width'])
        assert case['goal'][:2] == case['start_pose'][:2]
        assert case['goal'][2] == sign * math.pi
        assert swept_map_clearance(grid, .05, data['origin'], case['start_pose'], 0., sign*.4, math.pi/.4, radius=.116801).allowed


def test_oscillation_metrics_preserve_turn_direction_changes_and_total_rotation():
    metrics = MotionMetrics(dict(max_linear=.12, max_angular=.4))
    metrics.observe('final', 0., 0., .4)
    metrics.observe('final', .2, 0., -.4)
    metrics.observe('final', .4, 0., 0.)
    final = metrics.summary(.5)['streams']['final']
    assert final['angular_sign_changes'] == 1
    assert final['positive_angular_seconds'] == pytest.approx(.2)
    assert final['negative_angular_seconds'] == pytest.approx(.2)
    assert final['angular_distance_integral'] == pytest.approx(.16)
    assert final['signed_angular_integral'] == pytest.approx(0.)


def test_rotated_translated_frame_fixture_preserves_physical_l_route_and_map_clearance():
    import numpy as np
    from navigation_benchmark import map_to_world
    from ee3033_project.motion_safety import polyline_map_clearance
    folder = ROOT / 'validation/navigation_cases'
    original = json.loads((folder / 'motion_six_l90_forward.json').read_text())
    rotated = json.loads((folder / 'motion_six_l90_rotated_frame.json').read_text())
    assert map_to_world(rotated['start_pose'], rotated['map_from_odom']) == pytest.approx(original['start_pose'])
    assert map_to_world(rotated['goal'], rotated['map_from_odom']) == pytest.approx(original['goal'])
    route = np.array([[1.2, -.7], [1.2, .2], [.3, .2]])
    data = rotated['map']
    grid = np.array(data['cells']).reshape(data['height'], data['width'])
    assert polyline_map_clearance(grid, .05, data['origin'], route, radius=.116801).allowed


def test_navigation_phase_matches_production_mission_and_mapping_workflows():
    assert select_navigation_phase('navigate','known')=='mission'
    assert select_navigation_phase('automap','slam')=='mapping'
    assert select_navigation_phase('waypoint','known')=='mapping'
    assert select_navigation_phase('navigate','slam')=='mapping'
    assert select_navigation_phase('navigate','known','mapping')=='mapping'
    with pytest.raises(ValueError):select_navigation_phase('navigate','slam','mission')
    with pytest.raises(ValueError):select_navigation_phase('waypoint','known','mission')
