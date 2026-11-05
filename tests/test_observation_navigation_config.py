"""RPP reads each goal checker's tolerances instead of a second DWB XY latch."""
from pathlib import Path
import yaml


def test_observation_checker_does_not_inherit_an_incompatible_controller_xy_latch():
    config=Path(__file__).parents[1]/'src/ee3033_project/config/navigation.yaml'
    controller=yaml.safe_load(config.read_text())['controller_server']['ros__parameters']
    follower=controller['FollowPath']
    assert follower['plugin']=='ee3033_navigation::RegulatedPathFollower'
    assert 'xy_goal_tolerance' not in follower
    assert follower['use_collision_detection'] and not follower['allow_reversing']
    observation=controller['observation_goal_checker']
    assert observation['xy_goal_tolerance']==.10
    assert observation['xy_goal_tolerance']<=controller['goal_checker']['xy_goal_tolerance']
    assert observation['yaw_goal_tolerance']==.03
    assert controller['home_goal_checker']['yaw_goal_tolerance']>3.14


def test_home_controller_leaves_margin_for_independent_arrival_hold():
    root=Path(__file__).parents[1]
    for name in ['navigation.yaml','navigation_rpp_native.yaml']:
        c=yaml.safe_load((root/'src/ee3033_project/config'/name).read_text())['controller_server']['ros__parameters']
        # Nav2 terminal success must leave room for a fresh AMCL correction;
        # matching the outer verifier threshold caused a false arrival failure.
        assert 0<c['home_goal_checker']['xy_goal_tolerance']<c['goal_checker']['xy_goal_tolerance']
