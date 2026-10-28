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
