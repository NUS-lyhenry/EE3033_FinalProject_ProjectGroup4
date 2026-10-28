"""The DWB arrival critic must not deadlock a stricter observation checker."""
from pathlib import Path

import yaml


def test_dwb_switches_to_heading_only_inside_every_goal_checker_xy_tolerance():
    path=Path(__file__).parents[1]/'src/ee3033_project/config/navigation.yaml'
    controller=yaml.safe_load(path.read_text())['controller_server']['ros__parameters']
    dwb_xy=controller['FollowPath']['xy_goal_tolerance']
    tolerances=[controller[name]['xy_goal_tolerance']
                for name in controller['goal_checker_plugins']]
    assert 0<dwb_xy<min(tolerances)
    observation=controller['observation_goal_checker']
    assert observation['xy_goal_tolerance']<=controller['goal_checker']['xy_goal_tolerance']
    assert observation['xy_goal_tolerance']<=controller['home_goal_checker']['xy_goal_tolerance']
