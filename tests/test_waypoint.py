import math
import numpy as np
import pytest
from ee3033_project.waypoint import validate_waypoint,waypoint_goal


def room():
    grid=np.zeros((60,80),dtype=int);grid[[0,-1],:]=100;grid[:,[0,-1]]=100
    return grid


def test_reachable_waypoint_preserves_both_position_and_facing():
    target=(5.,3.,math.pi/2)
    goal,groups,final=waypoint_goal(room(),.1,(0,0,0),(2,3,0),target)
    assert final and goal==target and groups==0


def test_unknown_waypoint_uses_known_free_approach_instead_of_crossing_unknown():
    grid=np.full((60,80),-1);grid[5:55,5:40]=0
    target=(6.,3.,1.)
    goal,groups,final=waypoint_goal(grid,.1,(0,0,0),(2,3,0),target,clearance=.14)
    assert goal is not None and not final and groups>0
    assert 2<goal[0]<4 and grid[math.floor(goal[1]/.1),math.floor(goal[0]/.1)]==0


@pytest.mark.parametrize('target',[(0.,0.,0.),(-1.,2.,0.),(2.,3.,math.nan)])
def test_obstacles_outside_map_and_nonfinite_waypoints_are_rejected(target):
    with pytest.raises(ValueError):validate_waypoint(room(),.1,(0,0,0),target)


def test_failed_final_goal_is_deferred_but_a_visited_approach_does_not_prevent_final_facing():
    target=(5.,3.,1.)
    assert waypoint_goal(room(),.1,(0,0,0),(2,3,0),target,failed=[target[:2]])==(None,1,False)
    assert waypoint_goal(room(),.1,(0,0,0),(2,3,0),target,visited=[target[:2]])[2]


def test_waypoint_coordinates_respect_a_rotated_map_origin():
    target=(-3.,5.,1.)
    assert validate_waypoint(room(),.1,(0,0,math.pi/2),target)==target
