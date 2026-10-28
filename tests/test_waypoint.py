import math
import numpy as np
import pytest
from ee3033_project.waypoint import validate_waypoint,waypoint_goal
from ee3033_project.motion_safety import FOOTPRINT_RADIUS


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


def test_free_target_cell_beside_unknown_space_is_not_a_final_leg():
    from ee3033_project.motion_safety import polyline_map_clearance
    grid=np.full((60,80),-1);grid[5:55,5:40]=0
    target=(4.-.8*FOOTPRINT_RADIUS,3.05,1.)
    assert grid[30,math.floor(target[0]/.1)]==0
    assert not polyline_map_clearance(grid,.1,(0,0,0),[target[:2]]).allowed
    goal,groups,final=waypoint_goal(grid,.1,(0,0,0),(2.,3.,0.),target,clearance=.14)
    assert not final and goal is not None and groups>0
    assert goal[:2]!=target[:2]
    assert polyline_map_clearance(grid,.1,(0,0,0),[goal[:2]]).allowed


def test_free_target_cell_with_wall_in_footprint_is_not_silently_snapped():
    grid=room();grid[30,40]=100
    target=(4.-.8*FOOTPRINT_RADIUS,3.05,1.)
    assert validate_waypoint(grid,.1,(0,0,0),target)==target
    assert waypoint_goal(grid,.1,(0,0,0),(2.,3.,0.),target,clearance=.14)==(None,0,False)


def test_native_mask_rejects_final_waypoint_without_changing_requested_position():
    grid=room();mask=np.ones(grid.shape,bool);mask[:,40]=False
    target=(5.,3.,1.)
    assert waypoint_goal(grid,.1,(0,0,0),(2.,3.,0.),target,
                         clearance=.14,safe_mask=mask)==(None,0,False)


def test_safe_rotated_world_waypoint_preserves_exact_xy_and_final_yaw():
    target=(-3.,5.,1.)
    goal,groups,final=waypoint_goal(room(),.1,(0,0,np.pi/2),(-3.,2.,0.),target,clearance=.14)
    assert final and goal==target and groups==0
