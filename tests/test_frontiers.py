import numpy as np
import pytest
from ee3033_project.frontiers import frontier_goal
from ee3033_project.coverage import CoveragePlanner


def test_approach_stays_in_known_reachable_clear_space():
    grid=np.full((50,60),-1,dtype=int)
    grid[5:45,5:30]=0
    grid[5:45,5]=100
    goal,count=frontier_goal(grid,0.05,(-1,-1,0),(0,0,0))
    assert goal is not None and count>0
    planner=CoveragePlanner(grid,0.05,(-1,-1,0),clearance=0.22)
    assert planner.safe[planner.world_to_cell(*goal[:2])]


def test_enclosed_fully_mapped_room_has_no_frontiers():
    grid=np.full((40,40),100)
    grid[1:-1,1:-1]=0
    assert frontier_goal(grid,0.1,(0,0,0),(1,1,0)) == (None,0)


def test_frontier_failures_suppress_nearby_goals():
    grid=np.full((40,40),-1)
    grid[1:39,1:25]=0
    first,count=frontier_goal(grid,0.1,(0,0,0),(1,1,0))
    assert first and count
    second,_=frontier_goal(grid,0.1,(0,0,0),(1,1,0),failed=[first[:2]])
    if second:
        assert np.linalg.norm(np.array(second[:2])-first[:2])>=0.4


def test_frontier_outside_map_fails_closed():
    with pytest.raises(ValueError):
        frontier_goal(np.zeros((10,10)),0.1,(0,0,0),(-2,-2,0))
