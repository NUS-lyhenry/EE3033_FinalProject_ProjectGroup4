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


def test_slanted_frontier_is_not_discarded_as_single_cell_noise():
    grid=np.full((60,60),100)
    for row in range(1,59):
        for col in range(1,59):
            grid[row,col]=0 if row+col<60 else -1
    goal,count=frontier_goal(grid,.05,(0,0,0),(.8,.8,0))
    assert count==1 and goal is not None
    planner=CoveragePlanner(grid,.05,(0,0,0),clearance=.22)
    assert planner.safe[planner.world_to_cell(*goal[:2])]


def test_frontier_failure_excludes_the_approach_but_preserves_nearby_alternatives():
    grid=np.full((40,40),-1)
    grid[1:39,1:25]=0
    first,count=frontier_goal(grid,0.1,(0,0,0),(1,1,0))
    assert first and count
    second,_=frontier_goal(grid,0.1,(0,0,0),(1,1,0),failed=[first[:2]])
    if second:
        assert np.linalg.norm(np.array(second[:2])-first[:2])>=0.2


def test_native_costmap_mask_rejects_a_grid_safe_frontier_goal():
    grid=np.full((50,60),-1,dtype=int)
    grid[5:45,5:35]=0
    first,_=frontier_goal(grid,.05,(0,0,0),(.8,1.2,0),clearance=.14)
    assert first is not None
    planner=CoveragePlanner(grid,.05,(0,0,0),clearance=.15)
    mask=np.ones_like(grid,dtype=bool)
    r,c=planner.world_to_cell(*first[:2]);mask[r-1:r+2,c-1:c+2]=False
    second,count=frontier_goal(grid,.05,(0,0,0),(.8,1.2,0),clearance=.14,safe_mask=mask)
    assert count and second is not None
    assert mask[planner.world_to_cell(*second[:2])]


def test_small_unresolved_frontier_is_not_evidence_of_complete_mapping():
    grid=np.full((30,30),100)
    grid[1:29,1:29]=0
    grid[1,10]=-1
    goal,count=frontier_goal(grid,.05,(0,0,0),(.8,.8,0),min_frontier_cells=5)
    assert goal is None and count>0


def test_frontier_outside_map_fails_closed():
    with pytest.raises(ValueError):
        frontier_goal(np.zeros((10,10)),0.1,(0,0,0),(-2,-2,0))


def test_no_safe_space_cannot_be_reported_as_completed_mapping():
    grid=np.full((20,20),100)
    grid[9:12,9:12]=0
    with pytest.raises(ValueError,match='map remains incomplete'):
        frontier_goal(grid,.05,(0,0,0),(.525,.525,0))
