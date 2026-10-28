import math
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


def test_bold_frontier_goal_has_complete_footprint_clearance_from_unknown_cells():
    from ee3033_project.motion_safety import polyline_map_clearance
    from ee3033_project.path_validation import safe_grid
    grid=np.full((60,60),-1,dtype=int)
    grid[5:55,5:35]=0
    goal,count=frontier_goal(grid,.05,(0,0,0),(.8,1.2,0),clearance=.14)
    assert goal is not None and count>0
    assert polyline_map_clearance(grid,.05,(0,0,0),[goal[:2]]).allowed
    planner=CoveragePlanner(grid,.05,(0,0,0),clearance=.14)
    assert safe_grid(grid,.05)[planner.world_to_cell(*goal[:2])]


def test_robot_footprint_touching_wall_cannot_snap_to_a_nearby_safe_component():
    grid=np.full((60,60),-1,dtype=int)
    grid[5:55,5:35]=0
    grid[20,20]=100
    with pytest.raises(ValueError,match='footprint-clear'):
        frontier_goal(grid,.05,(0,0,0),(1.075,1.025,0.),clearance=.14)


def test_nearby_safe_anchor_cannot_jump_across_a_pillar_corner():
    from ee3033_project.frontiers import connection_cell
    from ee3033_project.motion_safety import polyline_map_clearance
    grid=np.zeros((250,250),dtype=int);grid[100,100]=100
    planner=CoveragePlanner(grid,.01,(0,0,0),clearance=.15)
    from ee3033_project.motion_safety import FOOTPRINT_RADIUS
    cell=(math.ceil((1.000+.55*FOOTPRINT_RADIUS)/.01),
          math.ceil((1.000+.9*FOOTPRINT_RADIUS)/.01))
    planner.safe[:]=False;planner.safe[cell]=True
    anchor=planner.cell_to_world(*cell)
    robot=(anchor[0],1.005-.55*FOOTPRINT_RADIUS,0.)
    assert np.linalg.norm(np.array(robot[:2])-anchor)<.18
    assert polyline_map_clearance(grid,.01,(0,0,0),[robot[:2]]).allowed
    assert polyline_map_clearance(grid,.01,(0,0,0),[anchor]).allowed
    assert not polyline_map_clearance(grid,.01,(0,0,0),[robot[:2],anchor]).allowed
    assert connection_cell(planner,robot) is None


def test_legacy_xy_visits_do_not_exclude_all_directions_across_a_room():
    grid=np.full((40,40),-1,dtype=int);grid[5:35,5:30]=0
    visits=[(float(x),float(y)) for x in np.arange(.425,1.5,.3)
            for y in np.arange(.425,1.75,.3)]
    goal,count=frontier_goal(grid,.05,(0,0,0),(.8,1.,0.),clearance=.14,visited=visits)
    assert goal is not None and count>0
    assert all(np.linalg.norm(np.asarray(goal[:2])-v)>=.12 for v in visits)


def doorway_frontier():
    from ee3033_project.motion_safety import FOOTPRINT_RADIUS
    resolution=.0125
    grid=np.zeros((240,240),dtype=int);grid[[0,-1],:]=100;grid[:,[0,-1]]=100
    grid[:,112:128]=100
    half_cells=math.ceil((FOOTPRINT_RADIUS+resolution)/resolution)
    grid[120-half_cells:120+half_cells,112:128]=0
    grid[1:-1,200:-1]=-1
    # Strictly wider than the preferred graph's doorway connectivity, while
    # leaving the complete unchanged body circle and map sampling guard safe.
    clearance=half_cells*resolution+resolution
    return grid,resolution,clearance


def test_frontier_beyond_a_physically_proven_narrow_connector_remains_reachable():
    from ee3033_project.path_validation import guarded_plan
    grid,resolution,clearance=doorway_frontier()
    robot=(.6,1.5,0.)
    assert frontier_goal(grid,resolution,(0,0,0),robot,clearance=clearance)==(None,1)
    mask=np.ones_like(grid,dtype=bool)
    goal,count=frontier_goal(grid,resolution,(0,0,0),robot,
        clearance=clearance,safe_mask=mask)
    assert goal is not None and count==1 and goal[0]>1.6
    assert guarded_plan(grid,resolution,(0,0,0),robot[:2],goal[:2],mask=mask).allowed
    mask[:,120]=False
    assert frontier_goal(grid,resolution,(0,0,0),robot,
        clearance=clearance,safe_mask=mask)==(None,1)


def test_comfortable_connected_frontiers_remain_preferred_over_narrow_connectors():
    grid,resolution,clearance=doorway_frontier()
    grid[225:-1,1:112]=-1
    goal,count=frontier_goal(grid,resolution,(0,0,0),(.6,1.5,0.),
        clearance=clearance,safe_mask=np.ones_like(grid,dtype=bool))
    assert goal is not None and count==2 and goal[0]<1.4
