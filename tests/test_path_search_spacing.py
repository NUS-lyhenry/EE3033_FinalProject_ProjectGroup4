"""Regression of the recorded side entrance, using unchanged occupancy/costs."""
from pathlib import Path
import math

import numpy as np
import pytest

from ee3033_project.path_validation import (
    path_search_grid, path_samples, guarded_plan, validate_path, costmap_mask, cell_world,
)
from ee3033_project.frontiers import frontier_goal
from ee3033_project.waypoint import waypoint_goal
from ee3033_project.robot_geometry import GEOMETRY_PROFILE, FOOTPRINT_RADIUS


def recorded_gap():
    with np.load(Path(__file__).parent/'fixtures/side_gap_5cm_20261030.npz') as d:
        grid=d['grid'];res=float(d['resolution']);origin=d['origin']
        mask=costmap_mask(grid.shape,res,origin,d['costs'],
                          float(d['cost_resolution']),d['cost_origin'])
    return grid,res,origin,mask


@pytest.mark.parametrize('resolution,factor',[(.05,3),(float(np.float32(.05)),3),(.025,2),(.0125,1),(.1,5)])
def test_search_subdivision_preserves_occupancy_mask_and_existing_finer_maps(resolution,factor):
    grid=np.array([[100,-1],[0,24]])
    mask=np.array([[False,False],[True,False]])
    original=grid.copy()
    refined,res,allowed=path_search_grid(grid,resolution,mask)
    assert res<=.020000001
    assert refined.shape==(2*factor,2*factor)
    assert np.array_equal(grid,original)
    for row,col in np.ndindex(grid.shape):
        assert np.all(refined[row*factor:(row+1)*factor,col*factor:(col+1)*factor]==grid[row,col])
        assert np.all(allowed[row*factor:(row+1)*factor,col*factor:(col+1)*factor]==mask[row,col])


@pytest.mark.skipif(GEOMETRY_PROFILE!='measured',reason='Recording uses the measured wheel-inclusive robot')
def test_recorded_neck_route_passes_original_footprint_and_nav2_checks():
    grid,res,origin,mask=recorded_gap()
    start=(1.0955788589730127,-.8707878304340277)
    goal=cell_world((13,56),res,origin)
    original=grid.copy()
    plan=guarded_plan(grid,res,origin,start,goal,mask=mask)
    assert plan.allowed,plan.reason
    assert plan.path[0]==start and plan.path[-1]==goal
    assert validate_path(grid,res,origin,plan.path,mask=mask,radius=FOOTPRINT_RADIUS).allowed
    assert np.array_equal(grid,original)
    # Every internal search edge uses at most 2 cm. End connectors retain
    # their separately validated exact requested coordinates.
    assert max(math.dist(a,b) for a,b in zip(plan.path[1:-2],plan.path[2:-1]))<=.020000001
    target=(*goal,.7)
    selected,_,final=waypoint_goal(grid,res,origin,(*start,0.),target,
                                   clearance=.04,safe_mask=mask)
    assert final and selected==target
    frontier,count=frontier_goal(grid,res,origin,(*start,0.),clearance=.04,safe_mask=mask)
    assert frontier is not None and count>0 and frontier[1]<-1.1
    assert validate_path(grid,res,origin,[frontier[:2]],mask=mask).allowed


@pytest.mark.skipif(GEOMETRY_PROFILE!='measured',reason='Recording uses measured geometry')
@pytest.mark.parametrize('blocked_by',['unknown','obstacle','native_costmap'])
def test_refinement_cannot_open_a_blocked_recorded_neck(blocked_by):
    grid,res,origin,mask=recorded_gap()
    start=(1.0955788589730127,-.8707878304340277)
    goal=cell_world((13,56),res,origin)
    if blocked_by=='native_costmap':mask[17,:]=False
    else:grid[17,:]=-1 if blocked_by=='unknown' else 100
    assert not guarded_plan(grid,res,origin,start,goal,mask=mask).allowed


def test_native_costmap_is_projected_at_refined_search_locations():
    grid=np.zeros((2,2),int)
    refined,res,_=path_search_grid(grid,.05)
    costs=np.zeros((4,4),int);costs[0,0]=253
    mask=costmap_mask(refined.shape,res,(0.,0.,0.),costs,.025,(0.,0.,0.))
    assert not mask[0,0] and mask[0,2]


@pytest.mark.skipif(GEOMETRY_PROFILE!='measured',reason='Recording uses measured geometry')
def test_recorded_original_map_recheck_uses_same_continuous_sampling_budget():
    with np.load(Path(__file__).parent/'fixtures/original_map_sampling_20261030.npz') as d:
        grid=d['grid'];resolution=float(d['resolution']);origin=d['origin']
        fine,res,_=path_search_grid(grid,resolution)
        mask=costmap_mask(fine.shape,res,origin,d['costs'],float(d['cost_resolution']),d['cost_origin'])
        sparse=d['sparse_path']
        assert validate_path(fine,res,origin,sparse,mask=mask).allowed
        assert not validate_path(grid,resolution,origin,sparse).allowed
        plan=guarded_plan(fine,res,origin,d['start'],d['goal'],mask=mask)
        assert plan.allowed,plan.reason
        assert validate_path(grid,resolution,origin,plan.path).allowed
        assert validate_path(fine,res,origin,plan.path,mask=mask).allowed
        assert max(math.dist(a,b) for a,b in zip(plan.path[:-1],plan.path[1:]))<=.012500001


def test_path_samples_preserve_each_corner_endpoint_and_segment():
    corners=[(0.,0.),(.031,0.),(.031,.027)]
    samples=path_samples(corners)
    assert samples[0]==corners[0] and samples[-1]==corners[-1]
    assert corners[1] in samples
    assert all(y==0. or x==.031 for x,y in samples)
    assert max(math.dist(a,b) for a,b in zip(samples[:-1],samples[1:]))<=.012500001


@pytest.mark.parametrize('points,spacing',[([], .01), ([(0.,0.),(math.nan,0.)],.01), ([(0.,0.),(1.,0.)],0.), ([(0.,0.),(1e9,0.)],.01)])
def test_invalid_or_unbounded_path_samples_fail_closed(points,spacing):
    with pytest.raises(ValueError):path_samples(points,spacing)
