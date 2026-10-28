"""A reachable base pose is not necessarily a usable camera observation."""
import math
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from ee3033_project.coverage import CoveragePlanner
from ee3033_project.visibility import observation_visibility, supercover_cells


def planner(grid=None,origin=(0.,0.,0.)):
    return CoveragePlanner(np.zeros((100,100)) if grid is None else grid,.05,origin,
                           clearance=.15)


def camera(**changes):
    values=dict(translation=np.array([0.,0.,.3]),yaw=0.,fov=1.35,
                min_range=.3,max_range=3.)
    values.update(changes)
    return SimpleNamespace(**values)


def test_supercover_keeps_both_sides_of_grid_edges_and_diagonal_corners():
    edge=supercover_cells((.5,1.),(2.5,1.),1.,(0.,0.,0.))
    assert edge==frozenset((r,c) for r in (0,1) for c in (0,1,2))
    diagonal=supercover_cells((.5,.5),(2.5,2.5),1.,(0.,0.,0.))
    assert diagonal==frozenset(((0,0),(0,1),(1,0),(1,1),(1,2),(2,1),(2,2)))


def test_supercover_nonzero_margin_includes_intersected_cell_square():
    narrow=supercover_cells((.5,.95),(2.5,.95),1.,(0.,0.,0.),margin=.01)
    wider=supercover_cells((.5,.95),(2.5,.95),1.,(0.,0.,0.),margin=.05)
    assert (1,1) not in narrow and (1,1) in wider


def test_free_point_has_known_camera_visibility():
    result=observation_visibility(planner(),(.5,1.,0.),(1.5,1.))
    assert result and result.visible and result.support==result.range_support==1.
    assert result.hypotheses==result.visible_hypotheses==1


@pytest.mark.parametrize('value',[25,65,100,-1])
def test_wall_or_unknown_blocks_ray_even_if_endpoints_are_free(value):
    grid=np.zeros((100,100));grid[20,30]=value
    result=observation_visibility(planner(grid),(.5,1.025,0.),(2.,1.025))
    assert not result and 'blocks camera line of sight' in result.reason


def test_grid_corner_touching_wall_cannot_be_skipped_by_ray_sampling():
    grid=np.zeros((100,100));grid[19,20]=100
    # Segment crosses (1,1); the obstacle only touches its upper/right corner.
    result=observation_visibility(planner(grid),(.5,.5,math.pi/4),(1.5,1.5),los_margin=.001)
    assert not result


def test_robot_can_go_around_wall_but_camera_still_cannot_see_through_it():
    grid=np.zeros((100,100));grid[5:40,30]=100
    p=planner(grid);distances,_=p.travel_field(.5,1.)
    candidate=(2.5,1.,math.pi)
    assert math.isfinite(distances[p.world_to_cell(*candidate[:2])])
    result=observation_visibility(p,candidate,(.8,1.))
    assert not result and 'obstacle' in result.reason


def test_doorway_is_visible_when_the_whole_ray_margin_fits():
    grid=np.zeros((100,100));grid[:,30]=100;grid[17:24,30]=0
    assert observation_visibility(planner(grid),(.5,1.025,0.),(2.,1.025))
    grid[20,30]=100
    assert not observation_visibility(planner(grid),(.5,1.025,0.),(2.,1.025))


def test_camera_mount_translation_and_yaw_are_applied_at_candidate_pose():
    cam=camera(translation=np.array([.15,.10,.3]),yaw=math.pi/2)
    result=observation_visibility(planner(),(1.,1.,math.pi/2),(.3,1.15),camera_model=cam)
    assert result and result.camera_origin==pytest.approx((.9,1.15))
    assert not observation_visibility(planner(),(1.,1.,math.pi/2),(.3,1.15),
                                      camera_model=camera())


def test_camera_fov_and_range_are_checked_separately_from_map_visibility():
    p=planner();point=(1.+math.cos(.5),1.+math.sin(.5))
    assert not observation_visibility(p,(1.,1.,0.),point,camera_model=camera(fov=.6))
    assert observation_visibility(p,(1.,1.,0.),point,camera_model=camera(fov=1.2))
    assert not observation_visibility(p,(1.,1.,0.),(1.1,1.))
    assert not observation_visibility(p,(1.,1.,0.),(3.5,1.))


def test_out_of_bounds_ray_margin_and_zero_configured_margin_fail_closed():
    assert not observation_visibility(planner(),(.02,1.,0.),(1.,1.))
    assert not observation_visibility(planner(),(.5,1.,0.),(6.,1.),camera_model=camera(max_range=8.))
    assert not observation_visibility(planner(),(.5,1.,0.),(1.5,1.),los_margin=0.)


def test_bearing_only_uses_distribution_support_and_returns_no_invented_target():
    result=observation_visibility(planner(),(1.,1.6,math.atan2(.4,.8)),
        source_origin=(1.,2.),bearing=0.,range_bounds=(.3,1.2))
    assert result and result.support>=.6
    assert result.hypotheses>5 and result.visible_hypotheses>3
    assert not hasattr(result,'x') and not hasattr(result,'target')


def test_bearing_only_candidate_across_wall_cannot_use_depths_behind_wall():
    grid=np.zeros((100,100));grid[:,36]=100
    result=observation_visibility(planner(grid),(2.5,2.,math.pi),
        source_origin=(1.,2.),bearing=0.,range_bounds=(.3,1.2))
    assert not result and result.hypotheses>0 and result.support==0.


def test_seeing_a_single_one_metre_guess_is_not_enough_bearing_support():
    p=planner();cam=camera(fov=.4)
    candidate=(2.,1.6,math.pi/2)
    assert observation_visibility(p,candidate,(2.,2.),camera_model=cam)
    result=observation_visibility(p,candidate,source_origin=(1.,2.),bearing=0.,
        range_bounds=(.3,2.5),camera_model=cam)
    assert not result and result.support<.6


def test_source_near_clip_does_not_discard_bearing_hypotheses_before_retreat():
    result=observation_visibility(planner(),(1.7,2.,0.),source_origin=(2.,2.),
        bearing=0.,range_bounds=(.08,.38),camera_model=camera(min_range=.3))
    assert result and result.support>=.6


def test_intermediate_near_retreat_can_improve_visibility_without_claiming_range():
    kwargs=dict(source_origin=(2.,2.),bearing=0.,range_bounds=(.08,.38),
                camera_model=camera(min_range=.6))
    assert not observation_visibility(planner(),(1.7,2.,0.),**kwargs)
    intermediate=observation_visibility(planner(),(1.7,2.,0.),require_range=False,**kwargs)
    assert intermediate and intermediate.range_support<.6
    assert not observation_visibility(planner(),(.5,2.,0.),(4.,2.),
                                      camera_model=camera(max_range=2.),require_range=False)


def test_vertical_crop_can_retain_bearing_but_not_claim_floor_visibility():
    cam=camera(contains_floor=lambda points:np.zeros(len(points),dtype=bool))
    assert observation_visibility(planner(),(.5,1.,0.),(1.5,1.),camera_model=cam)
    assert not observation_visibility(planner(),(.5,1.,0.),(1.5,1.),
                                      camera_model=cam,require_floor=True)


def test_missing_depth_bounds_invalid_geometry_and_deadlines_fail_closed():
    p=planner()
    assert not observation_visibility(p,(1.,1.,0.),source_origin=(1.,1.))
    assert not observation_visibility(p,(1.,1.,0.),source_origin=(1.,1.),bearing=0.,range_bounds=(2.,.3))
    assert not observation_visibility(p,(1.,1.,math.nan),(2.,1.))
    assert not observation_visibility(p,(1.,1.,0.),(2.,1.),deadline=time.monotonic()-1.)
    cancel=threading.Event();cancel.set()
    assert not observation_visibility(p,(1.,1.,0.),(2.,1.),cancel=cancel)
