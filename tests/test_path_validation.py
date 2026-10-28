import math
import numpy as np
from ee3033_project.path_validation import guarded_plan,validate_path,costmap_mask


def room():
    g=np.zeros((60,60),dtype=int)
    g[[0,-1],:]=100;g[:,[0,-1]]=100
    return g


def test_thin_wall_between_safe_endpoints_is_never_accepted():
    g=room();g[10:50,30]=100
    start,goal=(.7,1.5),(2.3,1.5)
    assert not validate_path(g,.05,(0.,0.,0.),[start,goal]).allowed
    plan=guarded_plan(g,.05,(0.,0.,0.),start,goal,[start,goal])
    assert plan.allowed and plan.replaced
    assert validate_path(g,.05,(0.,0.,0.),plan.path).allowed
    assert plan.path[0]==start and plan.path[-1]==goal


def test_safe_candidate_is_preserved():
    g=room();start,goal=(.5,.5),(2.,.5)
    plan=guarded_plan(g,.05,(0.,0.,0.),start,goal,[start,goal])
    assert plan.allowed and not plan.replaced


def test_unknown_barrier_has_no_fabricated_route():
    g=room();g[:,30]=-1
    plan=guarded_plan(g,.05,(0.,0.,0.),(.5,1.5),(2.5,1.5))
    assert not plan.allowed


def test_inflated_nav2_mask_can_block_a_raw_free_route():
    g=room();mask=np.ones(g.shape,dtype=bool);mask[:,30]=False
    plan=guarded_plan(g,.05,(0.,0.,0.),(.5,1.5),(2.5,1.5),[(.5,1.5),(2.5,1.5)],mask=mask)
    assert not plan.allowed


def test_start_or_goal_footprint_overlap_is_not_snapped_away():
    g=room()
    plan=guarded_plan(g,.05,(0.,0.,0.),(.5,.5),(.08,1.5))
    assert not plan.allowed and plan.reason.startswith('Goal footprint')


def test_rotated_grid_preserves_world_route():
    g=room();o=(3.,0.,math.pi/2)
    plan=guarded_plan(g,.05,o,(2.5,.5),(2.5,2.))
    assert plan.allowed and validate_path(g,.05,o,plan.path).allowed


def test_costmap_projection_respects_origin_rotation_and_unknown():
    cost=np.zeros((4,4),int);cost[1,2]=255;cost[2,1]=253
    mask=costmap_mask((4,4),.05,(0.,0.,0.),cost,.05,(0.,0.,0.))
    assert not mask[1,2] and not mask[2,1] and mask[0,0]


def test_canceled_or_expired_planning_never_returns_partial_path():
    g=room()
    assert not guarded_plan(g,.05,(0.,0.,0.),(.5,.5),(2.,2.),cancel=lambda:True).allowed
    assert not guarded_plan(g,.05,(0.,0.,0.),(.5,.5),(2.,2.),deadline=0.).allowed
