import math
import time
import threading
import cv2
import numpy as np
import pytest
from ee3033_project.calibration import mounting_rotation
from ee3033_project.vision_geometry import FloorCamera, camera_matrix, validate_mount_settings
from ee3033_project.coverage import CoveragePlanner, Viewpoint
from ee3033_project.target_acquisition import SingleTarget
from ee3033_project.trial import PROFILES, parameter_plan


def camera(**overrides):
    values = dict(width=640, height=480, k=np.array([[450., 0, 320], [0, 450., 240], [0, 0, 1]]),
                  d=np.zeros(5), rotation=mounting_rotation(math.radians(10), 0),
                  translation=np.array([.04, .02, .22]))
    values.update(overrides)
    return FloorCamera(**values)


def box_at(model, x, y):
    point = (np.array([x, y, 0])-model.translation)@model.rotation
    pixel, _ = cv2.projectPoints(point.reshape(1, 3), np.zeros(3), np.zeros(3), model.k, model.d)
    u, v = pixel.ravel()
    return [u-12, v-50, u+12, v]


def test_floor_localization_round_trip_without_a_fitted_profile():
    model = camera()
    for x, y in [(.65, .02), (1., -.1), (1.4, .2)]:
        point, sigma, distance = model.project_box(box_at(model, x, y))
        assert point[:2] == pytest.approx([x, y], abs=1e-8)
        assert point[2] == pytest.approx(0.)
        assert sigma > 0 and distance > 0
        reopened = FloorCamera.from_metadata(model.metadata())
        assert reopened.project_box(box_at(model, x, y))[0] == pytest.approx(point)


def test_geometric_uncertainty_increases_with_distance_and_is_not_zero_for_stable_pixels():
    model = camera()
    near = model.project_box(box_at(model, .65, .02))[1]
    far = model.project_box(box_at(model, 1.8, .02))[1]
    assert far > near*3
    assert near > .015
    with pytest.raises(ValueError):
        model.project_box(box_at(model, 2.5, .02))
    point, sigma, distance = model.project_box(box_at(model, 2.5, .02), coarse_max_range=3.)
    assert distance > 2. and sigma > .10


@pytest.mark.parametrize('box', [[0, 200, 30, 300], [300, 200, 330, 479],
                               [300, 20, 330, 50], [310, 250, 311, 251]])
def test_clipped_tiny_and_horizon_detections_never_become_coordinates(box):
    with pytest.raises(ValueError):
        camera().project_box(box)


def test_invalid_intrinsics_require_an_explicit_fov_and_keep_approximation_provenance():
    with pytest.raises(ValueError, match='CameraInfo'):
        camera_matrix(640, 480)
    k, d, source = camera_matrix(640, 480, horizontal_fov=1.2)
    assert k[0, 0] == pytest.approx(640/(2*math.tan(.6)))
    assert 'approximation' in source
    with pytest.raises(ValueError):
        camera_matrix(640, 480, [math.nan]*9)
    with pytest.raises(ValueError):
        camera(translation=[0, 0, 0])


def test_coverage_excludes_near_blind_zone_and_uses_mount_yaw_and_offset():
    model = camera(rotation=mounting_rotation(math.radians(10), math.pi/2))
    grid = np.zeros((60, 60), dtype=int)
    planner = CoveragePlanner(grid, .05, (0, 0, 0), camera_model=model, spacing=.7, headings=6)
    planner.prepare(1.5, 1.5)
    cells = planner.visible(30, 30, 0)
    assert cells
    origin = planner.cell_to_world(30, 30)
    world = np.array([planner.cell_to_world(*c) for c in cells])-origin
    assert model.contains_floor(world).all()
    assert all(y > 0 for x, y in world)
    assert not model.contains_floor([[.05, .02]])[0]


def test_grid_path_cost_penalizes_wall_detour_and_rejects_unreachable_view():
    grid = np.zeros((30, 40), dtype=int)
    grid[0:25, 15:17] = 100
    planner = CoveragePlanner(grid, .1, (0, 0, 0), clearance=.1)
    cells = frozenset({(1, 1)})
    planner.options = (Viewpoint(1.85, .85, 0, cells), Viewpoint(.85, 2.05, math.pi/2, cells))
    distances, _ = planner.travel_field(.85, .85)
    assert distances[planner.world_to_cell(1.85, .85)] > 4.
    assert planner.next_viewpoint(.85, .85, set(), yaw=0) == 1
    # Native planner can reject a dynamic blockage even when static map is free.
    assert planner.rank_viewpoints(.85, .85, set(), path_lengths={1: math.inf})[0][1] == 0
    with pytest.raises(TimeoutError):
        planner.travel_field(.95, .85, deadline=time.monotonic()-1)


def detection(stamp, **kwargs):
    return dict(stamp=stamp, confidence=.8, label='bottle', bearing=0., origin=[1., 1.], **kwargs)


def test_single_target_requires_distinct_fresh_frames_and_does_not_switch_during_acquisition():
    target = SingleTarget()
    assert not target.update([detection(10.)], 10., 9.)
    assert not target.update([detection(10.)], 10., 9.)
    assert target.update([detection(10.25)], 10.25, 9.)
    target.started = 1.
    wrong = detection(10.5); wrong['bearing'] = 1.
    assert not target.update([wrong], 10.5, 9.)
    assert target.candidate['stamp'] == 10.25
    assert not target.update([detection(7.)], 10.5, 9.)
    assert not target.expired(16.)
    target.advance_clock(1.,working=True)
    target.advance_clock(16.,working=False,traveling=True)
    target.advance_clock(76.,working=True)
    assert not target.expired(76.)  # the minute of navigation is excluded
    target.advance_clock(106.)
    assert target.expired(46.)
    target.reset()
    assert not target.update([detection(20., x=1., y=1.)], 20., 19.)
    assert not target.update([detection(20.25, x=2., y=1.)], 20.25, 19.)


def test_approach_is_reachable_and_bearing_only_centers_before_a_baseline():
    planner = CoveragePlanner(np.zeros((40, 40)), .1, (0, 0, 0))
    target = SingleTarget()
    target.candidate = detection(10., x=2., y=1., sigma=.3)
    goal = target.observation_goal(planner, (1., 1., 0.))
    assert goal and planner.safe[planner.world_to_cell(*goal[:2])]
    assert .45 < math.hypot(goal[0]-2, goal[1]-1) < 1.2
    target.candidate = detection(10.)
    assert target.observation_goal(planner, (1., 1., .5)) == (1., 1., 0.)
    alternative=target.observation_goal(planner, (1., 1., .5))
    assert alternative and math.dist(alternative[:2],(1.,1.))>=.3


def test_repeated_turns_cannot_exhaust_motion_budget_before_a_lateral_baseline():
    planner=CoveragePlanner(np.zeros((50,50)),.1,(0,0,0))
    target=SingleTarget();target.candidate=detection(10.)
    pose=(2.,2.,.5)
    first=target.observation_goal(planner,pose)
    assert first[:2]==pose[:2]
    # Simulate fresh detections repeatedly requesting small corrections.
    target.candidate=detection(10.5);target.candidate['bearing']=.1
    second=target.observation_goal(planner,pose)
    assert second[:2]==pose[:2]
    target.candidate=detection(11.);target.candidate['bearing']=.2
    third=target.observation_goal(planner,pose)
    assert third and math.dist(third[:2],pose[:2])>=.3
    assert target.turns_at(pose)==2


def test_bad_floor_depth_and_first_nav2_rejected_side_still_try_another_baseline():
    planner=CoveragePlanner(np.zeros((50,50)),.1,(0,0,0))
    target=SingleTarget();target.candidate=detection(10.,x=20.,y=20.)
    pose=(2.,2.,0.)
    # The north-side point is Nav2-blocked; south is usable.
    goal=target.observation_goal(planner,pose,force_baseline=True,
        allowed=lambda x,y:y<2.)
    assert goal and goal[1]<2. and math.dist(goal[:2],pose[:2])>=.3
    assert target.last_goal_reason=='Depth-free lateral baseline'


def test_fast_profile_synchronizes_both_stages_and_search_estimates():
    for phase in ('mapping', 'mission'):
        p = parameter_plan(PROFILES['fast'], phase)
        assert p['controller_server']['FollowPath.rotate_to_heading_angular_vel'] == .55
        assert p['project_velocity_gate']['max_angular'] == p['controller_server']['FollowPath.max_vel_theta'] == .60
        if phase == 'mission':
            assert p['project_mission']['search_linear_speed'] == .16


def test_uncertain_target_requests_a_closer_observation_not_the_current_pose():
    planner = CoveragePlanner(np.zeros((60, 60)), .05, (0, 0, 0))
    target = SingleTarget()
    target.candidate = detection(10., x=2., y=1., sigma=.12, close_view=False)
    pose = (1.4, 1., 0.)
    goal = target.observation_goal(planner, pose)
    assert goal and planner.safe[planner.world_to_cell(*goal[:2])]
    assert .35 < math.hypot(goal[0]-2, goal[1]-1) < 1.1
    assert abs(math.atan2(1.-goal[1],2.-goal[0])) > math.radians(15.)
    assert math.dist(goal[:2], pose[:2]) >= .12 or abs(goal[2]-pose[2]) >= .2


def test_centered_bearing_only_target_does_not_spend_a_step_on_a_noop():
    target = SingleTarget()
    target.candidate = detection(10.)
    planner = CoveragePlanner(np.zeros((40, 40)), .1, (0, 0, 0))
    # An already centered target should acquire parallax, not another turn.
    goal=target.observation_goal(planner, (1., 1., 0.))
    assert goal and math.dist(goal[:2],(1.,1.))>=.3
    assert planner.safe[planner.world_to_cell(*goal[:2])]


def test_camera_settings_reject_nonfinite_values_without_inventing_mount_dimensions():
    settings = dict(camera_height=0., camera_pitch=0., camera_yaw=0., camera_x=0.,
                    camera_y=0., horizontal_fov=0., camera_min_range=.35, camera_max_range=2.)
    assert validate_mount_settings(settings)['camera_height'] == 0.
    with pytest.raises(ValueError):
        validate_mount_settings(dict(settings, camera_height=math.nan))
    with pytest.raises(ValueError):
        validate_mount_settings(dict(settings, camera_min_range=1., camera_max_range=.5))


def test_horizontal_clipping_only_guides_a_mapped_retreat_not_depth():
    planner=CoveragePlanner(np.zeros((40,40)),.1,(0,0,0))
    target=SingleTarget();target.candidate=dict(stamp=10.,label='cup',confidence=.9,
        box=[0,100,60,350],steering_bearing=0.)
    goal=target.observation_goal(planner,(1.,1.,0.))
    assert goal and goal[0]<1. and goal[1]==pytest.approx(1.)
    assert math.dist(goal[:2],(1.,1.))==pytest.approx(.3)
    assert planner.safe[planner.world_to_cell(*goal[:2])]
