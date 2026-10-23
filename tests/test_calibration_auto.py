import copy
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from ee3033_project.calibration import camera_model, predicted_points, load_profile, save_profile
from ee3033_project.calibration_auto import (AutomaticRoute, bottle_sample, fit_automatic,
                                            corridor_clear, relative_pose)

SETUP = dict(bottle_height=.25, bottle_diameter=.065, camera_height=.2655, camera_x=-.035, camera_y=0.)
CAMERA = camera_model(640, 480, 'camera', [520.,0.,320.,0.,520.,240.,0.,0.,1.], [0.]*5)
TRUE = [555., 548., .13, .035]


def observations(noise=.05):
    views = []
    for distance, angles, holdout in [(d, (0.,-.13,.13), False) for d in (0.,.18,.36,.54,.72)]+[(.45,(0.,),True),(.27,(0.,),True)]:
        for yaw in angles:
            pose = [-distance, 0., yaw]
            s = bottle_sample(SETUP, .9, .04, pose, [])
            p = predicted_points(TRUE, SETUP, CAMERA, s)
            box = np.array([p[2:,0].min(),p[1,1],p[2:,0].max(),p[0,1]])
            box += np.random.default_rng(len(views)).normal(0, noise, 4)
            views.append(dict(pose=pose,box=box.tolist(),holdout=holdout))
    return views


def test_one_range_and_robot_poses_recover_geometry_without_mount_measurements(tmp_path):
    assumed = {**SETUP, 'camera_height': .20}
    p = fit_automatic(assumed, CAMERA, .9, observations())
    np.testing.assert_allclose([p['camera']['k'][0], p['camera']['k'][4]], TRUE[:2], atol=12)
    assert abs(p['setup']['camera_height']-SETUP['camera_height']) < .012
    assert abs(p['automatic']['anchor_bearing_rad']-.04) < .015
    assert all(s['error_m'] < .025 for s in p['validation'])
    assert load_profile(save_profile(p, tmp_path))['automatic']['method'] == 'one_range_pose_bundle'


def test_held_out_images_never_influence_fitted_parameters():
    views=observations()
    a=fit_automatic(SETUP,CAMERA,.9,views)
    for view in views:
        if view['holdout']:
            view['box'][3] += 3
    b=fit_automatic(SETUP,CAMERA,.9,views)
    assert a['camera']['k'] == b['camera']['k']
    assert a['validation'] != b['validation']


def test_moved_bottle_or_bad_anchor_cannot_be_silently_saved():
    views=observations()
    views[7]['box'][0] += 35
    views[7]['box'][2] += 35
    with pytest.raises(ValueError, match='rejected'):
        fit_automatic(SETUP,CAMERA,.9,views)
    with pytest.raises(ValueError):
        fit_automatic(SETUP,CAMERA,False,observations())


def test_missing_translation_and_missing_holdouts_are_rejected():
    views=observations()
    for view in views: view['pose'][0]=0.
    with pytest.raises(ValueError,match='translation'):
        fit_automatic(SETUP,CAMERA,.9,views)
    with pytest.raises(ValueError,match='held-out'):
        fit_automatic(SETUP,CAMERA,.9,[v for v in observations() if not v['holdout']])


def grid():
    return NS(header=NS(frame_id='map'),info=NS(resolution=.05,width=80,height=80,
        origin=NS(position=NS(x=-2.,y=-2.),orientation=NS(x=0.,y=0.,z=0.,w=1.))), data=[0]*6400)


def test_corridor_checks_entire_footprint_unknown_space_and_rotated_map():
    g=grid()
    assert corridor_clear(g,[0.,0.],0.)
    g.data[40*80+25]=-1
    assert not corridor_clear(g,[0.,0.],0.)
    assert not corridor_clear(grid(),[-1.5,0.],0.)
    g=grid();g.info.origin.orientation.z=math.sin(.4);g.info.origin.orientation.w=math.cos(.4)
    assert corridor_clear(g,[-.7,0.],.8)


def test_route_collects_seventeen_views_and_two_separate_check_positions():
    route=AutomaticRoute(0.,[0.,0.,0.]);pose=[0.,0.,0.];now=0.;captures=[]
    for _ in range(5000):
        linear,angular,capture=route.tick(now,pose,0.,True)
        pose=[pose[0]+linear*math.cos(pose[2])*.05,
              pose[1]+linear*math.sin(pose[2])*.05,pose[2]+angular*.05]
        if capture:
            captures.append((list(pose),route.goals[route.index][2]));route.captured(now)
        now+=.05
        if route.phase=='done':break
    assert route.phase=='done'
    assert len(captures)==17 and sum(check for _,check in captures)==2
    assert max(math.hypot(*p[:2]) for p,_ in captures) <= .74
    assert relative_pose([10.,20.,math.pi/2],[10.,19.,math.pi/2]) == pytest.approx([-1.,0.,0.])


def test_route_bounds_drift_and_timeout():
    route=AutomaticRoute(0.,[0.,0.,0.])
    with pytest.raises(ValueError,match='timed out'):route.tick(211.,[0.,0.,0.],0.,True)
    route.phase='translate'
    with pytest.raises(ValueError,match='drifted'):route.tick(.1,[0.,.09,0.],0.,True)


def test_brief_data_gap_commands_wait_until_fresh_and_settled_then_expires():
    from ee3033_project.calibration_auto import FreshDataPause
    pause=FreshDataPause()
    assert not pause.waiting(0.,False,False)
    assert pause.waiting(1.,True,False)
    assert pause.waiting(1.2,False,False)
    assert not pause.waiting(1.6,False,True)
    assert pause.waiting(2.,True,True)
    with pytest.raises(ValueError,match='two seconds'):
        pause.waiting(4.01,True,False)
