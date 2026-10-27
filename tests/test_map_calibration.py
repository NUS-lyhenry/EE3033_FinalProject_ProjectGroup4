import math
from types import SimpleNamespace
import numpy as np
import pytest

from ee3033_project.calibration import camera_model,predicted_points,save_profile,load_profile
from ee3033_project.map_calibration import make_plan,map_sample,fit_map_profile,MapCalibrationRun,free_segment

SETUP=dict(bottle_height=.215,bottle_diameter=.065,camera_x=-.035,camera_y=0.,camera_height=.2655)
CAMERA=camera_model(640,480,'default_cam',[520.,0.,320.,0.,520.,240.,0.,0.,1.],[0.]*5)
TRUE=[555.,548.,math.radians(14.2),.035]
GRID=dict(width=120,height=120,resolution=.05,origin=[-3.,-3.,0.],cells=[0]*14400)


def views():
    plan=make_plan(GRID,[0.,0.,0.],[.9,0.]);data=[]
    for goal in plan['goals']:
        sample=map_sample(SETUP,[.9,0.],goal['pose'],[])
        points=predicted_points(TRUE,SETUP,CAMERA,sample)
        box=[points[2:,0].min(),points[1,1],points[2:,0].max(),points[0,1]]
        data.append(dict(pose=goal['pose'],box=list(map(float,box)),holdout=goal['holdout']))
    return data


def test_known_map_point_fits_camera_without_any_measured_distance(tmp_path):
    p=fit_map_profile({**SETUP,'camera_height':.20},CAMERA,[.9,0.],views())
    np.testing.assert_allclose([p['camera']['k'][0],p['camera']['k'][4]],TRUE[:2],atol=2)
    assert math.degrees(p['pitch'])==pytest.approx(14.2,abs=.1)
    assert p['setup']['camera_height']==pytest.approx(.2655,abs=.005)
    assert all(v['error_m']<.01 for v in p['validation'])
    assert load_profile(save_profile(p,tmp_path))['map_calibration']['method']=='known_map_point'


def test_holdouts_do_not_change_the_fitted_pitch():
    a=views();p=fit_map_profile(SETUP,CAMERA,[.9,0.],a)
    for v in a:
        if v['holdout']:v['box'][3]+=3
    q=fit_map_profile(SETUP,CAMERA,[.9,0.],a)
    assert p['pitch']==q['pitch']


def test_plan_rejects_unknown_floor_and_bad_reference_point():
    grid={**GRID,'cells':[-1]*14400}
    with pytest.raises(ValueError,match='free floor'):make_plan(grid,[0,0,0],[.9,0])
    with pytest.raises(ValueError,match='0.40'):make_plan(GRID,[0,0,0],[.1,0])
    assert not free_segment(GRID,[-2.9,0],[-2.,0])


@pytest.mark.parametrize('hardware_sign',[1.,-1.])
def test_full_map_route_verifies_and_corrects_both_hardware_turn_polarities(hardware_sign):
    plan=make_plan(GRID,[0,0,0],[.9,0]);pose=[0.,0.,.25]
    run=MapCalibrationRun(0.,pose,pose,plan);now=0.;captured=[]
    for _ in range(6000):
        vx,wz,capture=run.tick(now,pose,pose,True)
        assert abs(vx)<=.04 and abs(wz)<=.15
        pose=[pose[0]+vx*math.cos(pose[2])*.05,pose[1]+vx*math.sin(pose[2])*.05,pose[2]+hardware_sign*wz*.05]
        if capture:captured.append(run.index);run.captured(now)
        now+=.05
        if run.phase=='done':break
    assert run.phase=='done'
    assert run.yaw_sign==hardware_sign
    assert len(captured)==17


def test_unverified_turn_response_aborts_before_positioning():
    run=MapCalibrationRun(0.,[0,0,0],[0,0,0],make_plan(GRID,[0,0,0],[.9,0]))
    with pytest.raises(ValueError,match='verify consistent'):
        for t in np.arange(0,5,.05):run.tick(float(t),[0,0,0],[0,0,0],True)


def test_wrong_way_heading_response_is_stopped():
    run=MapCalibrationRun(0.,[0,0,0],[0,0,0],make_plan(GRID,[0,0,0],[.9,0]))
    run.phase='position';run.yaw_sign=1.
    run.tick(.1,[0,0,.3],[0,0,.3],True)
    with pytest.raises(ValueError,match='increasing'):run.tick(.8,[0,0,.5],[0,0,.5],True)


def test_duplicate_proposals_merge_but_distinct_or_clipped_boxes_do_not():
    from ee3033_project.map_calibration import complete_candidates
    proposals=[dict(box=[100,100,150,350],confidence=.9),dict(box=[102,102,152,352],confidence=.8),
               dict(box=[300,100,350,350],confidence=.85),dict(box=[0,100,50,350],confidence=.99)]
    accepted=complete_candidates(proposals,CAMERA)
    assert [p['box'] for p in accepted]==[[100,100,150,350],[300,100,350,350]]


def test_map_coordinate_validation_and_rotated_map_clearance():
    for point in ([],[1.],[.9,0.,0.],None):
        with pytest.raises(ValueError,match='map point'):make_plan(GRID,[0,0,0],point)
    grid={**GRID,'origin':[3.,-3.,math.pi/2]}
    assert make_plan(grid,[0,0,0],[.9,0.])['target']==[.9,0.]
    grid['cells']=list(grid['cells']);grid['cells'][60*120+60]=100
    assert not free_segment(grid,[0.,0.],[-.8,0.])
