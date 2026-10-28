import math
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import cv2
import numpy as np
import pytest
from ee3033_project.calibration import mounting_rotation
from ee3033_project.coverage import CoveragePlanner
from ee3033_project.near_view import retreat_goal
from ee3033_project.target_acquisition import SingleTarget
from ee3033_project.vision_geometry import FloorCamera


def camera():
    return FloorCamera(640,480,np.array([[300.,0.,320.],[0.,300.,240.],[0.,0.,1.]]),
        np.zeros(5),mounting_rotation(math.radians(28.6),0.),np.array([0.,0.,.31]),
        min_range=.6,max_range=1.2)


def box_at(model,x):
    relative=(np.array([x,0.,0.])-model.translation)@model.rotation
    pixels,_=cv2.projectPoints(relative.reshape(1,3),np.zeros(3),np.zeros(3),model.k,model.d)
    u,v=pixels.ravel()
    return [u-15,v-80,u+15,v]


def planner():
    return CoveragePlanner(np.zeros((100,100)),.05,(0.,0.,0.),clearance=.15,camera_model=camera())


def near_detection(**kwargs):
    return dict(stamp=10.,label='cup',confidence=.9,bearing=0.,origin=[2.,2.],
        projection_valid=False,range_state='too_near',range_hint_m=.3,
        camera_min_range=.6,camera_max_range=1.2,reframe_required=True,**kwargs)


def test_range_rejection_produces_only_an_action_hint_and_does_not_relax_the_camera():
    model=camera();box=box_at(model,.4)
    with pytest.raises(ValueError,match='reliable camera range'):model.project_box(box)
    view=model.view_requirements(box)
    assert view['range_state']=='too_near' and view['reframe_required']
    assert view['range_hint_m']==pytest.approx(.4) and view['range_hint_is_measurement'] is False
    assert 'x' not in view and 'y' not in view
    assert model.min_range==.6 and model.max_range==1.2


def test_bottom_crop_requests_reframe_without_an_invented_contact_range():
    view=camera().view_requirements([290.,300.,350.,479.])
    assert view['reframe_required'] and view['reframe_reasons']==['vertical_clipping']
    assert view['range_hint_m'] is None and view['range_state']=='unknown'


def test_vertical_crop_with_valid_bearing_prioritizes_retreat_over_lateral_baseline():
    target=SingleTarget(enable_ab_maneuver=True);target.candidate=near_detection(reason_codes=['detection_vertical_clipping'])
    goal=target.observation_goal(planner(),(2.,2.,0.),allowed=lambda x,y:True)
    assert goal and goal[0]<2. and goal[1]==pytest.approx(2.)
    assert math.dist(goal[:2],(2.,2.))==pytest.approx(.3)
    assert target.last_goal_action=='retreat' and target.reframe_attempts==1


def test_unknown_rear_corridor_and_nav2_rejection_prevent_retreat():
    p=planner();p.safe[:,33:40]=False
    d=near_detection()
    goal,_=retreat_goal(p,(2.,2.,0.),d,allowed=lambda x,y:True)
    assert goal is None
    goal,_=retreat_goal(planner(),(2.,2.,0.),d,allowed=lambda x,y:x>=2.)
    assert goal is None


def test_clear_retreat_does_not_recompute_the_whole_map_path_field(monkeypatch):
    p=planner()
    def forbidden(*args,**kwargs):
        raise AssertionError('A proven direct corridor needs no full Dijkstra field')
    monkeypatch.setattr(p,'travel_field',forbidden)
    goal,_=retreat_goal(p,(2.,2.,0.),near_detection(),allowed=lambda x,y:True)
    assert goal and math.dist(goal[:2],(2.,2.))==pytest.approx(.30)


def test_reachable_endpoint_around_an_obstacle_still_cannot_be_a_straight_retreat():
    p=planner();p.safe[40,38:40]=False
    distances,_=p.travel_field(2.,2.)
    assert math.isfinite(distances[p.world_to_cell(1.7,2.)])
    assert retreat_goal(p,(2.,2.,0.),near_detection(),allowed=lambda x,y:True)[0] is None


def test_reframing_never_uses_invalid_floor_coordinates_as_the_target(monkeypatch):
    calls=[]
    def visible(*args,**kwargs):
        calls.append(kwargs);return NS(visible=True,reason='test clear')
    monkeypatch.setattr('ee3033_project.near_view.observation_visibility',visible)
    target=SingleTarget(enable_ab_maneuver=True);target.candidate=near_detection(x=999.,y=999.)
    assert target.observation_goal(planner(),(2.,2.,0.))
    assert calls and calls[0].get('target') is None and calls[0]['source_origin']==(2.,2.)


def test_reframe_uses_the_last_image_origin_after_robot_motion(monkeypatch):
    calls=[]
    def visible(*args,**kwargs):
        calls.append(kwargs);return NS(visible=True,reason='test clear')
    monkeypatch.setattr('ee3033_project.near_view.observation_visibility',visible)
    d=near_detection()
    assert retreat_goal(planner(),(1.8,2.,0.),d)[0]
    assert calls[0]['source_origin']==(2.,2.)


def test_reframe_requires_actual_image_origin_and_cannot_fabricate_it_from_pose():
    d=near_detection();d.pop('origin')
    goal,reason=retreat_goal(planner(),(2.,2.,0.),d)
    assert goal is None and 'camera origin' in reason


@pytest.mark.parametrize('field,value',[('origin',2.),('bearing',math.nan),('camera_min_range','bad')])
def test_invalid_reframe_geometry_rejects_without_crashing(field,value):
    d=near_detection();d[field]=value
    assert retreat_goal(planner(),(2.,2.,0.),d)[0] is None


def test_near_field_moves_are_bounded_without_spending_abc_stations(monkeypatch):
    monkeypatch.setattr('ee3033_project.near_view.observation_visibility',
        lambda *a,**kw:NS(visible=True,reason='test clear'))
    target=SingleTarget(enable_ab_maneuver=True);p=planner();pose=(2.,2.,0.)
    for stamp in (10.,11.):
        d=near_detection();d['stamp']=stamp;d['origin']=list(pose[:2]);target.candidate=d
        goal=target.observation_goal(p,pose)
        assert goal and target.last_goal_action=='retreat'
        pose=goal
    d=near_detection();d['origin']=list(pose[:2]);target.candidate=d
    assert target.observation_goal(p,pose) is None
    assert target.reframe_attempts==2 and target.reframe_distance<=.6+1e-9
    assert len(target.positions)==0
    assert len(target.reframe_positions)==3


def legacy_perception(point):
    """Run the real legacy locate body without ROS, retaining its range guards."""
    source=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/perception.py'
    cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='Perception')
    locate=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='locate')
    ns=dict(math=math,np=np,floor_point=lambda *a:np.asarray(point,dtype=float),
        in_envelope=lambda *a:True,Time=NS(from_msg=lambda value:value),
        transform_arrays=lambda tr:(np.eye(3),np.zeros(3)))
    exec(compile(ast.Module(body=[locate],type_ignores=[]),str(source),'exec'),ns)
    image=NS(width=640,height=480,header=NS(frame_id='camera',stamp=0.))
    p=NS(p=dict(localization_method='hybrid',pixel_margin=8.,max_range=3.,
        camera_min_range=.6,camera_max_range=1.2,map_frame='map'),
        calibration=dict(camera=dict(width=640,height=480,frame_id='camera'),
                         setup=dict(camera_x=0.,camera_y=0.),base_frame='base_footprint'),
        projection_in_range=True,buffer=NS(lookup_transform=lambda *a:None))
    return p,image,ns['locate']


def test_legacy_profile_cannot_bypass_configured_minimum_or_cropped_box_rejection():
    p,image,locate=legacy_perception([.4,0.,0.])
    with pytest.raises(ValueError,match='reliable camera range'):
        locate(p,[290.,200.,350.,350.],image,None,None)
    p,image,locate=legacy_perception([.8,0.,0.])
    with pytest.raises(ValueError,match='clipped'):
        locate(p,[290.,200.,350.,479.],image,None,None)


def test_legacy_coarse_far_point_remains_outside_the_accepted_measurement_range():
    p,image,locate=legacy_perception([1.4,0.,0.])
    assert locate(p,[290.,200.,350.,350.],image,None,None)==pytest.approx([1.4,0.,0.])
    assert p.projection_in_range is False


def test_two_failed_framing_views_still_allow_valid_a_and_lateral_b():
    target=SingleTarget(enable_ab_maneuver=True);p=planner();pose=(2.,2.,0.)
    for stamp in (10.,11.):
        d=near_detection();d['stamp']=stamp;d['origin']=list(pose[:2])
        target.candidate=d
        goal=target.observation_goal(p,pose)
        assert goal is not None and target.last_goal_action=='retreat'
        pose=goal
    # Only here does the camera first recover a complete, usable A view.
    assert target.positions==[]
    target.candidate=dict(stamp=12.,label='cup',confidence=.9,bearing=0.,
        origin=list(pose[:2]),projection_valid=False,range_state='within_range',
        reframe_required=False,bearing_valid=True)
    goal=target.observation_goal(p,pose,force_baseline=True)
    assert goal is not None
    assert abs(goal[1]-pose[1])>=.25
    assert target.last_goal_reason=='Depth-free lateral baseline'
    assert target.positions==[tuple(pose[:2])]
    assert target.reframe_attempts==2 and target.reframe_distance<=.6+1e-9
    assert not target.expired(100.)


def test_repeated_near_view_does_not_duplicate_recovery_position():
    target=SingleTarget(enable_ab_maneuver=True);p=planner();pose=(2.,2.,0.)
    target.candidate=near_detection()
    target.observation_goal(p,pose)
    target.observation_goal(p,pose)
    assert target.reframe_positions==[(2.,2.)]
    assert target.positions==[]
    assert target.reframe_attempts<=2 and target.reframe_distance<=.6+1e-9
