"""Exercise the production goal classification and clock methods without ROS."""
import ast
import math
from pathlib import Path
import time
from types import MethodType, SimpleNamespace as NS

from ee3033_project.core import angle_difference, quaternion_yaw
from ee3033_project.target_acquisition import SingleTarget


source=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/mission.py'
cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='Mission')
methods=[n for n in cls.body if isinstance(n,ast.FunctionDef)
         and n.name in ('goal_requires_translation','update_localization_clock')]
namespace=dict(math=math,time=time,angle_difference=angle_difference,quaternion_yaw=quaternion_yaw)
exec(compile(ast.Module(body=methods,type_ignores=[]),str(source),'exec'),namespace)


def pose(x,y=0.,yaw=0.):
    return NS(header=NS(frame_id='map'),pose=NS(position=NS(x=x,y=y),
        orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))))


def mission(goal,translation):
    m=NS(current_goal=goal,goal_kind='acquire',goal_translation=translation,
         acquisition=SingleTarget(),pending_navigation=None,state='ACQUIRING',navigation_status={'stage':'TRACKING'})
    for name in ('goal_requires_translation','update_localization_clock'):
        setattr(m,name,MethodType(namespace[name],m))
    m.acquisition.started=1.
    return m


def test_retry_keeps_original_translation_at_eleven_centimetres_and_excludes_travel(monkeypatch):
    goal=pose(-.30);m=mission(goal,True)
    assert m.goal_requires_translation(goal,'acquire',True,(-.19,0.,0.)) is True
    # Without a retry, the historical .12 m threshold classifies this distance
    # as a turn; preserving task identity is what prevents the work-time error.
    assert m.goal_requires_translation(goal,'acquire',False,(-.19,0.,0.)) is False
    clock=[0.];monkeypatch.setattr(time,'monotonic',lambda:clock[0])
    m.update_localization_clock();clock[0]=10.;m.update_localization_clock()
    assert m.acquisition.travel_seconds==10. and m.acquisition.active_seconds==0.
    m.navigation_status['stage']='FINAL_ALIGN';m.update_localization_clock()
    clock[0]=15.;m.update_localization_clock()
    assert m.acquisition.travel_seconds==10. and m.acquisition.active_seconds==5.


def test_pure_turn_retry_does_not_become_travel(monkeypatch):
    goal=pose(0.,yaw=1.2);m=mission(goal,False)
    assert m.goal_requires_translation(goal,'acquire',True,(0.,0.,.5)) is False
    clock=[0.];monkeypatch.setattr(time,'monotonic',lambda:clock[0])
    m.update_localization_clock();clock[0]=8.;m.update_localization_clock()
    assert m.acquisition.active_seconds==8. and m.acquisition.travel_seconds==0.


def test_different_goal_or_kind_does_not_inherit_old_translation_flag():
    m=mission(pose(-.30),True)
    assert m.goal_requires_translation(pose(-.19,yaw=.5),'acquire',True,(-.19,0.,0.)) is False
    assert m.goal_requires_translation(pose(-.30),'home',True,(-.19,0.,0.)) is False
    assert m.goal_requires_translation(pose(-.60),'acquire',True,(-.19,0.,0.)) is True


def test_deferred_retry_stamp_can_change_without_changing_motion_identity():
    m=mission(pose(-.30),True)
    same=pose(-.30);same.header.stamp=NS(sec=123,nanosec=0)
    assert m.goal_requires_translation(same,'acquire',True,(-.19,0.,0.)) is True
