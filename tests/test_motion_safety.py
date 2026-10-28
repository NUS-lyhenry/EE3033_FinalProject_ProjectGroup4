"""Final-command safety checks exercise geometry and the production gate body."""
import ast
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace as NS
import numpy as np
import pytest

from ee3033_project import motion_safety as motion
from ee3033_project.core import quaternion_yaw, stamp_seconds
from ee3033_project.reverse_safety import reverse_clearance


@pytest.mark.parametrize('v,w',[(.12,.8),(.24,.4),(-.24,-.8),(.1,-.8),(0.,2.),(.3,0.),(0.,0.)])
def test_speed_limiting_never_changes_turn_curvature_or_sign(v,w):
    vv,ww,scale=motion.limit_twist(v,w,.12,.4)
    assert abs(vv)<=.12+1e-12 and abs(ww)<=.4+1e-12
    assert vv==pytest.approx(v*scale) and ww==pytest.approx(w*scale)
    assert 0<scale<=1.
    if v:assert ww/vv==pytest.approx(w/v)


def test_invalid_limits_cannot_publish_unbounded_motion():
    for args in [(1.,0.,0.,.4),(1.,0.,.12,-1.),(float('nan'),0.,.12,.4)]:
        with pytest.raises(ValueError):motion.limit_twist(*args)


def grid():return np.zeros((100,100),dtype=np.int8),.05,(-2.5,-2.5,0.)


@pytest.mark.parametrize('value,reason',[(-1,'unknown'),(25,'obstacle'),(100,'obstacle')])
def test_full_footprint_blocks_cell_beside_arc_not_just_its_centre(value,reason):
    data,res,origin=grid()
    # 0.20 m forward, 0.10 m lateral: outside the centreline, inside the body.
    data[52,54]=value
    result=motion.swept_map_clearance(data,res,origin,(0.,0.,0.),.12,0.,2.)
    assert not result.allowed and reason in result.reason


def test_polyline_check_cannot_skip_a_thin_wall_between_distant_waypoints():
    data,res,origin=grid();data[:,55]=100
    result=motion.polyline_map_clearance(data,res,origin,[[0.,0.],[1.,0.]])
    assert not result.allowed


def test_rotated_map_origin_has_equivalent_collision_and_known_free_results():
    data,res,origin=grid();data[52,54]=100
    for pose,org in [((0.,0.,0.),origin),((0.,0.,math.pi/2),(2.5,-2.5,math.pi/2))]:
        assert not motion.swept_map_clearance(data,res,org,pose,.12,0.,2.).allowed
    data[:]=0
    assert motion.swept_map_clearance(data,res,origin,(0.,0.,0.),.12,.4,1.).allowed
    assert not motion.swept_map_clearance(data,res,origin,(2.45,0.,0.),.12,0.,1.).allowed


def test_point_to_cell_square_detects_corner_intersection():
    data=np.zeros((50,50),np.int8);data[20,20]=100
    # Robot centre is outside radius of the occupied centre (.025,.025), but
    # the occupied square's lower-left corner intersects the physical disc.
    result=motion.polyline_map_clearance(data,.05,(-1.,-1.,0.),[[-.7*motion.FOOTPRINT_RADIUS,-.7*motion.FOOTPRINT_RADIUS]])
    assert not result.allowed


def test_scan_sweep_checks_turning_arc_and_does_not_invent_returns():
    arc=motion.swept_arc(.12,.4,1.)
    hit=arc[-1,:2]+[0.,.10]
    assert not motion.swept_scan_clearance([hit],.12,.4,1.).allowed
    assert motion.swept_scan_clearance([[2.,2.]],.12,.4,1.).allowed
    assert motion.swept_scan_clearance([], -.04,0.,1.).allowed


def test_circular_rotation_cannot_avoid_an_obstacle_inside_the_footprint():
    assert not motion.swept_scan_clearance([[.9*motion.FOOTPRINT_RADIUS,0.]],0.,.4,1.).allowed
    assert motion.swept_scan_clearance([[.3,0.]],0.,.4,1.).allowed


def twist():return NS(linear=NS(x=0.,y=0.,z=0.),angular=NS(x=0.,y=0.,z=0.))


def gate_fixture(v=.12,w=.4):
    source=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/velocity_gate.py'
    cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef))
    env=dict(Node=object,np=np,math=math,time=time,json=json,Twist=twist,
        Time=lambda:None,TransformException=LookupError,String=lambda **kw:NS(**kw),
        reverse_clearance=reverse_clearance,stamp_seconds=stamp_seconds,quaternion_yaw=quaternion_yaw,
        FOOTPRINT_RADIUS=motion.FOOTPRINT_RADIUS,geometry_metadata=lambda:{},limit_twist=motion.limit_twist,
        swept_arc=motion.swept_arc,swept_map_clearance=motion.swept_map_clearance,
        swept_scan_clearance=motion.swept_scan_clearance)
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),env)
    gate=env['VelocityGate'].__new__(env['VelocityGate'])
    now=time.monotonic()
    gate.p=dict(mapping_mode=False,allow_motion=True,heartbeat_timeout=.7,command_timeout=.5,scan_timeout=.7,
        max_linear=.12,max_angular=.4,rotation_clearance=.14,reverse_policy='recovery',base_frame='base_footprint',
        stop_distance=.22,enforce_swept_map=True,swept_horizon=1.,braking_deceleration=.30,footprint_radius=.15)
    cmd=twist();cmd.linear.x=v;cmd.angular.z=w
    gate.commands={'nav':(cmd,now),'manual':(twist(),-math.inf)}
    gate.automatic=False;gate.enabled=True
    gate.enable_time=gate.scan_time=now;gate.automatic_time=-math.inf
    gate.reverse_idle=None;gate.reverse_start=None;gate.last_status=0.
    stamp=NS(sec=100,nanosec=0)
    gate.scan=NS(header=NS(stamp=stamp,frame_id='base_footprint'),range_min=.08,range_max=8.,
        angle_min=-math.pi,angle_increment=2*math.pi/360,ranges=[math.inf]*360)
    data,res,origin=grid()
    q=NS(x=0.,y=0.,z=0.,w=1.)
    gate.map=NS(info=NS(resolution=res,height=100,width=100,
        origin=NS(position=NS(x=origin[0],y=origin[1]),orientation=q)),data=data.ravel())
    gate.tf=NS(lookup_transform=lambda *args:NS(header=NS(stamp=stamp),
        transform=NS(translation=NS(x=0.,y=0.,z=0.),rotation=q)))
    gate.get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000))
    gate.get_parameter=lambda name:NS(value=gate.p[name])
    outputs=[];statuses=[]
    gate.pub=NS(publish=outputs.append);gate.status_pub=NS(publish=lambda m:statuses.append(json.loads(m.data)))
    return gate,outputs,statuses


def test_production_gate_preserves_curvature_before_swept_check():
    gate,outputs,statuses=gate_fixture(.12,.8)
    gate.tick()
    assert outputs[-1].linear.x==pytest.approx(.06)
    assert outputs[-1].angular.z==pytest.approx(.4)
    assert statuses[-1]['speed_scale']==.5
    assert statuses[-1]['output_arc_frame']=='map'


def test_production_gate_stops_both_components_when_front_obstacle_blocks():
    gate,outputs,statuses=gate_fixture()
    gate.scan.ranges[180]=.20
    gate.tick()
    assert outputs[-1].linear.x==outputs[-1].angular.z==0.
    assert statuses[-1]['reason']=='forward_obstacle'


def test_production_gate_rejects_future_footprint_collision_and_unknown_space():
    for value in [25,-1]:
        gate,outputs,statuses=gate_fixture(.12,0.)
        gate.map.data[50*100+54]=value
        gate.tick()
        assert outputs[-1].linear.x==outputs[-1].angular.z==0.
        assert statuses[-1]['reason'].startswith('swept_footprint_')


def test_production_gate_fails_closed_on_missing_or_stale_pose_map():
    gate,outputs,statuses=gate_fixture()
    gate.map=None;gate.tick()
    assert outputs[-1].linear.x==outputs[-1].angular.z==0.
    assert statuses[-1]['reason']=='swept_map_missing'
    gate,outputs,statuses=gate_fixture()
    gate.get_clock=lambda:NS(now=lambda:NS(nanoseconds=102_000_000_000))
    gate.tick()
    assert statuses[-1]['reason']=='swept_transform_stale'
    assert outputs[-1].linear.x==outputs[-1].angular.z==0.


def test_production_gate_requires_declared_scan_frame_instead_of_guessing_base():
    gate,outputs,statuses=gate_fixture();gate.scan.header.frame_id=''
    gate.tick()
    assert statuses[-1]['reason']=='scan_transform_unavailable'
    assert outputs[-1].linear.x==outputs[-1].angular.z==0.


def test_laser_offset_is_applied_before_body_collision_check():
    gate,outputs,statuses=gate_fixture()
    gate.scan.header.frame_id='offset_laser';gate.scan.ranges[180]=.3
    class Time:
        @classmethod
        def from_msg(cls,stamp):return stamp
    gate.laser_hits.__func__.__globals__['Time']=Time
    def lookup(target,frame,stamp):
        assert target=='base_footprint' and frame=='offset_laser'
        return NS(transform=NS(translation=NS(x=-.2,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    gate.tf=NS(lookup_transform=lookup)
    hits,count=gate.laser_hits()
    assert count==360 and hits[0]==pytest.approx([.1,0.])
    assert not motion.swept_scan_clearance(hits,.12,.4,1.).allowed


def test_reenable_requires_a_new_command_after_stop():
    gate,outputs,statuses=gate_fixture()
    gate.on_enable(NS(data=False))
    assert gate.commands['nav'][1]==-math.inf
    # Simulate a late old Nav2 command while disabled, then a new generation.
    stale=twist();stale.linear.x=.12;stale.angular.z=.4
    gate.on_command(stale)
    gate.on_enable(NS(data=True))
    gate.tick()
    assert outputs[-1].linear.x==outputs[-1].angular.z==0.
    assert statuses[-1]['reason']=='command_stale'
