"""Exercise real simulated-robot callbacks with message-only ROS stubs."""
import ast
from collections import deque
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
import numpy as np

sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from simulation_geometry import clearance,raycast
from simulation_motion import motion_profile,advance_motion
from ee3033_project.motion_safety import PHYSICAL_RADIUS
from simulation_scenarios import default_scenario,generate_scenario,validate_scenario


def rotation():return NS(x=0.,y=0.,z=0.,w=1.)
def position():return NS(x=0.,y=0.,z=0.)
def transform():return NS(header=NS(),transform=NS(translation=position(),rotation=rotation()))
def odometry():return NS(header=NS(),pose=NS(pose=NS(position=position(),orientation=rotation())),
    twist=NS(twist=NS(linear=position(),angular=position())))
def set_yaw(q,a):q.z=math.sin(a/2);q.w=math.cos(a/2)

source=Path(__file__).parents[1]/'scripts/simulation_robot.py'
cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='SimulatedRobot')
namespace=dict(Node=object,math=math,np=np,time=time,json=__import__('json'),
    motion_profile=motion_profile,advance_motion=advance_motion,PHYSICAL_RADIUS=PHYSICAL_RADIUS,
    validate_scenario=validate_scenario,clearance=clearance,raycast=raycast,
    TransformStamped=transform,Odometry=odometry,LaserScan=lambda:NS(header=NS()),set_yaw=set_yaw)
exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),namespace)
SimulatedRobot=namespace['SimulatedRobot']


def test_scenario_apply_changes_sensor_walls_and_resets_actual_odometry_without_old_motion():
    robot=SimulatedRobot.__new__(SimulatedRobot)
    robot.world_lock=threading.RLock();robot.accept_commands=True;robot.command_queue=deque()
    robot.scene=default_scenario();robot.pose=[.6,-.4,.8];robot.command=(.12,.4)
    robot.command_time=robot.last_tick=time.monotonic();robot.phase='mission'
    robot.scene_status_after=-math.inf;robot.collision_rejections=3
    stamp=NS(sec=1,nanosec=0)
    robot.get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000,to_msg=lambda:stamp))
    transforms=[];odoms=[];scans=[]
    robot.tf=NS(sendTransform=transforms.append);robot.odom=NS(publish=odoms.append);robot.scan=NS(publish=scans.append)
    scene=generate_scenario(122)
    robot.pause_commands();robot.apply_scenario(scene,reset_pose=True)
    # A queued old command is ignored throughout the reset window.
    robot.on_command(NS(linear=NS(x=.12),angular=NS(z=.4)))
    robot.resume_commands();robot.tick();robot.scan_tick()
    assert robot.pose==scene['start'] and robot.phase=='mapping'
    assert robot.collision_rejections==0
    assert robot.command==(0.,0.) and odoms[-1].twist.twist.linear.x==0.
    assert [odoms[-1].pose.pose.position.x,odoms[-1].pose.pose.position.y]==scene['start'][:2]
    assert odoms[-1].header.frame_id=='odom_combined'
    assert np.array_equal(robot.walls,np.asarray(scene['walls']))
    angles=scans[-1].angle_min+np.arange(450)*scans[-1].angle_increment+robot.pose[2]
    assert np.allclose(scans[-1].ranges,raycast(robot.pose[:2],angles,np.asarray(scene['walls'])))
    # Old phase messages cannot make the new world render a target during mapping.
    robot.on_status(NS(data='{"stamp": 0.5, "phase": "mission"}'))
    assert robot.phase=='mapping'


def test_target_only_change_preserves_actual_robot_pose_and_continuous_odometry_origin():
    robot=SimulatedRobot.__new__(SimulatedRobot)
    robot.world_lock=threading.RLock();robot.accept_commands=False
    robot.pose=[.6,-.4,.8];robot.phase='mission';robot.command=(0.,0.)
    robot.get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000))
    scene=default_scenario();scene['target']=[-.7,.7,0.]
    robot.apply_scenario(scene,reset_pose=False)
    assert robot.pose==[.6,-.4,.8] and robot.phase=='mission'
    assert robot.target.tolist()==[-.7,.7,0.]
