"""Independent checks of elapsed-time, collisions and actual odometry."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from simulation_motion import advance_motion,motion_profile
from test_simulation_robot_reset import SimulatedRobot,namespace,transform,odometry

WALLS=np.array([[[-2.,-2.],[2.,-2.]],[[2.,-2.],[2.,2.]],[[2.,2.],[-2.,2.]],[[-2.,2.],[-2.,-2.]]])


def test_exact_arc_and_delayed_timer_preserve_all_elapsed_time():
    p,blocked=advance_motion([0.,0.,0.],[.12,.4],1.,WALLS,.14)
    assert not blocked
    assert p==pytest.approx([.3*math.sin(.4),.3*(1-math.cos(.4)),.4])


def test_swept_disc_cannot_tunnel_through_a_wall_with_free_final_position():
    walls=np.concatenate([WALLS,[[[0.,-1.],[0.,1.]]]])
    p,blocked=advance_motion([-.5,0.,0.],[2.,0.],.5,walls,.14)
    assert blocked and p[0]<=-.14


def test_collision_keeps_safe_rotation_but_does_not_claim_translation():
    walls=np.concatenate([WALLS,[[[.15,-1.],[.15,1.]]]])
    p,blocked=advance_motion([0.,0.,0.],[.3,.4],.5,walls,.14)
    assert blocked and p[0]<.011 and p[2]==pytest.approx(.2)


def robot_fixture(profile,clock):
    r=SimulatedRobot.__new__(SimulatedRobot)
    r.pose=[0.,0.,0.];r.motion=motion_profile(profile);r.command_queue=[]
    r.command=(.12,.4);r.command_time=0.;r.last_tick=0.;r.accept_commands=True
    r.walls=WALLS;r.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:NS(sec=1,nanosec=0)))
    r.tf=NS(sendTransform=lambda _:None);out=[];r.odom=NS(publish=out.append)
    return r,out


def test_frozen_chassis_reports_zero_twist_despite_nonzero_command(monkeypatch):
    monkeypatch.setattr(namespace['time'],'monotonic',lambda:.2)
    r,out=robot_fixture('frozen',.2);r.physics_tick()
    assert r.pose==[0.,0.,0.]
    assert out[-1].twist.twist.linear.x==0. and out[-1].twist.twist.angular.z==0.


def test_deadband_uses_actual_heading_change_for_odometry(monkeypatch):
    monkeypatch.setattr(namespace['time'],'monotonic',lambda:.2)
    r,out=robot_fixture(dict(angular_deadband=.08),.2);r.command=(.12,.04)
    r.physics_tick()
    assert r.pose==pytest.approx([.024,0.,0.])
    assert out[-1].twist.twist.linear.x==pytest.approx(.12)
    assert out[-1].twist.twist.angular.z==0.


def test_command_latency_and_expiry_are_integrated_inside_long_timer(monkeypatch):
    monkeypatch.setattr(namespace['time'],'monotonic',lambda:1.)
    r,out=robot_fixture(dict(command_delay=.1),1.);r.command=(.12,0.);r.command_queue=[(0.,r.command)]
    r.physics_tick()
    # Active at t=.1 through t=.5; the remaining .6 s is real stopped time.
    assert r.pose[0]==pytest.approx(.048)
    assert out[-1].twist.twist.linear.x==pytest.approx(.048)
    assert out[-1].twist.twist.angular.z==0.


@pytest.mark.parametrize('value',['bad',dict(command_delay=-1),dict(frozen=1),dict(extra=True)])
def test_invalid_physical_profiles_are_rejected(value):
    with pytest.raises(ValueError):motion_profile(value)
