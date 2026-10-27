"""Safety and progress decisions for bounded navigation recovery."""
import math
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from ee3033_project.navigation_progress import NavigationProgress
from ee3033_project.reverse_safety import reverse_clearance
from ee3033_project.core import stamp_seconds, quaternion_yaw


def evaluate(checker, poses, distance=1., error=1., rotation_only=False):
    result=None
    for stamp,pose in poses:
        result=checker.check(stamp,pose,distance,error,rotation_only)
    return result


def test_frozen_and_oscillating_chassis_cannot_renew_progress():
    for moving in (lambda t:(0.,0.,0.),lambda t:(.025*math.sin(t),0.,.06*math.sin(t))):
        checker=NavigationProgress()
        assert evaluate(checker,[(t,moving(t)) for t in range(12)]) is None
        assert checker.check(12,moving(12),1.,1.)


def test_heading_progress_grace_is_bounded_for_translation():
    checker=NavigationProgress()
    for t in range(18):
        assert checker.check(t,(0.,0.,t*.1),1.,1.) is None
    assert checker.check(18,(0.,0.,1.8),1.,1.)


def test_real_translation_and_converging_final_heading_are_not_stuck():
    checker=NavigationProgress()
    for t in range(40):
        assert checker.check(t,(t*.04,0.,0.),1.,0.) is None
    checker.reset()
    for t in range(20):
        assert checker.check(t,(0.,0.,t*.04),.05,1.-t*.04,True) is None


def test_unconverged_final_heading_aborts_but_arrived_heading_does_not():
    checker=NavigationProgress()
    assert evaluate(checker,[(t,(0.,0.,0.)) for t in range(13)],.05,.8,True)
    checker.reset()
    assert evaluate(checker,[(t,(0.,0.,0.)) for t in range(30)],.05,.02,True) is None


def test_near_home_still_requires_translation_progress_until_xy_tolerance():
    checker=NavigationProgress()
    assert evaluate(checker,[(t,(0.,0.,0.)) for t in range(13)],.19,0.)


def test_final_heading_gets_its_own_window_after_eleven_second_journey():
    checker=NavigationProgress()
    for t in range(12):
        assert checker.check(t,(t*.10,0.,0.),max(.11,1.2-t*.10),.8) is None
    assert checker.check(11.9,(1.19,0.,0.),.09,.8) is None
    assert checker.phase=='heading' and checker.phase_started==11.9
    # The old implementation timed out here using the journey's first sample.
    assert checker.check(12.,(1.19,0.,0.),.09,.8) is None
    assert checker.check(23.8,(1.19,0.,0.),.09,.8) is None
    assert checker.check(24.,(1.19,0.,0.),.09,.8)=='Final heading is not converging'


def test_near_heading_window_survives_threshold_noise_without_resetting():
    checker=NavigationProgress()
    assert checker.check(0.,(0.,0.,0.),.09,.8) is None
    for t in range(1,12):
        assert checker.check(float(t),(0.,0.,0.),.11+.02*math.sin(t),.8) is None
        assert checker.phase=='heading' and checker.phase_started==0.
    assert checker.check(12.,(0.,0.,0.),.14,.8)=='Final heading is not converging'


def test_leaving_arrival_zone_restarts_translation_then_heading_windows():
    checker=NavigationProgress()
    assert checker.check(0.,(0.,0.,0.),.09,.8) is None
    assert checker.check(10.,(-.10,0.,0.),.16,.8) is None
    assert checker.phase=='translation' and checker.phase_started==10.
    assert checker.check(11.,(0.,0.,0.),.09,.8) is None
    assert checker.phase=='heading' and checker.phase_started==11.
    assert checker.check(22.9,(0.,0.,0.),.09,.8) is None
    assert checker.check(23.,(0.,0.,0.),.09,.8)=='Final heading is not converging'


def test_goal_specific_xy_tolerance_and_optional_home_heading():
    home=NavigationProgress();acquire=NavigationProgress()
    for t in range(13):
        assert home.check(t,(0.,0.,0.),.115,.8,xy_tolerance=.12,heading_required=False) is None
    assert home.phase=='heading'
    assert acquire.check(0.,(0.,0.,0.),.115,.8,xy_tolerance=.10) is None
    assert acquire.check(12.,(0.,0.,0.),.115,.8,xy_tolerance=.10).startswith('No translation progress')


def test_heading_only_spin_stays_bounded_even_with_no_translation_phase():
    checker=NavigationProgress()
    for t in range(12):
        assert checker.check(t,(0.,0.,float(t)),.30,.8,rotation_only=True) is None
    assert checker.check(12.,(0.,0.,12.),.30,.8,rotation_only=True)=='Final heading is not converging'


def test_hysteresis_does_not_loosen_the_actual_xy_arrival_tolerance():
    checker=NavigationProgress()
    assert checker.check(0.,(0.,0.,0.),.09,0.) is None
    # Stop at a residual XY error inside the phase exit band but outside arrival.
    for t in range(1,13):
        result=checker.check(t,(-.04,0.,0.),.14,0.)
    assert result is None  # the first window still includes the actual drift
    assert checker.check(13.,(-.04,0.,0.),.14,0.)=='Arrival position is not converging'


def safe_map():
    return np.zeros((100,100),dtype=np.int8),.05,(-2.5,-2.5,0.),(0.,0.,0.),np.diag([.0004,.0004,.0004])


def test_reverse_allowed_in_known_corridor_and_uncertainty_expands_envelope():
    grid,res,origin,pose,cov=safe_map()
    okay,reason,envelope=reverse_clearance(grid,res,origin,pose,cov)
    assert okay and envelope>.19
    cov[0,0]=.0064
    assert reverse_clearance(grid,res,origin,pose,cov)[2]>envelope


@pytest.mark.parametrize('obstacle',[-1,100])
def test_unknown_and_occupied_rear_swept_cells_block_reverse(obstacle):
    grid,res,origin,pose,cov=safe_map()
    # x=-0.35, y=0 is behind the current footprint but in the swept envelope.
    grid[50,43]=obstacle
    assert not reverse_clearance(grid,res,origin,pose,cov)[0]


def test_reverse_rejects_bad_covariance_and_outside_map():
    grid,res,origin,pose,cov=safe_map()
    for bad in (np.diag([.04,.04,.01]),np.diag([.001,.001,.09]),np.diag([-1.,.001,.001])):
        assert not reverse_clearance(grid,res,origin,pose,bad)[0]
    assert not reverse_clearance(grid,res,origin,(-2.4,0.,0.),cov)[0]


def test_reverse_rejects_nonphysical_radius_reserve_and_nan_map():
    grid,res,origin,pose,cov=safe_map()
    assert not reverse_clearance(grid,res,origin,pose,cov,radius=-1.)[0]
    assert not reverse_clearance(grid,res,origin,pose,cov,reserve=-1.)[0]
    grid=grid.astype(float);grid[50,43]=float('nan')
    assert not reverse_clearance(grid,res,origin,pose,cov)[0]


def test_rotated_map_keeps_the_same_reverse_geometry():
    grid,res,origin,pose,cov=safe_map()
    grid[50,43]=100
    assert not reverse_clearance(grid,res,origin,pose,cov)[0]
    assert not reverse_clearance(grid,res,(2.5,-2.5,math.pi/2),(0.,0.,math.pi/2),cov)[0]


def test_all_navigation_trees_only_reverse_after_refresh_retry():
    root=Path(__file__).parents[1]/'src/ee3033_project/behavior_trees'
    for name in ('navigate_with_bounded_recovery.xml','navigate_observation.xml','navigate_home.xml'):
        tree=ET.parse(root/name).getroot().find('BehaviorTree')
        outer=tree[0]
        assert outer.tag=='RecoveryNode' and outer.attrib['number_of_retries']=='1'
        assert outer[0].tag=='RecoveryNode'
        assert outer[0][1].find('Wait') is not None
        assert outer[1].tag=='BackUp'
        assert float(outer[1].attrib['backup_dist'])<=.20
        assert float(outer[1].attrib['backup_speed'])<=.04


def gate_fixture():
    # Exercise the production method body on a ROS-free message/TF fixture.
    path=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/velocity_gate.py'
    cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef))
    env=dict(Node=object,np=np,math=math,Time=lambda:None,TransformException=LookupError,
             reverse_clearance=reverse_clearance,stamp_seconds=stamp_seconds,quaternion_yaw=quaternion_yaw)
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),env)
    gate=env['VelocityGate'].__new__(env['VelocityGate'])
    stamp=NS(sec=100,nanosec=0);q=NS(x=0.,y=0.,z=0.,w=1.)
    gate.transform=NS(header=NS(stamp=stamp),transform=NS(translation=NS(x=0.,y=0.,z=0.),rotation=q))
    gate.tf=NS(lookup_transform=lambda *args:gate.transform)
    gate.get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000))
    grid,res,origin,pose,cov=safe_map()
    gate.map=NS(info=NS(resolution=res,height=100,width=100,
        origin=NS(position=NS(x=origin[0],y=origin[1]),orientation=q)),data=grid.ravel())
    full=np.zeros((6,6));full[np.ix_([0,1,5],[0,1,5])]=cov
    gate.localization=NS(header=NS(stamp=stamp),pose=NS(covariance=full.ravel()))
    gate.p=dict(reverse_distance=.20,reverse_radius=.15,reverse_reserve=.04,
                base_frame='base_footprint',odom_frame='odom_combined')
    gate.reverse_start=None
    return gate


def test_reverse_gate_enforces_continuous_distance_and_time_bounds():
    gate=gate_fixture()
    assert gate.approve_reverse(10.)[0]
    assert not gate.approve_reverse(18.1)[0]
    gate=gate_fixture();assert gate.approve_reverse(10.)[0]
    gate.transform.transform.translation.x=-.201
    assert not gate.approve_reverse(11.)[0]


def test_reverse_gate_requires_fresh_localization_even_when_map_is_clear():
    gate=gate_fixture()
    gate.localization.header.stamp=NS(sec=90,nanosec=0)
    assert gate.approve_reverse(10.)==(False,'reverse_localization_stale')
