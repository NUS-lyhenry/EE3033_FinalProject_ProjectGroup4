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
from ee3033_project.motion_safety import FOOTPRINT_RADIUS
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


def test_optional_legacy_tree_bounds_reverse_and_active_profiles_do_not_duplicate_recovery():
    root=Path(__file__).parents[1]/'src/ee3033_project/behavior_trees'
    # Explicit opt-in legacy experiment retains its bounded recovery contract.
    tree=ET.parse(root/'navigate_with_bounded_recovery.xml').getroot().find('BehaviorTree')
    outer=tree[0]
    assert outer.tag=='RecoveryNode' and outer.attrib['number_of_retries']=='1'
    assert outer[0].tag=='RecoveryNode' and outer[0][1].find('Wait') is not None
    assert outer[1].tag=='BackUp'
    assert float(outer[1].attrib['backup_dist'])<=.20
    assert float(outer[1].attrib['backup_speed'])<=.04
    # Home/observation actions delegate recovery to the single supervisor.
    for name in ('navigate_observation.xml','navigate_home.xml'):
        tree=ET.parse(root/name).getroot().find('BehaviorTree')
        assert not list(tree.iter('RecoveryNode'))
        assert not list(tree.iter('BackUp'))


def gate_fixture():
    # Exercise the production method body on a ROS-free message/TF fixture.
    path=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/velocity_gate.py'
    cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef))
    env=dict(Node=object,np=np,math=math,Time=lambda:None,TransformException=LookupError,
             reverse_clearance=reverse_clearance,FOOTPRINT_RADIUS=FOOTPRINT_RADIUS,stamp_seconds=stamp_seconds,quaternion_yaw=quaternion_yaw)
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


# Mission is an action owner, not a second recovery state machine. Load its
# production callbacks without ROS imports to check that boundary directly.
def mission_callbacks():
    import json
    import time
    source=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/mission.py'
    cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='Mission')
    env=dict(math=math,time=time,json=json,Path=Path,
             NavigateToPose=NS(Goal=lambda:NS(behavior_tree='')),
             quaternion_yaw=quaternion_yaw,angle_difference=lambda a,b:math.atan2(math.sin(a-b),math.cos(a-b)))
    exec(compile(ast.Module(body=[n for n in cls.body if isinstance(n,ast.FunctionDef)],
                           type_ignores=[]),str(source),'exec'),env)
    return env


def test_mission_deadline_cancels_without_scheduling_a_navigation_retry():
    calls=[]
    node=NS(cancel=lambda state,reason:calls.append((state,reason)),
            send_goal=lambda *a:pytest.fail('Owner timeout must not resend a navigation goal'))
    mission_callbacks()['navigation_timed_out'](node,'Navigation goal time limit exceeded')
    assert calls==[('GOAL_FAILED','Navigation goal time limit exceeded')]
    node.cancel_next='GOAL_FAILED';node.goal_finished=lambda success:calls.append(success)
    mission_callbacks()['after_cancel'](node)
    assert calls[-1] is False and node.cancel_next is None


def test_mission_does_not_run_another_stall_detector_during_supervisor_recovery(monkeypatch):
    import time
    monkeypatch.setattr(time,'monotonic',lambda:100.)
    node=NS(p={'goal_timeout':75.,'mission_timeout':900.,'acquisition_seconds':45.,
               'accept_timeout':8.},get_parameter=lambda name:NS(value=75.),
            navigation_map=NS(poll=lambda now:None),update_localization_clock=lambda:None,
            state='ACQUIRING',ACTIVE={'ACQUIRING'},goal_handle=object(),goal_pending=False,
            heartbeat=lambda *a:None,odom_pose=(0.,0.,0.),started=90.,
            nav_progress=NS(check=lambda *a,**kw:pytest.fail('Mission must not compete with supervisor')),
            readiness_error=lambda **kw:None,pending_navigation=None,
            acquisition=NS(active_seconds=0.,expired=lambda now:False),return_budget_exhausted=lambda:False,
            goal_started=99.,last_status=100.)
    mission_callbacks()['tick'](node)
    assert node.state=='ACQUIRING'


def test_mission_navigation_diagnostics_are_goal_local_fresh_and_deduplicated():
    import json
    import time
    callback=mission_callbacks()['on_navigation_status']
    node=NS(state='ACQUIRING',goal_handle=NS(goal_id=NS(uuid=bytes(range(16)))),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000)),
            started_stamp=90.,goal_requested_stamp=95.,goal_handle_serial=7,goal_serial=7,
            goal_kind='acquire',goal_started=time.monotonic()-4.,
            navigation_status={},navigation_recovery_seen=set(),navigation_recoveries=[])
    event=dict(goal_id=bytes(range(16)).hex(),stamp=99.,recovery_count=1,
               stage='REFRESHING',last_failure_reason='No translation progress')
    callback(node,NS(data=json.dumps(event)))
    callback(node,NS(data=json.dumps(dict(event,stage='TRACKING'))))
    assert len(node.navigation_recoveries)==1
    assert node.navigation_recoveries[0]['reason']=='No translation progress'
    callback(node,NS(data=json.dumps(dict(event,recovery_count=2,stage='REVERSING'))))
    assert len(node.navigation_recoveries)==2
    for stale in (dict(event,goal_id='ff'*16),dict(event,stamp=89.),
                  dict(event,stamp=94.),dict(event,stamp=101.),dict(event,recovery_count=True)):
        callback(node,NS(data=json.dumps(stale)))
    assert len(node.navigation_recoveries)==2 and node.navigation_status['stage']=='REVERSING'
    node.state='CANCELING'
    callback(node,NS(data=json.dumps(event)))
    assert node.navigation_status['stage']=='REVERSING'


def test_supervisor_reverse_and_refresh_are_not_charged_as_localization_work(monkeypatch):
    import time
    from ee3033_project.target_acquisition import SingleTarget
    clock=[0.];monkeypatch.setattr(time,'monotonic',lambda:clock[0])
    node=NS(state='ACQUIRING',acquisition=SingleTarget(),pending_navigation=None,
            goal_translation=True,navigation_status={'stage':'REVERSING'})
    node.acquisition.started=1.
    callback=mission_callbacks()['update_localization_clock']
    callback(node);clock[0]=10.;callback(node)
    assert node.acquisition.active_seconds==node.acquisition.travel_seconds==0.
    node.navigation_status['stage']='TRACKING';callback(node)
    clock[0]=20.;callback(node)
    node.navigation_status['stage']='FINAL_ALIGN';callback(node)
    clock[0]=25.;callback(node)
    assert node.acquisition.travel_seconds==10. and node.acquisition.active_seconds==5.


@pytest.mark.parametrize('kind,heading,override,expected',[
    ('home',True,'custom-home.xml',''),
    ('home',False,'custom-home.xml','custom-home.xml'),
    ('home',False,'','navigate_home.xml'),
    ('acquire',False,'','navigate_observation.xml'),
    ('search',False,'',''),
])
def test_mission_home_heading_requirement_selects_the_correct_navigation_tree(monkeypatch,kind,heading,override,expected):
    import sys
    import types
    ament=types.ModuleType('ament_index_python')
    packages=types.ModuleType('ament_index_python.packages')
    packages.get_package_share_directory=lambda name:'/tmp/project-share'
    monkeypatch.setitem(sys.modules,'ament_index_python',ament)
    monkeypatch.setitem(sys.modules,'ament_index_python.packages',packages)
    sent=[]
    node=NS(goal_pending=False,goal_handle=None,nav=NS(server_is_ready=lambda:True,
                send_goal_async=lambda goal:(sent.append(goal) or NS(add_done_callback=lambda cb:None))),
            defer_for_costmap=lambda *a:False,navigation_map=NS(allowed=lambda *a:True),
            goal_serial=0,pose=lambda:(0.,0.,0.),goal_requires_translation=lambda *a:True,
            localization_motion_log=[],acquisition=NS(last_goal_reason='Observation',last_goal_action=None),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000)),
            p={'home_behavior_tree':override,'return_heading_required':heading},
            set_state=lambda *a:None,index=0,route=[1],heartbeat=lambda *a:None)
    goal=NS(pose=NS(position=NS(x=.4,y=.2),orientation=NS(x=0.,y=0.,z=0.,w=1.)))
    mission_callbacks()['send_goal'](node,goal,kind)
    assert len(sent)==1
    assert sent[0].behavior_tree.endswith(expected) if expected else sent[0].behavior_tree==''
