"""Independent physical boundaries for optional mapper planning preferences.

The measured circle is unchanged when length and width are corrected. These
tests explicitly select measured in a fresh process, including in a legacy run.
A native mask supplies the externally checked planning corridor; a soft policy
may prefer more room, but cannot enlarge unknown space or shrink the chassis.
"""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def measured_boundaries():
    code = r'''
import json
from pathlib import Path
import numpy as np
from ee3033_project.robot_geometry import geometry_metadata
from ee3033_project.gap import RESERVED_RADIUS,parameter_plan
from ee3033_project.motion_safety import FOOTPRINT_RADIUS
from ee3033_project.path_validation import guarded_plan,validate_path
from ee3033_project.waypoint import waypoint_goal
from navigation_benchmark_geometry import occupancy_grid,physically_reachable

def probe(name,resolution,clearance,*,native=True,unknown=False):
    case=json.loads((Path('validation/navigation_cases')/(name+'.json')).read_text())
    m=occupancy_grid(case['scenario'],resolution)
    grid=np.asarray(m['cells']).reshape(m['height'],m['width'])
    start=case['scenario']['start'];goal=case['goal']
    if unknown:
        # The complete vertical band is unobserved, including the open doorway.
        xx=m['origin'][0]+(np.arange(m['width'])+.5)*resolution
        grid[:,np.abs(xx)<.05]=-1
    kwargs={'safe_mask':np.ones_like(grid,dtype=bool)} if native else {}
    try:
        chosen,groups,exact=waypoint_goal(grid,resolution,m['origin'],start,goal,
            clearance=clearance,**kwargs)
        error=None
    except ValueError as exc:
        chosen=None;exact=False;error=str(exc)
    check=guarded_plan(grid,resolution,m['origin'],start[:2],goal[:2])
    return dict(exact=bool(exact),chosen=chosen,error=error,physical_path=check.allowed,
        oracle=physically_reachable(case['scenario'],goal,radius=.116801),
        chosen_footprint_safe=chosen is None or validate_path(
            grid,resolution,m['origin'],[chosen[:2]],radius=.116801).allowed)

result=dict(geometry=geometry_metadata(),reserved_radius=RESERVED_RADIUS,
    disabled_plan=parameter_plan(False,'mapping'),enabled_plan=parameter_plan(True,'mapping'),
    footprint=FOOTPRINT_RADIUS)
result['door28']=probe('too_narrow',.0125,.14)
result['door28_without_native']=probe('too_narrow',.0125,.14,native=False)
result['door28_coarse']=probe('too_narrow',.05,.001)
result['door28_medium']=probe('too_narrow',.025,.001)
result['door20']=probe('measured_too_narrow',.0125,.001)
result['unknown28']=probe('too_narrow',.0125,.001,unknown=True)
print(json.dumps(result))
'''
    env=dict(os.environ,EE3033_ROBOT_GEOMETRY='measured',
             PYTHONPATH=os.pathsep.join((str(ROOT/'src/ee3033_project'),str(ROOT/'scripts'))))
    result=subprocess.run([sys.executable,'-c',code],cwd=ROOT,env=env,
                          capture_output=True,text=True,timeout=25)
    assert result.returncode==0,result.stderr
    return json.loads(result.stdout)


def test_measured_width_controls_optional_side_reserve_but_not_circle(measured_boundaries):
    data=measured_boundaries
    assert data['geometry']['length']==.115
    assert data['geometry']['width']==.18
    assert data['footprint']==.116801
    assert data['reserved_radius']==pytest.approx(.17)
    assert data['disabled_plan']['global_costmap/global_costmap']['robot_radius']==.106801
    assert data['enabled_plan']['global_costmap/global_costmap']['robot_radius']==pytest.approx(.17)
    assert 'footprint_radius' not in data['disabled_plan']['project_velocity_gate']


def test_fresh_native_corridor_may_override_only_the_soft_margin(measured_boundaries):
    door=measured_boundaries['door28']
    assert door['oracle'] and door['physical_path']
    assert door['exact'], 'A proven .28 m doorway must not be disconnected only by soft .14 m EDT preference'
    assert door['chosen_footprint_safe']


def test_no_native_corridor_proof_preserves_the_explicit_soft_margin(measured_boundaries):
    assert not measured_boundaries['door28_without_native']['exact']


@pytest.mark.parametrize('case',['door28_coarse','door28_medium'])
def test_coarse_map_uncertainty_cannot_be_erased_by_lowering_soft_margin(measured_boundaries,case):
    door=measured_boundaries[case]
    assert door['oracle'], 'Physical world remains .28 m; only occupancy evidence is coarser'
    assert not door['physical_path'] and not door['exact']


def test_too_narrow_door_stays_blocked_even_with_tiny_soft_margin(measured_boundaries):
    door=measured_boundaries['door20']
    assert not door['oracle'] and not door['physical_path'] and not door['exact']
    assert door['chosen_footprint_safe']
    assert measured_boundaries['footprint']==.116801


def test_unknown_doorway_is_never_a_proven_direct_route(measured_boundaries):
    door=measured_boundaries['unknown28']
    assert door['oracle'], 'Physical opening alone does not establish observed free space'
    assert not door['physical_path'] and not door['exact']
    assert door['chosen_footprint_safe']
