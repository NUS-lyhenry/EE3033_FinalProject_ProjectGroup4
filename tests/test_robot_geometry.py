"""Profile selection is explicit and reproducible in fresh processes."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT=Path(__file__).parents[1]


def selected(profile=None,extra=None):
    env=dict(os.environ)
    env['PYTHONPATH']=str(ROOT/'src/ee3033_project')
    env.pop('EE3033_ROBOT_GEOMETRY',None)
    if profile is not None:env['EE3033_ROBOT_GEOMETRY']=profile
    if extra:env.update(extra)
    code='import json; from ee3033_project.robot_geometry import geometry_metadata; print(json.dumps(geometry_metadata()))'
    return subprocess.run([sys.executable,'-c',code],env=env,capture_output=True,text=True)


def test_default_measured_circle_conservatively_contains_all_four_measured_corners():
    value=json.loads(selected().stdout)
    assert value['profile']=='measured'
    assert value['length']==.115 and value['width']==.18
    exact=math.hypot(.09,.0575)
    assert value['body_radius']==.106801
    assert exact<=value['body_radius']<exact+1e-6
    assert value['padding']==.01 and value['footprint_radius']==.116801
    assert value['model']=='conservative_circumscribed_circle'
    assert value['front_stop_distance']==.186801
    assert value['rotation_clearances']==dict(cautious=.136801,balanced=.126801,bold=.116801)


def test_explicit_legacy_profile_preserves_the_old_pressure_geometry():
    value=json.loads(selected('legacy').stdout)
    assert value['profile']=='legacy' and value['length'] is None
    assert value['width']==.20 and value['body_radius']==.14
    assert value['padding']==.01 and value['footprint_radius']==.15
    assert value['front_stop_distance']==.22
    assert value['rotation_clearances']==dict(cautious=.16,balanced=.15,bold=.14)
    assert value['dimension_source']=='legacy_radius_and_estimated_width'


@pytest.mark.parametrize('value',['','auto','LEGACY','Measured','0',' measured','legacy '])
def test_invalid_selection_fails_without_silently_using_another_profile(value):
    result=selected(value)
    assert result.returncode!=0 and 'must be measured or legacy' in result.stderr


def test_unrelated_environment_never_implicitly_selects_legacy_or_changes_dimensions():
    value=json.loads(selected(extra={'EE3033_ROBOT_WIDTH':'5.0','ROBOT_RADIUS':'10','CI':'true'}).stdout)
    assert value['profile']=='measured' and value['body_radius']==.106801


def test_metadata_return_value_cannot_change_future_profile_reports():
    from ee3033_project.robot_geometry import geometry_metadata,GEOMETRY_METADATA
    value=geometry_metadata();value['rotation_clearances']['bold']=0.
    assert geometry_metadata()['rotation_clearances']['bold']>0
    assert GEOMETRY_METADATA['rotation_clearances']['bold']>0
