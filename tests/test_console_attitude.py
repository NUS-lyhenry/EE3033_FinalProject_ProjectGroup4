"""Execute the production mixin without ROS; no driver or motion is started."""
import ast
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest

from ee3033_project.assessment import ACTIVE, atomic_json
from ee3033_project.attitude_geometry import (ATTITUDE_TOPIC, AttitudeBuffer, default_config,
    load_config, mount_fingerprint, reference_valid, validate_config)
from ee3033_project.calibration import mounting_rotation
from ee3033_project.stationary import StationaryWindow


def console(tmp_path):
    path = Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/console_attitude.py'
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
    scope = dict(math=math, np=np, threading=threading, Path=Path, ACTIVE=ACTIVE,
        atomic_json=atomic_json, ATTITUDE_TOPIC=ATTITUDE_TOPIC, AttitudeBuffer=AttitudeBuffer,
        default_config=default_config, load_config=load_config, mount_fingerprint=mount_fingerprint,
        reference_valid=reference_valid, validate_config=validate_config,
        mounting_rotation=mounting_rotation, StationaryWindow=StationaryWindow,
        TransformException=LookupError, Time=lambda **kw: kw,
        Vector3Stamped=object, Odometry=object, qos_profile_sensor_data=None)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), scope)
    node = scope['AttitudeSettings']()
    node.p = dict(session_directory=str(tmp_path), phase='mission')
    node.stage = None
    node.lock = threading.Lock()
    node.state = {'state': 'COMPLETE'}
    node.vision_settings = dict(camera_height=.33, camera_pitch=.499, camera_yaw=0.,
                               camera_x=0., camera_y=0., use_pitch_override=True)
    node.control_generation = 3
    node.create_subscription = lambda *a: None
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=101_100_000_000))
    node.require_idle = lambda: None
    node.require_zero_command = lambda: None
    node.init_attitude()
    for stamp in np.arange(100., 101.051, .05):
        node.attitude_samples.add(float(stamp), .01, .06)
        node.attitude_stationary.add(float(stamp), 0., 0., 0., 0., 0., 'odom')
    return node


def test_reference_captures_controller_angles_and_only_changes_controller_mount(tmp_path):
    node = console(tmp_path)
    node.attitude_settings(dict(action='configure', mode='controller', pivot=[0., 0., .033]), 3)
    assert not node.attitude_snapshot()['ready']
    result = node.attitude_settings(dict(action='reference'), 3)
    assert result['ready'] and result['reference']['valid']
    assert result['reference']['pitch'] == pytest.approx(.06)
    assert result['reference']['translation'] == pytest.approx([-.01, 0., .33])
    assert node.vision_settings['camera_x'] == 0.  # Fixed compatibility mount untouched.
    assert load_config(node.attitude_path)['reference']['roll'] == pytest.approx(.01)


@pytest.mark.parametrize('speed,accepted',[(.032,True),(.04,False)])
def test_reference_permits_measured_bounded_balance_but_refuses_sustained_speed(tmp_path,speed,accepted):
    node=console(tmp_path)
    node.attitude_settings(dict(action='configure',mode='controller',pivot=[0.,0.,.033]),3)
    node.attitude_stationary.reset()
    for stamp in np.arange(100.,101.051,.05):
        node.attitude_stationary.add(float(stamp),.013*math.sin(2*math.pi*stamp),0.,0.,speed,.02,'odom')
    if accepted:
        assert node.attitude_settings(dict(action='reference'),3)['reference']['valid']
    else:
        with pytest.raises(ValueError,match='Mean linear speed'):
            node.attitude_settings(dict(action='reference'),3)
        assert node.attitude_config['reference'] is None


def test_mount_or_pivot_change_invalidates_reference(tmp_path):
    node = console(tmp_path)
    node.attitude_settings(dict(action='configure', mode='controller', pivot=[0., 0., .033]), 3)
    node.attitude_settings(dict(action='reference'), 3)
    node.vision_settings['camera_pitch'] += .01
    assert not node.attitude_snapshot()['reference']['valid']
    node.attitude_settings(dict(action='configure', mode='controller', pivot=[0., 0., .04]), 3)
    assert node.attitude_snapshot()['reference'] is None


def test_stop_generation_change_cannot_persist_or_replace_previous_configuration(tmp_path):
    node = console(tmp_path)
    node.attitude_settings(dict(action='configure', mode='fixed', pivot=None), 3)
    old = node.attitude_path.read_bytes()
    with pytest.raises(ValueError, match='Stop superseded'):
        node.attitude_settings(dict(action='configure', mode='controller', pivot=[0., 0., .033]), 2)
    assert node.attitude_path.read_bytes() == old


def test_reference_refuses_client_angles_moving_body_and_nonzero_command(tmp_path):
    node = console(tmp_path)
    node.attitude_settings(dict(action='configure', mode='controller', pivot=[0., 0., .033]), 3)
    with pytest.raises(ValueError, match='captured'):
        node.attitude_settings(dict(action='reference', pitch=.1), 3)
    node.attitude_stationary.add(101.09, .2, 0., 0., .2, 0., 'odom')
    with pytest.raises(ValueError):
        node.attitude_settings(dict(action='reference'), 3)
    assert node.attitude_config['reference'] is None
    node = console(tmp_path)
    node.require_zero_command = lambda: (_ for _ in ()).throw(ValueError('nonzero command'))
    with pytest.raises(ValueError, match='nonzero'):
        node.attitude_settings(dict(action='reference'), 3)
    assert node.attitude_config['reference'] is None


def test_snapshot_is_safe_while_console_lock_is_held(tmp_path):
    node = console(tmp_path)
    with node.lock:
        assert node.attitude_snapshot()['mode'] == 'fixed'


def test_missing_capture_time_camera_tf_returns_api_error_without_saving_reference(tmp_path):
    node = console(tmp_path)
    node.vision_settings['camera_height'] = 0.
    node.tf = NS(lookup_transform=lambda *a: (_ for _ in ()).throw(LookupError('No TF')))
    node.attitude_settings(dict(action='configure', mode='controller', pivot=[0., 0., .033]), 3)
    before = node.attitude_path.read_bytes()
    with pytest.raises(ValueError, match='unavailable at the reference capture time'):
        node.attitude_settings(dict(action='reference'), 3)
    assert node.attitude_path.read_bytes() == before
    assert node.attitude_config['reference'] is None


def test_zero_command_is_checked_before_and_after_reference_capture(tmp_path):
    node = console(tmp_path)
    node.attitude_settings(dict(action='configure', mode='controller', pivot=[0., 0., .033]), 3)
    checks = []
    def command_check():
        checks.append(len(checks))
        if len(checks) == 2:
            raise ValueError('Output changed while capturing reference')
    node.require_zero_command = command_check
    with pytest.raises(ValueError, match='Output changed'):
        node.attitude_settings(dict(action='reference'), 3)
    assert checks == [0, 1]
    assert node.attitude_config['reference'] is None
