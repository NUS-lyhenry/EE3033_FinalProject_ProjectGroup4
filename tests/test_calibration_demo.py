"""The clickable demo exercises the real fitter without ROS or motor access."""
import importlib.util
from pathlib import Path

import pytest

from ee3033_project.calibration import load_profile

spec = importlib.util.spec_from_file_location('calibration_demo', Path(__file__).resolve().parents[1]/'scripts/simulate_calibration.py')
demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)


def test_clickable_simulation_can_complete_real_fit_and_validation(tmp_path):
    session = demo.Simulation(tmp_path)
    def command(name, values=None):
        session.command(name, dict(generation=session.generation, **(values or {})))
    command('setup', demo.PHYSICAL)
    for index in range(8):
        if index == 6:
            command('fit')
            with pytest.raises(ValueError):
                command('save')
        command('freeze')
        sample = session.observation()
        command('sample', {**sample, 'freeze_id': session.frozen['id'], 'validation': index >= 6})
        if index < 7:
            command('scene')
    command('save')
    assert len(session.profile['training']) == 6
    assert all(v['passed'] for v in session.profile['validation'])
    assert session.profile['simulated'] is True
    with pytest.raises(ValueError):
        load_profile(session.saved)
    assert session.snapshot()['simulation_reference']['scene'] == 8
    assert session.image().startswith(b'\xff\xd8')


def test_demo_rejects_recording_without_frozen_frame_and_stale_controls(tmp_path):
    session = demo.Simulation(tmp_path)
    with pytest.raises(ValueError, match='Freeze'):
        session.command('sample', dict(generation=0))
    session.command('stop', {})
    with pytest.raises(ValueError, match='context changed'):
        session.command('move', dict(generation=0, mode='forward'))


def test_automatic_demo_needs_only_three_inputs_and_saves_simulation_evidence(tmp_path, monkeypatch):
    clock=[100.]
    monkeypatch.setattr(demo.time,'monotonic',lambda:clock[0])
    session=demo.Simulation(tmp_path)
    session.command('automatic',dict(generation=0,bottle_height=.25,bottle_diameter=.065,distance=.9))
    for _ in range(18):
        clock[0]+=.31
        state=session.snapshot()
        if state['motion']=='auto':session.command('heartbeat',dict(generation=state['generation']))
    assert session.automatic['state']=='complete'
    assert len(session.profile['automatic']['observations'])==18
    assert len(session.profile['validation'])==2
    assert session.profile['simulated'] is True
    with pytest.raises(ValueError):load_profile(session.saved)


def test_automatic_demo_stops_when_browser_heartbeat_is_lost(tmp_path, monkeypatch):
    clock=[100.]
    monkeypatch.setattr(demo.time,'monotonic',lambda:clock[0])
    session=demo.Simulation(tmp_path)
    session.command('automatic',dict(generation=0,bottle_height=.25,bottle_diameter=.065,distance=.9))
    clock[0]+=1.
    state=session.snapshot()
    assert state['motion']=='idle' and state['automatic']['state']=='stopped'
    assert session.saved is None
