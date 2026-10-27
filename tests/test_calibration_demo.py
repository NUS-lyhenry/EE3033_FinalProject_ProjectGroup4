"""The map-point simulation uses the real fitter and cannot create robot profiles."""
import importlib.util
from pathlib import Path
import pytest
from ee3033_project.calibration import load_profile
spec=importlib.util.spec_from_file_location('calibration_demo',Path(__file__).resolve().parents[1]/'scripts/simulate_calibration.py')
demo=importlib.util.module_from_spec(spec);spec.loader.exec_module(demo)


def test_map_point_simulation_fits_and_saves_without_a_distance_input(tmp_path,monkeypatch):
    clock=[100.];monkeypatch.setattr(demo.time,'monotonic',lambda:clock[0]);session=demo.Simulation(tmp_path)
    def command(name,**body):session.command(name,dict(generation=session.generation,**body))
    command('target',xy=[.9,0.]);command('start',bottle_height=.215,bottle_diameter=.065)
    for _ in range(17):
        clock[0]+=.31;session.snapshot();command('heartbeat')
    assert session.automatic['state']=='complete'
    assert len(session.profile['map_calibration']['observations'])==17
    assert session.image().startswith(b'\xff\xd8')
    with pytest.raises(ValueError):load_profile(session.saved)


def test_demo_requires_map_point_and_stops_without_heartbeat(tmp_path,monkeypatch):
    clock=[100.];monkeypatch.setattr(demo.time,'monotonic',lambda:clock[0]);session=demo.Simulation(tmp_path)
    with pytest.raises(ValueError,match='position'):session.command('start',dict(generation=0))
    session.command('target',dict(generation=0,xy=[.9,0.]))
    session.command('start',dict(generation=0,bottle_height=.215,bottle_diameter=.065))
    clock[0]+=1.;assert session.snapshot()['motion']=='idle'
    assert session.saved is None
    with pytest.raises(ValueError,match='context'):session.command('heartbeat',dict(generation=0))
