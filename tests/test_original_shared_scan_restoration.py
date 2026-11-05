"""Explicit operator-requested restoration: original radial invalid-data feed."""
from pathlib import Path
import math
import yaml
from ee3033_project.scan_core import resample
ROOT=Path(__file__).parents[1]


def test_near_returns_are_unknown_nan_not_zero_or_free_rays_at_operator_twenty_cm():
    values=[.05,.122355,.134265,.152020,.199,.20,.25,math.inf]
    result=resample(values,-math.pi,2*math.pi/len(values),0.,8.,len(values),.20)
    assert all(math.isnan(value) for value in result[:5])
    assert result[5]==.20 and result[6]==.25 and math.isinf(result[7])


def test_all_navigation_profiles_and_final_gate_use_original_shared_filtered_topic():
    config=ROOT/'src/ee3033_project/config'
    for path in config.glob('navigation*.yaml'):
        values=yaml.safe_load(path.read_text())
        for name in ('local_costmap','global_costmap'):
            scan=values[name][name]['ros__parameters']['obstacle_layer']['scan']
            assert scan['topic']=='/scan_fixed'
        assert values['amcl']['ros__parameters']['scan_topic']=='/scan_fixed'
    project=yaml.safe_load((config/'project.yaml').read_text())
    assert 'scan_topic' not in project['project_velocity_gate']['ros__parameters']
    assert "LaserScan, '/scan_fixed'" in (ROOT/'src/ee3033_project/ee3033_project/velocity_gate.py').read_text()
