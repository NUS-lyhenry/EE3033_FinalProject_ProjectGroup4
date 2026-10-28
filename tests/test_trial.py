import math
import pytest
from ee3033_project.trial import parameter_plan, validate_trial


def values(**overrides):
    return dict({'max_linear':.12,'max_angular':.4,'goal_timeout':60.},**overrides)


def test_trial_plan_keeps_controller_and_gate_in_sync():
    plan=parameter_plan(values())
    assert plan['controller_server']['FollowPath.desired_linear_vel']==plan['project_velocity_gate']['max_linear']
    assert plan['controller_server']['FollowPath.rotate_to_heading_angular_vel']==plan['project_velocity_gate']['max_angular']
    assert 'FollowPath.max_vel_theta' not in plan['controller_server']
    assert plan['controller_server']['FollowPath.max_angular_vel']==.4
    assert plan['controller_server']['FollowPath.max_linear_vel']==.12
    assert plan['project_mapper']=={'goal_timeout':60.}
    assert plan['project_navigation_supervisor']==plan['project_mapper']
    assert all('clearance' not in key and 'stop_distance' not in key
               for entries in plan.values() for key in entries)


@pytest.mark.parametrize('key,bad',[
    ('max_linear',.19),('max_linear',0.),('max_angular',math.nan),
    ('max_angular',math.inf),('goal_timeout',121),('max_linear',True),
    ('max_linear','0.12'),
])
def test_trial_rejects_invalid_values(key,bad):
    trial=values()
    trial[key]=bad
    with pytest.raises(ValueError):
        validate_trial(trial)


def test_trial_requires_exact_fields():
    with pytest.raises(ValueError):
        validate_trial({'max_linear':.12})
    with pytest.raises(ValueError):
        validate_trial({**values(),'rotation_clearance':0.})
