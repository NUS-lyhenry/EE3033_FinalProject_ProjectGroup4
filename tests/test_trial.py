import math
import pytest
from ee3033_project.trial import parameter_plan, validate_trial


def values(**overrides):
    return dict({'max_linear':.12,'max_angular':.4,'goal_timeout':60.,'lookahead_dist':.18,'alignment_angle':.35},**overrides)


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
    ('max_linear',.31),('max_linear',0.),('max_angular',math.nan),
    ('max_angular',math.inf),('max_angular',1.05),
    ('goal_timeout',301),('max_linear',True),
    ('max_linear','0.12'),
    ('lookahead_dist',0.),('lookahead_dist',.31),
    ('lookahead_dist',float('nan')),('alignment_angle',.01),
    ('alignment_angle',1.58),('alignment_angle',True),
])
def test_trial_rejects_invalid_values(key,bad):
    trial=values()
    trial[key]=bad
    with pytest.raises(ValueError):
        validate_trial(trial)


@pytest.mark.parametrize('phase',['mapping','mission'])
@pytest.mark.parametrize('linear,angular,timeout',[
    (.19,.65,125.), (.30,1.00,300.),
])
def test_higher_limits_apply_consistently(phase,linear,angular,timeout):
    requested=values(max_linear=linear,max_angular=angular,goal_timeout=timeout)
    assert validate_trial(requested)==requested
    plan=parameter_plan(requested,phase)
    assert plan['project_velocity_gate']=={'max_linear':linear,'max_angular':angular}
    assert plan['controller_server']['FollowPath.desired_linear_vel']==linear
    assert plan['controller_server']['FollowPath.max_linear_vel']==linear
    assert plan['controller_server']['FollowPath.rotate_to_heading_angular_vel']==angular
    assert plan['controller_server']['FollowPath.max_angular_vel']==angular
    assert plan['project_navigation_supervisor']=={'goal_timeout':timeout}
    if phase=='mapping':
        assert plan['project_mapper']=={'goal_timeout':timeout}
    else:
        assert plan['project_mission']=={'goal_timeout':timeout,
            'search_linear_speed':linear,'search_angular_speed':angular}


def test_trial_requires_exact_fields():
    with pytest.raises(ValueError):
        validate_trial({'max_linear':.12})
    with pytest.raises(ValueError):
        validate_trial({**values(),'rotation_clearance':0.})


@pytest.mark.parametrize('phase',['mapping','mission'])
def test_tracking_settings_apply_without_changing_speed_or_protection(phase):
    requested=values(lookahead_dist=.08,alignment_angle=.10)
    plan=parameter_plan(requested,phase)
    controller=plan['controller_server']
    assert controller['FollowPath.lookahead_dist']==.08
    assert controller['FollowPath.min_lookahead_dist']==.08
    assert controller['FollowPath.max_lookahead_dist']==.08
    assert controller['FollowPath.rotate_to_heading_min_angle']==.10
    assert plan['project_velocity_gate']=={'max_linear':.12,'max_angular':.4}
    assert not any('collision' in key or 'radius' in key
                   for params in plan.values() for key in params)


def test_legacy_speed_request_has_old_tracking_defaults():
    trial=validate_trial(dict(max_linear=.12,max_angular=.4,goal_timeout=60.))
    assert trial==values()
