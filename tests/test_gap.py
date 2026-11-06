import copy
from pathlib import Path
import pytest
import yaml
from ee3033_project import gap
from ee3033_project.robot_geometry import GEOMETRY_PROFILE,FOOTPRINT_PADDING,FOOTPRINT_RADIUS,MIN_CONFIGURED_COLLISION_RADIUS,ROTATION_CLEARANCES


def snapshot(enabled=False, phase='mapping', audacity=gap.DEFAULT_AUDACITY):
    result = gap.parameter_plan(enabled, phase, audacity)
    for name in gap.COSTMAPS:
        result[name].update(footprint='[]',footprint_padding=.01,resolution=.05,
                           **{'inflation_layer.enabled':True})
    result['controller_server']['progress_checker.plugin']=gap.PROGRESS_PLUGIN
    return result


def test_floor_and_reserved_policy_match_both_stages():
    for phase in ('mapping','mission'):
        off,on=snapshot(False,phase),snapshot(True,phase)
        assert gap.describe(off,phase)['enabled'] is False
        assert gap.describe(on,phase)['enabled'] is True
        for name in gap.COSTMAPS:
            assert off[name]['robot_radius']==gap.BASE_RADIUS
            assert on[name]['robot_radius']==pytest.approx(gap.RESERVED_RADIUS)
        consumer='project_mapper' if phase=='mapping' else 'project_mission'
        assert off[consumer]['clearance']==pytest.approx(.04)
        assert on[consumer]['clearance']==pytest.approx(.04+gap.RESERVED_RADIUS-gap.BASE_RADIUS)


@pytest.mark.parametrize('phase', ['mapping', 'mission'])
@pytest.mark.parametrize('enabled', [False, True])
def test_inflation_tracks_the_selected_robot_envelope(phase, enabled):
    actual=snapshot(enabled, phase)
    radius=gap.RESERVED_RADIUS if enabled else gap.BASE_RADIUS
    expected=max(.12, radius+FOOTPRINT_PADDING)
    for name in gap.COSTMAPS:
        assert actual[name]['inflation_layer.inflation_radius']==pytest.approx(expected)
    assert gap.describe(actual,phase)['synchronized']


@pytest.mark.parametrize('phase', ['mapping', 'mission'])
def test_twelve_cm_inflation_is_rejected_for_the_larger_reserved_envelope(phase):
    actual=snapshot(True, phase)
    actual[gap.COSTMAPS[0]]['inflation_layer.inflation_radius']=.12
    with pytest.raises(ValueError,match='unsupported footprint/inflation'):
        gap.describe(actual,phase)


def test_installed_yaml_defaults_match_ui_clearance_defaults():
    config=Path(__file__).resolve().parents[1]/'src/ee3033_project/config'
    project=yaml.safe_load((config/'project.yaml').read_text())
    nav=yaml.safe_load((config/'navigation.yaml').read_text())
    actual={}
    for name,entries in gap.parameter_plan(False,'mission').items():
        keys=list(entries)
        if name in gap.COSTMAPS:
            outer,inner=name.split('/')
            params=nav[outer][inner]['ros__parameters']
            keys+=['footprint','footprint_padding','resolution',
                   'inflation_layer.enabled','inflation_layer.inflation_radius']
        elif name=='controller_server':
            params=nav[name]['ros__parameters']
            keys+=['progress_checker.plugin']
        else:params=project[name]['ros__parameters']
        actual[name]={}
        for key in keys:
            value=params
            for part in key.split('.'):value=value[part]
            actual[name][key]=value
    # Source YAML defaults are measured; legacy launch explicitly overrides only
    # profile geometry/rotation while preserving the same soft clearances.
    if GEOMETRY_PROFILE=='legacy':
        legacy = gap.parameter_plan(False, 'mission')
        for name in gap.COSTMAPS + ('project_velocity_gate','project_mission'):
            actual[name].update(legacy[name])
    result=gap.describe(actual,'mission')
    assert result['audacity']=='bold' and result['synchronized']
    assert result['enabled'] is False
    assert result['tuning']['defaults'] == result['tuning']['requested'] == gap.MANUAL_DEFAULTS
    assert result['tuning']['live']['inflation_radius'] == pytest.approx(
        max(gap.MANUAL_DEFAULTS['inflation_radius'],
            gap.MANUAL_DEFAULTS['robot_radius'] + gap.MANUAL_DEFAULTS['footprint_padding']))


def test_partial_updates_are_never_labelled_verified():
    result=snapshot()
    result[gap.COSTMAPS[0]]['inflation_layer.inflation_radius']=.19
    assert gap.describe(result,'mapping')['enabled'] is None
    assert not gap.describe(result,'mapping')['synchronized']


@pytest.mark.parametrize('key,value',[('footprint','[[0,0],[1,0],[0,1]]'),
    ('footprint_padding',0.),('inflation_layer.enabled',False),('inflation_layer.inflation_radius',gap.BASE_RADIUS+FOOTPRINT_PADDING-.01)])
def test_unsupported_geometry_cannot_masquerade_as_policy(key,value):
    result=copy.deepcopy(snapshot())
    result[gap.COSTMAPS[0]][key]=value
    with pytest.raises(ValueError): gap.describe(result,'mapping')


@pytest.mark.parametrize('value',[None,0,1,'true',{},[]])
def test_toggle_requires_boolean(value):
    with pytest.raises(ValueError): gap.parameter_plan(value,'mapping')


@pytest.mark.parametrize('phase', ['mapping', 'mission'])
@pytest.mark.parametrize('enabled', [False, True])
def test_three_levels_preserve_geometry_and_gap_reserve(phase, enabled):
    clearances=[]
    for level, clearance in [('cautious', .10), ('balanced', .08), ('bold', .04)]:
        actual=snapshot(enabled,phase,level)
        data=gap.describe(actual,phase)
        assert data['audacity']==level and data['enabled'] is enabled
        assert data['synchronized']
        consumer='project_mapper' if phase=='mapping' else 'project_mission'
        expected=clearance+(gap.RESERVED_RADIUS-gap.BASE_RADIUS if enabled else 0)
        assert actual[consumer]['clearance']==pytest.approx(expected)
        for name in gap.COSTMAPS:
            assert actual[name]['robot_radius']==pytest.approx(gap.RESERVED_RADIUS if enabled else gap.BASE_RADIUS)
            assert actual[name]['footprint_padding']==.01
        assert data['effective_min_opening']==pytest.approx(2*max(expected,(gap.RESERVED_RADIUS if enabled else gap.BASE_RADIUS)+FOOTPRINT_PADDING,FOOTPRINT_RADIUS))
        clearances.append(expected)
    assert clearances[0]>clearances[1]>clearances[2]


@pytest.mark.parametrize('value', [None, 1, True, [], {}, 'fast', 'BOLD'])
def test_unknown_audacity_is_rejected(value):
    with pytest.raises(ValueError): gap.parameter_plan(False,'mapping',value)


def test_level_readback_requires_both_costmaps_to_agree():
    actual=snapshot(False,'mapping','bold')
    actual[gap.COSTMAPS[1]]['inflation_layer.inflation_radius']=.19
    data=gap.describe(actual,'mapping')
    assert not data['synchronized'] and data['audacity'] is None


def test_audacity_preserves_original_translation_checker_at_every_level():
    for level in gap.AUDACITY:
        plan=gap.parameter_plan(False,'mapping',level)
        assert plan['controller_server']=={}
    assert gap.PROGRESS_PLUGIN=='nav2_controller::SimpleProgressChecker'
    assert [gap.TURNING[level]['stop'] for level in gap.AUDACITY]==list(ROTATION_CLEARANCES.values())


def test_heading_checker_is_not_mislabelled_as_original_progress_policy():
    actual=snapshot(False,'mapping','bold')
    actual['controller_server']['progress_checker.plugin']='nav2_controller::PoseProgressChecker'
    with pytest.raises(ValueError,match='Reload navigation'):gap.describe(actual,'mapping')


def test_partial_turning_update_is_not_verified():
    actual=snapshot(False,'mapping','bold')
    actual['project_velocity_gate']['rotation_clearance']=.16
    assert not gap.describe(actual,'mapping')['synchronized']


@pytest.mark.parametrize('width,expected',[(.10,[False,False,True]),(.18,[False,True,True]),(.24,[True,True,True])])
def test_levels_change_connected_passage_reachability(width, expected):
    import numpy as np
    from scipy.ndimage import label
    from ee3033_project.coverage import CoveragePlanner
    # Two rooms joined by a corridor. Test the planning reserve independently
    # of Nav2's unchanged physical footprint, padding and lidar gate.
    grid=np.full((100,100),100)
    grid[5:95,5:40]=0;grid[5:95,60:95]=0
    cells=round(width/.01)
    grid[50-cells//2:50-cells//2+cells,40:60]=0
    for level,passable in zip(gap.AUDACITY,expected):
        clearance=gap.parameter_plan(False,'mapping',level)['project_mapper']['clearance']
        planner=CoveragePlanner(grid,.01,(0,0,0),clearance=clearance)
        components,_=label(planner.safe,structure=[[0,1,0],[1,1,1],[0,1,0]])
        assert bool(components[50,20] and components[50,20]==components[50,80]) is passable


@pytest.mark.parametrize('level', ['cautious','balanced','bold'])
def test_low_planning_threshold_cannot_shrink_padded_physical_opening(level):
    data=gap.describe(snapshot(False,'mapping',level),'mapping')
    assert data['effective_min_opening']==pytest.approx(2*FOOTPRINT_RADIUS)
    if GEOMETRY_PROFILE=='measured':
        assert data['effective_min_opening']==pytest.approx(.233602)


def test_custom_planning_radius_respects_gap_reserve_and_independent_collision_floor():
    tuning = dict(robot_radius=round(gap.BASE_RADIUS * .6, 6), footprint_radius=.18,
                  footprint_padding=.02, inflation_radius=.11)
    for enabled in (False, True):
        policy = dict(enabled=enabled, audacity='bold', tuning=tuning)
        actual = snapshot(enabled)
        for name, values in gap.parameter_plan(phase='mapping', **policy).items():
            actual[name].update(values)
        described = gap.describe(actual, 'mapping', policy)
        radius = gap.RESERVED_RADIUS if enabled else tuning['robot_radius']
        assert described['synchronized']
        assert described['tuning']['live']['robot_radius'] == radius
        assert described['tuning']['live']['footprint_radius'] == .18
        assert described['tuning']['live']['inflation_radius'] == max(.11, radius+.02)
        assert described['effective_min_opening'] >= .36
        assert not gap.describe(snapshot(enabled), 'mapping', policy)['synchronized']
    with pytest.raises(ValueError):
        gap.validate_tuning({'robot_radius': tuning['robot_radius']-.001})
    with pytest.raises(ValueError):
        gap.validate_tuning({'footprint_radius': MIN_CONFIGURED_COLLISION_RADIUS-.001})
    reduced = gap.parameter_plan(False, 'mapping', tuning={'footprint_radius': MIN_CONFIGURED_COLLISION_RADIUS})
    assert reduced['project_velocity_gate']['footprint_radius'] == MIN_CONFIGURED_COLLISION_RADIUS


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('level', gap.AUDACITY)
def test_legacy_settings_become_manual_without_changing_effective_values(enabled, level):
    legacy = dict(enabled=enabled, audacity=level,
                  tuning={'robot_radius':.10, 'clearance':.20, 'footprint_radius':.16})
    manual = gap.manual_policy(legacy)
    assert not manual['enabled'] and manual['audacity'] == gap.DEFAULT_AUDACITY
    assert gap.manual_policy(manual) == manual
    assert gap.parameter_plan(phase='mapping', **manual) == gap.parameter_plan(phase='mapping', **legacy)


def test_turning_stop_can_be_set_to_sixty_percent_of_default():
    minimum = round(ROTATION_CLEARANCES[gap.DEFAULT_AUDACITY] * .60, 6)
    tuning = {'rotation_clearance': minimum}
    for phase in ('mapping', 'mission'):
        gate = gap.parameter_plan(False, phase, tuning=tuning)['project_velocity_gate']
        assert gate['rotation_clearance'] == minimum
        assert gate['footprint_radius'] == FOOTPRINT_RADIUS
    with pytest.raises(ValueError):
        gap.validate_tuning({'rotation_clearance': minimum - .000001})
