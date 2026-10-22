import copy
import pytest
from ee3033_project import gap


def snapshot(enabled=False, phase='mapping', audacity='cautious'):
    result = gap.parameter_plan(enabled, phase, audacity)
    for name in gap.COSTMAPS:
        result[name].update(footprint='[]',footprint_padding=.01,resolution=.05,
                           **{'inflation_layer.enabled':True,'inflation_layer.inflation_radius':.24})
    return result


def test_floor_and_reserved_policy_match_both_stages():
    for phase in ('mapping','mission'):
        off,on=snapshot(False,phase),snapshot(True,phase)
        assert gap.describe(off,phase)['enabled'] is False
        assert gap.describe(on,phase)['enabled'] is True
        for name in gap.COSTMAPS:
            assert off[name]['robot_radius']==.14
            assert on[name]['robot_radius']==pytest.approx(.18)
        consumer='project_mapper' if phase=='mapping' else 'project_mission'
        assert off[consumer]['clearance']==pytest.approx(.22)
        assert on[consumer]['clearance']==pytest.approx(.26)


def test_partial_updates_are_never_labelled_verified():
    result=snapshot()
    result[gap.COSTMAPS[0]]['robot_radius']=.18
    assert gap.describe(result,'mapping')['enabled'] is None
    assert not gap.describe(result,'mapping')['synchronized']


@pytest.mark.parametrize('key,value',[('footprint','[[0,0],[1,0],[0,1]]'),
    ('footprint_padding',0.),('inflation_layer.enabled',False),('inflation_layer.inflation_radius',.15)])
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
    for level, clearance in [('cautious', .22), ('balanced', .18), ('bold', .14)]:
        actual=snapshot(enabled,phase,level)
        data=gap.describe(actual,phase)
        assert data['audacity']==level and data['enabled'] is enabled
        assert data['synchronized']
        consumer='project_mapper' if phase=='mapping' else 'project_mission'
        expected=clearance+(.04 if enabled else 0)
        assert actual[consumer]['clearance']==pytest.approx(expected)
        for name in gap.COSTMAPS:
            assert actual[name]['robot_radius']==pytest.approx(.18 if enabled else .14)
            assert actual[name]['footprint_padding']==.01
        assert data['effective_min_opening']==pytest.approx(2*max(expected,(.19 if enabled else .15)))
        clearances.append(expected)
    assert clearances[0]>clearances[1]>clearances[2]


@pytest.mark.parametrize('value', [None, 1, True, [], {}, 'fast', 'BOLD'])
def test_unknown_audacity_is_rejected(value):
    with pytest.raises(ValueError): gap.parameter_plan(False,'mapping',value)


def test_level_readback_requires_both_costmaps_to_agree():
    actual=snapshot(False,'mapping','bold')
    actual[gap.COSTMAPS[1]]['robot_radius']=.18
    data=gap.describe(actual,'mapping')
    assert not data['synchronized'] and data['audacity'] is None


@pytest.mark.parametrize('width,expected',[(.30,[False,False,True]),(.40,[False,True,True]),(.50,[True,True,True])])
def test_levels_change_connected_passage_reachability(width, expected):
    import numpy as np
    from scipy.ndimage import label
    from ee3033_project.coverage import CoveragePlanner
    # Two rooms joined by a corridor. Test the planning reserve independently
    # of Nav2's unchanged physical footprint, padding and lidar gate.
    grid=np.full((100,100),100)
    grid[5:95,5:40]=0;grid[5:95,60:95]=0
    cells=round(width/.05)
    grid[50-cells//2:50-cells//2+cells,40:60]=0
    for level,passable in zip(gap.AUDACITY,expected):
        clearance=gap.parameter_plan(False,'mapping',level)['project_mapper']['clearance']
        planner=CoveragePlanner(grid,.05,(0,0,0),clearance=clearance)
        components,_=label(planner.safe,structure=[[0,1,0],[1,1,1],[0,1,0]])
        assert bool(components[50,20] and components[50,20]==components[50,80]) is passable
