import copy
import pytest
from ee3033_project import gap


def snapshot(enabled=False, phase='mapping'):
    result = gap.parameter_plan(enabled, phase)
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
