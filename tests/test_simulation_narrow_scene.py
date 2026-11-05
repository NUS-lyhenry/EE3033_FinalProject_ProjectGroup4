"""Scene setup must admit the measured robot without changing its envelope."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from simulation_scenarios import validate_scenario, ROBOT_CLEARANCE
from ee3033_project.robot_geometry import FOOTPRINT_RADIUS


def scene(width,offset):
    low=.165+offset-width/2
    high=low+width
    return dict(version=1,name='Measured narrow doorway',seed=None,
        bounds=[-1.,-1.,1.5,1.5],start=[-.4,.165+offset,0.],target=[.9,.165+offset,0.],
        walls=[[[-1.,-1.],[1.5,-1.]],[[1.5,-1.],[1.5,1.5]],
               [[1.5,1.5],[-1.,1.5]],[[-1.,1.5],[-1.,-1.]],
               [[.3,-1.],[.3,low]],[[.3,high],[.3,1.5]]])


@pytest.mark.parametrize('width',[.30,.35])
@pytest.mark.parametrize('offset',[0.,.011,.025])
def test_measured_30_and_35_cm_openings_survive_grid_phase(width,offset):
    value=scene(width,offset)
    assert ROBOT_CLEARANCE==FOOTPRINT_RADIUS==.116801
    assert validate_scenario(value)==value


def test_door_narrower_than_padded_body_is_still_rejected():
    with pytest.raises(ValueError,match='not connected'):
        validate_scenario(scene(.22,.011))
