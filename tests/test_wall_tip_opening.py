"""Opening acceptance and wall-tip tracking reserve; no footprint reduction."""
import numpy as np
import pytest
from ee3033_project.robot_geometry import FOOTPRINT_RADIUS,PHYSICAL_RADIUS,ROUTE_RADIUS
from ee3033_project.path_validation import guarded_plan,validate_path,path_samples

@pytest.mark.parametrize('resolution',[.05,float(np.float32(.05))])
def test_six_free_5cm_cells_pass_even_with_preferred_tracking_budget(resolution):
    g=np.zeros((60,80),int);g[[0,-1],:]=100;g[:,[0,-1]]=100
    g[:,40]=100;g[27:33,40]=0
    result=guarded_plan(g,resolution,(0.,0.,0.),(.7,1.5),(3.3,1.5))
    assert result.allowed,result.reason
    assert validate_path(g,resolution,(0.,0.,0.),result.path,radius=FOOTPRINT_RADIUS).allowed
    assert FOOTPRINT_RADIUS==pytest.approx(.116801) and PHYSICAL_RADIUS==pytest.approx(.106801)


def test_tracking_budget_rejects_hugging_tip_without_changing_physical_geometry():
    g=np.zeros((60,60),int);g[25:35,30]=100
    # This connector clears the body but spends the tracking/error budget.
    path=path_samples([(.5,1.12),(2.2,1.12)],.005)
    assert validate_path(g,.05,(0.,0.,0.),path).allowed
    assert not validate_path(g,.05,(0.,0.,0.),path,radius=ROUTE_RADIUS).allowed
    rerouted=guarded_plan(g,.05,(0.,0.,0.),path[0],path[-1],path,radius=ROUTE_RADIUS)
    assert rerouted.allowed and rerouted.replaced


def test_home_budget_is_longer_but_absolute_and_keeps_recovery_limits():
    from ee3033_project.navigation_supervision import navigation_budget
    budget,reverse=navigation_budget(10.,60.,2,True,'/trees/navigate_home.xml')
    assert budget.deadline==190. and budget.maximum==2 and reverse
    assert not budget.expired(189.9) and budget.expired(190.)
    assert navigation_budget(0.,300.,2,False,'navigate_home.xml')[0].deadline==300.
    assert navigation_budget(0.,60.,2,True,'navigate_mapping.xml')[0].deadline==60.
