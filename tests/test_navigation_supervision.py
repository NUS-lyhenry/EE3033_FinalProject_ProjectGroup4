import math
import pytest
from ee3033_project.navigation_supervision import ProgressWatch, RecoveryBudget, path_reference


def test_rotation_and_replanned_heading_do_not_extend_translation_deadline():
    w=ProgressWatch()
    failure=None
    for i in range(121):
        t=i*.1
        failure=w.update(t,(0.,0.,t*.3),.8,aligning=True,reference=t*.4)
    assert failure=='No translation progress'


def test_only_net_translation_counts_and_small_oscillations_do_not():
    for pose,expected in [((.081,0.,0.),None),((.02,0.,1.),'No translation progress')]:
        w=ProgressWatch()
        assert w.update(0.,(0.,0.,0.),0.) is None
        assert w.update(12.,pose,0.)==expected


def test_converging_turn_is_allowed_but_frozen_heading_has_early_failure():
    w=ProgressWatch()
    assert w.update(0.,(0.,0.,0.),1.,aligning=True,reference=1.) is None
    assert w.update(3.,(0.,0.,.5),.5,aligning=True,reference=1.) is None
    assert w.update(6.,(0.,0.,.51),.49,aligning=True,reference=1.)=='Path heading is not converging'


def test_final_heading_has_its_own_window_and_does_not_require_translation():
    w=ProgressWatch()
    w.update(0.,(0.,0.,0.),1.)
    assert w.update(11.,(.4,0.,0.),.4,arrived=True,reference=.4) is None
    assert w.update(14.,(.4,0.,.3),.1,arrived=True,reference=.4) is None
    assert w.update(17.,(.4,0.,.3),.1,arrived=True,reference=.4)=='Final heading is not converging'
    assert w.update(20.,(.4,0.,.4),0.,arrived=True,reference=.4) is None


def test_sensor_nan_is_never_progress():
    w=ProgressWatch()
    assert 'Invalid' in w.update(0.,(math.nan,0.,0.),0.)


def test_two_recoveries_have_fixed_order_and_absolute_deadline():
    b=RecoveryBudget(10.,60.)
    assert b.next(20.,False)=='refresh'
    assert b.next(30.,True)=='reverse'
    assert b.next(40.,True) is None
    assert b.deadline==70.
    assert b.expired(70.)


def test_no_route_never_authorizes_reverse():
    b=RecoveryBudget(0.)
    assert b.next(5.,False)=='refresh'
    assert b.next(10.,False) is None
    assert b.count==1


def test_expired_budget_never_authorizes_another_action():
    b=RecoveryBudget(0.,5.)
    assert b.next(5.,True) is None
    with pytest.raises(ValueError):RecoveryBudget(0.,0.)


def test_heading_follows_path_arc_instead_of_cutting_to_later_point():
    # 18 cm ahead lies on the vertical leg, not across the corner chord.
    angle=path_reference([(0.,0.),(.1,0.),(.1,.2)],(0.,0.,0.))
    assert angle==pytest.approx(math.atan2(.08,.1))
    assert path_reference([(0.,0.)],(0.,0.,0.)) is None
