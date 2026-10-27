"""Map-prior MMSE math and safety conditions, independent of ROS."""
import math
import numpy as np
import pytest
from ee3033_project.target_estimate import MapMMSE, map_signature


def detection(stamp=10.,x=.5,y=.5,sigma=.2,**values):
    return dict(stamp=stamp,x=x,y=y,sigma=sigma,confidence=.9,label='bottle',**values)


def tracker(grid=None,resolution=.02,origin=(0.,0.,0.),**kwargs):
    t=MapMMSE(**kwargs)
    t.set_map(np.zeros((50,50),int) if grid is None else grid,resolution,origin)
    return t


def update(t,n=4,**kwargs):
    for i in range(n):
        d=detection(stamp=10+i*.1,**kwargs)
        assert t.update(d,d['stamp'])
    return t.snapshot(d['stamp'])


def test_truncated_gaussian_matches_analytic_mmse_not_clamped_ml():
    t=tracker(np.zeros((300,100),int),resolution=.01,origin=(0.,-1.,0.),common_fraction=0.)
    r=update(t,n=1,x=1.1,y=.5,sigma=.2)
    # N(1.10,.20^2) truncated at [0,1]: analytic mean=.8717844, sigma=.1036300.
    assert r['x']==pytest.approx(.87178445,abs=1e-4)
    assert r['covariance'][0][0]**.5==pytest.approx(.1036300,abs=1e-4)
    assert r['x']<1. and r['stable'] is False


def test_interior_fusion_uses_inverse_covariance_with_common_error_floor():
    t=tracker(np.zeros((500,500),int),.02,(-5.,-5.,0.))
    assert t.update(detection(x=0.,y=0.,sigma=1.),10.)
    assert t.update(detection(stamp=10.1,x=1.,y=0.,sigma=2.),10.1)
    r=t.snapshot(10.1)
    assert r['likelihood_mean'][0]==pytest.approx(.2)
    assert r['likelihood_covariance'][0][0]>=3.2
    t=tracker(np.zeros((500,500),int),.02,(-5.,-5.,0.))
    r=update(t,30,x=0.,y=0.,sigma=.3)
    assert r['sigma']>=math.sqrt(.8)*.3
    assert r['stable'] and r['samples']==30


def test_unknown_cells_remain_possible_and_no_robot_radius_inflation():
    grid=np.zeros((100,100),int);grid[:,0]=100;grid[:,50:]=-1
    t=tracker(grid,.01)
    r=update(t,x=.025,y=.5,sigma=.01)
    assert .01<r['x']<.05 # target near wall, inside robot-centre exclusion distance
    t=tracker(grid,.01)
    r=update(t,x=.7,y=.5,sigma=.05)
    assert r['x']>.65 and not r['prior_conflict']
    assert np.all(t.prior_grid[:,0]==0.)
    assert np.all(t.prior_grid[:,50:]>0.)


def test_seed_component_and_rotated_map_support():
    grid=np.zeros((40,40),int);grid[:,20]=100
    t=tracker(grid,.1);t.set_seed(.5,.5)
    assert np.all(t.points[:,0]<2.)
    t=tracker(np.zeros((100,100),int),.01,(2.,3.,math.pi/2))
    r=update(t,x=1.5,y=3.5,sigma=.1)
    assert r['x']==pytest.approx(1.5,abs=1e-5)
    assert r['y']==pytest.approx(3.5,abs=1e-5)
    assert t.prior_grid[t.cell(r['x'],r['y'])]>0.


def test_nonconvex_posterior_projects_to_support_and_reports_projection_error():
    grid=np.zeros((100,100),int);grid[20:80,20:80]=100
    t=tracker(grid,.01)
    r=update(t,n=1,x=.5,y=.5,sigma=.3)
    assert r['posterior_mean']==pytest.approx([.5,.5],abs=1e-6)
    assert r['projected']
    assert t.prior_grid[t.cell(r['x'],r['y'])]>0
    assert r['sigma']>=math.hypot(r['x']-.5,r['y']-.5)


def test_map_conflict_does_not_manufacture_accepted_precision():
    t=tracker()
    r=update(t,x=3.,sigma=.05)
    assert r['prior_conflict'] and not r['stable']
    assert r['prior_mass']<1e-5
    assert np.isfinite(r['covariance']).all()


def test_confirmation_duplicate_stale_misses_deferred_and_exact_freeze():
    t=tracker();d=detection()
    assert t.update(d,10.)
    assert not t.update(d,10.)
    assert t.snapshot(10.)['samples']==1
    t.miss(10.1,True)
    assert t.streak==1
    for i in range(1,4):assert t.update(detection(stamp=10+i*.2),10+i*.2)
    r=t.snapshot(10.6);assert r['stable']
    t.freeze(r)
    assert not t.update(detection(stamp=11.,x=.7),11.)
    for k in ('estimate_id','x','y','covariance'):assert t.snapshot(100.)[k]==r[k]
    assert t.snapshot(100.)['confirmed']
    t.reset();assert t.snapshot(100.) is None
    r=update(t);assert t.snapshot(13.)['stale'] and not t.snapshot(13.)['stable']
    assert t.snapshot(41.) is None
    t.miss(13.);assert t.streak==0
    t.set_map(np.ones((50,50),int),.02,(0.,0.,0.))
    assert t.snapshot(10.3) is None


@pytest.mark.parametrize('changes',[{'x':math.nan},{'sigma':0.},{'sigma':math.inf},
    {'stamp':3.},{'confidence':.5},{'covariance':[[1,2],[2,1]]},
    {'covariance':[[1,1],[0,1]]}])
def test_invalid_observations_cannot_make_estimate(changes):
    t=tracker();d=detection();d.update(changes)
    assert not t.update(d,10.)
    assert t.snapshot(10.) is None


def test_no_map_no_observation_and_map_fingerprint_consistency():
    t=MapMMSE();assert not t.update(detection(),10.)
    assert t.snapshot(10.) is None
    t=tracker();assert t.snapshot(10.) is None
    assert t.signature==map_signature(np.zeros((50,50),int),.02,(0.,0.,0.))
    r=update(t);t.set_map(np.zeros((50,50),int),.02,(0.,0.,0.))
    assert t.snapshot(10.3)['estimate_id']==r['estimate_id']


def test_window_expiration_requires_four_new_frames_without_intermediate_misses():
    t=tracker();update(t)
    assert t.update(detection(stamp=20.),20.)
    assert t.snapshot(20.)['samples']==1
    assert not t.snapshot(20.)['stable']
    for stamp in (20.1,20.2,20.3):assert t.update(detection(stamp=stamp),stamp)
    assert t.snapshot(20.3)['stable']
