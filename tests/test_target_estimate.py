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


def test_broad_likelihood_is_weak_information_not_a_hidden_sigma_limit():
    t=tracker();r=update(t,sigma=1000.)
    assert r['prior_mass']<1e-5
    assert not r['prior_conflict'] and r['stable']
    assert [r['x'],r['y']]==pytest.approx([.5,.5],abs=1e-6)
    assert r['sigma']==pytest.approx(math.sqrt(1/12),abs=1e-4)


def bearing(stamp=10.,**values):
    result=dict(stamp=stamp,source_frame=stamp,origin=[.5,4.],bearing=0.,
        bearing_sigma=.02,position_sigma=.02,confidence=.9,label='bottle',
        settled=True,center_error=0.)
    result.update(values)
    return result


def test_bearing_only_retains_map_extent_depth_and_does_not_infer_floor_range_limits():
    t=tracker(np.zeros((160,160),int),.05)
    for i in range(4):
        d=bearing(10.+i*.1,range_state='too_near',range_min=.6,range_max=1.2)
        assert t.update_bearing(d,d['stamp'])
    r=t.snapshot(10.3)
    assert r['stable'] and r['source']=='bearing_only'
    assert r['x']>3. and r['covariance'][0][0]>1.
    assert r['covariance'][0][0]>10*r['covariance'][1][1]
    assert 0<r['position_confidence']<.2 and r['confidence_radius_m']==.2
    assert r['confidence']==pytest.approx(.9)
    assert r['source_frames']==pytest.approx([10.,10.1,10.2,10.3])
    assert r['first_source_stamp']==10. and r['last_source_stamp']==pytest.approx(10.3)
    assert r['map_signature']==t.signature


def test_repeated_correlated_bearings_confirm_without_manufacturing_precision():
    t=tracker(np.zeros((160,160),int),.05)
    assert t.update_bearing(bearing(),10.)
    first=t.snapshot(10.)
    for i in range(1,30):assert t.update_bearing(bearing(10.+i*.1),10.+i*.1)
    result=t.snapshot(12.9)
    assert result['samples']==30 and result['stable']
    for key in ('sigma','covariance','position_confidence','x','y'):
        assert np.asarray(result[key])==pytest.approx(np.asarray(first[key]),abs=1e-12)


@pytest.mark.parametrize('first_kind',['floor','bearing'])
def test_one_image_cannot_contribute_both_floor_and_bearing_or_a_restamped_duplicate(first_kind):
    t=tracker()
    ray=bearing(origin=[0.,.5])
    first=t.update if first_kind=='floor' else t.update_bearing
    second=t.update_bearing if first_kind=='floor' else t.update
    assert first(detection(source_frame=10.) if first_kind=='floor' else ray,10.)
    before=t.snapshot(10.)
    raw=ray if first_kind=='floor' else detection(source_frame=10.)
    assert not second(raw,10.)
    assert not second(dict(raw,stamp=10.1),10.1)
    assert t.snapshot(10.1)['samples']==1 and t.streak==1
    assert t.snapshot(10.1)['covariance']==before['covariance']


def test_position_probability_is_normalized_posterior_disk_mass_not_detector_score():
    t=tracker(np.zeros((200,200),int),.02,(-2.,-2.,0.))
    r=update(t,n=1,x=0.,y=0.,sigma=.2)
    # Isotropic Gaussian radial probability 1-exp(-r^2/(2*sigma^2)).
    expected=1-math.exp(-.2**2/(2*.2**2))
    assert r['position_confidence']==pytest.approx(expected,abs=.006)
    assert r['confidence']==.9 and r['position_confidence']<r['confidence']
    assert r['source']=='floor_projection'
    t=tracker()
    r=update(t,n=1,sigma=1000.)
    assert r['position_confidence']==pytest.approx(math.pi*.2**2,abs=.003)
    assert 0<=r['position_confidence']<=1


def test_bearing_posterior_keeps_separated_modes_and_disk_centers_on_reported_point():
    grid=np.full((60,120),100,int)
    grid[25:35,25:35]=grid[25:35,85:95]=0
    t=tracker(grid,.02)
    assert t.update_bearing(bearing(origin=[0.,.6],bearing_sigma=.03),10.)
    r=t.snapshot(10.)
    assert r['projected'] and r['sigma']>.5
    assert .2<r['position_confidence']<.8
    ray_log,_=t._bearing_likelihood()
    weights=t.prior*np.exp(ray_log-np.max(ray_log));weights/=weights.sum()
    reported=np.array([r['x'],r['y']])
    expected=weights[np.sum((t.points-reported)**2,axis=1)<=.2**2].sum()
    mean_probability=weights[np.sum((t.points-r['posterior_mean'])**2,axis=1)<=.2**2].sum()
    assert r['position_confidence']==pytest.approx(expected)
    assert mean_probability<r['position_confidence']


def test_only_explicit_depth_bounds_can_limit_bearing_distance():
    t=tracker(np.zeros((160,160),int),.05)
    assert t.update_bearing(bearing(depth_bounds_m=[.6,1.2]),10.)
    r=t.snapshot(10.)
    assert 1.1<r['x']<1.7 and r['sigma']<.25


@pytest.mark.parametrize('changes',[
    {'settled':False},{'bearing_sigma':0.},{'bearing_sigma':1e308},
    {'position_sigma':-1.},{'origin':[0.]},{'bearing':math.nan},
    {'center_error':.2},{'confidence':.5},{'depth_bounds_m':[2.,1.]},
    {'source_frame':11.},{'source_frame':1.},
])
def test_invalid_bearing_evidence_cannot_create_posterior(changes):
    t=tracker()
    assert not t.update_bearing(bearing(**changes),10.)
    assert t.snapshot(10.) is None


def test_map_with_no_forward_bearing_support_rejects_without_nonfinite_output():
    t=tracker()
    assert not t.update_bearing(bearing(origin=[2.,.5]),10.)
    assert t.snapshot(10.) is None and not t.bearings


def test_bearing_window_map_reset_and_mixed_samples_remain_bounded():
    t=tracker(max_samples=4)
    for i in range(8):
        stamp=10.+i*.1
        if i%2:assert t.update(detection(stamp=stamp),stamp)
        else:assert t.update_bearing(bearing(stamp,origin=[0.,.5]),stamp)
    assert t.snapshot(10.7)['samples']==4
    assert len(t.samples)+len(t.bearings)==4
    assert t.update_bearing(bearing(20.,origin=[0.,.5]),20.)
    assert t.snapshot(20.)['samples']==1 and not t.snapshot(20.)['stable']
    t.set_map(np.zeros((50,51),int),.02,(0.,0.,0.))
    assert not t.bearings and t.snapshot(20.) is None


def test_joint_factor_replaces_raw_rays_and_cannot_refresh_itself_with_identical_sources():
    t=tracker()
    assert t.update_bearing(bearing(origin=[0.,.5]),10.)
    factor=detection(stamp=10.7,sigma=.1,source='bearing_intersection',
        source_frames=[10.+i*.1 for i in range(8)],sites=2)
    assert t.update_factor(factor,10.7)
    result=t.snapshot(10.7)
    assert not t.bearings and len(t.samples)==1
    assert result['source']=='bearing_intersection' and len(result['source_frames'])==8
    assert not t.update_bearing(bearing(10.8,origin=[0.,.5]),10.8)
    assert not t.update_factor(dict(factor,stamp=10.8),10.8)
    assert t.snapshot(10.8)['stamp']==10.7
    t.freeze(result)
    frozen=t.snapshot(100.)
    for key in ('position_confidence','confidence_radius_m','source_frames','first_source_stamp','last_source_stamp'):
        assert frozen[key]==result[key]


def test_relabeling_timestamp_does_not_reconsume_an_evicted_source_frame():
    t=tracker(max_samples=4)
    for i in range(6):assert t.update(detection(stamp=10.+i*.1),10.+i*.1)
    assert 10. not in t.snapshot(10.5)['source_frames']
    assert not t.update(detection(stamp=10.6,source_frame=10.),10.6)
    assert t.snapshot(10.6)['samples']==4


def test_unsupported_bearing_rejection_preserves_full_previous_history_at_capacity():
    t=tracker(max_samples=4)
    for i in range(4):assert t.update_bearing(bearing(10.+i*.1,origin=[0.,.5]),10.+i*.1)
    before=t.snapshot(10.3)
    assert not t.update_bearing(bearing(10.4,origin=[2.,.5]),10.4)
    after=t.snapshot(10.4)
    for key in ('estimate_id','covariance','source_frames','samples','frames','stable'):
        assert after[key]==before[key]
    assert len(t.bearings)==4


def test_rejected_joint_refit_restores_all_raw_and_factor_history():
    t=tracker()
    assert t.update(detection(),10.)
    assert t.update_bearing(bearing(10.1,origin=[0.,.5]),10.1)
    factor=detection(stamp=10.7,source='bearing_intersection',
        source_frames=[10.+i*.1 for i in range(8)],sites=2)
    before=t.snapshot(10.1)
    assert not t.update_factor(dict(factor,covariance=[[0,0],[0,0]]),10.7)
    assert len(t.samples)==len(t.bearings)==1 and t.factor is None
    for key in ('estimate_id','covariance','source_frames','samples','frames'):
        assert t.snapshot(10.7)[key]==before[key]
    assert t.update_factor(factor,10.7)
    before=t.snapshot(10.7)
    changed=dict(factor,stamp=10.8,source_frames=factor['source_frames']+[10.8],covariance=[[0,0],[0,0]])
    assert not t.update_factor(changed,10.8)
    assert not t.bearings and len(t.samples)==1 and t.factor['stamp']==10.7
    for key in ('estimate_id','covariance','source_frames','samples','frames'):
        assert t.snapshot(10.8)[key]==before[key]
