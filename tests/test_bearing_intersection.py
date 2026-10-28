import math
import numpy as np
import pytest
from ee3033_project.triangulation import BearingIntersection
from ee3033_project.target_estimate import MapMMSE
from ee3033_project.vision_geometry import override_pitch, validate_mount_settings
from ee3033_project.calibration import mounting_rotation


def view(stamp, origin, target=(1., 1.), **changes):
    angle=math.atan2(target[1]-origin[1],target[0]-origin[0])
    return dict(stamp=stamp, odom_origin=list(origin), odom_bearing=angle,
                map_odom=[0.,0.,0.], bearing_sigma=math.radians(1.),
                position_sigma=.02, center_error=0., settled=True,
                label='cup', confidence=.9, **changes)


def collect(t, origin, start, target=(1., 1.)):
    result=None
    for i in range(4):result=t.add(view(start+i*.2, origin, target))
    return result


def test_two_distinct_sites_recover_target_with_nonzero_covariance():
    t=BearingIntersection()
    assert collect(t,(0.,0.),10.) is None
    result=collect(t,(0.,1.),12.)
    assert [result['x'],result['y']]==pytest.approx([1.,1.],abs=1e-6)
    assert np.linalg.eigvalsh(result['covariance']).min()>.0004
    assert result['sites']==2 and len(result['source_frames'])==8
    assert t.add(view(12.6,(0.,1.))) is None


def test_rotation_without_baseline_and_nearly_parallel_rays_are_rejected():
    t=BearingIntersection()
    collect(t,(0.,0.),10.,(2.,0.))
    assert collect(t,(0.,.1),12.,(2.,0.)) is None
    assert 'angle' in t.reason


def test_four_intermittent_frames_with_two_second_gaps_fit_each_site_window():
    t=BearingIntersection()
    for stamp in (10.,12.,14.,16.):
        assert t.add(view(stamp,(0.,0.))) is None
    assert t.progress()['sites'][0]['frames']==4
    assert t.progress()['sites'][0]['ready']
    result=None
    # Transit does not invalidate a completed A capture window. B is sampled
    # independently, and the existing full-observation age check still applies.
    for stamp in (30.,32.,34.,36.):
        result=t.add(view(stamp,(0.,1.)))
    assert result is not None
    assert [result['x'],result['y']]==pytest.approx([1.,1.],abs=1e-6)
    assert result['sites']==2 and len(result['source_frames'])==8


def test_four_frames_spanning_more_than_six_seconds_do_not_complete_site():
    t=BearingIntersection()
    for stamp in (10.,12.1,14.2,16.3):
        assert t.add(view(stamp,(0.,0.))) is None
    assert not t.progress()['sites'][0]['ready']
    for stamp in (20.,22.,24.,26.):
        assert t.add(view(stamp,(0.,1.))) is None
    assert sum(s['ready'] for s in t.progress()['sites'])==1


def test_frames_from_distinct_positions_cannot_fill_one_capture_window():
    t=BearingIntersection()
    for stamp,origin in ((10.,(0.,0.)),(12.,(0.,0.)),(14.,(0.,1.)),(16.,(0.,1.))):
        assert t.add(view(stamp,origin)) is None
    assert len(t.progress()['sites'])==2
    assert not any(s['ready'] for s in t.progress()['sites'])


def test_completed_site_still_expires_under_full_observation_age_limit():
    t=BearingIntersection(max_age=120.)
    assert collect(t,(0.,0.),10.) is None
    assert collect(t,(0.,1.),142.) is None
    assert len(t.progress()['sites'])==1


@pytest.mark.parametrize('changes',[dict(settled=False),dict(center_error=math.radians(3.)),
                                  dict(confidence=.59),dict(bearing_sigma=0.)])
def test_invalid_frames_do_not_complete_an_intermittent_capture(changes):
    t=BearingIntersection()
    for stamp in (10.,12.,14.):
        assert t.add(view(stamp,(0.,0.))) is None
    invalid=view(16.,(0.,0.));invalid.update(changes)
    assert t.add(invalid) is None
    assert not t.progress()['sites'][0]['ready']
    for stamp in (20.,22.,24.,26.):
        assert t.add(view(stamp,(0.,1.))) is None


def test_map_jump_recollects_and_unsettled_frames_never_count():
    t=BearingIntersection()
    collect(t,(0.,0.),10.)
    d=view(12.,(0.,1.));d['map_odom']=[.3,0.,0.]
    assert t.add(d) is None
    assert len(t.sites)==1
    t.reset()
    for i in range(5):
        d=view(20.+i*.2,(0.,0.));d['settled']=False
        assert t.add(d) is None
    assert not t.sites


def test_joint_factor_replaces_floor_likelihood_and_cannot_be_double_counted():
    mmse=MapMMSE();mmse.set_map(np.zeros((40,40)),.1,(0,0,0))
    for i in range(4):
        stamp=10+i*.2
        assert mmse.update(dict(stamp=stamp,x=1.3,y=1.,sigma=.3,confidence=.9,label='cup'),stamp)
    t=BearingIntersection();collect(t,(0.,0.),10.)
    factor=collect(t,(0.,1.),12.)
    assert mmse.update_factor(factor,12.6)
    assert len(mmse.samples)==1
    result=mmse.snapshot(12.6)
    assert result['stable'] and result['sites']==2
    assert result['x']==pytest.approx(1.,abs=.05)
    assert not mmse.update_factor(factor,12.6)
    assert not mmse.update(dict(stamp=12.7,x=2.,y=2.,sigma=.01,confidence=.9,label='cup'),12.7)
    new=t.add(view(12.8,(0.,1.)))
    assert mmse.update_factor(new,12.8)
    assert len(mmse.samples)==1
    assert mmse.snapshot(16.)['stale']


def test_pitch_override_retains_tf_translation_contract_and_yaw():
    yaw=.35
    rotation=override_pitch(mounting_rotation(0.,yaw),math.radians(28.6))
    assert rotation==pytest.approx(mounting_rotation(math.radians(28.6),yaw))
    assert rotation.T@rotation==pytest.approx(np.eye(3))
    settings=dict(camera_height=0.,camera_pitch=math.radians(28.6),camera_yaw=0.,camera_x=0.,
                  camera_y=0.,horizontal_fov=0.,camera_min_range=.35,camera_max_range=2.,use_pitch_override=True)
    assert validate_mount_settings(settings)['use_pitch_override'] is True
    with pytest.raises(ValueError):validate_mount_settings(dict(settings,use_pitch_override=1))


def test_correct_pitch_recovers_far_contact_that_old_pitch_projects_beyond_range():
    import cv2
    from ee3033_project.vision_geometry import FloorCamera
    k=np.array([[303.59,0.,313.76],[0.,302.97,258.72],[0.,0.,1.]])
    mount=np.array([-.035,0.,.2655]);truth=np.array([1.,0.,0.])
    rotation=mounting_rotation(math.radians(28.6),0.)
    optical=(truth-mount)@rotation
    pixel,_=cv2.projectPoints(optical.reshape(1,3),np.zeros(3),np.zeros(3),k,np.zeros(5))
    model=FloorCamera(640,480,k,np.zeros(5),rotation,mount)
    assert model.intersection(pixel.ravel())==pytest.approx(truth,abs=1e-8)
    old=FloorCamera(640,480,k,np.zeros(5),mounting_rotation(math.radians(14),0.),mount)
    try:assert np.linalg.norm(old.intersection(pixel.ravel())[:2]-mount[:2])>3.
    except ValueError:pass


def test_two_collinear_near_retreats_cannot_starve_a_new_lateral_capture():
    t=BearingIntersection();target=(1.,0.)
    for origin,stamp in (((0.,0.),10.),((-.3,0.),12.),((-.6,0.),14.)):
        assert collect(t,origin,stamp,target) is None
    assert t.reason_code=='bearing_weak_crossing_angle' and len(t.sites)==3
    first=t.add(view(16.,(-.6,.5),target))
    assert first is None and t.frame_accepted
    assert 'bearing_redundant_site_replaced' in t.last_events
    result=None
    for stamp in (16.2,16.4,16.6):result=t.add(view(stamp,(-.6,.5),target))
    assert result is not None
    assert [result['x'],result['y']]==pytest.approx(target,abs=1e-6)
    assert len(t.sites)==3 and result['sites']==3
    assert len(result['source_frames'])==len(set(result['source_frames']))==12
    kept={p['stamp'] for site in t.sites for p in site}
    assert set(result['source_frames'])==kept
    mmse=MapMMSE();mmse.set_map(np.zeros((80,80)),.05,(-2.,-2.,0.))
    assert mmse.update_factor(result,16.6)
    assert not mmse.update_factor(result,16.6)
    assert len(mmse.samples)==1


def test_published_factor_sites_cannot_be_evicted_by_a_fourth_view():
    t=BearingIntersection()
    collect(t,(0.,0.),10.);assert collect(t,(0.,1.),12.)
    assert collect(t,(.5,0.),14.)
    sources=t.last_sources.copy()
    origins=[site[-1]['origin'].copy() for site in t.sites]
    assert t.add(view(16.,(.5,1.))) is None
    assert t.reason_code=='bearing_site_limit'
    assert t.last_sources==sources and len(t.sites)==3
    assert all(np.array_equal(a,site[-1]['origin']) for a,site in zip(origins,t.sites))


def test_incomplete_unused_site_can_be_replaced_without_losing_factor_evidence():
    t=BearingIntersection()
    collect(t,(0.,0.),10.);assert collect(t,(0.,1.),12.)
    sources=t.last_sources.copy()
    t.add(view(14.,(.5,0.)))  # one frame at a site unused by the published A/B factor
    assert len(t.sites)==3
    t.add(view(15.,(.5,1.)))
    assert 'bearing_redundant_site_replaced' in t.last_events
    assert t.last_sources==sources
    assert sources <= {p['stamp'] for site in t.sites for p in site}


def test_parallel_complete_captures_are_not_replaced_by_more_parallel_evidence():
    t=BearingIntersection();target=(2.,0.)
    for origin,stamp in (((0.,0.),10.),((-.3,0.),12.),((-.6,0.),14.)):
        collect(t,origin,stamp,target)
    assert t.add(view(16.,(-.9,0.),target)) is None
    assert t.reason_code=='bearing_site_limit' and len(t.sites)==3
