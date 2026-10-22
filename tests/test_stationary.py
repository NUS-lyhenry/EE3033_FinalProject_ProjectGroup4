import math
import pytest
from ee3033_project.stationary import StationaryWindow


def feed(window, motion, seconds=2.5, dt=.02):
    result=None
    for i in range(round(seconds/dt)+1):
        t=i*dt
        x,y,yaw,v,w=motion(t,i)
        window.add(t,x,y,yaw,v,w,'odom')
        result=window.evaluate(t)
    return result


def test_brief_balancing_velocity_spikes_with_bounded_pose_are_accepted():
    result=feed(StationaryWindow(),lambda t,i:(.005*math.sin(12*t),.01*math.sin(8*t),
                .03*math.sin(10*t),.086 if i%5==0 else .008,.232 if i%5==0 else .015))
    assert result['verified']
    assert result['linear_mean']<.035 and result['angular_mean']<.10


@pytest.mark.parametrize('motion,reason',[
    (lambda t,i:(.08*t,0.,0.,.08,0.),'linear'),
    (lambda t,i:(0.,0.,.2*t,0.,.2),'angular'),
    (lambda t,i:(.03*t,0.,0.,.03,0.),'Position'),
    (lambda t,i:(0.,0.,.08*t,0.,.08),'Heading'),
    (lambda t,i:(0.,0.,0.,.08*(-1)**i,0.),'linear'),
])
def test_sustained_motion_or_pose_drift_is_rejected(motion,reason):
    result=feed(StationaryWindow(),motion)
    assert not result['verified'] and reason in result['reason']


def test_requires_full_duration_and_fresh_distinct_samples():
    window=StationaryWindow()
    assert not feed(window,lambda t,i:(0.,0.,0.,0.,0.),seconds=1.9)['verified']
    for _ in range(30):window.add(1.9,0.,0.,0.,0.,0.,'odom')
    assert not window.evaluate(1.9)['verified']
    assert 'stale' in window.evaluate(3.)['reason']


@pytest.mark.parametrize('change',['gap','time_reversal','frame','nonfinite'])
def test_discontinuous_data_resets_the_window(change):
    window=StationaryWindow()
    assert feed(window,lambda t,i:(0.,0.,0.,0.,0.))['verified']
    stamp=3. if change=='gap' else 2. if change=='time_reversal' else 2.52
    window.add(stamp,math.nan if change=='nonfinite' else 0.,0.,0.,0.,0.,
               'new_odom' if change=='frame' else 'odom')
    assert not window.evaluate(stamp)['verified']


def test_heading_wrap_does_not_look_like_a_full_rotation():
    result=feed(StationaryWindow(),lambda t,i:(0.,0.,math.atan2(math.sin(math.pi+.02*math.sin(t)),
                                        math.cos(math.pi+.02*math.sin(t))),0.,.01))
    assert result['verified'] and result['heading_span']<.05


def test_time_weighted_mean_is_not_biased_by_a_burst_of_samples():
    window=StationaryWindow(seconds=1.,max_gap=.6)
    for t,v in [(0.,.08),(.5,.08),(.501,0.),(.502,0.),(.503,0.),(.504,0.),(1.,0.)]:
        window.add(t,0.,0.,0.,v,0.,'odom')
    result=window.evaluate(1.)
    assert not result['verified'] and result['linear_mean']>.04
