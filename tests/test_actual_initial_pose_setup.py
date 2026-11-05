"""A setup resend never extends120s or changes a pose after Mission Start."""
from pathlib import Path
import io
import runpy
from types import SimpleNamespace as NS
import pytest
namespace=runpy.run_path(str(Path(__file__).parents[1]/'scripts/actual_yolo_search_comparison.py'))
Driver=namespace['Driver']


def driver(monkeypatch,*,recover=True,reason='No AMCL pose; set the assigned initial pose before starting',state='WAITING',started=0.):
    clock=NS(now=0.)
    fake=NS(monotonic=lambda:clock.now,time=lambda:1000.+clock.now,sleep=lambda delta:setattr(clock,'now',clock.now+delta))
    monkeypatch.setitem(Driver.wait.__globals__,'time',fake)
    instance=Driver.__new__(Driver);instance.stream=io.StringIO();instance.calls=[];instance.records={}
    mission=dict(state=state,reason='Waiting',readiness_reason=reason,ready=False,started_stamp=started)
    instance.get=lambda:dict(simulated=True,preview_backend='main-ros',mission=mission)
    def post(action,body):
        instance.calls.append((clock.now,action,body))
        if recover:mission['ready']=True
        return dict(success=True)
    instance.post=post;instance.save=lambda name,data:instance.records.__setitem__(name,data)
    instance.initial_pose_setup=dict(pose=[1.,2.,.3],began=0.,resent=False)
    return instance,clock


def test_one_same_assigned_pose_resend_recovers_only_before_start(monkeypatch):
    instance,clock=driver(monkeypatch)
    assert instance.wait(lambda s:s['mission']['ready'],120.,'mission-ready')['mission']['ready']
    assert instance.calls==[(20.,'initial_pose',dict(x=1.,y=2.,yaw=.3))]
    assert instance.records['prestart-initial-pose-resend.json']['remaining_original_deadline_seconds']==100.
    assert clock.now==20.5


def test_failed_recovery_keeps_original120second_deadline_and_only_one_attempt(monkeypatch):
    instance,clock=driver(monkeypatch,recover=False)
    with pytest.raises(TimeoutError):instance.wait(lambda s:s['mission']['ready'],120.,'mission-ready')
    assert len(instance.calls)==1 and clock.now==120.


@pytest.mark.parametrize('settings',[dict(reason='Map-to-base TF missing or stale'),dict(state='SEARCHING',started=1.),dict(state='WAITING',started=1.)])
def test_other_readiness_faults_or_a_started_task_never_get_pose_resends(monkeypatch,settings):
    instance,clock=driver(monkeypatch,recover=False,**settings)
    with pytest.raises(TimeoutError):instance.wait(lambda s:s['mission']['ready'],21.,'mission-ready',allow_failure=True)
    assert instance.calls==[]
