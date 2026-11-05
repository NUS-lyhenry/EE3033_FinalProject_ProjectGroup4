"""Owner boundaries that must remain safe while search evidence is reset."""
import json
from types import SimpleNamespace as NS
import pytest
from test_mission_policy import mission, namespace


@pytest.mark.parametrize('changes', [dict(available=True), dict(target_locked=True),
    dict(epoch_started_stamp=9.5), dict(mission_stamp=8.), dict(search_epoch=0)])
def test_reset_only_accepts_an_empty_unlocked_matching_cutoff_ack(changes):
    m=mission();m.state='RESETTING';m.search_epoch=1;m.epoch_started_stamp=10.
    m.filter_epoch_ack=False
    reply=dict(available=False,target_locked=False,mission_stamp=9.,search_epoch=1,
               epoch_started_stamp=10.)
    reply.update(changes)
    m.on_estimate(NS(data=json.dumps(reply)))
    assert not m.filter_epoch_ack


def test_matching_reset_ack_can_release_the_wait_without_an_estimate():
    m=mission();m.state='RESETTING';m.search_epoch=1;m.epoch_started_stamp=10.
    m.filter_epoch_ack=False
    m.on_estimate(NS(data=json.dumps(dict(available=False,target_locked=False,
        mission_stamp=9.,search_epoch=1,epoch_started_stamp=10.))))
    assert m.filter_epoch_ack


def test_duplicate_image_does_not_extend_the_loss_clock(monkeypatch):
    clock=NS(value=100.)
    monkeypatch.setitem(namespace,'time',NS(monotonic=lambda:clock.value))
    m=mission();m.state='LOCALIZING';m.target_detected=True
    m.p=dict(target_class='cup');m.acquisition.started=1.;m.view_frames=set()
    for stamp in (10.,10.2):
        detection=dict(stamp=stamp,label='cup',confidence=.9,bearing=0.,bearing_valid=True)
        m.on_detections(NS(data=json.dumps(dict(detections=[detection]))))
    before=m.last_usable_bearing
    clock.value=200.
    m.on_detections(NS(data=json.dumps(dict(detections=[detection]))))
    assert m.last_usable_bearing==before
    m.on_detections(NS(data=json.dumps(dict(detections=[]))))
    assert m.last_usable_bearing==before
