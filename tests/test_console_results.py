"""Read-only Console result serialization; no ROS, network or hardware."""
import ast
from collections import deque
import json
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import pytest


def console():
    path=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/console.py'
    tree=ast.parse(path.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Console')
    cls.bases=[]
    cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in (
        'displayed_target_estimate','on_target_estimate','on_status')]
    namespace=dict(math=math,json=json,time=NS(monotonic=lambda:100.,strftime=lambda _:''))
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),namespace)
    node=namespace['Console']()
    node.state=dict(state='LOCALIZING',started_stamp=10.,search_epoch=1,epoch_started_stamp=11.)
    node.p=dict(phase='mission');node.stage=None;node.retry_after_stamp=-math.inf
    node.target_estimate=None;node.lock=threading.Lock();node.events=deque();node.freshness={}
    node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=12_000_000_000))
    node.workflow_status=lambda state:None
    return node


def target():
    return dict(x=.7,y=.2,sigma=.18,variance=.18**2,covariance=[[.18**2,0],[0,.18**2]],
        observation_stamp=11.,estimator='map-constrained MMSE',source='bottom_inverse_single',
        source_frames=[11.],frames=1,samples=1,measurement_stable=False,class_='bottle',quality='approximate',
        **{'class':'bottle'})


@pytest.mark.parametrize('reported',[False,True])
def test_single_source_record_never_becomes_a_stable_four_frame_result(reported):
    node=console();node.state.update(target=target(),target_reported=reported)
    shown=node.displayed_target_estimate()
    assert shown['source_frames']==[11.] and shown['frames']==shown['samples']==1
    assert shown['stable'] is False and shown['measurement_stable'] is False
    assert shown['confirmed'] is reported and shown['display_only'] is not reported
    assert shown['sigma']==.18 and shown['covariance']==target()['covariance']


def test_stopping_preserves_an_accepted_unreported_coordinate_and_original_age():
    node=console();node.state.update(state='STOPPED',target=target(),target_reported=False)
    node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=30_000_000_000))
    shown=node.displayed_target_estimate()
    assert shown['x']==.7 and shown['stamp']==11. and shown['age']==pytest.approx(19.)
    assert shown['stale'] and not shown['confirmed'] and not shown['stable']


def test_unknown_source_count_is_not_invented():
    node=console();value=target()
    for key in ('source_frames','frames','samples'):value.pop(key)
    node.state.update(target=value,target_reported=True)
    shown=node.displayed_target_estimate()
    assert 'frames' not in shown and 'samples' not in shown


def test_old_epoch_empty_reply_cannot_clear_a_new_estimate():
    node=console();node.target_estimate={'kept':True}
    node.on_target_estimate(NS(data=json.dumps(dict(available=False,mission_stamp=10.,search_epoch=0))))
    assert node.target_estimate=={'kept':True}


def test_new_epoch_clears_unreported_preview_using_its_exposure_boundary():
    node=console();reset=[];node.visual=NS(reset=lambda stamp:reset.append(stamp))
    node.target_estimate={'old':True}
    node.on_status(NS(data=json.dumps(dict(state='RESETTING',reason='reset',phase='mission',
        started_stamp=10.,search_epoch=2,epoch_started_stamp=11.5))))
    assert node.target_estimate is None and reset==[11.5]


def test_invalid_record_does_not_create_a_coordinate():
    node=console();value=target();value['x']=None
    node.state.update(target=value,target_reported=True)
    assert node.displayed_target_estimate() is None
