"""Fault injection begins only after a public, validated planning response."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


def response(accepted=True,status=4,frame='map',points=2):
    result=NS(status=status,result=NS(path=NS(header=NS(frame_id=frame),poses=[None]*points)))
    return NS(accepted=accepted,get_result_async=lambda:result)


def preflight(responses):
    source=Path(__file__).with_name('integration_navigation_faults.py')
    method=next(n for n in ast.parse(source.read_text()).body
        if isinstance(n,ast.FunctionDef) and n.name=='wait_for_checked_route')
    clock=[0.]
    sent=[]
    world=NS(running=False)
    def send(request):
        assert not world.running, 'Planner readiness must never enable chassis motion'
        sent.append(request)
        return responses.pop(0)
    client=NS(wait_for_server=lambda **kw:True,send_goal_async=send)
    env=dict(ActionClient=lambda *a:client,ComputePathToPose=NS(Goal=NS),
        time=NS(monotonic=lambda:clock[0],sleep=lambda seconds:clock.__setitem__(0,clock[0]+seconds)),
        json=json,wait=lambda value,timeout:value)
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),env)
    return env['wait_for_checked_route'],world,sent,env


@pytest.mark.parametrize('unready',[
    response(accepted=False),response(status=6),response(frame='odom_combined'),response(points=0),
])
def test_readiness_retries_terminal_unready_attempts_and_requires_checked_map_route(unready):
    check,world,sent,_=preflight([unready,response()])
    pose=object()
    result=check(world,pose)
    assert result==dict(attempts=2,accepted=True,status=4,frame_id='map',points=2)
    assert len(sent)==2 and all(request.goal is pose for request in sent)
    assert not world.running


def test_readiness_timeout_preserves_failed_result_and_never_enables_motion():
    check,world,sent,_=preflight([response(status=6)])
    with pytest.raises(AssertionError,match='"status": 6'):
        check(world,object(),timeout=.05)
    assert len(sent)==1 and not world.running


def test_nonterminal_planning_timeout_never_launches_overlapping_retry():
    check,world,sent,env=preflight([response(),response()])
    calls=[]
    def wait(value,timeout):
        calls.append(value)
        if len(calls)==2:
            raise TimeoutError('Planning still running')
        return value
    env['wait']=wait
    with pytest.raises(TimeoutError,match='still running'):
        check(world,object())
    assert len(sent)==1 and not world.running
