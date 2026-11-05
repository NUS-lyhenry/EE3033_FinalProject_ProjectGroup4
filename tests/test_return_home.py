"""Operator return dispatch and Stop precedence using production callbacks."""
import ast,math,time,threading
from pathlib import Path
from types import SimpleNamespace as NS,MethodType
import pytest
ROOT=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project'
def callback(file,name):
    tree=ast.parse((ROOT/file).read_text());cls=next(x for x in tree.body if isinstance(x,ast.ClassDef))
    f=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name==name)
    scope=dict(time=time,math=math,Bool=NS)
    exec(compile(ast.Module(body=[f],type_ignores=[]),str(ROOT/file),'exec'),scope)
    return scope[name]
def mapper():
    c=NS(home=(0.,0.,0.),state='FAILED',operator_return_pending=False,p={'allow_motion':True},nav=NS(server_is_ready=lambda:True),error=lambda:None,job=None,finished_at=9.,return_verified=False,return_reason=None,partial=False,pending=False,nav_handle=None,drive_pub=NS(publish=lambda x:None),sent=[],get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)))
    c.send=lambda p:c.sent.append(p);c.fail=lambda msg:setattr(c,'state','FAILED')
    for name in ('return_home','begin_operator_return','stop','done'):setattr(c,name,MethodType(callback('mapper.py',name),c))
    return c
def response():return NS(success=False,message='')
def test_mapper_failed_task_returns_to_original_home_without_resetting_it():
    c=mapper();r=c.return_home(None,response())
    assert r.success and c.state=='RETURNING' and c.sent==[(0.,0.,0.)]
    assert c.partial and c.finished_at==0.
def test_mapper_return_waits_for_old_goal_terminal():
    c=mapper();c.state='MAPPING';calls=[];c.nav_handle=NS(cancel_goal_async=lambda:calls.append('cancel'))
    assert c.return_home(None,response()).success
    assert c.state=='CANCELING' and calls==['cancel'] and c.sent==[]
    c.nav_handle=None;c.done(False)
    assert c.state=='RETURNING' and c.sent==[c.home]
def test_mapper_stop_supersedes_queued_return():
    c=mapper();c.pending=True;c.state='MAPPING'
    c.return_home(None,response());c.stop(None,response())
    assert not c.operator_return_pending
    c.pending=False;c.done(False)
    assert c.state=='STOPPED' and c.sent==[]
@pytest.mark.parametrize('condition',['no_home','motion_disabled','stale_sensors','canceling','saving'])
def test_mapper_rejects_return_without_required_evidence(condition):
    c=mapper()
    if condition=='no_home':c.home=None
    elif condition=='motion_disabled':c.p['allow_motion']=False
    elif condition=='stale_sensors':c.error=lambda:'Stale sensors'
    else:c.state='CANCELING' if condition=='canceling' else 'SAVING'
    assert not c.return_home(None,response()).success
    assert c.sent==[]
