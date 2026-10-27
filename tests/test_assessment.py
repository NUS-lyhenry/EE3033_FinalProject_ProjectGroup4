import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
from ee3033_project.assessment import MapCatalog, StageProcess, TrialRecord, mission_options


def map_files(root, name='onsite_test'):
    directory=root/name;directory.mkdir()
    (directory/'map.pgm').write_bytes(b'P5\n2 2\n255\n'+bytes([0,254,254,0]))
    (directory/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return directory/'map.yaml'


def test_timer_spans_both_stages_transition_and_stop(tmp_path):
    now=[100.];clock=lambda:now[0]
    record=TrialRecord(tmp_path,clock=clock,boot_id='one')
    record.start('mapping');now[0]=130
    record.status('mapping',{'state':'STOPPED','reason':'Map review'})
    now[0]=150
    reopened=TrialRecord(tmp_path,clock=clock,boot_id='one')
    assert reopened.snapshot()['elapsed_seconds']==50
    reopened.start('mission');now[0]=180
    reopened.status('mission',{'state':'RETURNING','target':{'x':1,'y':2}})
    now[0]=200
    reopened.status('mission',{'state':'COMPLETE','target':{'x':1,'y':2},'return_verified':True})
    now[0]=300
    result=reopened.snapshot()
    assert result['elapsed_seconds']==100 and result['timing_valid']
    assert not result['running']
    assert (tmp_path/result['id']/'assessment.json').is_file()


@pytest.mark.parametrize('status',[
    {'state':'COMPLETE','target':None,'return_verified':True},
    {'state':'COMPLETE','target':{'x':1},'return_verified':False},
    {'state':'COMPLETE','target':{'x':1}},
    {'state':'NOT_FOUND','return_verified':True},
    {'state':'FAILED'},
])
def test_unverified_or_missing_target_never_finishes_timer(tmp_path,status):
    record=TrialRecord(tmp_path,boot_id='one');record.start('mapping')
    record.status('mission',status)
    assert record.snapshot()['finished'] is None


def test_reboot_is_an_interruption_not_a_fabricated_time(tmp_path):
    record=TrialRecord(tmp_path,clock=lambda:100,boot_id='old');record.start('mapping')
    resumed=TrialRecord(tmp_path,clock=lambda:3,boot_id='new')
    assert resumed.snapshot()['interrupted']
    assert not resumed.snapshot()['timing_valid']
    with pytest.raises(ValueError):resumed.start('mission')
    resumed.reset();resumed.start('mapping');assert resumed.snapshot()['timing_valid']


def test_mission_only_run_is_not_presented_as_combined_time(tmp_path):
    record=TrialRecord(tmp_path,boot_id='one');record.start('mission')
    assert not record.snapshot()['timing_valid']


def test_review_keeps_partial_label_and_never_edits_map(tmp_path):
    path=map_files(tmp_path);before={p:p.read_bytes() for p in path.parent.iterdir()}
    catalog=MapCatalog(tmp_path);entry=catalog.register(path,'partial')
    assert not entry['reviewed']
    reviewed=catalog.review(entry['id'])
    assert reviewed['reviewed'] and reviewed['quality']=='partial'
    assert all(p.read_bytes()==content for p,content in before.items())
    assert catalog.list()[0]['id']==entry['id']


@pytest.mark.parametrize('map_id',['../secret/map.yaml','/tmp/map.yaml','onsite_test/map.pgm',None,''])
def test_map_selection_cannot_escape_configured_directory(tmp_path,map_id):
    map_files(tmp_path)
    with pytest.raises(ValueError):MapCatalog(tmp_path).resolve(map_id)


def test_symlinked_map_and_external_image_are_rejected(tmp_path):
    path=map_files(tmp_path)
    outside=tmp_path.parent/(tmp_path.name+'-outside');outside.mkdir()
    other=map_files(outside)
    (tmp_path/'outside').symlink_to(other.parent,target_is_directory=True)
    with pytest.raises(ValueError):MapCatalog(tmp_path).resolve('outside/map.yaml')
    path.write_text(path.read_text().replace('map.pgm',str(other.with_suffix('.pgm'))))
    with pytest.raises(ValueError):MapCatalog(tmp_path).resolve('onsite_test/map.yaml')


@pytest.mark.parametrize('values',[{'return_tolerance':True},{'return_tolerance':float('nan')},
    {'return_tolerance':0},{'preferred_target_sigma':True},{'preferred_target_sigma':float('nan')},
    {'preferred_target_sigma':0.},{'preferred_target_sigma':1.01},{'preferred_target_sigma':'0.1'},{'target_class':'cup; touch /tmp/injected'},
    {'target_class':'cup:=bottle'},{'localization_method':'guess'}])
def test_mission_setup_validates_real_settings_not_shell_fragments(values):
    with pytest.raises(ValueError):mission_options(values)


def test_valid_floor_target_setup_uses_explicit_projection():
    assert mission_options({'localization_method':'ground_plane','return_tolerance':.1})=={
        'target_class':'cup','localization_method':'ground_plane','return_tolerance':.1,'preferred_target_sigma':.1}
    assert mission_options({'preferred_target_sigma':.25})['preferred_target_sigma']==.25


def test_stage_manager_stops_only_its_owned_process_group(tmp_path):
    calls=[]
    def factory(args,**kwargs):
        calls.append(args)
        return subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'],**kwargs)
    unrelated=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'])
    manager=StageProcess(tmp_path,allow_motion=False,popen=factory)
    try:
        assert manager.launch('mission','/tmp/map with spaces.yaml',{},tmp_path)
        owned=manager.process
        assert 'map:=/tmp/map with spaces.yaml' in calls[0]
        assert 'allow_motion:=false' in calls[0]
        with pytest.raises(RuntimeError):manager.launch('mapping',map_directory=tmp_path)
        manager.stop()
        assert owned.poll() is not None
        assert unrelated.poll() is None
        assert manager.process is None
    finally:
        if manager.process:manager.stop()
        unrelated.terminate();unrelated.wait(timeout=3)


def test_stage_exit_is_reported_without_automatic_restart(tmp_path):
    calls=[]
    def factory(args,**kwargs):
        calls.append(args)
        return subprocess.Popen([sys.executable,'-c','pass'],**kwargs)
    manager=StageProcess(tmp_path,popen=factory);manager.launch('mapping',map_directory=tmp_path)
    manager.process.wait(timeout=3)
    for _ in range(3):assert manager.snapshot()['fault'] and not manager.snapshot()['running']
    assert len(calls)==1
    manager.stop()


def test_stop_during_transition_prevents_successor_launch(tmp_path):
    calls=[]
    def forbidden(*args,**kwargs):calls.append(args);raise AssertionError('Must not start')
    manager=StageProcess(tmp_path,popen=forbidden);manager.cancel.set()
    assert manager.launch('mapping',map_directory=tmp_path) is False
    assert calls==[]
