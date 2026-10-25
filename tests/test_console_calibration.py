import copy
from pathlib import Path
from types import SimpleNamespace

from ee3033_project.calibration import save_profile, validate_holdout
from ee3033_project.console_calibration import CameraSetup
from ee3033_project.assessment import StageProcess, TrialRecord
from test_calibration import profile, sample


class Setup(CameraSetup):
    def __init__(self, directory):
        self.p = dict(calibration_directory=str(directory),calibration_file='',phase='mapping')
        self.stage = SimpleNamespace(calibration_file='')
        self.init_camera_setup()


def saved_profile(directory, simulated=False):
    data=profile()
    data['simulated']=simulated
    for d,y in [(.9,-.08),(1.25,.09)]:
        data['validation'].append(validate_holdout(data,sample(d,y)))
    return save_profile(data,directory)


def test_validated_camera_setup_is_reused_after_console_restart(tmp_path):
    first=Setup(tmp_path)
    assert not first.calibration_status['ready']
    path=saved_profile(tmp_path)
    first.refresh_camera_profile()
    assert first.calibration_status['ready']
    assert first.stage.calibration_file==str(Path(path).resolve())
    reopened=Setup(tmp_path)
    assert reopened.p['calibration_file']==first.p['calibration_file']


def test_invalid_and_simulated_profiles_never_replace_valid_calibration(tmp_path):
    expected=saved_profile(tmp_path)
    saved_profile(tmp_path,simulated=True)
    (tmp_path/'camera_broken.json').write_text('{invalid')
    setup=Setup(tmp_path)
    assert setup.calibration_status['path']==expected


def test_camera_preparation_does_not_start_assessment_timer(tmp_path):
    record=TrialRecord(tmp_path)
    record.status('calibration',dict(state='CALIBRATING',reason='Positioning bottle'))
    assert record.snapshot()['started'] is None


def test_calibration_stage_owns_drivers_and_uses_separate_launch_arguments(tmp_path):
    calls=[]
    manager=StageProcess(tmp_path,popen=lambda args,**kw:calls.append(args) or SimpleNamespace(poll=lambda:None))
    manager.calibration_directory=str(tmp_path/'profiles')
    manager.console_port=8888
    manager.launch('calibration')
    args=calls[0]
    assert 'calibration.launch.py' in args and 'parent_port:=8888' in args
    assert 'camera:=true' in args
    assert not any(a.startswith(('map_directory:=','console:=')) for a in args)


def test_automatic_camera_launch_passes_map_and_only_explicit_fresh_pose_seed(tmp_path):
    calls=[]
    manager=StageProcess(tmp_path,popen=lambda args,**kw:calls.append(args) or SimpleNamespace(poll=lambda:None))
    manager.calibration_initial_pose=[1.,2.,.3]
    manager.launch('calibration','/maps/room.yaml')
    assert 'map:=/maps/room.yaml' in calls[0]
    assert 'seed_pose:=true' in calls[0] and 'initial_yaw:=0.3' in calls[0]


def test_calibration_rejects_unsaved_or_wrong_live_mapping_map():
    import pytest
    from ee3033_project.assessment import require_current_mapping_map
    with pytest.raises(ValueError,match='Save the current map first'):
        require_current_mapping_map('mapping',True,None,'/maps/old/map.yaml')
    with pytest.raises(ValueError,match='just saved'):
        require_current_mapping_map('mapping',True,'/maps/new/map.yaml','/maps/old/map.yaml')
    require_current_mapping_map('mapping',True,'/maps/new/map.yaml','/maps/new/map.yaml')
    require_current_mapping_map('mapping',False,None,'/maps/old/map.yaml')
    require_current_mapping_map('calibration',True,None,'/maps/old/map.yaml')
