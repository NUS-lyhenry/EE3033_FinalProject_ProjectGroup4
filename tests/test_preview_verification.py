import importlib.util
from pathlib import Path

import pytest


spec=importlib.util.spec_from_file_location('preview_verification',
    Path(__file__).parents[1]/'scripts/verify_algorithm_preview.py')
preview=importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


def test_map_handoff_waits_for_new_path_and_catalogue(monkeypatch):
    states=iter([
        dict(saved_map=None),
        dict(saved_map='old.yaml'),
        dict(saved_map='new.yaml'),
        dict(saved_map='new.yaml'),
    ])
    catalogues=iter([{'maps':[]},{'maps':[dict(id='new',path='new.yaml')]}])
    def get(path):
        if path=='maps':return next(catalogues)
        return dict(simulated=True,preview_backend='main-ros',
                    mission=dict(state='MAPPED',saved_map='new.yaml'),
                    pose=[.1,.2,.3],**next(states))
    monkeypatch.setattr(preview.time,'sleep',lambda _:None)
    state,entry=preview.wait_saved_map(get)
    assert entry['id']=='new' and state['pose']==[.1,.2,.3]


def test_map_handoff_refuses_a_hardware_console():
    with pytest.raises(RuntimeError,match='non-simulation'):
        preview.wait_saved_map(lambda _:dict(simulated=False))


def test_map_handoff_has_a_bounded_catalogue_wait(monkeypatch):
    clock=iter([0.,0.,31.])
    monkeypatch.setattr(preview.time,'monotonic',lambda:next(clock))
    monkeypatch.setattr(preview.time,'sleep',lambda _:None)
    with pytest.raises(TimeoutError,match='catalogue'):
        preview.wait_saved_map(lambda _:dict(simulated=True,preview_backend='main-ros'))
