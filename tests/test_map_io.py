from pathlib import Path
from types import SimpleNamespace
import json
import numpy as np
import pytest
import yaml
from ee3033_project.map_io import read_pgm, validate_saved_map, repair_legacy_map, save_new_map


def map_file(root, free=.196, pixels=None):
    root.mkdir(parents=True,exist_ok=True)
    pixels=np.array([[0,205,254],[205,254,0]],dtype=np.uint8) if pixels is None else pixels
    image=root/'map.pgm'
    image.write_bytes(f'P5\n# fixture\n{pixels.shape[1]} {pixels.shape[0]}\n255\n'.encode()+pixels.tobytes())
    path=root/'map.yaml'
    path.write_text(yaml.safe_dump(dict(image='map.pgm',mode='trinary',resolution=.05,
        origin=[0,0,0],negate=0,free_thresh=free,occupied_thresh=.65)))
    return path


def test_unknown_survives_save_reload_with_explicit_compatible_threshold(tmp_path):
    path=map_file(tmp_path/'map')
    result=validate_saved_map(path)
    assert result==dict(width=3,height=2,occupied=2,free=2,unknown=2,roundtrip_verified=True)


def test_legacy_threshold_is_rejected_and_repair_is_a_new_unreviewed_copy(tmp_path):
    path=map_file(tmp_path/'old',free=.25)
    (path.parent/'project_map.json').write_text(json.dumps(dict(quality='partial',reviewed=True)))
    original={p.name:p.read_bytes() for p in path.parent.iterdir()}
    with pytest.raises(ValueError,match='thresholds change'):validate_saved_map(path)
    fixed=Path(repair_legacy_map(path,tmp_path/'copies'))
    assert fixed!=path and validate_saved_map(fixed)['unknown']==2
    assert {p.name:p.read_bytes() for p in path.parent.iterdir()}==original
    metadata=json.loads((fixed.parent/'project_map.json').read_text())
    assert metadata['quality']=='partial' and metadata['reviewed'] is False
    assert repair_legacy_map(fixed) is None


def test_noncanonical_gray_map_is_never_automatically_rewritten(tmp_path):
    path=map_file(tmp_path/'map',free=.25,pixels=np.array([[0,205,220,254]],dtype=np.uint8))
    with pytest.raises(ValueError,match='noncanonical'):repair_legacy_map(path)
    assert list(tmp_path.iterdir())==[path.parent]


def test_binary_pgm_first_pixel_whitespace_value_is_not_eaten(tmp_path):
    p=tmp_path/'test.pgm'
    p.write_bytes(b'P5\n2 1\n255\n'+bytes([10,32]))
    assert read_pgm(p).tolist()==[[10,32]]


def test_save_requires_verified_roundtrip_and_passes_explicit_thresholds(tmp_path,monkeypatch):
    calls=[]
    def saver(args,**kwargs):
        calls.append(args)
        destination=Path(args[args.index('-f')+1]).parent
        map_file(destination)
        return SimpleNamespace(returncode=0,stdout='',stderr='')
    monkeypatch.setattr('ee3033_project.map_io.subprocess.run',saver)
    path=Path(save_new_map(tmp_path))
    assert 'free_thresh_default:=0.196' in calls[0]
    assert json.loads((path.parent/'map_validation.json').read_text())['roundtrip_verified']
