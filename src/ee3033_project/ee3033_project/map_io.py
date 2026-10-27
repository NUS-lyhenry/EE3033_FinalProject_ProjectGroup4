"""Allocate a unique map destination, then use the standard Nav2 saver."""
from pathlib import Path
import json
import shutil
import subprocess
import tempfile
import numpy as np
import yaml


def read_pgm(path):
    """Read Nav2's binary/ASCII grayscale PGM without image-library conversions."""
    raw=Path(path).read_bytes();index=0
    def token():
        nonlocal index
        while index<len(raw):
            if raw[index:index+1]==b'#':
                end=raw.find(b'\n',index)
                index=len(raw) if end<0 else end+1
            elif raw[index:index+1].isspace():index+=1
            else:break
        start=index
        while index<len(raw) and not raw[index:index+1].isspace() and raw[index:index+1]!=b'#':index+=1
        if start==index:raise ValueError('Truncated PGM header')
        return raw[start:index]
    mode=token();width,height,maximum=(int(token()) for _ in range(3))
    if mode not in (b'P2',b'P5') or min(width,height)<=0 or maximum!=255:
        raise ValueError('Expected an 8-bit grayscale PGM')
    if mode==b'P5':
        if index>=len(raw) or not raw[index:index+1].isspace():raise ValueError('Invalid PGM separator')
        if raw[index:index+2]==b'\r\n':index+=2
        else:index+=1
        values=np.frombuffer(raw[index:],dtype=np.uint8)
    else:
        values=np.array([int(token()) for _ in range(width*height)],dtype=np.uint8)
    if values.size!=width*height:raise ValueError('PGM pixel count does not match dimensions')
    return values.reshape(height,width)


def validate_saved_map(path):
    """Verify that standard trinary pixels reload with the same meanings."""
    path=Path(path)
    data=yaml.safe_load(path.read_text())
    image=(path.parent/data['image']).resolve()
    if image.parent!=path.parent.resolve():raise ValueError('Map image must be beside its YAML')
    if data.get('mode','trinary')!='trinary' or data.get('negate')!=0:
        raise ValueError('Expected canonical Nav2 trinary map encoding')
    pixels=read_pgm(image)
    levels,counts=np.unique(pixels,return_counts=True)
    if not set(levels.tolist())<={0,205,254}:
        raise ValueError('Map has noncanonical trinary pixels; inspect rather than rewriting')
    free,occupied=float(data['free_thresh']),float(data['occupied_thresh'])
    if not 0<=free<occupied<=1:raise ValueError('Invalid map thresholds')
    occupancy=1-pixels.astype(float)/255
    loaded=np.where(occupancy>occupied,100,np.where(occupancy<free,0,-1))
    expected=np.where(pixels==0,100,np.where(pixels==254,0,-1))
    if not np.array_equal(loaded,expected):
        raise ValueError('Map thresholds change unknown/free/occupied cells on reload')
    return dict(width=int(pixels.shape[1]),height=int(pixels.shape[0]),
        occupied=int(np.count_nonzero(loaded==100)),free=int(np.count_nonzero(loaded==0)),
        unknown=int(np.count_nonzero(loaded<0)),roundtrip_verified=True)


def repair_legacy_map(path, root=None):
    """Create a corrected copy of the known old saver format; never edit source.

    Returns None for a map that already roundtrips. Unsupported maps are left
    untouched and rejected; gray 205 alone is not enough to infer its meaning.
    """
    path=Path(path).expanduser().resolve()
    try:
        validate_saved_map(path)
        return None
    except ValueError as exc:
        if 'thresholds change' not in str(exc):raise
    data=yaml.safe_load(path.read_text())
    if (data.get('free_thresh')!=.25 or data.get('occupied_thresh')!=.65
            or data.get('mode','trinary')!='trinary' or data.get('negate')!=0):
        raise ValueError('Not the known legacy Nav2 saver format; source preserved')
    destination=Path(root).expanduser().resolve() if root else path.parent.parent
    destination.mkdir(parents=True,exist_ok=True)
    directory=Path(tempfile.mkdtemp(prefix='onsite_corrected_',dir=str(destination)))
    image=(path.parent/data['image']).resolve()
    shutil.copy2(image,directory/image.name)
    data.update(image=image.name,free_thresh=.196)
    corrected=directory/'map.yaml'
    corrected.write_text(yaml.safe_dump(data,sort_keys=False))
    validation=validate_saved_map(corrected)
    validation.update(source_map=str(path),correction='Preserved canonical unknown pixels with free_thresh 0.196')
    (directory/'map_validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    metadata=path.parent/'project_map.json'
    previous=json.loads(metadata.read_text()) if metadata.is_file() else {}
    (directory/'project_map.json').write_text(json.dumps(dict(quality=previous.get('quality','unverified'),
        reviewed=False,source_map=str(path),correction='unknown_threshold'),indent=2)+'\n')
    return str(corrected)


def save_new_map(root):
    root=Path(root).expanduser()
    root.mkdir(parents=True,exist_ok=True)
    directory=Path(tempfile.mkdtemp(prefix='onsite_',dir=str(root)))
    result=subprocess.run(['ros2','run','nav2_map_server','map_saver_cli',
                           '-f',str(directory/'map'),'--ros-args',
                           '-p','save_map_timeout:=10.0',
                           '-p','free_thresh_default:=0.196',
                           '-p','occupied_thresh_default:=0.65',
                           '-p','map_subscribe_transient_local:=true'],
                          capture_output=True,text=True,timeout=18)
    if result.returncode or not (directory/'map.yaml').is_file():
        raise RuntimeError('Map save failed: '+(result.stdout+result.stderr)[-600:])
    validation=validate_saved_map(directory/'map.yaml')
    (directory/'map_validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    return str(directory/'map.yaml')
