"""Allocate a unique map destination, then use the standard Nav2 saver."""
from pathlib import Path
import subprocess
import tempfile


def save_new_map(root):
    root=Path(root).expanduser()
    root.mkdir(parents=True,exist_ok=True)
    directory=Path(tempfile.mkdtemp(prefix='onsite_',dir=str(root)))
    result=subprocess.run(['ros2','run','nav2_map_server','map_saver_cli',
                           '-f',str(directory/'map'),'--ros-args',
                           '-p','save_map_timeout:=10.0',
                           '-p','map_subscribe_transient_local:=true'],
                          capture_output=True,text=True,timeout=18)
    if result.returncode or not (directory/'map.yaml').is_file():
        raise RuntimeError('Map save failed: '+(result.stdout+result.stderr)[-600:])
    return str(directory/'map.yaml')
