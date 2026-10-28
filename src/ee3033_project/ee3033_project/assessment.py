"""Assessment records, saved-map catalog, and owned stage processes.

This module has no ROS imports. It never sends motion commands or edits a map.
"""
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import threading
import time

import yaml

ACTIVE = frozenset(('PLANNING','SEARCHING','OBSERVING','CANCELING','RETURNING','TURNING',
                    'VERIFYING','VERIFYING_HOME','MAPPING','SAVING','CALIBRATING','ACQUIRING','LOCALIZING','RECOVERING'))


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.record-', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(data, stream, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def require_preserved_unknown(path):
    """Block only the identified canonical PGM/threshold mismatch on load.

    Other image formats and noncanonical maps remain the loader's concern;
    gray 205 alone is not sufficient to reinterpret an arbitrary map.
    """
    from .map_io import read_pgm
    path=Path(path)
    try:
        data=yaml.safe_load(path.read_text())
        if (data.get('mode','trinary')!='trinary' or data.get('negate')!=0
                or float(data.get('free_thresh',0))<=50/255):return
        image=path.parent/data['image']
        if image.suffix.lower()!='.pgm':return
        pixels=read_pgm(image)
        levels=set(pixels.ravel().tolist())
    except (ValueError,TypeError,OSError,KeyError,yaml.YAMLError):
        return
    if 205 in levels and levels<={0,205,254}:
        raise ValueError('This map would load unknown cells as free space. Select and review its corrected copy before searching.')


class TrialRecord:
    """One continuous monotonic timer, including stage transition and failures.

    Reloads on the same host boot preserve timing. A reboot is recorded as an
    interruption; it must never silently manufacture a successful trial time.
    """
    def __init__(self, root, clock=time.monotonic, boot_id=None):
        self.root = Path(root).expanduser()
        self.clock = clock
        self.boot_id = boot_id or Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.lock = threading.RLock()
        current = self.root/'current.json'
        self.data = json.loads(current.read_text()) if current.is_file() else self.empty()
        if self.data.get('started') is not None and self.data.get('boot_id') != self.boot_id:
            self.data['interrupted'] = True
            self.persist()

    def empty(self):
        return dict(id=None, started=None, finished=None, boot_id=self.boot_id,
                    started_utc=None, elapsed_seconds=0., interrupted=False,
                    includes_mapping=False, stages={}, events=[], maps=[], result=None)

    def persist(self):
        if self.data['started'] is not None and not self.data['interrupted']:
            self.data['elapsed_seconds'] = max(0., (self.data['finished'] or self.clock())-self.data['started'])
        atomic_json(self.root/'current.json', self.data)
        if self.data['id']:
            atomic_json(self.root/self.data['id']/'assessment.json', self.data)

    def reset(self):
        with self.lock:
            self.data = self.empty()
            self.persist()

    def repeat_search(self, map_id):
        """Archive the last attempt and reuse its map without claiming new mapping."""
        with self.lock:
            if not isinstance(map_id,str) or not map_id:
                raise ValueError('Select the loaded map before repeating a search')
            self.persist()
            previous = self.data['id']
            self.data = self.empty()
            self.data.update(maps=[map_id], reused_map_from_trial=previous)
            self.persist()

    def restore(self, data):
        """Restore an archived attempt when a successor was explicitly rejected."""
        with self.lock:
            self.data = json.loads(json.dumps(data))
            self.persist()

    def start(self, phase):
        with self.lock:
            if self.data['interrupted'] or self.data['finished'] is not None:
                raise ValueError('Begin a new trial before starting again')
            now = self.clock()
            if self.data['started'] is None:
                self.root.mkdir(parents=True, exist_ok=True)
                self.data.update(id=Path(tempfile.mkdtemp(prefix='trial_',dir=self.root)).name,
                                 started=now,started_utc=utc_now(),includes_mapping=phase=='mapping')
            if phase not in self.data['stages']:
                self.data['stages'][phase] = {'started':now,'started_utc':utc_now()}
                self.persist()

    def status(self, phase, status):
        if phase=='calibration':
            return
        with self.lock:
            if status.get('state') in ACTIVE and status['state']!='CANCELING':
                self.start(phase)
            if self.data['started'] is None:
                return
            previous = next((event for event in reversed(self.data['events']) if event['phase']==phase), None)
            if previous and previous['state']==status.get('state'):
                return
            self.data['events'].append(dict(phase=phase,state=status.get('state'),
                reason=status.get('reason',''),utc=utc_now(),elapsed_seconds=self.clock()-self.data['started']))
            self.data['stages'].setdefault(phase,{})['last_status'] = status
            if (phase=='mission' and status.get('state') in ('COMPLETE','NOT_FOUND','FOUND_UNLOCALIZED','SEARCH_INCOMPLETE')
                    and status.get('return_verified') is True
                    and (status.get('state')!='COMPLETE' or status.get('target'))):
                self.data['finished'] = self.clock()
                self.data['result'] = status
            self.persist()

    def add_map(self, map_id):
        with self.lock:
            if map_id not in self.data['maps']:
                self.data['maps'].append(map_id)
                self.persist()

    def snapshot(self):
        with self.lock:
            result = json.loads(json.dumps(self.data))
            if result['started'] is not None and not result['interrupted']:
                result['elapsed_seconds'] = max(0., (result['finished'] or self.clock())-result['started'])
            result['running'] = result['started'] is not None and result['finished'] is None
            result['timing_valid'] = result['includes_mapping'] and not result['interrupted']
            return result


class MapCatalog:
    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()

    def resolve(self, map_id):
        if not isinstance(map_id,str) or not map_id or Path(map_id).is_absolute():
            raise ValueError('Select a map from the saved-map list')
        path = (self.root/map_id).resolve()
        if not path.is_relative_to(self.root) or path.name!='map.yaml' or not path.is_file():
            raise ValueError('Saved map is unavailable or outside the map directory')
        data = yaml.safe_load(path.read_text())
        if not isinstance(data,dict) or not isinstance(data.get('image'),str):
            raise ValueError('Invalid map YAML')
        image = (path.parent/data['image']).resolve()
        if image.parent!=path.parent or not image.is_file():
            raise ValueError('Map image must exist beside its YAML')
        resolution = data.get('resolution')
        if isinstance(resolution,bool) or not isinstance(resolution,(int,float)) or not math.isfinite(resolution) or resolution<=0:
            raise ValueError('Invalid map resolution')
        origin=data.get('origin')
        if (not isinstance(origin,list) or len(origin)!=3 or any(isinstance(v,bool)
                or not isinstance(v,(int,float)) or not math.isfinite(v) for v in origin)):
            raise ValueError('Map origin must contain three finite values')
        free,occupied=data.get('free_thresh'),data.get('occupied_thresh')
        if (type(data.get('negate')) is not int or data['negate'] not in (0,1)
                or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v)
                       for v in (free,occupied)) or not 0<=free<occupied<=1):
            raise ValueError('Invalid map occupancy thresholds')
        return path

    def entry(self, map_id):
        path = self.resolve(map_id)
        meta = path.parent/'project_map.json'
        info = json.loads(meta.read_text()) if meta.is_file() else {}
        return dict(id=str(path.relative_to(self.root)),path=str(path),
                    saved_utc=datetime.fromtimestamp(path.stat().st_mtime, timezone.utc).isoformat(),
                    quality=info.get('quality','unverified'),reviewed=info.get('reviewed',False),
                    reviewed_utc=info.get('reviewed_utc'),resolution=yaml.safe_load(path.read_text())['resolution'])

    def list(self):
        if not self.root.exists(): return []
        result=[]
        for path in sorted(self.root.glob('*/map.yaml'),key=lambda p:p.stat().st_mtime,reverse=True)[:100]:
            try: result.append(self.entry(str(path.relative_to(self.root))))
            except (ValueError,OSError,TypeError,yaml.YAMLError,json.JSONDecodeError): continue
        return result

    def register(self, path, quality='snapshot'):
        path = Path(path).resolve()
        if not path.is_relative_to(self.root):
            raise ValueError('Saved map is outside the configured directory')
        map_id = str(path.relative_to(self.root))
        self.resolve(map_id)
        meta = path.parent/'project_map.json'
        if not meta.exists():
            atomic_json(meta,dict(quality=quality,reviewed=False))
        return self.entry(map_id)

    def review(self, map_id):
        entry = self.entry(map_id)
        # Metadata records operator review; YAML and pixels are never modified.
        atomic_json(Path(entry['path']).parent/'project_map.json',
                    dict(quality=entry['quality'],reviewed=True,reviewed_utc=utc_now()))
        return self.entry(map_id)


def calibration_pose_source(phase, saved_path, selected_path, loaded_id, selected_id):
    """Identify the live coordinate system that belongs to the selected map."""
    if phase == 'mapping' and saved_path and selected_path and Path(saved_path).resolve() == Path(selected_path).resolve():
        return 'mapping'
    if loaded_id and loaded_id == selected_id:
        return 'amcl'
    return None


def require_current_mapping_map(phase, running, saved_path, selected_path):
    """Do not discard a live mapping session to load an unrelated old map."""
    if phase != 'mapping' or not running:
        return
    if not saved_path:
        raise ValueError('Save the current map first; the current mapping session has no saved map')
    if not selected_path or Path(saved_path).resolve() != Path(selected_path).resolve():
        raise ValueError('Select the map just saved from the current mapping session')


def mission_options(values):
    target = values.get('target_class','cup')
    method = values.get('localization_method','hybrid')
    tolerance = values.get('return_tolerance',.20)
    sigma = values.get('preferred_target_sigma',.10)
    if not isinstance(target,str) or not re.fullmatch(r'[a-z0-9][a-z0-9 ._-]{0,79}',target):
        raise ValueError('Use the detector class label, for example cup or bottle')
    if method not in ('lidar','ground_plane','hybrid'):
        raise ValueError('Select lidar or ground_plane localization')
    if isinstance(tolerance,bool) or not isinstance(tolerance,(int,float)) or not math.isfinite(tolerance) or not .02<=tolerance<=1.:
        raise ValueError('Return tolerance must be 0.02–1.00 m')
    if isinstance(sigma,bool) or not isinstance(sigma,(int,float)) or not math.isfinite(sigma) or not .01<=sigma<=1.:
        raise ValueError('Preferred precision must be 0.01–1.00 m (one sigma)')
    return dict(target_class=target,localization_method=method,return_tolerance=float(tolerance),
                preferred_target_sigma=float(sigma))


class StageProcess:
    """Own exactly one launch process group. Never restart crashed nodes."""
    def __init__(self, log_root, allow_motion=False, hardware=None, model=None, popen=subprocess.Popen,
                 calibration_file=''):
        self.root = Path(log_root).expanduser()
        self.allow_motion = allow_motion
        self.hardware = hardware or dict(base=True,lidar=True,camera=True)
        self.model = model
        self.calibration_file = calibration_file
        self.navigation_profile = 'original'
        self.scan_self_mask_radius = .08
        self.mapping_resolution = .05
        self.vision_config = ''
        self.calibration_directory = '~/ee3033_calibrations'
        self.calibration_port, self.console_port = 8780, 8765
        self.calibration_initial_pose = None
        self.popen = popen
        self.process = None
        self.log = None
        self.phase = None
        self.fault = None
        self.loading = False
        self.cancel = threading.Event()

    def snapshot(self):
        exited = self.process is not None and self.process.poll() is not None
        return dict(managed=True,phase=self.phase,running=self.process is not None and not exited,
                    transitioning=self.loading,fault=self.fault or ('Stage exited; inspect logs before manual intervention' if exited else None),
                    log_path=str(self.log) if self.log else None)

    def stop(self, timeout=12):
        if self.process is None: return
        proc = self.process
        # ROS launch reaps its children on SIGINT. Refuse a successor if any
        # member of the old process group survives, even if launch itself exits.
        try: os.killpg(proc.pid,signal.SIGINT)
        except ProcessLookupError: pass
        try: proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            self.fault='Stage shutdown timed out; inspect before restarting'
            raise RuntimeError(self.fault) from exc
        try:
            os.killpg(proc.pid,0)
        except ProcessLookupError:
            self.process=None
            return
        self.fault='Stage children are still running; successor was not started'
        raise RuntimeError(self.fault)

    def launch(self, phase, map_path=None, options=None, map_directory=None):
        if self.process is not None:
            raise RuntimeError('Stop the previous stage before loading another')
        if phase not in ('mapping','mission','calibration'): raise ValueError('Invalid stage')
        if self.cancel.is_set(): return False
        self.root.mkdir(parents=True,exist_ok=True)
        directory=Path(tempfile.mkdtemp(prefix=phase+'_',dir=self.root))
        self.log=directory/'launch.log'
        args=['ros2','launch','ee3033_project',phase+'.launch.py',
              'allow_motion:='+str(bool(self.allow_motion)).lower(),
              'self_mask_radius:='+str(self.scan_self_mask_radius)]
        if phase=='mapping':
            args.append('mapping_resolution:='+str(self.mapping_resolution))
        if phase=='calibration':
            args += ['port:='+str(self.calibration_port), 'parent_port:='+str(self.console_port),
                     'output_directory:='+str(self.calibration_directory)]
            if map_path:
                args.append('map:='+str(map_path))
            if map_path and self.calibration_initial_pose is not None:
                args += ['seed_pose:=true', *[key+':='+str(value) for key, value in
                         zip(('initial_x','initial_y','initial_yaw'), self.calibration_initial_pose)]]
            if self.model: args.append('model:='+self.model)
        else:
            args += ['console:=false','map_directory:='+str(map_directory),
                     'navigation_profile:='+self.navigation_profile]
        for name,enabled in self.hardware.items():
            if name!='camera' or phase in ('mission','calibration'): args.append(name+':='+str(bool(enabled)).lower())
        if phase=='mission':
            if map_path is None: raise ValueError('A reviewed saved map is required')
            options=mission_options(options or {})
            args.append('results_directory:='+str(self.root.parent/'mission_runs'))
            args += ['map_partial:='+str(bool(getattr(self,'map_partial',False))).lower(), 'map:='+str(map_path),*[(key+':='+str(value)) for key,value in options.items()]]
            if self.model: args.append('model:='+self.model)
            if self.calibration_file: args.append('calibration_file:='+self.calibration_file)
            if self.vision_config: args.append('vision_config:='+self.vision_config)
        with self.log.open('w') as stream:
            self.process=self.popen(args,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        self.phase,self.fault=phase,None
        if self.cancel.is_set():
            self.stop()
            return False
        return True
