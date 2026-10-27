"""Preparation-stage status and automatic reuse of validated camera profiles."""
import json
from pathlib import Path
from urllib.request import Request, urlopen

from .calibration import load_profile


class CameraSetup:
    def init_camera_setup(self):
        self.calibration_live = None
        self.calibration_status = dict(ready=False, path=None, message='Camera calibration has not been saved.')
        self.calibration_identity = None
        self.explicit_calibration = bool(self.p['calibration_file'])
        self.refresh_camera_profile()

    def refresh_camera_profile(self):
        root = Path(self.p['calibration_directory']).expanduser()
        def modified(path):
            try: return path.stat().st_mtime_ns
            except OSError: return -1
        candidates = ([Path(self.p['calibration_file']).expanduser()]
                       if self.explicit_calibration and self.p['phase'] != 'calibration'
                       else sorted(root.glob('camera_*.json'), key=modified, reverse=True)
                       if self.p['phase'] == 'calibration' else [])
        for path in candidates:
            try:
                stat = path.stat()
                identity = (str(path), stat.st_mtime_ns, stat.st_size)
                if identity == self.calibration_identity:
                    return
                profile = load_profile(path)
                self.calibration_identity = identity
                self.p['calibration_file'] = str(path.resolve())
                self.calibration_status = dict(ready=True, path=self.p['calibration_file'],
                    created_utc=profile['created_utc'], camera=profile['camera']['frame_id'],
                    message=('Explicitly selected legacy profile will be used for this mission.'
                             if self.explicit_calibration else
                             'Legacy profile saved. Select it with calibration_file to opt in; default missions use camera geometry.'))
                if self.stage:
                    self.stage.calibration_file = self.p['calibration_file']
                return
            except (OSError, ValueError, KeyError, TypeError):
                continue
        self.calibration_identity = None
        self.calibration_status = dict(ready=False, path=None, message='Camera calibration has not been saved, or its profile is unavailable.')
        if not self.explicit_calibration:
            self.p['calibration_file'] = ''
            if self.stage:
                self.stage.calibration_file = ''

    def camera_setup_tick(self):
        self.refresh_camera_profile()
        if self.p['phase'] != 'calibration' or not self.stage or not self.stage.process:
            self.calibration_live = None
            return
        try:
            with urlopen(f'http://127.0.0.1:{self.p["calibration_port"]}/api/calibration/state', timeout=.25) as response:
                live = json.load(response)
            self.calibration_live = dict(ready=True, motion=live['motion'],
                automatic=live.get('automatic'), preview=live.get('preview'), capture=live.get('capture'),
                target=live.get('target'), plan=live.get('plan'),
                detected_count=live.get('detected_count', 0))
            with self.lock:
                self.state = dict(phase='calibration', state='WAITING' if live['motion']=='idle' and live.get('automatic',{}).get('state')!='fitting' else 'CALIBRATING',
                    reason=live['reason'], ready=True, home=None, target=None)
        except (OSError, ValueError, KeyError):
            self.calibration_live = dict(ready=False, motion=None)

    def require_camera_idle(self):
        if self.p['phase'] == 'calibration' and self.stage and self.stage.process:
            if not self.calibration_live or not self.calibration_live.get('ready'):
                raise ValueError('Wait for fresh camera setup status before changing stages')
            if self.calibration_live.get('motion') != 'idle':
                raise ValueError('Stop camera positioning before changing stages')

    def set_calibration_target(self, body):
        if self.p['phase'] != 'calibration' or not self.selected_map:
            raise ValueError('Load a reviewed map and camera setup first')
        if self.stage.loading:
            raise ValueError('Wait for camera setup to finish loading')
        self.require_camera_idle()
        if body.get('map_id') != self.selected_map['id']:
            raise ValueError('Bottle position belongs to a different map; refresh the page')
        base = f'http://127.0.0.1:{self.p["calibration_port"]}/api/calibration/'
        with urlopen(base+'state', timeout=1.) as response:
            state = json.load(response)
        payload = dict(generation=state['generation'], map_file=self.selected_map['path'], xy=[body.get('x'), body.get('y')])
        request = Request(base+'target', json.dumps(payload).encode(),
            {'Content-Type':'application/json', 'X-Project-Token':state['token']}, method='POST')
        from urllib.error import HTTPError
        try:
            with urlopen(request, timeout=2.) as response:
                result = json.load(response)
        except HTTPError as exc:
            raise ValueError(json.load(exc).get('error', 'Bottle position rejected')) from exc
        result['message'] = 'Bottle position selected. Place its base at the marker, then start calibration below.'
        return result
