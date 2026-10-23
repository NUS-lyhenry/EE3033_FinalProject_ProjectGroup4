"""Preparation-stage status and automatic reuse of validated camera profiles."""
import json
from pathlib import Path
from urllib.request import urlopen

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
                      else sorted(root.glob('camera_*.json'), key=modified, reverse=True))
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
                    message='Saved calibration will be used automatically when the mission loads.')
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
