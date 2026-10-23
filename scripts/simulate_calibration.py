#!/usr/bin/env python3
"""Clickable calibration demo. Uses the real fitter, imports no ROS or robot IO."""
import argparse
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import secrets
import sys
import tempfile
import threading
import time
from urllib.parse import urlparse

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/ee3033_project'))
from ee3033_project.calibration import (camera_model, fit_profile, predicted_points,
    save_profile, validate_holdout, validate_sample, validate_setup, number)
from ee3033_project.calibration_auto import fit_automatic, bottle_sample

PHYSICAL = dict(bottle_height=.25, bottle_diameter=.065, camera_height=.20, camera_x=0., camera_y=0.)
CAMERA = camera_model(640, 480, 'simulated_camera', [520., 0., 320., 0., 510., 240., 0., 0., 1.], [0.]*5)
CAMERA['source'] = 'SIMULATED pinhole camera'
POSITIONS = [(.65, -.09), (.8, .1), (1., -.2), (1.15, .22), (1.35, -.18), (1.5, .12), (.9, -.08), (1.25, .09)]


class Simulation:
    def __init__(self, directory):
        self.lock = threading.RLock()
        self.directory = directory
        self.token = secrets.token_urlsafe(24)
        self.generation = self.scene = 0
        self.distance, self.lateral = POSITIONS[0]
        self.setup = self.profile = self.frozen = self.saved = None
        self.samples = []
        self.automatic = dict(state='idle', reason='Simulation ready: use bottle 0.25 m × 0.065 m and distance 0.90 m.', collected=0, total=18)
        self.auto_views = []
        self.auto_index = 0
        self.motion = 'idle'
        self.motion_reason = 'Simulation stopped'
        self.reason = self.automatic['reason']
        self.last_tick = self.began = self.lease = time.monotonic()

    def stop(self, reason):
        self.motion, self.motion_reason = 'idle', reason
        self.generation += 1
        if self.automatic['state']=='collecting':
            self.automatic.update(state='stopped', reason=reason)

    def update(self):
        now = time.monotonic()
        dt = min(.2, now-self.last_tick)
        self.last_tick = now
        if self.motion == 'idle':
            return
        if self.motion == 'auto':
            if now-self.lease > .6:
                self.stop('Simulation stopped: browser heartbeat lost')
                return
            if now-self.auto_last < .30:
                return
            self.auto_last = now
            distance, yaw, holdout = self.auto_goals[self.auto_index]
            pose = [-distance, 0., yaw]
            sample = bottle_sample(PHYSICAL, .9, .04, pose, [])
            self.distance, self.lateral = sample['distance'], sample['lateral']
            self.auto_views.append(dict(pose=pose, box=self.observation()['box'], holdout=holdout))
            self.auto_index += 1
            self.automatic.update(collected=self.auto_index, reason='Simulating automatic view collection')
            if self.auto_index == len(self.auto_goals):
                try:
                    self.profile = fit_automatic(self.setup, CAMERA, self.auto_distance, self.auto_views)
                    self.profile['simulated'] = True
                    self.samples = self.profile['training']
                    self.saved = save_profile(self.profile, self.directory)
                    self.automatic.update(state='complete', reason='Simulation complete; demo profile cannot be used on a robot')
                    self.reason = self.automatic['reason']
                except ValueError as exc:
                    self.automatic.update(state='failed', reason=str(exc))
                    self.reason = str(exc)
                self.stop(self.automatic['reason'])
            return
        if now-self.lease > .6 or now-self.began > 1.5:
            self.stop('Simulated movement stopped; check reference measurements again')
        elif self.motion in ('forward', 'backward'):
            self.distance = max(.5, self.distance+(-1 if self.motion == 'forward' else 1)*.035*dt)
        else:
            target = self.distance*math.tan(.035)
            self.lateral += float(np.clip(target-self.lateral, -.12*dt, .12*dt))

    def observation(self):
        sample = dict(distance=self.distance, lateral=self.lateral)
        p = predicted_points([555., 548., .13, .035], PHYSICAL, CAMERA, sample)
        return {**sample, 'box': [float(p[2:, 0].min()), float(p[1, 1]), float(p[2:, 0].max()), float(p[0, 1])]}

    def image(self):
        frame = np.full((480, 640, 3), (53, 69, 74), dtype=np.uint8)
        frame[240:] = (103, 120, 119)
        for y in range(250, 480, 45):
            cv2.line(frame, (0, y), (640, y), (124, 139, 134), 1)
        for x in range(-640, 1300, 160):
            cv2.line(frame, (320, 240), (x, 480), (124, 139, 134), 1)
        x1, y1, x2, y2 = map(round, self.observation()['box'])
        mid, neck = (x1+x2)//2, max(3, (x2-x1)//5)
        shoulder = y1+(y2-y1)//5
        polygon = np.array([[mid-neck,y1],[mid+neck,y1],[mid+neck,shoulder-5],
                            [x2,shoulder+8],[x2,y2-4],[x2-4,y2],[x1+4,y2],
                            [x1,y2-4],[x1,shoulder+8],[mid-neck,shoulder-5]])
        cv2.fillPoly(frame, [polygon], (125, 191, 151))
        cv2.rectangle(frame, (x1+2, shoulder+20), (x2-2, min(y2-5, shoulder+50)), (219, 225, 188), -1)
        cv2.rectangle(frame, (mid-neck, y1-2), (mid+neck, y1+5), (68, 123, 84), -1)
        cv2.putText(frame, 'SIMULATION - NO ROBOT CONNECTED', (18, 28), cv2.FONT_HERSHEY_SIMPLEX, .5, (224, 239, 231), 1)
        return cv2.imencode('.jpg', frame)[1].tobytes()

    def snapshot(self):
        self.update()
        return copy.deepcopy(dict(token=self.token, generation=self.generation,
            allow_motion=True, read_only=False, simulated=True,
            automatic={**self.automatic, 'prerequisites': dict(ready=True,reason='Simulated saved map and localization ready')},
            simulation_reference=dict(distance=self.distance, lateral=self.lateral, scene=self.scene+1, total=len(POSITIONS)),
            health={k: True for k in ('camera','odometry','lidar','output','ownership')}, ownership=True,
            motion=self.motion, motion_reason=self.motion_reason, reason=self.reason,
            camera=CAMERA, setup=self.setup, samples=self.samples, profile=self.profile, saved=self.saved,
            detected_count=1, box=self.observation()['box'], detection_fresh=True,
            frozen={k:v for k,v in self.frozen.items() if k!='jpeg'} if self.frozen else None))

    def command(self, name, body):
        self.update()
        if name == 'stop':
            self.stop('Simulation stopped by operator')
            return
        if body.get('generation') != self.generation:
            raise ValueError('Movement context changed; retry after the next update')
        if name == 'heartbeat':
            self.lease = time.monotonic()
            return
        if name == 'unfreeze':
            self.frozen = None
            return
        if self.motion != 'idle':
            raise ValueError('Wait for simulated movement to stop')
        if name == 'automatic':
            self.setup = validate_setup({**PHYSICAL, 'bottle_height': body.get('bottle_height'),
                                        'bottle_diameter': body.get('bottle_diameter')})
            self.auto_distance = number(body.get('distance'), 'Measured floor distance (m)', .45, 2.5)
            self.auto_goals = [(0.,0.,False)]+[(d,a,False) for d in (0.,.18,.36,.54,.72) for a in (0.,-.13,.13)]
            self.auto_goals += [(.45,0.,True),(.27,0.,True)]
            self.auto_index, self.auto_views = 0, []
            self.profile, self.saved, self.frozen = None, None, None
            self.motion = 'auto'
            self.auto_last = self.began = self.lease = time.monotonic()
            self.automatic.update(state='collecting',collected=0,reason='Simulating automatic alignment and collection')
        elif name == 'setup':
            self.setup = validate_setup(body)
            self.samples, self.profile, self.frozen, self.saved = [], None, None, None
            self.reason = 'Setup saved. Freeze the bottle, fill the reference offsets and record a training sample.'
        elif name == 'scene':
            self.scene = (self.scene+1) % len(POSITIONS)
            self.distance, self.lateral = POSITIONS[self.scene]
            self.frozen = None
            self.reason = 'New simulated position. Freeze its image before recording.'
        elif name == 'move':
            if body.get('mode') not in ('center', 'forward', 'backward'):
                raise ValueError('Invalid simulated movement')
            self.motion, self.frozen = body['mode'], None
            self.motion_reason = 'Simulating '+self.motion
            self.began = self.lease = self.last_tick = time.monotonic()
        elif name == 'freeze':
            if not self.setup:
                raise ValueError('Save setup first')
            self.frozen = dict(id=secrets.token_hex(8), box=self.observation()['box'], jpeg=self.image())
            self.reason = 'Image frozen. Fill the simulated reference offsets, then record the measurement.'
        elif name == 'sample':
            if not self.frozen or body.get('freeze_id') != self.frozen['id']:
                raise ValueError('Freeze a new image first')
            sample = validate_sample(body, CAMERA)
            if body.get('validation'):
                if not self.profile:
                    raise ValueError('Fit before checking validation positions')
                self.profile['validation'].append(validate_holdout(self.profile, sample))
            else:
                if any(math.hypot(sample['distance']-s['distance'], sample['lateral']-s['lateral']) < .08 for s in self.samples):
                    raise ValueError('Choose a new position with Next bottle position')
                self.samples.append(sample)
                self.profile = None
            self.frozen, self.saved = None, None
            self.reason = 'Measurement recorded. Select Next bottle position for another observation.'
        elif name == 'fit':
            self.profile = fit_profile(self.setup, CAMERA, self.samples)
            self.profile['simulated'] = True
            self.reason = 'Fit complete. Use positions 7 and 8 as NEW validation samples.'
        elif name == 'remove':
            entries = self.samples if body.get('kind') == 'training' else self.profile['validation'] if self.profile else []
            index = body.get('index')
            if type(index) is not int or not 0 <= index < len(entries):
                raise ValueError('Select an existing sample')
            entries.pop(index)
            if body.get('kind') == 'training':
                self.profile = None
            self.saved = None
        elif name == 'save':
            if not self.profile:
                raise ValueError('Fit and validate first')
            self.saved = save_profile(self.profile, self.directory)
            self.reason = 'Simulation complete. The demo profile is labelled simulated and cannot be used by the robot.'
        else:
            raise ValueError('Unknown demo command')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8791)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='group4_camera_demo_') as directory:
        demo = Simulation(directory)
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def reply(self, code, data, kind='application/json'):
                body = json.dumps(data, allow_nan=False).encode() if kind == 'application/json' else data
                self.send_response(code)
                for key, value in [('Content-Type',kind),('Content-Length',str(len(body))),('Cache-Control','no-store'),('X-Frame-Options','DENY'),('X-Content-Type-Options','nosniff')]:
                    self.send_header(key, value)
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = urlparse(self.path).path
                files = {'/': ('calibration.html','text/html; charset=utf-8'), '/calibration.js': ('calibration.js','text/javascript'), '/calibration.css': ('calibration.css','text/css')}
                if path in files:
                    name, kind = files[path]
                    return self.reply(200, (ROOT/'src/ee3033_project/web'/name).read_bytes(), kind)
                with demo.lock:
                    if path == '/api/calibration/state':
                        return self.reply(200, demo.snapshot())
                    if path == '/api/calibration/image.jpg':
                        return self.reply(200, demo.frozen['jpeg'] if demo.frozen else demo.image(), 'image/jpeg')
                return self.reply(404, dict(error='Unknown demo endpoint'))

            def do_POST(self):
                allowed = {f'http://127.0.0.1:{server.server_address[1]}', f'http://localhost:{server.server_address[1]}'}
                if not secrets.compare_digest(self.headers.get('X-Project-Token',''),demo.token) or self.headers.get('Origin',next(iter(allowed))) not in allowed:
                    return self.reply(403, dict(error='Invalid session token or origin'))
                try:
                    length = int(self.headers.get('Content-Length','0'))
                    if not 0 <= length <= 8192:
                        raise ValueError('Request too large')
                    body = json.loads(self.rfile.read(length) or b'{}')
                    if not isinstance(body, dict):
                        raise ValueError('JSON object required')
                    with demo.lock:
                        demo.command(urlparse(self.path).path.removeprefix('/api/calibration/'), body)
                    return self.reply(200, dict(success=True))
                except (ValueError, TypeError, KeyError) as exc:
                    return self.reply(400, dict(error=str(exc)))

        server = ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
        server.daemon_threads = True
        print(f'CLICKABLE SIMULATION (no robot): http://127.0.0.1:{server.server_address[1]}',flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()


if __name__ == '__main__':
    main()
