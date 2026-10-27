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
from ee3033_project.calibration import camera_model, predicted_points, save_profile, validate_setup
from ee3033_project.map_calibration import make_plan, map_sample, fit_map_profile

PHYSICAL = dict(bottle_height=.215, bottle_diameter=.065, camera_height=.2655, camera_x=-.035, camera_y=0.)
CAMERA = camera_model(640,480,'simulated_camera',[520.,0.,320.,0.,520.,240.,0.,0.,1.],[0.]*5)
GRID = dict(width=120,height=120,resolution=.05,origin=[-3.,-3.,0.],cells=[0]*14400)


class Simulation:
    def __init__(self,directory):
        self.lock=threading.RLock();self.directory=directory;self.token=secrets.token_urlsafe(24)
        self.generation=0;self.motion='idle';self.pose=[0.,0.,0.];self.target=None;self.plan=None
        self.profile=self.saved=self.setup=None;self.auto_views=[]
        self.automatic=dict(state='idle',reason='Choose bottle map coordinates below (simulation only)')
        self.reason=self.automatic['reason'];self.lease=self.last=time.monotonic()

    def stop(self,reason):
        self.motion='idle';self.generation+=1;self.reason=reason
        if self.automatic['state']=='collecting':self.automatic.update(state='stopped',reason=reason)

    def observation(self):
        target=self.target['xy'] if self.target else [.9,0.]
        sample=map_sample(PHYSICAL,target,self.pose,[])
        p=predicted_points([555.,548.,math.radians(14.2),.035],PHYSICAL,CAMERA,sample)
        return [float(p[2:,0].min()),float(p[1,1]),float(p[2:,0].max()),float(p[0,1])]

    def image(self):
        frame=np.full((480,640,3),(53,69,74),dtype=np.uint8);frame[240:]=(103,120,119)
        x1,y1,x2,y2=map(round,self.observation())
        cv2.rectangle(frame,(x1,y1),(x2,y2),(125,191,151),-1)
        cv2.putText(frame,'SIMULATION - NO ROBOT',(18,28),cv2.FONT_HERSHEY_SIMPLEX,.6,(224,239,231),1)
        return cv2.imencode('.jpg',frame)[1].tobytes()

    def update(self):
        now=time.monotonic()
        if self.motion!='auto':return
        if now-self.lease>.6:self.stop('Simulation stopped: browser heartbeat lost');return
        if now-self.last<.3:return
        self.last=now;goal=self.plan['goals'][len(self.auto_views)];self.pose=list(goal['pose'])
        self.auto_views.append(dict(pose=list(self.pose),box=self.observation(),holdout=goal['holdout']))
        self.automatic.update(collected=len(self.auto_views),reason='Simulating map-pose collection')
        if len(self.auto_views)==len(self.plan['goals']):
            try:
                self.profile=fit_map_profile(self.setup,CAMERA,self.target['xy'],self.auto_views)
                self.profile['simulated']=True;self.saved=save_profile(self.profile,self.directory)
                self.automatic.update(state='complete',reason='Simulation complete; profile cannot be used on the robot')
            except ValueError as exc:self.automatic.update(state='failed',reason=str(exc))
            self.stop(self.automatic['reason'])

    def snapshot(self):
        self.update();box=self.observation()
        return copy.deepcopy(dict(token=self.token,generation=self.generation,allow_motion=True,read_only=False,
            simulated=True,motion=self.motion,reason=self.reason,target=self.target,plan=self.plan,
            automatic={**self.automatic,'prerequisites':dict(ready=self.target is not None,reason='Simulated map point ready' if self.target else 'Choose a simulated map point')},
            health={k:True for k in ('camera','odometry','lidar','output','ownership')},ownership=True,
            capture=dict(ready=True),camera=CAMERA,profile=self.profile,saved=self.saved,setup=self.setup,
            detected_count=1,detection_fresh=True,bottle_detections=[dict(box=box)],preview=dict(fresh=True)))

    def command(self,name,body):
        self.update()
        if name=='stop':self.stop('Simulation stopped by operator');return
        if body.get('generation')!=self.generation:raise ValueError('Control context changed')
        if name=='heartbeat':self.lease=time.monotonic();return
        if self.motion!='idle':raise ValueError('Stop the current simulation first')
        if name=='target':
            self.plan=make_plan(GRID,self.pose,body.get('xy',[]));self.target=dict(xy=self.plan['target'])
            self.reason='Simulated point selected; enter dimensions and Start';return
        if name!='start':raise ValueError('Use target/start/stop in the map-point workflow')
        if not self.target:raise ValueError('Select the bottle map position first')
        self.setup=validate_setup({**PHYSICAL,'bottle_height':body.get('bottle_height'),'bottle_diameter':body.get('bottle_diameter')})
        self.plan=make_plan(GRID,self.pose,self.target['xy']);self.auto_views=[];self.profile=self.saved=None
        self.automatic=dict(state='collecting',reason='Simulating map-pose collection',collected=0,total=17)
        self.motion='auto';self.last=self.lease=time.monotonic()


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
                        return self.reply(200, demo.image(), 'image/jpeg')
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
