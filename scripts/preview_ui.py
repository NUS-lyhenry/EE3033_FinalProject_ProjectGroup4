#!/usr/bin/env python3
"""Read-only, explicitly simulated UI preview. No ROS imports or robot access."""
import argparse
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
WEB=ROOT/'src/ee3033_project/web'


def preview_state():
    return dict(simulated=True,read_only=True,phase='mapping',allow_motion=False,
        token='read-only-preview',control_generation=0,
        mission=dict(state='WAITING',reason='SIMULATED PREVIEW · connect the robot for live data',
                     home=None,target=None,ready=False,elapsed_seconds=0,return_verified=False),
        stage=dict(managed=True,running=False,phase=None,transitioning=False,fault=None),
        assessment=dict(elapsed_seconds=0,started=None,finished=None,running=False,timing_valid=False,maps=[]),
        mission_options=dict(target_class='cup',localization_method='lidar',return_tolerance=.2),
        map_revision=0,pose=None,route=[],path=[],health={},events=[],camera_source=None,
        battery=dict(voltage=None,fresh=False,age=None),control_fault=None,initial_pose_sent=False)


class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def reply(self,status,body,kind='application/json'):
        if kind=='application/json':body=json.dumps(body).encode()
        self.send_response(status);self.send_header('Content-Type',kind)
        self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)))
        self.end_headers();self.wfile.write(body)
    def do_GET(self):
        path=self.path.split('?')[0]
        files={'/':('index.html','text/html; charset=utf-8'),'/app.js':('app.js','text/javascript'),'/style.css':('style.css','text/css')}
        if path in files:
            name,kind=files[path];return self.reply(200,(WEB/name).read_bytes(),kind)
        if path=='/api/state':return self.reply(200,preview_state())
        if path=='/api/map':return self.reply(200,None)
        if path=='/api/maps':return self.reply(200,{'maps':[]})
        if path=='/api/result':return self.reply(200,{'simulated':True,'mission':preview_state()['mission']})
        return self.reply(404,{'error':'No live robot data in preview'})
    def do_POST(self):self.reply(403,{'error':'Read-only preview cannot send robot commands'})


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8790);args=parser.parse_args()
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    print(f'SIMULATED read-only UI: http://127.0.0.1:{server.server_address[1]}',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()


if __name__=='__main__':main()
