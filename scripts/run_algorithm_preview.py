#!/usr/bin/env python3
"""Run the actual managed console/stage launches with simulated physical sensors.

Run only inside the dedicated Docker bridge on a non-robot ROS domain. The web
relay tags simulated evidence; all control requests go unchanged to Console.
"""
import argparse
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import signal
import secrets
import threading
from urllib.parse import urlparse

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.parameter import Parameter

from ee3033_project.console import Console
from simulation_robot import SimulatedRobot
from simulation_control import SimulationScenarioControl


class SimulationConsole(SimulationScenarioControl,Console):
    def snapshot(self):
        data = super().snapshot()
        data.update(simulated=True, preview_interactive=True, preview_backend='main-ros',
            preview_hint='Project navigation, SLAM and YOLO with simulated sensors and chassis. No physical robot is connected.')
        data.update(self.simulation_snapshot())
        return data

    def workflow_get(self, path):
        data = super().workflow_get(path)
        if path == '/api/result':
            data.update(simulated=True, simulation_notice='Real main algorithms with simulated sensors/kinematics; not physical test evidence.')
            data.update(self.simulation_snapshot())
        return data


def proxy(console, public_port):
    if not isinstance(console,SimulationConsole):
        raise ValueError('Simulation relay requires the hardware-free SimulationConsole')
    console_port=console.p['port']
    class Relay(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def forward(self):
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 <= length <= 8192:
                self.send_error(413)
                return
            origin = self.headers.get('Origin')
            if origin and origin not in (f'http://127.0.0.1:{public_port}', f'http://localhost:{public_port}'):
                self.send_error(403)
                return
            scenario_path=urlparse(self.path).path
            if scenario_path in ('/api/simulation_scenario','/api/simulation_camera'):
                return self.scenario(length,scenario_path)
            headers = {k: v for k, v in self.headers.items() if k.lower() not in ('host', 'origin', 'connection')}
            if origin:
                headers['Origin'] = f'http://127.0.0.1:{console_port}'
            client = http.client.HTTPConnection('127.0.0.1', console_port, timeout=35)
            try:
                client.request(self.command, self.path, self.rfile.read(length), headers)
                response = client.getresponse()
                body = response.read()
                self.send_response(response.status)
                for key, value in response.getheaders():
                    if key.lower() in ('content-type', 'cache-control', 'x-content-type-options', 'x-frame-options'):
                        self.send_header(key, value)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except (OSError, http.client.HTTPException):
                self.send_error(502, 'Local ROS console unavailable')
            finally:
                client.close()

        def scenario(self,length,path):
            def reply(code,data):
                body=json.dumps(data,allow_nan=False).encode()
                self.send_response(code)
                for k,v in [('Content-Type','application/json'),('Content-Length',str(len(body))),
                            ('Cache-Control','no-store'),('X-Content-Type-Options','nosniff')]:
                    self.send_header(k,v)
                self.end_headers();self.wfile.write(body)
            if self.command=='GET':return reply(200,console.camera_get() if path=='/api/simulation_camera' else console.scenario_get())
            if not secrets.compare_digest(self.headers.get('X-Project-Token',''),console.token):
                return reply(403,dict(error='Missing simulation console session token'))
            try:
                body=json.loads(self.rfile.read(length) or b'{}')
                with console.control_lock:
                    generation=self.headers.get('X-Control-Generation')
                    if generation!=str(console.control_generation):
                        raise ValueError('Control context changed; refresh before changing the scenario')
                    action=console.camera_command if path=='/api/simulation_camera' else console.scenario_command
                    result=action(body,expected_generation=int(generation))
                return reply(200,result)
            except (ValueError,TypeError,KeyError) as exc:return reply(400,dict(error=str(exc)))
            except (OSError,RuntimeError) as exc:return reply(503,dict(error=str(exc)))
        do_GET = do_POST = forward
    server = ThreadingHTTPServer(('0.0.0.0', 8790), Relay)
    server.daemon_threads = True
    return server


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--texture', required=True)
    parser.add_argument('--public-port', type=int, default=8792)
    parser.add_argument('--output-directory', default='/tmp/ee3033_sim')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None, '0', '6') or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Requires isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    if not Path('/.dockerenv').exists():
        raise SystemExit('Run in the dedicated hardware-free Docker container')
    rclpy.init()
    root = str(Path(args.output_directory).resolve())
    if not root.startswith('/tmp/'):
        raise SystemExit('Simulation output must remain under /tmp inside the container')
    Path(root).mkdir(parents=True, exist_ok=True)
    # This world's independently specified camera is 10 degrees downward.
    # Keep its known TF geometry separate from the physical robot's 28.6 preset.
    vision = Path(root)/'simulation_vision.yaml'
    vision.write_text(json.dumps({'project_perception': {'ros__parameters': {'camera_height': 0., 'camera_pitch': 0., 'camera_yaw': 0., 'camera_x': 0., 'camera_y': 0., 'horizontal_fov': 0., 'camera_min_range': .35, 'camera_max_range': 2., 'use_pitch_override': False}}}))
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    # Inherit Console's mapping defaults, as the physical assessment entry does.
    # A separate fine-grid override can prevent stationary SLAM bootstrapping.
    console = SimulationConsole(parameter_overrides=[Parameter(k, value=v) for k, v in dict(
        port=8765, phase='mapping', allow_motion=True, manage_stages=True,
        stage_base=False, stage_lidar=False, stage_camera=False, model=args.model,
        map_directory=root+'/maps', session_directory=root+'/results',
        software_version=os.environ.get('PREVIEW_MAIN_COMMIT', 'main'),
        calibration_directory=root+'/calibration',vision_config=str(vision)).items()])
    robot = SimulatedRobot(args.texture)
    console.attach_simulation(robot,root)
    relay = proxy(console, args.public_port)
    threading.Thread(target=relay.serve_forever, daemon=True).start()
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (console, robot):
        executor.add_node(node)
    print(f'Actual ROS console + physical sensor model: http://127.0.0.1:{args.public_port}/#/workflow', flush=True)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        relay.shutdown()
        relay.server_close()
        executor.shutdown(timeout_sec=3)
        console.destroy_node()
        robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
