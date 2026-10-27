#!/usr/bin/env python3
"""Run the actual managed console/stage launches with simulated physical sensors.

Run only inside the dedicated Docker bridge on a non-robot ROS domain. The web
relay tags simulated evidence; all control requests go unchanged to Console.
"""
import argparse
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.parameter import Parameter

from ee3033_project.console import Console
from simulation_robot import SimulatedRobot


class SimulationConsole(Console):
    def snapshot(self):
        data = super().snapshot()
        data.update(simulated=True, preview_interactive=True, preview_backend='main-ros',
            preview_hint='Current main programs with SLAM, Nav2 and YOLO; only sensors and the chassis are simplified. Original speeds and timeouts are retained.')
        return data

    def workflow_get(self, path):
        data = super().workflow_get(path)
        if path == '/api/result':
            data.update(simulated=True, simulation_notice='Real main algorithms with simulated sensors/kinematics; not physical test evidence.')
        return data


def proxy(console_port, public_port):
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
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    console = SimulationConsole(parameter_overrides=[Parameter(k, value=v) for k, v in dict(
        port=8765, phase='mapping', allow_motion=True, manage_stages=True,
        stage_base=False, stage_lidar=False, stage_camera=False, model=args.model,
        map_directory=root+'/maps', session_directory=root+'/results',
        software_version=os.environ.get('PREVIEW_MAIN_COMMIT', 'main'),
        calibration_directory=root+'/calibration').items()])
    robot = SimulatedRobot(args.texture)
    relay = proxy(8765, args.public_port)
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
