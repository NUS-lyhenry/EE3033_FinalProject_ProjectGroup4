#!/usr/bin/env python3
"""Paired real-Mission ROS fixture with synthetic images and navigation actions.

No YOLO, Nav2 controller, SLAM or hardware is exercised. Mock navigation follows
collision-clear grid paths with translation/rotation durations accelerated by
the same factor in both versions. Reported task_seconds is measured wall time
from Mission Start through return verification, excluding mapping.
"""
import argparse
import json
import math
import os
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ee3033_project.bottom_range import BottomCamera, CALIBRATION_K, CALIBRATION_D
from ee3033_project.core import angle_difference, quaternion_yaw
from ee3033_project.coverage import CoveragePlanner
from ee3033_project.mission import Mission
from ee3033_project.target_filter import TargetFilter
from ee3033_project.vision_geometry import FloorCamera
from integration_ros import FakeRobot, overrides, wait


class SearchFixture(FakeRobot):
    def __init__(self, scene, target, scale):
        self.scene, self.layout, self.scale = scene, target, scale
        self.mission = None
        self.trajectory = []
        self.first_detection = self.first_report = None
        self.path_metres = self.turn_radians = self.nominal_motion_seconds = 0.
        self.goal_kinds = []
        super().__init__('absent')
        grid = np.zeros((25, 35), int)
        grid[[0, -1], :] = 100
        grid[:, [0, -1]] = 100
        if scene != 'open':
            grid[8:, 17:19] = 100
        if scene == 'alcove':
            grid[8:10, 22:31] = 100
            grid[8:19, 29:31] = 100
        if scene == 'open' and target == 'occluded':
            grid[8:14, 14] = 100
        self.camera = FloorCamera.from_metadata(BottomCamera(640, 480, CALIBRATION_K, CALIBRATION_D).metadata())
        self.geometry = CoveragePlanner(grid, .1, (-.75, -.75, 0.), clearance=.15,
            spacing=.65, headings=6, camera_model=self.camera)
        self.geometry.prepare(0., 0.)
        xy = {'near': (.6, 0.), 'far': (1.9, 1.3), 'occluded': (1.9, .7)}.get(target)
        self.target_cell = min(self.geometry.target_cells,
            key=lambda cell: math.dist(self.geometry.cell_to_world(*cell), xy)) if xy else None
        self.target = self.geometry.cell_to_world(*self.target_cell) if xy else None
        m = self.fixture_map
        m.info.width, m.info.height = grid.shape[1], grid.shape[0]
        m.info.origin.position.x = m.info.origin.position.y = -.75
        m.data = grid.ravel().tolist()
        self.map_pub.publish(m)
        self.raw_pub = self.create_publisher(String, '/project/target_detections', 10)
        self.zero_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(.03, lambda: self.zero_pub.publish(Twist()))

    def perception_status(self):
        status = super().perception_status()
        status.update(localization_method='bottom_inverse', observation_ready=True)
        return status

    def observations(self):
        m = self.mission
        if m is None or not m.started_stamp or m.state not in m.ACTIVE:
            return
        stamp = time.time()
        row, col = self.geometry.world_to_cell(*self.xy_yaw[:2])
        visible = self.target_cell is not None and self.target_cell in self.geometry.visible(
            row, col, self.xy_yaw[2], camera_model=self.camera, position=self.xy_yaw[:2])
        detections, observations = [], []
        if visible:
            if self.first_detection is None:
                self.first_detection = stamp-m.started_stamp
            x, y = self.target
            bearing = math.atan2(y-self.xy_yaw[1], x-self.xy_yaw[0])
            error = angle_difference(bearing, self.xy_yaw[2])
            d = dict(stamp=stamp, label='bottle', confidence=.95,
                box=[300., 100., 340., 258.], image_width=640,
                bearing=bearing, bearing_valid=True, origin=list(self.xy_yaw[:2]),
                capture_heading=self.xy_yaw[2], range_model='bottom_inverse_17pt_20261031',
                settled=False, centered=abs(error)<.05, center_error=error,
                center_error_px=error*300., projection_valid=True,
                ranging_mode='calibrated', range_state='within_range',
                range_m=math.dist(self.target, self.xy_yaw[:2]),
                attitude=dict(valid=False), search_eligible=True, x=x, y=y, sigma=.35)
            detections = [d]
            observations = [dict(stamp=stamp, label='bottle', confidence=.95,
                box=d['box'], x=x, y=y, sigma=.35, method='bottom_inverse',
                source_frame=stamp, quality_flags=['synthetic_fixture'])]
        self.raw_pub.publish(String(data=json.dumps(dict(stamp=stamp, detections=detections))))
        self.obs_pub.publish(String(data=json.dumps(dict(stamp=stamp, observations=observations,
            bearings=detections, diagnostics=dict(localization_method='bottom_inverse')))))

    def segment(self, handle, destination):
        start = list(self.xy_yaw)
        distance = math.dist(start[:2], destination[:2])
        turn = angle_difference(destination[2], start[2])
        nominal = distance/.12+abs(turn)/.4
        duration = nominal/self.scale
        steps = max(1, math.ceil(duration/.025))
        for i in range(1, steps+1):
            if handle.is_cancel_requested:
                return False
            fraction = i/steps
            self.xy_yaw = [start[0]+(destination[0]-start[0])*fraction,
                           start[1]+(destination[1]-start[1])*fraction,
                           start[2]+turn*fraction]
            self.path_metres += distance/steps
            self.turn_radians += abs(turn)/steps
            self.nominal_motion_seconds += nominal/steps
            self.trajectory.append(dict(stamp=time.time(), pose=list(self.xy_yaw), kind=self.mission.goal_kind))
            if duration:
                time.sleep(duration/steps)
        return True

    async def execute(self, handle):
        self.active_goals += 1
        self.max_active_goals = max(self.max_active_goals, self.active_goals)
        kind = self.mission.goal_kind
        self.goal_kinds.append(kind)
        if kind == 'home':
            self.home_goals += 1
        else:
            self.search_goals += 1
        try:
            p = handle.request.pose.pose
            destination = (p.position.x, p.position.y, quaternion_yaw(p.orientation))
            points = []
            if math.dist(self.xy_yaw[:2], destination[:2]) >= .01:
                field, parents = self.geometry.travel_field(*self.xy_yaw[:2])
                goal = self.geometry.world_to_cell(*destination[:2])
                if not math.isfinite(float(field[goal])):
                    handle.abort()
                    return NavigateToPose.Result()
                cells, cell = [], goal
                while cell in parents:
                    cells.append(cell)
                    cell = parents[cell]
                points = [self.geometry.cell_to_world(*c) for c in reversed(cells)]
                points.append(destination[:2])
            for xy in points:
                heading = math.atan2(xy[1]-self.xy_yaw[1], xy[0]-self.xy_yaw[0])
                if not self.segment(handle, (*self.xy_yaw[:2], heading)) or not self.segment(handle, (*xy, heading)):
                    handle.canceled()
                    return NavigateToPose.Result()
            if not self.segment(handle, destination):
                handle.canceled()
            else:
                handle.succeed()
            return NavigateToPose.Result()
        finally:
            self.active_goals -= 1


def run(args):
    if os.environ.get('ROS_DOMAIN_ID') != '156' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('This fixture is restricted to isolated domain 156')
    directory = Path(args.output)
    directory.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    robot = SearchFixture(args.scene, args.target, args.time_scale)
    mission = Mission(parameter_overrides=overrides(allow_motion=True, target_class='bottle',
        results_directory=str(directory/'mission'), search_strategy=args.strategy,
        viewpoint_spacing=.65, max_goals=120, max_candidates=250,
        verify_seconds=.5, verify_timeout=3., cancel_timeout=3.,
        acquisition_seconds=20., mission_timeout=900., return_tolerance=.20))
    robot.mission = mission
    filt = TargetFilter(parameter_overrides=overrides(target_class='bottle'))
    executor = MultiThreadedExecutor(num_threads=8)
    nodes = (robot, mission, filt)
    for node in nodes:
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    error = None
    try:
        if not wait(lambda: mission.readiness_error() is None and mission.nav.server_is_ready(), 10.):
            raise RuntimeError('Fixture readiness failed: '+str(mission.readiness_error()))
        client = robot.create_client(Trigger, '/project/start')
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError('Mission Start service unavailable')
        future = client.call_async(Trigger.Request())
        if not wait(future.done, 5.) or not future.result().success:
            raise RuntimeError('Mission Start failed')
        deadline = time.monotonic()+args.timeout
        while time.monotonic()<deadline and mission.state not in ('COMPLETE','SEARCH_INCOMPLETE','NOT_FOUND','FOUND_UNLOCALIZED','FAILED','STOPPED'):
            if mission.target_reported and robot.first_report is None:
                robot.first_report = time.monotonic()-mission.started
            time.sleep(.02)
        if mission.target_reported and robot.first_report is None:
            robot.first_report = time.monotonic()-mission.started
        result = mission.summary()
        if not mission.finished_at:
            error = 'Fixture wall-time budget exhausted'
        expected = result['state'] == 'COMPLETE' if args.target != 'absent' else result['state'] in ('SEARCH_INCOMPLETE','NOT_FOUND')
        accepted = bool(expected and result['return_verified'] and not error and robot.max_active_goals <= 1
                        and (args.target == 'absent' or result['target_reported'] and result['target']))
        payload = dict(evidence='Native production Mission/Filter over ROS; synthetic visibility detections and collision-clear mock navigation; no YOLO/Nav2-controller/physical claim',
            source_version=args.version, scene=args.scene, target_layout=args.target, strategy=args.strategy,
            time_scale=args.time_scale, map_resolution=.1, target_truth=robot.target,
            first_detection_seconds=robot.first_detection, first_report_seconds=robot.first_report,
            task_seconds=result['elapsed_seconds'] if mission.finished_at else None,
            task_timing='Measured Start to report, home return and verified stop; mapping excluded; mock motion accelerated',
            path_metres=robot.path_metres, turn_radians=robot.turn_radians,
            nominal_mock_motion_seconds=robot.nominal_motion_seconds,
            home_goals=robot.home_goals, max_overlapping_actions=robot.max_active_goals,
            accepted=accepted, error=error, result=result)
        (directory/'comparison.json').write_text(json.dumps(payload, indent=2, allow_nan=False)+'\n')
        (directory/'trajectory.json').write_text(json.dumps(robot.trajectory, separators=(',', ':'))+'\n')
        print(json.dumps({k:v for k,v in payload.items() if k not in ('result','evidence')}, allow_nan=False), flush=True)
        return 0 if accepted else 1
    finally:
        if mission.state in mission.ACTIVE:
            mission.stop(None, Trigger.Response())
            wait(lambda:robot.active_goals == 0, 3.)
        executor.shutdown(timeout_sec=4.)
        thread.join(timeout=4.)
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=('open','divider','alcove'), required=True)
    parser.add_argument('--target', choices=('near','far','occluded','absent'), required=True)
    parser.add_argument('--strategy', choices=('greedy','adaptive'), required=True)
    parser.add_argument('--version', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--time-scale', type=float, default=12.)
    parser.add_argument('--timeout', type=float, default=70.)
    raise SystemExit(run(parser.parse_args()))
