"""Native Humble search with synthetic sensors/actions on an isolated domain.

No physical drivers or motor output publishers. Exercises actual Mission ROS
callbacks, fresh-frame coverage, bounded retries, dynamic goals and Stop.
"""
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from ee3033_project.mission import Mission
from ee3033_project.velocity_gate import VelocityGate
from integration_ros import FakeRobot, overrides, wait


class SearchRobot(FakeRobot):
    def __init__(self, mode):
        self.mode = mode
        self.mission = None
        self.frame_key = None
        self.frame_stamp = None
        super().__init__('absent')

    def perception_status(self):
        data = super().perception_status()
        mission = self.mission
        if mission is not None and mission.state == 'OBSERVING':
            if self.mode == 'duplicate_frames':
                if self.frame_key != mission.observe_after_stamp:
                    self.frame_key = mission.observe_after_stamp
                    self.frame_stamp = time.time()
                data['stamp'] = self.frame_stamp
            elif self.mode == 'prearrival_frames':
                data['stamp'] = mission.observe_after_stamp-.01
            elif self.mode == 'unlocalized_candidate':
                data['detected_candidates'] = 1
        return data

    async def execute(self, handle):
        pose = handle.request.pose.pose.position
        home = abs(pose.x) < 1e-6 and abs(pose.y) < 1e-6
        result = await super().execute(handle)
        if self.mode == 'misaligned' and not home:
            self.xy_yaw[2] += 1.
        return result


class AdaptiveSearch(unittest.TestCase):
    def run_case(self, mode):
        rclpy.init()
        with tempfile.TemporaryDirectory(prefix='group4_adaptive_') as root:
            robot = SearchRobot(mode)
            mission = Mission(cli_args=['--ros-args','-r','/cmd_vel:=/test/motor_output'],
                parameter_overrides=overrides(allow_motion=True, target_class='bottle',
                    viewpoint_spacing=1., max_goals=3, max_candidates=1,
                    observe_seconds=.25, min_observation_frames=3,
                    max_observation_retries=1, verify_seconds=.25, verify_timeout=2.,
                    sensor_timeout=.8, perception_timeout=1.5, results_directory=root))
            robot.mission = mission
            gate = VelocityGate(parameter_overrides=overrides(
                allow_motion=False, output_topic='/test/motor_output'))
            executor = MultiThreadedExecutor(num_threads=6)
            for node in (robot,mission,gate):
                executor.add_node(node)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            try:
                self.assertTrue(wait(lambda: mission.readiness_error() is None
                                     and mission.nav.server_is_ready()))
                start = robot.create_client(Trigger, '/project/start')
                self.assertTrue(start.wait_for_service(timeout_sec=3))
                response = start.call_async(Trigger.Request())
                self.assertTrue(wait(response.done))
                self.assertTrue(response.result().success, response.result().message)
                if mode == 'stop':
                    self.assertTrue(wait(lambda: mission.state == 'OBSERVING'))
                    stop = robot.create_client(Trigger, '/project/stop')
                    self.assertTrue(stop.wait_for_service(timeout_sec=3))
                    response = stop.call_async(Trigger.Request())
                    self.assertTrue(wait(response.done))
                expected = 'STOPPED' if mode == 'stop' else 'SEARCH_INCOMPLETE'
                self.assertTrue(wait(lambda: mission.state in ('STOPPED','FAILED','SEARCH_INCOMPLETE','NOT_FOUND'),
                                     timeout=15), mission.summary())
                time.sleep(.3)
                self.assertEqual(mission.state, expected, mission.summary())
                self.assertIsNone(mission.target)
                self.assertGreater(len(mission.planner.options),len(mission.route))
                self.assertLessEqual(robot.max_active_goals,1)
                self.assertEqual(robot.commands[-1][1:],(0.,0.))
                self.assertFalse(mission.goal_pending)
                self.assertIsNone(mission.goal_handle)
                if mode == 'valid':
                    self.assertTrue(mission.seen)
                    self.assertFalse(mission.failures)
                elif mode == 'stop':
                    self.assertEqual(robot.home_goals,0)
                    goals = robot.search_goals
                    time.sleep(.4)
                    self.assertEqual(robot.search_goals,goals)
                else:
                    self.assertFalse(mission.seen)
                    self.assertEqual(len(mission.failures),len(mission.route))
                    if mode == 'misaligned':
                        self.assertGreater(robot.search_goals,len(mission.route))
                payload = json.loads((mission.run_dir/'result.json').read_text())
                self.assertEqual(payload['state'],expected)
                self.assertTrue(payload['candidate_threshold_exceeded'])
                self.assertEqual(payload['unobserved_cells'],
                                 len(mission.planner.target_cells-mission.seen))
                print('PASS adaptive search:',mode,expected,'observed',len(mission.seen))
            finally:
                executor.shutdown(timeout_sec=3)
                thread.join(timeout=3)
                for node in (mission,gate,robot):
                    node.destroy_node()
                rclpy.shutdown()

    def test_valid_frames_credit_coverage_and_replan(self): self.run_case('valid')
    def test_duplicate_frames_do_not_credit_coverage(self): self.run_case('duplicate_frames')
    def test_prearrival_frames_do_not_credit_coverage(self): self.run_case('prearrival_frames')
    def test_detected_but_unlocalized_target_does_not_credit_empty_coverage(self): self.run_case('unlocalized_candidate')
    def test_false_navigation_heading_retries_without_coverage(self): self.run_case('misaligned')
    def test_stop_during_observation_prevents_successor(self): self.run_case('stop')


if __name__ == '__main__':
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6'):
        raise SystemExit('Use an isolated ROS domain, e.g.107')
    unittest.main(verbosity=2)
