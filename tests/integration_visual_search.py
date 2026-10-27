"""Real Mission/filter/gate callbacks with synthetic detections and Nav2 actions."""
import json
import math
import os
import tempfile
import threading
import time
import unittest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ee3033_project.mission import Mission
from ee3033_project.target_filter import TargetFilter
from ee3033_project.velocity_gate import VelocityGate
from integration_ros import FakeRobot, overrides, wait


class VisualRobot(FakeRobot):
    def __init__(self, mode):
        self.mode = mode
        self.mission = None
        self.states = []
        self.max_steps = 0
        super().__init__('absent')
        self.raw_pub = self.create_publisher(String, '/project/target_detections', 10)

    def accept(self, request):
        if self.mode == 'stop_pending' and self.mission.goal_kind == 'acquire':
            time.sleep(.5)
        return super().accept(request)

    def observations(self):
        m = self.mission
        if m is None:
            return
        self.states.append(m.state)
        self.max_steps = max(self.max_steps, m.acquisition.steps)
        stamp = time.time()
        detections, observations = [], []
        if self.searching and self.search_goals:
            if self.mode != 'lost' or m.acquisition.started is None:
                bearing = math.atan2(.5-self.xy_yaw[1], .5-self.xy_yaw[0])
                detections = [dict(stamp=stamp, label='bottle', confidence=.9,
                                   bearing=bearing, origin=self.xy_yaw[:2], x=.5, y=.5, sigma=.3)]
            if self.mode == 'direct' or (self.mode == 'approach' and m.goal_kind == 'acquire' and m.state == 'LOCALIZING'):
                observations = [dict(stamp=stamp, x=.5, y=.5, confidence=.9, label='bottle', sigma=.03)]
        self.raw_pub.publish(String(data=json.dumps(dict(stamp=stamp, detections=detections))))
        self.obs_pub.publish(String(data=json.dumps(dict(stamp=stamp, observations=observations))))


class VisualSearch(unittest.TestCase):
    def case(self, mode):
        rclpy.init()
        with tempfile.TemporaryDirectory() as root:
            robot = VisualRobot(mode)
            mission = Mission(cli_args=['--ros-args', '-r', '/cmd_vel:=/test/motor_output'],
                parameter_overrides=overrides(allow_motion=True, target_class='bottle',
                    viewpoint_spacing=1., max_goals=2, observe_seconds=.3,
                    acquisition_seconds=3.5, acquisition_steps=3,
                    verify_seconds=.25, verify_timeout=2., results_directory=root))
            robot.mission = mission
            filt = TargetFilter()
            gate = VelocityGate(parameter_overrides=overrides(allow_motion=False, output_topic='/test/motor_output'))
            executor = MultiThreadedExecutor(num_threads=6)
            nodes = (robot, mission, filt, gate)
            for node in nodes:
                executor.add_node(node)
            thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
            try:
                self.assertTrue(wait(lambda: mission.readiness_error() is None and mission.nav.server_is_ready()))
                client = robot.create_client(Trigger, '/project/start')
                self.assertTrue(client.wait_for_service(timeout_sec=3))
                response = client.call_async(Trigger.Request())
                self.assertTrue(wait(response.done))
                self.assertTrue(response.result().success, response.result().message)
                if mode.startswith('stop'):
                    predicate = (lambda: mission.state == 'ACQUIRING' and mission.goal_pending) if mode == 'stop_pending' else (lambda: mission.state == 'LOCALIZING')
                    self.assertTrue(wait(predicate), mission.summary())
                    stop = robot.create_client(Trigger, '/project/stop')
                    self.assertTrue(stop.wait_for_service(timeout_sec=3))
                    response = stop.call_async(Trigger.Request()); self.assertTrue(wait(response.done))
                expected = 'STOPPED' if mode.startswith('stop') else 'COMPLETE' if mode in ('direct', 'approach') else 'NOT_FOUND'
                self.assertTrue(wait(lambda: mission.state in ('COMPLETE', 'NOT_FOUND', 'FAILED', 'STOPPED'), 25), mission.summary())
                time.sleep(.7)
                self.assertEqual(mission.state, expected, mission.summary())
                self.assertLessEqual(robot.max_active_goals, 1)
                self.assertLessEqual(robot.max_steps, 3)
                self.assertIsNone(mission.goal_handle)
                self.assertFalse(mission.goal_pending)
                self.assertIn('LOCALIZING', robot.states)
                self.assertEqual(robot.count_publishers('/cmd_vel'), 0)
                if expected == 'COMPLETE':
                    self.assertTrue(mission.return_verified)
                    self.assertEqual(sum(r.get('event') == 'target_reported' for r in robot.results), 1)
                elif expected == 'STOPPED':
                    count = robot.search_goals
                    time.sleep(.4)
                    self.assertEqual(robot.search_goals, count)
                else:
                    self.assertIsNone(mission.target)
                    self.assertTrue(mission.failures)
                print('PASS visual single-target:', mode, expected, flush=True)
            finally:
                executor.shutdown(timeout_sec=3); thread.join(timeout=3)
                for node in nodes:
                    node.destroy_node()
                rclpy.shutdown()

    def test_reliable_coordinates_report_without_approach(self): self.case('direct')
    def test_uncertain_candidate_approaches_then_reports(self): self.case('approach')
    def test_lost_candidate_resumes_bounded_search(self): self.case('lost')
    def test_unlocalizable_candidate_exhausts_budget(self): self.case('budget')
    def test_stop_during_localization(self): self.case('stop_localizing')
    def test_stop_with_late_approach_acceptance(self): self.case('stop_pending')


if __name__ == '__main__':
    if os.environ.get('ROS_DOMAIN_ID') in (None, '0', '6'):
        raise SystemExit('Use an isolated ROS domain')
    unittest.main(verbosity=2)
