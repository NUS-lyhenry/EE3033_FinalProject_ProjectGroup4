"""One real exposure -> Filter -> Mission -> canceled action -> verified return.

Synthetic observations and navigation actions, real ROS transport and production
callbacks. No hardware or motor endpoints. Run only in an isolated ROS domain.
"""
import json
import math
import os
import tempfile
import threading
import time
import unittest

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist

from ee3033_project.mission import Mission
from ee3033_project.target_filter import TargetFilter
from integration_ros import FakeRobot, overrides, wait


class ExposureRobot(FakeRobot):
    def __init__(self, mode):
        self.mode = mode
        self.mission = None
        self.center_goals = 0
        self.goal_kinds = []
        self.terminal_order = []
        super().__init__('absent')
        self.raw_pub = self.create_publisher(String, '/project/target_detections', 10)
        self.zero_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(.03, lambda: self.zero_pub.publish(Twist()))

    def perception_status(self):
        return dict(stamp=time.time(), ready=True, target_class='bottle',
                    localization_method='bottom_inverse', geometry_ready=True,
                    observation_ready=True, detected_candidates=0)

    def observations(self):
        pass  # This fixture sends exactly one exposure explicitly.

    def make_odometry(self, stamp):
        odom = super().make_odometry(stamp)
        # Normal balancing motion must not create a sampling prerequisite.
        phase = time.monotonic() * 18.
        odom.pose.pose.position.x += .018 * math.sin(phase)
        odom.twist.twist.linear.x = .08
        odom.twist.twist.angular.z = .20
        return odom

    def accept(self, request):
        if self.mode == 'late_center_stop' and self.mission.goal_kind == 'acquire':
            time.sleep(.4)
        return super().accept(request)

    async def execute(self, handle):
        kind = self.mission.goal_kind
        self.goal_kinds.append(kind)
        self.active_goals += 1
        self.max_active_goals = max(self.max_active_goals, self.active_goals)
        try:
            if kind == 'home':
                self.home_goals += 1
                if self.mode == 'return_failure':
                    handle.abort()
                else:
                    p = handle.request.pose.pose.position
                    self.xy_yaw = [p.x, p.y, 0.]
                    time.sleep(.15)
                    handle.succeed()
            else:
                if kind == 'acquire':
                    self.center_goals += 1
                    p = handle.request.pose.pose.position
                    assert math.dist((p.x, p.y), self.xy_yaw[:2]) < .03, 'Refinement translated'
                else:
                    self.search_goals += 1
                    self.xy_yaw = [.2, 0., 0.]
                # A failed centering action is deliberately held until its
                # bounded cancellation. No second image rescues the result.
                until = time.monotonic() + 15.
                while not handle.is_cancel_requested and time.monotonic() < until:
                    time.sleep(.01)
                if handle.is_cancel_requested:
                    if kind in ('search', 'search_scan') and self.mode in ('delayed_search_cancel', 'search_cancel_stop'):
                        time.sleep(1.8)
                    handle.canceled()
                else:
                    handle.abort()
            self.terminal_order.append(kind)
            return NavigateToPose.Result()
        finally:
            self.active_goals -= 1

    def exposure(self, centered):
        stamp = time.time()
        angle = 0. if centered else .26
        box = [300., 100., 340., 258.] if centered else [380., 100., 420., 258.]
        origin = list(self.xy_yaw[:2])
        x, y = origin[0] + .7 * math.cos(angle), origin[1] + .7 * math.sin(angle)
        d = dict(stamp=stamp, label='bottle', confidence=.95, box=box,
                 image_width=640, bearing=angle, bearing_valid=True,
                 origin=origin, capture_heading=0.,
                 range_model='bottom_inverse_17pt_20261031',
                 settled=False, centered=centered,
                 center_error=angle, center_error_px=0. if centered else 80.,
                 projection_valid=True, ranging_mode='calibrated',
                 range_state='within_range', range_m=.7,
                 attitude=dict(valid=False), search_eligible=True,
                 x=x, y=y, sigma=.35)
        o = dict(stamp=stamp, label='bottle', confidence=.95, box=box,
                 x=x, y=y, sigma=.35, method='bottom_inverse',
                 source_frame=stamp, quality_flags=['attitude_unverified'])
        self.raw_pub.publish(String(data=json.dumps(dict(stamp=stamp, detections=[d]))))
        self.obs_pub.publish(String(data=json.dumps(dict(stamp=stamp,
            observations=[o], bearings=[d],
            diagnostics=dict(localization_method='bottom_inverse')))))
        return stamp


class SingleFrameMission(unittest.TestCase):
    def run_case(self, mode):
        self.assertNotIn(os.environ.get('ROS_DOMAIN_ID'), (None, '0', '6'))
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        rclpy.init()
        with tempfile.TemporaryDirectory(prefix='single_frame_mission_') as directory:
            robot = ExposureRobot(mode)
            mission = Mission(parameter_overrides=overrides(
                allow_motion=True, target_class='bottle', results_directory=directory,
                clearance=.11, viewpoint_spacing=1., max_goals=4, max_candidates=12,
                verify_seconds=.5, verify_timeout=3., cancel_timeout=3.,
                max_target_variance=.01, acquisition_seconds=20.))
            robot.mission = mission
            if mode == 'planning':
                # Hold the real selection worker until acceptance cancels it.
                # A target can appear while search is still choosing a waypoint.
                def hold_selection(current):
                    while not mission.plan_cancel.is_set():
                        time.sleep(.01)
                    raise RuntimeError('Synthetic selection canceled after target')
                mission.select_viewpoint = hold_selection
            filt = TargetFilter(parameter_overrides=overrides(target_class='bottle'))
            executor = MultiThreadedExecutor(num_threads=8)
            for node in (robot, mission, filt):
                executor.add_node(node)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            def call(name):
                client = robot.create_client(Trigger, '/project/' + name)
                self.assertTrue(client.wait_for_service(timeout_sec=3))
                future = client.call_async(Trigger.Request())
                self.assertTrue(wait(future.done, 5))
                self.assertTrue(future.result().success, future.result().message)
            try:
                self.assertTrue(wait(lambda: mission.readiness_error() is None
                    and mission.nav.server_is_ready()))
                call('start')
                self.assertAlmostEqual(mission.planner.clearance, .11)
                if mode == 'planning':
                    self.assertTrue(wait(lambda: mission.state == 'PLANNING'
                        and mission.selection_future is not None
                        and filt.mission_stamp == mission.started_stamp), mission.summary())
                else:
                    self.assertTrue(wait(lambda: robot.search_goals > 0
                        and mission.state == 'SEARCHING'
                        and filt.mission_stamp == mission.started_stamp), mission.summary())
                stamp = robot.exposure(mode in ('centered', 'return_failure', 'planning'))
                self.assertTrue(wait(lambda: mission.target is not None, 5), mission.summary())
                if mode in ('stop', 'late_center_stop', 'search_cancel_stop'):
                    if mode == 'search_cancel_stop':
                        self.assertTrue(wait(lambda: mission.state == 'CANCELING'
                            and mission.cancel_next == 'ACQUIRE', 5), mission.summary())
                    else:
                        self.assertTrue(wait(lambda: mission.goal_kind == 'acquire'
                            and (mission.goal_pending or mission.goal_handle is not None), 5), mission.summary())
                    call('stop')
                    self.assertTrue(wait(lambda: mission.state == 'STOPPED'
                        and robot.active_goals == 0 and not mission.goal_pending, 6), mission.summary())
                    time.sleep(.5)
                    self.assertEqual(robot.home_goals, 0)
                    self.assertFalse(mission.return_verified)
                    self.assertFalse(mission.target_reported)
                else:
                    expected = 'STOPPED' if mode == 'return_failure' else 'COMPLETE'
                    self.assertTrue(wait(lambda: mission.state == expected, 12), mission.summary())
                    self.assertEqual(mission.return_verified, expected == 'COMPLETE')
                    self.assertTrue(mission.target_reported)
                    self.assertGreaterEqual(robot.home_goals, 1)
                    if mode in ('centered', 'planning'):
                        self.assertEqual(robot.center_goals, 0)
                    elif mode in ('off_center', 'delayed_search_cancel'):
                        self.assertEqual(robot.center_goals, 1)
                        self.assertGreater(time.time() - stamp, 2.)
                target = mission.target
                self.assertIsNotNone(target)
                self.assertEqual(target['source'], 'bottom_inverse_single')
                self.assertEqual(target['source_frames'], [stamp])
                self.assertEqual(target['frames'], 1)
                self.assertFalse(target['measurement_stable'])
                self.assertEqual(target['observation_stamp'], stamp)
                self.assertGreaterEqual(target['sigma'], .35 - 1e-9)
                self.assertEqual(robot.max_active_goals, 1)
                self.assertLessEqual(robot.center_goals, 1)
                self.assertTrue(all(k in ('search', 'search_scan', 'acquire', 'home')
                                    for k in robot.goal_kinds))
                if robot.home_goals:
                    self.assertEqual(robot.terminal_order[0], robot.goal_kinds[0])
                print(json.dumps(dict(case=mode, source_frames=1,
                    center_goals=robot.center_goals, home_goals=robot.home_goals,
                    overlapping_actions=robot.max_active_goals,
                    final_state=mission.state, original_stamp_preserved=True)), flush=True)
            finally:
                executor.shutdown(timeout_sec=4)
                thread.join(timeout=4)
                for node in (robot, mission, filt):
                    node.destroy_node()
                rclpy.shutdown()

    def test_single_centered_exposure(self):
        self.run_case('centered')

    def test_single_exposure_while_planning_is_not_discarded(self):
        self.run_case('planning')

    def test_single_exposure_survives_failed_centering_and_age(self):
        self.run_case('off_center')

    def test_slow_search_cancellation_does_not_strand_accepted_coordinate(self):
        self.run_case('delayed_search_cancel')

    def test_stop_preserves_accepted_coordinate_without_return(self):
        self.run_case('stop')

    def test_stop_during_late_center_acceptance(self):
        self.run_case('late_center_stop')

    def test_stop_while_original_search_cancellation_is_pending(self):
        self.run_case('search_cancel_stop')

    def test_return_failure_preserves_coordinate(self):
        self.run_case('return_failure')


if __name__ == '__main__':
    unittest.main()
