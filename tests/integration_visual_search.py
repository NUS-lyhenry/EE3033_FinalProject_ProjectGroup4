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
from ee3033_project.ros_utils import LATCHED
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
        self.sensor_frames = 0
        self.global_goals = 0
        self.global_after_lock = 0
        self.estimates = {}
        super().__init__('absent')
        self.create_subscription(String,'/project/target_estimate',self.on_estimate,LATCHED)
        self.raw_pub = self.create_publisher(String, '/project/target_detections', 10)

    def on_estimate(self,msg):
        data=json.loads(msg.data)
        if data.get('estimate_id'):self.estimates[data['estimate_id']]=data

    def accept(self, request):
        if self.mission.goal_kind == 'search':
            self.global_goals += 1
            self.global_after_lock += int(self.mission.target_detected)
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
        self.sensor_frames += 1
        detections, observations, bearings = [], [], []
        deferred = False
        if self.searching and self.search_goals:
            if self.mode != 'lost' or m.acquisition.started is None:
                bearing = math.atan2(.5-self.xy_yaw[1], .5-self.xy_yaw[0])
                detections = [dict(stamp=stamp, label='bottle', confidence=.9,
                                   bearing=bearing, origin=self.xy_yaw[:2], x=.5, y=.5, sigma=.3,
                                   close_view=self.mode == 'close_stop' and m.goal_kind == 'acquire')]
            if self.mode == 'direct' or (self.mode in ('approach', 'close_stop') and m.goal_kind == 'acquire' and m.state == 'LOCALIZING'):
                observations = [dict(stamp=stamp, x=.5, y=.5, confidence=.9, label='bottle', sigma=.03)]
            if self.mode == 'precision_boundary':
                observations = [dict(stamp=stamp, x=.5, y=.5, confidence=.9, label='bottle', sigma=.1)]
            if self.mode in ('soft_precision', 'stop_soft','tunable_precision') and m.acquisition.started is not None:
                observations = [dict(stamp=stamp, x=.5, y=.5, confidence=.9, label='bottle', sigma=.25)]
            if self.mode == 'deferred_balance':
                deferred = self.sensor_frames % 2 == 0
                if not deferred:
                    observations = [dict(stamp=stamp, x=.5, y=.5, confidence=.9, label='bottle', sigma=.03)]
            if self.mode == 'centered_late':
                detections = [dict(stamp=stamp, label='bottle', confidence=.9,
                                   bearing=self.xy_yaw[2], origin=self.xy_yaw[:2])]
                if m.acquisition.started is not None and time.monotonic()-m.acquisition.started > 2.2:
                    observations = [dict(stamp=stamp, x=.5, y=.5, confidence=.9, label='bottle', sigma=.03)]
            if self.mode=='temporary_loss' and m.acquisition.started is not None:
                lost_for=time.monotonic()-m.acquisition.started
                if lost_for<4.:
                    detections=[]
                else:
                    observations=[dict(stamp=stamp,x=.5,y=.5,confidence=.9,label='bottle',sigma=.03)]
            if self.mode=='bearing_only' and detections:
                d=detections[0]
                for key in ('x','y','sigma','close_view'):d.pop(key,None)
                center=(d['bearing']-self.xy_yaw[2]+math.pi)%(2*math.pi)-math.pi
                d.update(center_error=center,settled=m.state=='LOCALIZING',
                    odom_origin=self.xy_yaw[:2],odom_bearing=d['bearing'],map_odom=[0.,0.,0.],
                    position_sigma=.02,bearing_sigma=math.radians(1.),box=[280,0,360,479])
                bearings=[dict(d)]
        self.raw_pub.publish(String(data=json.dumps(dict(stamp=stamp, detections=detections))))
        self.obs_pub.publish(String(data=json.dumps(dict(stamp=stamp, observations=observations, bearings=bearings, deferred=deferred))))


class VisualSearch(unittest.TestCase):
    def case(self, mode):
        rclpy.init()
        with tempfile.TemporaryDirectory() as root:
            robot = VisualRobot(mode)
            mission = Mission(cli_args=['--ros-args', '-r', '/cmd_vel:=/test/motor_output'],
                parameter_overrides=overrides(allow_motion=True, target_class='bottle',
                    viewpoint_spacing=1., max_goals=2, observe_seconds=.3,
                    acquisition_seconds=12. if mode in ('temporary_loss','bearing_only') else 3.5,
                    acquisition_steps=5 if mode=='bearing_only' else 3,
                    max_target_variance=.25**2 if mode=='tunable_precision' else .1**2,
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
                    if mode == 'stop_soft':
                        predicate = lambda: mission.state == 'LOCALIZING' and mission.best_target is not None
                    self.assertTrue(wait(predicate), mission.summary())
                    robot.states.append(mission.state)
                    stop = robot.create_client(Trigger, '/project/stop')
                    self.assertTrue(stop.wait_for_service(timeout_sec=3))
                    response = stop.call_async(Trigger.Request()); self.assertTrue(wait(response.done))
                expected = 'STOPPED' if mode.startswith('stop') else 'COMPLETE' if mode in ('direct', 'approach', 'close_stop', 'soft_precision', 'precision_boundary', 'deferred_balance', 'centered_late','tunable_precision','temporary_loss','bearing_only') else 'FOUND_UNLOCALIZED'
                self.assertTrue(wait(lambda: mission.state in ('COMPLETE', 'FOUND_UNLOCALIZED', 'SEARCH_INCOMPLETE', 'NOT_FOUND', 'FAILED', 'STOPPED'), 25), mission.summary())
                time.sleep(.7)
                self.assertEqual(mission.state, expected, mission.summary())
                self.assertLessEqual(robot.max_active_goals, 1)
                self.assertLessEqual(mission.summary()['localization_elapsed_seconds'], mission.p['acquisition_seconds']+1.0)
                self.assertIsNone(mission.goal_handle)
                self.assertFalse(mission.goal_pending)
                self.assertIn('LOCALIZING', robot.states)
                self.assertEqual(robot.count_publishers('/cmd_vel'), 0)
                if expected == 'COMPLETE':
                    self.assertTrue(mission.return_verified)
                    self.assertEqual(sum(r.get('event') == 'target_reported' for r in robot.results), 1)
                    target=mission.target;posterior=robot.estimates[target['estimate_id']]
                    if mode=='bearing_only':
                        self.assertEqual(target['source'],'bearing_intersection')
                        self.assertGreaterEqual(target['sites'],2)
                        self.assertGreaterEqual(len(target['source_frames']),8)
                        self.assertLess(math.hypot(target['x']-.5,target['y']-.5),.08)
                    if mode=='temporary_loss':
                        self.assertGreater(mission.acquisition.active_seconds,4.)
                        self.assertEqual(robot.global_after_lock,0)
                    for key in ('x','y','covariance','posterior_mean','projected','map_signature'):
                        self.assertEqual(target[key],posterior[key])
                    self.assertEqual(target['estimator'],'map-constrained MMSE')
                    self.assertTrue(posterior['confirmed'])
                    if mode == 'soft_precision':
                        self.assertEqual(mission.target['quality'], 'approximate')
                        self.assertGreater(mission.target['sigma'], .1)
                        self.assertAlmostEqual(mission.target['variance'],mission.target['sigma']**2)
                    elif mode in ('precision_boundary','tunable_precision'):
                        self.assertEqual(mission.target['quality'], 'preferred_precision')
                elif expected == 'STOPPED':
                    self.assertIsNone(mission.target)
                    count = robot.search_goals
                    time.sleep(.4)
                    self.assertEqual(robot.search_goals, count)
                else:
                    self.assertIsNone(mission.target)
                    self.assertTrue(mission.target_detected)
                    self.assertTrue(mission.return_verified)
                    self.assertEqual(robot.global_goals, 1)
                    self.assertEqual(robot.global_after_lock, 0)
                print('PASS visual single-target:', mode, expected, flush=True)
            finally:
                executor.shutdown(timeout_sec=3); thread.join(timeout=3)
                for node in nodes:
                    node.destroy_node()
                rclpy.shutdown()

    def test_reliable_coordinates_report_without_approach(self): self.case('direct')
    def test_uncertain_candidate_approaches_then_reports(self): self.case('approach')
    def test_approach_stops_as_soon_as_a_close_view_is_available(self): self.case('close_stop')
    def test_lost_candidate_returns_without_resuming_search(self): self.case('lost')
    def test_unlocalizable_candidate_exhausts_budget(self): self.case('budget')
    def test_stop_during_localization(self): self.case('stop_localizing')
    def test_stop_with_late_approach_acceptance(self): self.case('stop_pending')
    def test_configured_quarter_metre_precision_accepts_shared_posterior(self): self.case('tunable_precision')

    def test_high_uncertainty_stable_target_reports_after_refinement_budget(self): self.case('soft_precision')
    def test_stop_prevents_reporting_the_retained_soft_estimate(self): self.case('stop_soft')
    def test_preferred_precision_boundary_handles_float_rounding(self): self.case('precision_boundary')
    def test_deferred_visible_motion_frames_are_not_detector_misses(self): self.case('deferred_balance')
    def test_centered_target_can_finish_confirmation_after_the_old_early_exit(self): self.case('centered_late')
    def test_four_second_loss_is_reacquired_without_global_search(self):self.case('temporary_loss')
    def test_bearing_only_mission_automatically_collects_A_B_and_reports_MMSE(self):self.case('bearing_only')


if __name__ == '__main__':
    if os.environ.get('ROS_DOMAIN_ID') in (None, '0', '6'):
        raise SystemExit('Use an isolated ROS domain')
    unittest.main(verbosity=2)
