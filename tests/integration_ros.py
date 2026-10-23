"""Real ROS 2 integration with mock Nav2/sensors; never connects to motors.

Run on an isolated domain: ROS_DOMAIN_ID=88 python3 tests/integration_ros.py
Uses installed Humble action/message libraries, not Python interface stubs.
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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from ee3033_project.mission import Mission
from ee3033_project.target_filter import TargetFilter
from ee3033_project.velocity_gate import VelocityGate
from ee3033_project.mapper import Mapper
from ee3033_project.ros_utils import LATCHED, set_yaw
from ee3033_project.core import quaternion_yaw


def overrides(**values):
    return [Parameter(k, value=v) for k,v in values.items()]


def wait(predicate, timeout=12.0):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class FakeRobot(Node):
    def __init__(self, scenario):
        super().__init__('mock_robot')
        self.scenario = scenario
        self.xy_yaw = [0.0,0.0,0.0]
        self.live = True
        self.searching = False
        self.search_goals = 0
        self.home_goals = 0
        self.canceled = 0
        self.active_goals = 0
        self.max_active_goals = 0
        self.results = []
        self.commands = []
        self.tf = TransformBroadcaster(self)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.amcl_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_fixed', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/project/perception_status', 10)
        self.obs_pub = self.create_publisher(String, '/project/target_observations', 10)
        self.create_subscription(Bool, '/project/search_active',
                                 lambda m: setattr(self,'searching',m.data), LATCHED)
        self.create_subscription(String, '/project/mission_result',
                                 lambda m: self.results.append(json.loads(m.data)), LATCHED)
        self.create_subscription(Twist, '/test/motor_output',
                                 lambda m: self.commands.append((time.monotonic(),m.linear.x,m.angular.z)), 10)
        self.server = ActionServer(self, NavigateToPose, '/navigate_to_pose',
                                    execute_callback=self.execute,
                                    goal_callback=self.accept,
                                    cancel_callback=self.cancel,
                                    callback_group=ReentrantCallbackGroup())
        self.timer = self.create_timer(0.05,self.publish)
        self.obs_timer = self.create_timer(0.15,self.observations)
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.info.width = m.info.height = 24
        m.info.resolution = 0.1
        m.info.origin.position.x = m.info.origin.position.y = -1.2
        m.info.origin.orientation.w = 1.0
        m.data = [100 if r in (0,23) or c in (0,23) else 0
                  for r in range(24) for c in range(24)]
        self.map_pub.publish(m)

    def accept(self, request):
        del request
        if self.scenario == 'delayed_accept':
            time.sleep(0.5)
        return GoalResponse.ACCEPT

    def cancel(self, goal):
        del goal
        self.canceled += 1
        return CancelResponse.ACCEPT

    async def execute(self, handle):
        self.active_goals += 1
        self.max_active_goals = max(self.max_active_goals,self.active_goals)
        pose = handle.request.pose.pose
        home = abs(pose.position.x) < 1e-6 and abs(pose.position.y) < 1e-6
        if home:
            self.home_goals += 1
        else:
            self.search_goals += 1
        if home and self.scenario == 'false_home_success':
            self.xy_yaw = [0.6,0.6,0.0]
        else:
            self.xy_yaw = [pose.position.x,pose.position.y,quaternion_yaw(pose.orientation)]
        duration = 1.4 if self.scenario in ('found','stale','delayed_accept') and not home else 0.18
        if self.scenario == 'mapping_timeout':
            duration = 2.0
        deadline = time.monotonic()+duration
        if self.scenario == 'stale' and not home:
            self.live = False
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                handle.canceled()
                self.active_goals -= 1
                return NavigateToPose.Result()
            time.sleep(0.02)
        if self.scenario == 'search_abort' and not home:
            handle.abort()
        else:
            handle.succeed()
        self.active_goals -= 1
        return NavigateToPose.Result()

    def publish(self):
        if not self.live:
            return
        stamp = self.get_clock().now().to_msg()
        tr = TransformStamped()
        tr.header.stamp, tr.header.frame_id, tr.child_frame_id = stamp,'map','base_footprint'
        tr.transform.translation.x, tr.transform.translation.y = self.xy_yaw[:2]
        set_yaw(tr.transform.rotation,self.xy_yaw[2])
        self.tf.sendTransform(tr)
        amcl = PoseWithCovarianceStamped()
        amcl.header.stamp,amcl.header.frame_id = stamp,'map'
        amcl.pose.pose.position.x,amcl.pose.pose.position.y = self.xy_yaw[:2]
        amcl.pose.pose.orientation.w = 1.0
        amcl.pose.covariance[0] = amcl.pose.covariance[7] = amcl.pose.covariance[35] = 0.01
        self.amcl_pub.publish(amcl)
        odom = self.make_odometry(stamp)
        if odom is not None:self.odom_pub.publish(odom)
        scan = LaserScan()
        scan.header.stamp,scan.header.frame_id = stamp,'laser'
        scan.angle_min,scan.angle_max,scan.angle_increment = -math.pi,math.pi,2*math.pi/99
        scan.range_min,scan.range_max = 0.08,8.0
        scan.ranges = [3.0]*100
        self.scan_pub.publish(scan)
        self.status_pub.publish(String(data=json.dumps(self.perception_status())))

    def perception_status(self):
        return {'stamp':time.time(), 'ready':True, 'target_class':'bottle',
                'detected_candidates':0}

    def make_odometry(self, stamp):
        odom=Odometry()
        odom.header.stamp,odom.header.frame_id,odom.child_frame_id=stamp,'odom_combined','base_footprint'
        odom.pose.pose.position.x,odom.pose.pose.position.y=self.xy_yaw[:2]
        set_yaw(odom.pose.pose.orientation,self.xy_yaw[2])
        return odom

    def observations(self):
        stamp = time.time()
        obs = []
        if self.scenario in ('found','false_home_success') and self.searching and self.search_goals:
            obs = [{'stamp':stamp,'x':0.5,'y':0.5,'confidence':0.9,
                    'label':'bottle','sigma':0.08}]
        self.obs_pub.publish(String(data=json.dumps({'stamp':stamp,'observations':obs})))

    def destroy_node(self):
        self.server.destroy()
        super().destroy_node()


class Integration(unittest.TestCase):
    def run_mission(self, scenario):
        rclpy.init()
        root = tempfile.TemporaryDirectory(prefix='group4_test_')
        robot = FakeRobot(scenario)
        mission = Mission(cli_args=['--ros-args','-r','/cmd_vel:=/test/motor_output'],parameter_overrides=overrides(
            allow_motion=True, target_class='bottle', viewpoint_spacing=1.0, max_goals=4, max_candidates=12,
            observe_seconds=0.20, verify_seconds=0.25, verify_timeout=1.5,
            cancel_timeout=2.0, sensor_timeout=0.6, perception_timeout=1.5,
            results_directory=root.name))
        filt = TargetFilter()
        gate = VelocityGate(parameter_overrides=overrides(
            allow_motion=True, output_topic='/test/motor_output'))
        executor = MultiThreadedExecutor(num_threads=6)
        for node in (robot,mission,filt,gate):
            executor.add_node(node)
        thread = threading.Thread(target=executor.spin,daemon=True)
        thread.start()
        try:
            self.assertTrue(wait(lambda: mission.readiness_error() is None
                                 and mission.nav.server_is_ready()))
            client = robot.create_client(Trigger,'/project/start')
            self.assertTrue(client.wait_for_service(timeout_sec=3))
            future = client.call_async(Trigger.Request())
            self.assertTrue(wait(future.done))
            self.assertTrue(future.result().success, future.result().message)
            if scenario == 'delayed_accept':
                self.assertTrue(wait(lambda: mission.goal_pending))
                stop_client = robot.create_client(Trigger,'/project/stop')
                self.assertTrue(stop_client.wait_for_service(timeout_sec=3))
                stopped = stop_client.call_async(Trigger.Request())
                self.assertTrue(wait(stopped.done))
            self.assertTrue(wait(lambda: mission.state in ('COMPLETE','NOT_FOUND','FAILED','STOPPED'),
                                 timeout=18),mission.summary())
            # Allow any late acceptance/cancellation and terminal heartbeat to settle.
            time.sleep(0.7)
            expected = {'found':'COMPLETE','absent':'NOT_FOUND','search_abort':'NOT_FOUND',
                        'stale':'FAILED','delayed_accept':'STOPPED',
                        'false_home_success':'FAILED'}[scenario]
            self.assertEqual(mission.state, expected, mission.summary())
            self.assertLessEqual(robot.max_active_goals,1)
            self.assertTrue(robot.commands)
            self.assertEqual(robot.commands[-1][1:],(0.0,0.0))
            self.assertFalse(mission.goal_pending)
            self.assertIsNone(mission.goal_handle)
            payload = json.loads((mission.run_dir/'result.json').read_text())
            self.assertEqual(payload['state'],expected)
            if scenario == 'found':
                self.assertEqual(payload['target']['x'],0.5)
                self.assertGreaterEqual(robot.canceled,1)
                self.assertEqual(robot.home_goals,1)
                self.assertTrue(any(m.get('event') == 'target_reported' for m in robot.results))
            if scenario == 'search_abort':
                self.assertEqual(len(payload['failed_viewpoints']),len(mission.route))
            if scenario in ('stale','delayed_accept'):
                self.assertEqual(robot.home_goals,0)
            if scenario == 'false_home_success':
                self.assertIn('verification',payload['reason'])
        finally:
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            for node in (mission,filt,gate,robot):
                node.destroy_node()
            rclpy.shutdown()
            root.cleanup()

    def test_found_and_return(self):
        self.run_mission('found')

    def test_target_absent_returns_but_does_not_claim_success(self):
        self.run_mission('absent')

    def test_failed_viewpoints_are_skipped_and_recorded(self):
        self.run_mission('search_abort')

    def test_sensor_loss_cancels_and_stops(self):
        self.run_mission('stale')

    def test_stop_during_goal_acceptance_cancels_late_goal(self):
        self.run_mission('delayed_accept')

    def test_nav2_success_outside_home_cannot_complete_mission(self):
        self.run_mission('false_home_success')


class GateIntegration(unittest.TestCase):
    def test_velocity_limits_obstacle_stop_and_missing_heartbeat(self):
        rclpy.init()
        io = Node('gate_test_inputs')
        gate = VelocityGate(parameter_overrides=overrides(
            allow_motion=True, output_topic='/test/gated_velocity',
            command_timeout=0.25, heartbeat_timeout=0.25, scan_timeout=0.25))
        command_pub = io.create_publisher(Twist,'/project/nav_cmd_vel',10)
        enable_pub = io.create_publisher(Bool,'/project/drive_enabled',LATCHED)
        scan_pub = io.create_publisher(LaserScan,'/scan_fixed',qos_profile_sensor_data)
        received = []
        io.create_subscription(Twist,'/test/gated_velocity',
                               lambda m: received.append((m.linear.x,m.angular.z)),10)
        controls = {'heartbeat':True,'commands':True,'scans':True,
                    'front':3.0,'back':3.0,'linear':0.8,'stale':False}

        def publish():
            if controls['heartbeat']:
                enable_pub.publish(Bool(data=True))
            if controls['commands']:
                cmd = Twist()
                cmd.linear.x,cmd.angular.z = controls['linear'],0.9
                command_pub.publish(cmd)
            if controls['scans']:
                msg = LaserScan()
                msg.header.stamp = io.get_clock().now().to_msg()
                if controls['stale']:
                    msg.header.stamp.sec -= 3
                msg.range_min,msg.range_max = 0.08,8.0
                msg.angle_min,msg.angle_increment = -math.pi,math.pi/2
                msg.ranges = [controls['back'],3.0,controls['front'],3.0]
                scan_pub.publish(msg)
        timer = io.create_timer(0.03,publish)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(io)
        executor.add_node(gate)
        thread = threading.Thread(target=executor.spin,daemon=True)
        thread.start()
        try:
            self.assertTrue(wait(lambda: received and received[-1] == (0.15,0.45)))
            controls['front'] = 0.10
            self.assertTrue(wait(lambda: received[-1] == (0.0,0.0)))
            controls['front'],controls['back'],controls['linear'] = 3.0,0.1,-0.8
            time.sleep(0.2)
            self.assertEqual(received[-1],(0.0,0.0))
            controls['back'] = 3.0
            self.assertTrue(wait(lambda: received[-1] == (-0.15,0.45)))
            controls['heartbeat'] = False
            self.assertTrue(wait(lambda: received[-1] == (0.0,0.0)))
            controls['heartbeat'] = True
            self.assertTrue(wait(lambda: received[-1] == (-0.15,0.45)))
            controls['commands'] = False
            self.assertTrue(wait(lambda: received[-1] == (0.0,0.0)))
            controls['commands'],controls['stale'] = True,True
            time.sleep(0.4)
            self.assertEqual(received[-1],(0.0,0.0))
        finally:
            io.destroy_timer(timer)
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            gate.destroy_node()
            io.destroy_node()
            rclpy.shutdown()


class MappingIntegration(unittest.TestCase):
    def test_timeout_cancels_before_saving_partial_map(self):
        rclpy.init()
        root=tempfile.TemporaryDirectory(prefix='group4_partial_test_')
        robot=FakeRobot('mapping_timeout')
        mapper=Mapper(parameter_overrides=overrides(allow_motion=True,
            settle_seconds=.1,frontier_settle_seconds=.2,map_directory=root.name))
        executor=MultiThreadedExecutor(num_threads=5)
        for node in (robot,mapper): executor.add_node(node)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        try:
            self.assertTrue(wait(lambda:mapper.error() is None and mapper.nav.server_is_ready()))
            client=robot.create_client(Trigger,'/project/start')
            self.assertTrue(client.wait_for_service(timeout_sec=3))
            future=client.call_async(Trigger.Request());self.assertTrue(wait(future.done))
            self.assertTrue(future.result().success)
            self.assertTrue(wait(lambda:mapper.state=='RETURNING' and mapper.nav_handle is not None))
            mapper.goal_time=time.monotonic()-61
            self.assertTrue(wait(lambda:mapper.state in ('MAPPED_PARTIAL','FAILED'),15))
            self.assertEqual(mapper.state,'MAPPED_PARTIAL',mapper.reason)
            self.assertEqual(robot.canceled,1)
            self.assertEqual(robot.active_goals,0)
            self.assertIsNone(mapper.nav_handle)
            self.assertTrue(Path(mapper.saved_map).is_file())
            print('PASS: mapping timeout confirmed cancellation before separate partial-map save')
        finally:
            executor.shutdown(timeout_sec=3);thread.join(timeout=3)
            mapper.destroy_node();robot.destroy_node();rclpy.shutdown();root.cleanup()

    def test_frontier_exhaustion_returns_and_saves_a_separate_map(self):
        rclpy.init()
        root=tempfile.TemporaryDirectory(prefix='group4_mapping_test_')
        original=Path(root.name)/'lab_map.yaml'
        original.write_text('Original lab map must remain unchanged\n')
        robot=FakeRobot('absent')
        mapper=Mapper(parameter_overrides=overrides(
            allow_motion=True,settle_seconds=0.1,frontier_settle_seconds=0.2,
            map_directory=root.name))
        executor=MultiThreadedExecutor(num_threads=5)
        for node in (robot,mapper):
            executor.add_node(node)
        thread=threading.Thread(target=executor.spin,daemon=True)
        thread.start()
        try:
            self.assertTrue(wait(lambda:mapper.error() is None and mapper.nav.server_is_ready()))
            client=robot.create_client(Trigger,'/project/start')
            self.assertTrue(client.wait_for_service(timeout_sec=3))
            future=client.call_async(Trigger.Request())
            self.assertTrue(wait(future.done))
            self.assertTrue(future.result().success,future.result().message)
            self.assertTrue(wait(lambda:mapper.state in ('MAPPED','MAPPED_PARTIAL','FAILED'),15))
            self.assertEqual(mapper.state,'MAPPED',mapper.reason)
            self.assertEqual(robot.home_goals,1)
            self.assertTrue(Path(mapper.saved_map).is_file())
            self.assertTrue(Path(mapper.saved_map).with_suffix('.pgm').is_file())
            self.assertEqual(original.read_text(),'Original lab map must remain unchanged\n')
        finally:
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            mapper.destroy_node()
            robot.destroy_node()
            rclpy.shutdown()
            root.cleanup()


if __name__ == '__main__':
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6'):
        raise SystemExit('Use an isolated ROS_DOMAIN_ID, e.g. 88. Never the robot domain.')
    unittest.main(verbosity=2)
