"""Native action/progress/cancellation supervisor tests; no robot or motor writer."""
import math
import os
import threading
import time
import unittest

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import BackUp, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from ee3033_project.navigation_supervisor import NavigationSupervisor
from ee3033_project.ros_utils import LATCHED, set_yaw


def until(predicate, seconds=8):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.02)
    raise AssertionError('Timed out waiting for supervisor fixture')


def result(future,seconds=8):
    until(future.done,seconds)
    return future.result()


class Fixture(Node):
    def __init__(self):
        super().__init__('navigation_supervisor_fixture')
        self.group=ReentrantCallbackGroup()
        self.mode='success'
        self.pose=[0.,0.,0.]
        self.enabled=True
        self.live=True
        self.nav_count=self.back_count=self.cancels=self.active=self.max_active=0
        self.clear_count=0
        self.drive=self.create_publisher(Bool,'/project/drive_enabled',LATCHED)
        self.odom=self.create_publisher(Odometry,'/odom',10)
        self.command=self.create_publisher(Twist,'/cmd_vel',10)
        self.gate=self.create_publisher(String,'/project/velocity_gate_status',10)
        self.path=self.create_publisher(Path,'/plan',10)
        self.tf=TransformBroadcaster(self)
        self.static=StaticTransformBroadcaster(self)
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom_combined'
        t.header.stamp=self.get_clock().now().to_msg();t.transform.rotation.w=1.
        self.static.sendTransform(t)
        self.raw=ActionServer(self,NavigateToPose,'/project/nav2_navigate_to_pose',
            execute_callback=self.navigate,goal_callback=self.accept,cancel_callback=self.cancel,callback_group=self.group)
        self.reverse=ActionServer(self,BackUp,'/backup',execute_callback=self.backup,
            cancel_callback=self.cancel,callback_group=self.group)
        self.clear_services=[self.create_service(ClearEntireCostmap,path,self.clear,callback_group=self.group)
                       for path in ('/local_costmap/clear_entirely_local_costmap',
                                    '/global_costmap/clear_entirely_global_costmap')]
        self.create_timer(.03,self.tick,callback_group=self.group)

    def tick(self):
        self.drive.publish(Bool(data=self.enabled))
        if not self.live:return
        stamp=self.get_clock().now().to_msg()
        t=TransformStamped();t.header.frame_id='odom_combined';t.child_frame_id='base_footprint'
        t.header.stamp=stamp;t.transform.translation.x,t.transform.translation.y=self.pose[:2]
        set_yaw(t.transform.rotation,self.pose[2]);self.tf.sendTransform(t)
        o=Odometry();o.header=t.header;o.child_frame_id='base_footprint'
        o.pose.pose.position.x,o.pose.pose.position.y=self.pose[:2]
        set_yaw(o.pose.pose.orientation,self.pose[2]);self.odom.publish(o)
        self.command.publish(Twist())
        self.gate.publish(String(data='{"reason":"passing","output":[0,0]}'))
        if self.mode in ('recover','frozen'):
            p=Path();p.header.frame_id='map';p.header.stamp=stamp
            for x in (0.,.2,.4):
                v=PoseStamped();v.header=p.header;v.pose.position.x=x;v.pose.orientation.w=1.
                p.poses.append(v)
            self.path.publish(p)

    def accept(self,request):
        if self.mode=='late':time.sleep(.4)
        return GoalResponse.ACCEPT

    def cancel(self,handle):
        self.cancels+=1
        return CancelResponse.ACCEPT

    def clear(self,request,response):
        self.clear_count+=1
        return response

    async def navigate(self,h):
        self.nav_count+=1
        number=self.nav_count
        self.active+=1;self.max_active=max(self.max_active,self.active)
        try:
            if self.mode=='no_path':
                h.abort();return NavigateToPose.Result()
            if self.mode=='success' or (self.mode=='recover' and number==3):
                self.pose=[h.request.pose.pose.position.x,h.request.pose.pose.position.y,0.]
                time.sleep(.15);h.succeed();return NavigateToPose.Result()
            if self.mode=='false_success':
                h.succeed();return NavigateToPose.Result()
            deadline=time.monotonic()+8
            while not h.is_cancel_requested and time.monotonic()<deadline:time.sleep(.01)
            if h.is_cancel_requested:h.canceled()
            else:h.abort()
            return NavigateToPose.Result()
        finally:self.active-=1

    async def backup(self,h):
        self.back_count+=1;self.active+=1;self.max_active=max(self.max_active,self.active)
        try:
            self.pose[0]-=.20
            time.sleep(.1)
            h.succeed();return BackUp.Result()
        finally:self.active-=1

    def destroy_node(self):
        self.raw.destroy();self.reverse.destroy()
        super().destroy_node()


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.fixture=Fixture()
        self.supervisor=NavigationSupervisor(parameter_overrides=[Parameter(k,value=v) for k,v in dict(
            goal_timeout=6.,translation_window=.35,translation_distance=.08,
            heading_window=.2,stationary_seconds=.06,stationary_timeout=.8,
            sensor_timeout=.2,enable_grace=.5).items()])
        self.executor=MultiThreadedExecutor(num_threads=8)
        for node in (self.fixture,self.supervisor):self.executor.add_node(node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        self.client=ActionClient(self.fixture,NavigateToPose,'/navigate_to_pose')
        self.assertTrue(self.client.wait_for_server(timeout_sec=3))
        until(lambda:self.supervisor.nav.server_is_ready() and self.supervisor.odom is not None)

    def tearDown(self):
        self.supervisor.stop.set();self.supervisor.cancel_raw()
        self.executor.shutdown(timeout_sec=3);self.thread.join(timeout=3)
        self.supervisor.destroy_node();self.fixture.destroy_node();rclpy.shutdown()

    def begin(self,mode):
        self.fixture.mode=mode
        time.sleep(.08)
        g=NavigateToPose.Goal();g.pose.header.frame_id='map'
        g.pose.pose.position.x=.4;g.pose.pose.orientation.w=1.
        h=result(self.client.send_goal_async(g));self.assertTrue(h.accepted)
        return h

    def test_success_is_proxied_with_verified_pose(self):
        h=self.begin('success');r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(self.fixture.nav_count,1)
        self.assertEqual(self.fixture.back_count,0)

    def test_false_success_outside_goal_tolerance_is_refused(self):
        h=self.begin('false_success');r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_ABORTED)
        self.assertIn('outside',self.supervisor.status['reason'])
        self.assertEqual(self.fixture.nav_count,1)

    def test_stall_retries_then_bounded_reverse_without_overlapping_actions(self):
        h=self.begin('recover');r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_SUCCEEDED,self.supervisor.status)
        self.assertEqual(self.fixture.nav_count,3)
        self.assertEqual(self.fixture.back_count,1)
        self.assertEqual(self.fixture.max_active,1)
        self.assertEqual(self.supervisor.status['recovery_count'],2)

    def test_unreachable_goal_never_triggers_blind_reverse(self):
        h=self.begin('no_path');r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_ABORTED)
        self.assertEqual(self.fixture.nav_count,2)
        self.assertEqual(self.fixture.back_count,0)
        self.assertEqual(self.fixture.max_active,1)

    def test_stop_cancels_current_goal_without_successor(self):
        h=self.begin('frozen');until(lambda:self.fixture.active==1)
        self.fixture.enabled=False
        r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_ABORTED)
        until(lambda:self.fixture.active==0)
        self.assertEqual(self.fixture.nav_count,1)
        self.assertEqual(self.fixture.back_count,0)

    def test_client_cancel_waits_for_underlying_cancel(self):
        h=self.begin('frozen');until(lambda:self.fixture.active==1)
        result(h.cancel_goal_async())
        r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_CANCELED)
        until(lambda:self.fixture.active==0)
        self.assertEqual(self.fixture.nav_count,1)

    def test_stop_during_late_raw_acceptance_cancels_without_new_goal(self):
        h=self.begin('late')
        until(lambda:self.supervisor.raw_future is not None)
        self.fixture.enabled=False
        r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_ABORTED)
        until(lambda:self.fixture.nav_count==1 and self.fixture.active==0)
        self.assertEqual(self.fixture.back_count,0)
        self.assertFalse(self.supervisor.blocked)

    def test_sensor_loss_cancels_without_attempting_recovery(self):
        h=self.begin('frozen');until(lambda:self.fixture.active==1)
        self.fixture.live=False
        r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_ABORTED)
        self.assertEqual(self.fixture.back_count,0)
        self.assertIn('stale',self.supervisor.status['reason'])


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6'):
        raise SystemExit('Run in an isolated ROS_DOMAIN_ID')
    unittest.main(verbosity=2)
