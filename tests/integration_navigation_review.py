"""Native ROS review boundaries beyond the ordinary supervisor scenarios."""
import os
import math
import threading
import time
import unittest

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from ee3033_project.navigation_supervisor import NavigationSupervisor
from integration_navigation_supervisor import Fixture,until,result


class CoastingFixture(Fixture):
    def __init__(self):
        self.last_tick=time.monotonic();self.coast_started=None;self.raw_success=None
        self.balance_started=None;self.balance_anchor=None
        super().__init__()

    def tick(self):
        now=time.monotonic();dt=min(.05,now-self.last_tick);self.last_tick=now
        if self.mode=='coasting':self.pose[2]+=.4*dt
        if self.coast_started is not None:
            elapsed=now-self.coast_started
            self.pose[2]+=max(0.,.4-.8*elapsed)*dt
        if self.mode=='balanced_success' and self.balance_started is not None:
            elapsed=now-self.balance_started
            self.pose=[self.balance_anchor[0]+.006*math.sin(2*math.pi*elapsed),
                self.balance_anchor[1],.02*math.sin(2*math.pi*elapsed)+.0025*elapsed]
            self.reported_linear=.006*2*math.pi*math.cos(2*math.pi*elapsed)
            self.reported_angular=.02*2*math.pi*math.cos(2*math.pi*elapsed)+.0025
        # Parent intentionally reports zero twist: changing actual pose is the
        # independent evidence which must prevent an early stationary verdict.
        super().tick()

    async def navigate(self,h):
        if self.mode=='balanced_success':
            self.nav_count+=1;self.active+=1;self.max_active=max(self.max_active,self.active)
            try:
                self.balance_anchor=[h.request.pose.pose.position.x,h.request.pose.pose.position.y]
                self.balance_started=time.monotonic()
                time.sleep(.15);self.raw_success=time.monotonic();h.succeed()
                return NavigateToPose.Result()
            finally:self.active-=1
        if self.mode!='success_coast':return await super().navigate(h)
        self.nav_count+=1;self.active+=1;self.max_active=max(self.max_active,self.active)
        try:
            self.pose=[h.request.pose.pose.position.x,h.request.pose.pose.position.y,0.]
            self.coast_started=time.monotonic()
            time.sleep(.05)
            self.raw_success=time.monotonic();h.succeed()
            return NavigateToPose.Result()
        finally:self.active-=1


class ReviewBoundaries(unittest.TestCase):
    def setUp(self):
        rclpy.init();self.fixture=CoastingFixture()
        self.supervisor=NavigationSupervisor(parameter_overrides=[Parameter(k,value=v) for k,v in dict(
            goal_timeout=6.,translation_window=.35,translation_distance=.08,heading_window=.2,
            stationary_seconds=.06,stationary_timeout=.8,sensor_timeout=.2,enable_grace=.5).items()])
        self.executor=MultiThreadedExecutor(num_threads=8)
        for n in (self.fixture,self.supervisor):self.executor.add_node(n)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        self.client=ActionClient(self.fixture,NavigateToPose,'/navigate_to_pose')
        self.assertTrue(self.client.wait_for_server(timeout_sec=3))
        until(lambda:self.supervisor.nav.server_is_ready() and self.supervisor.odom is not None)

    def tearDown(self):
        self.supervisor.stop.set();self.supervisor.cancel_raw()
        self.executor.shutdown(timeout_sec=3);self.thread.join(timeout=3)
        self.supervisor.destroy_node();self.fixture.destroy_node();rclpy.shutdown()

    def begin(self,mode):
        self.fixture.mode=mode;time.sleep(.08)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
        goal.pose.pose.position.x=.4;goal.pose.pose.orientation.w=1.
        handle=result(self.client.send_goal_async(goal));self.assertTrue(handle.accepted)
        return handle

    def test_acceptance_after_cancel_timeout_is_canceled_and_server_stays_locked(self):
        # The production task budget reads the committed live ROS parameter.
        self.supervisor.set_parameters([Parameter('goal_timeout',value=.20)])
        self.supervisor.p['cancel_timeout']=.04
        handle=self.begin('late');answer=result(handle.get_result_async())
        self.assertEqual(answer.status,GoalStatus.STATUS_ABORTED)
        until(lambda:self.fixture.nav_count==1 and self.fixture.active==0)
        self.assertGreaterEqual(self.fixture.cancels,1)
        self.assertTrue(self.supervisor.busy and self.supervisor.blocked)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.pose.orientation.w=1.
        self.assertFalse(result(self.client.send_goal_async(goal)).accepted)
        self.assertEqual(self.fixture.back_count,0)

    def test_yaw_only_motion_does_not_permanently_lock_navigation(self):
        handle=self.begin('coasting');answer=result(handle.get_result_async())
        self.assertEqual(answer.status,GoalStatus.STATUS_ABORTED)
        until(lambda:not self.supervisor.busy)
        self.assertFalse(self.supervisor.blocked)
        self.assertTrue(self.supervisor.status['stationary_check']['position_only'])

    def test_outer_success_waits_for_actual_pose_to_finish_coasting(self):
        self.supervisor.p['stationary_timeout']=2.
        handle=self.begin('success_coast');answer=result(handle.get_result_async())
        self.assertEqual(answer.status,GoalStatus.STATUS_SUCCEEDED,self.supervisor.status)
        self.assertGreaterEqual(time.monotonic()-self.fixture.coast_started,.5)
        self.assertGreaterEqual(self.supervisor.status['stationary_check']['duration'],.5)
        self.assertTrue(self.supervisor.status['stationary_check']['verified'])
        self.assertFalse(self.supervisor.blocked)

    def test_balance_motion_and_slow_yaw_allow_successive_goals_without_lock(self):
        self.supervisor.p.update(stationary_seconds=.5,stationary_timeout=2.)
        handle=self.begin('balanced_success');answer=result(handle.get_result_async())
        self.assertEqual(answer.status,GoalStatus.STATUS_SUCCEEDED,self.supervisor.status)
        check=self.supervisor.status['stationary_check']
        self.assertTrue(check['verified']);self.assertGreater(check['heading_span'],.02)
        self.assertEqual(check['limits']['heading_span'],.12)
        until(lambda:not self.supervisor.busy)
        second=self.begin('balanced_success');answer=result(second.get_result_async())
        self.assertEqual(answer.status,GoalStatus.STATUS_SUCCEEDED,self.supervisor.status)
        self.assertFalse(self.supervisor.blocked)
        self.assertEqual(self.fixture.nav_count,2)
        self.assertEqual(self.fixture.max_active,1)


if __name__=='__main__':
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','6')
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    unittest.main(verbosity=2)
