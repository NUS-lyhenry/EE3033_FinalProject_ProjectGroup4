"""Native ROS mapper retry policy; synthetic frontiers/actions, no hardware.

The planner fixture represents two frontier areas disappearing from the map
after successful observation. Geometry itself is covered by test_frontiers.py.
"""
import os
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import rclpy
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger

from integration_ros import FakeRobot, overrides, wait
from ee3033_project.mapper import Mapper


AREAS={'A':(.6,0.,0.),'B':(0.,.6,0.)}


class RetryRobot(FakeRobot):
    def __init__(self):
        self.mode='abort'
        self.order=[]
        self.completed=set()
        self.events=[]
        self.release_cancel=threading.Event()
        super().__init__('absent')

    def accept(self,request):
        if self.mode=='late_timeout' and not self.order:
            time.sleep(.5)
        return super().accept(request)

    async def execute(self,handle):
        pose=handle.request.pose.pose
        name='A' if abs(pose.position.x-.6)<.01 else 'B' if abs(pose.position.y-.6)<.01 else 'home'
        self.active_goals+=1
        self.max_active_goals=max(self.max_active_goals,self.active_goals)
        self.order.append(name)
        first=name=='A' and self.order.count('A')==1
        failed=name=='A' and (first or self.mode=='always_abort')
        if failed and self.mode in ('timeout','late_timeout','stop_cancel','cancel_timeout','sensor_loss'):
            if self.mode=='sensor_loss':self.live=False
            deadline=time.monotonic()+8
            while not handle.is_cancel_requested and time.monotonic()<deadline:
                time.sleep(.01)
            if self.mode in ('stop_cancel','cancel_timeout'):
                self.release_cancel.wait(timeout=8)
            self.events.append(('terminal',name))
            handle.canceled()
        elif failed:
            self.events.append(('terminal',name))
            handle.abort()
        else:
            self.xy_yaw=[pose.position.x,pose.position.y,0.]
            time.sleep(.12)
            if handle.is_cancel_requested:
                self.events.append(('terminal',name));handle.canceled()
            else:
                self.completed.add(name)
                self.events.append(('terminal',name));handle.succeed()
        self.active_goals-=1
        return NavigateToPose.Result()


class MappingRetryIntegration(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.root=tempfile.TemporaryDirectory(prefix='group4_retry_')
        self.robot=RetryRobot()
        self.mapper=Mapper(parameter_overrides=overrides(allow_motion=True,
            settle_seconds=.05,frontier_settle_seconds=.1,map_directory=self.root.name))
        def frontiers(data,resolution,origin,robot,failed=(),clearance=.22):
            remaining=[(name,p) for name,p in AREAS.items() if name not in self.robot.completed]
            for _,pose in remaining:
                if all((pose[0]-x)**2+(pose[1]-y)**2>=.4**2 for x,y in failed):
                    return pose,len(remaining)
            return None,len(remaining)
        self.patch=patch('ee3033_project.mapper.frontier_goal',side_effect=frontiers)
        self.patch.start()
        self.executor=MultiThreadedExecutor(num_threads=5)
        for n in (self.mapper,self.robot):self.executor.add_node(n)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()

    def tearDown(self):
        self.robot.release_cancel.set()
        self.executor.shutdown(timeout_sec=3);self.thread.join(timeout=3)
        self.mapper.destroy_node();self.robot.destroy_node();rclpy.shutdown()
        self.patch.stop();self.root.cleanup()

    def start(self,mode):
        self.robot.mode=mode
        self.assertTrue(wait(lambda:self.mapper.error() is None and self.mapper.nav.server_is_ready()))
        self.call('start')

    def call(self,name):
        client=self.robot.create_client(Trigger,'/project/'+name)
        self.assertTrue(client.wait_for_service(timeout_sec=3))
        f=client.call_async(Trigger.Request());self.assertTrue(wait(f.done))
        self.assertTrue(f.result().success,f.result().message)

    def finish(self,expected='MAPPED'):
        self.assertTrue(wait(lambda:self.mapper.state in ('MAPPED','MAPPED_PARTIAL','FAILED'),15),self.mapper.reason)
        self.assertEqual(self.mapper.state,expected,self.mapper.reason)
        self.assertEqual(self.robot.order,['A','B','A','home'])
        self.assertEqual(self.robot.max_active_goals,1)
        self.assertEqual(self.mapper.retry_round,1)
        self.assertEqual(len(self.mapper.failed),2 if expected=='MAPPED_PARTIAL' else 1)

    def test_abort_explores_other_area_then_retries_once(self):
        self.start('abort');self.finish()

    def test_timeout_waits_for_cancellation_then_explores_and_retries(self):
        self.start('timeout')
        self.assertTrue(wait(lambda:self.mapper.nav_handle is not None))
        self.mapper.goal_time=time.monotonic()-61
        self.finish()
        self.assertEqual(self.robot.canceled,1)

    def test_late_acceptance_is_canceled_before_any_successor(self):
        self.start('late_timeout')
        self.assertTrue(wait(lambda:self.mapper.pending))
        self.mapper.goal_time=time.monotonic()-61
        self.finish()
        self.assertEqual(self.robot.canceled,1)

    def test_repeated_failure_is_bounded_and_preserves_partial_map(self):
        self.start('always_abort');self.finish('MAPPED_PARTIAL')

    def test_stop_during_cancel_prevents_resume_or_retry(self):
        self.start('stop_cancel')
        self.assertTrue(wait(lambda:self.mapper.nav_handle is not None))
        self.mapper.goal_time=time.monotonic()-61
        self.assertTrue(wait(lambda:self.mapper.resume_after_cancel and self.robot.canceled==1))
        self.call('stop');self.robot.release_cancel.set()
        self.assertTrue(wait(lambda:self.mapper.state=='STOPPED'))
        time.sleep(.4)
        self.assertEqual(self.robot.order,['A'])
        self.assertFalse(self.mapper.resume_after_cancel)

    def test_unconfirmed_cancel_never_starts_another_goal(self):
        self.start('cancel_timeout')
        self.assertTrue(wait(lambda:self.mapper.nav_handle is not None))
        self.mapper.goal_time=time.monotonic()-61
        self.assertTrue(wait(lambda:self.mapper.resume_after_cancel and self.robot.canceled==1))
        self.mapper.cancel_deadline=time.monotonic()-1
        self.assertTrue(wait(lambda:self.mapper.state=='FAILED'))
        self.robot.release_cancel.set();time.sleep(.4)
        self.assertEqual(self.robot.order,['A'])

    def test_sensor_loss_stops_without_exploring_another_area(self):
        self.start('sensor_loss')
        self.assertTrue(wait(lambda:self.mapper.state=='FAILED'))
        self.assertTrue(wait(lambda:self.robot.active_goals==0))
        self.assertEqual(self.robot.order,['A'])

    def test_mapping_time_limit_prevents_a_new_exploration_or_retry(self):
        self.start('timeout')
        self.assertTrue(wait(lambda:self.mapper.nav_handle is not None))
        self.mapper.started=time.monotonic()-601
        self.mapper.goal_time=time.monotonic()-61
        self.assertTrue(wait(lambda:self.mapper.state in ('MAPPED_PARTIAL','FAILED'),15))
        self.assertEqual(self.mapper.state,'MAPPED_PARTIAL',self.mapper.reason)
        self.assertEqual(self.robot.order,['A','home'])
        self.assertEqual(self.mapper.retry_round,0)
        self.assertEqual(self.robot.max_active_goals,1)


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6'):
        raise SystemExit('Use an isolated ROS_DOMAIN_ID; never the robot domain.')
    unittest.main(verbosity=2)
