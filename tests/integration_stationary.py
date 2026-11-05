"""Real mission stop verification with synthetic odometry/final output only.

Exercises VERIFYING after a successful return, including imperfect odometry.
No hardware, navigation goals or motor output publishers are started.
"""
import math
import os
import threading
import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from ee3033_project.mission import Mission
from ee3033_project.ros_utils import set_yaw
from integration_ros import FakeRobot,overrides,wait


class StopRobot(FakeRobot):
    def __init__(self, mode):
        self.mode,self.began,self.ticks=mode,None,0
        super().__init__('absent')
        self.commands_pub=self.create_publisher(Twist,'/test/stop_output',10)
        self.create_timer(.05,self.final_command)

    def final_command(self):
        if self.began is not None and self.mode=='stale_commands':return
        msg=Twist()
        if self.began is not None and self.mode=='nonzero_commands':msg.linear.x=.01
        self.commands_pub.publish(msg)

    def make_odometry(self, stamp):
        msg=super().make_odometry(stamp)
        if self.began is None:return msg
        t=time.monotonic()-self.began
        self.ticks+=1
        if self.mode=='noisy':
            msg.twist.twist.linear.x=.086 if self.ticks%5==0 else .008
            msg.twist.twist.angular.z=.232 if self.ticks%5==0 else .015
            msg.pose.pose.position.x=.005*math.sin(12*t)
            msg.pose.pose.position.y=.01*math.sin(8*t)
            set_yaw(msg.pose.pose.orientation,.03*math.sin(10*t))
        elif self.mode in ('linear','creeping'):
            v=.08 if self.mode=='linear' else .03
            msg.twist.twist.linear.x=v;msg.pose.pose.position.x=v*t
        elif self.mode in ('angular','slow_turn'):
            w=.2 if self.mode=='angular' else .08
            msg.twist.twist.angular.z=w;set_yaw(msg.pose.pose.orientation,w*t)
        elif self.mode=='odom_gaps' and self.ticks%8:return None
        elif self.mode=='invalid_pose':msg.pose.pose.position.x=math.nan
        return msg


class StopVerification(unittest.TestCase):
    def run_case(self,mode,expected):
        rclpy.init()
        robot=StopRobot(mode)
        mission=Mission(cli_args=['--ros-args','-r','/cmd_vel:=/test/stop_output'],
            parameter_overrides=overrides(target_class='bottle',verify_timeout=3.5))
        executor=MultiThreadedExecutor(num_threads=4)
        for node in (robot,mission):executor.add_node(node)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        try:
            self.assertTrue(wait(lambda:mission.readiness_error() is None))
            mission.home=(0.,0.,0.)
            mission.target=dict(x=.5,y=.5,frame='map',**{'class':'bottle'})
            mission.started=time.monotonic()
            robot.began=mission.started
            mission.goal_kind='home'
            mission.goal_finished(True)  # Nav2 return succeeded; verify independently.
            if mode=='operator_stop':
                time.sleep(.4)
                from std_srvs.srv import Trigger
                mission.stop(Trigger.Request(),Trigger.Response())
            self.assertTrue(wait(lambda:mission.state in ('COMPLETE','FAILED','STOPPED'),timeout=8),mission.summary())
            self.assertEqual(mission.state,expected,mission.summary())
            self.assertEqual(mission.return_verified,expected=='COMPLETE')
            if expected=='COMPLETE':
                check=mission.summary()['stationary_check']
                self.assertTrue(check['verified']);self.assertGreaterEqual(check['duration'],1.999)
                self.assertGreater(check['samples'],10)
            if mode!='operator_stop' and expected=='STOPPED':
                self.assertIn('verification',mission.reason)
                self.assertIsNotNone(mission.internal_error)
            print('PASS stop verification:',mode,mission.state,mission.stationary_check)
        finally:
            executor.shutdown(timeout_sec=3);thread.join(timeout=3)
            for node in (mission,robot):node.destroy_node()
            rclpy.shutdown()

    def test_balancing_spikes_with_bounded_pose_can_complete(self):self.run_case('noisy','COMPLETE')
    def test_sustained_translation_cannot_complete(self):self.run_case('linear','STOPPED')
    def test_sustained_yaw_cannot_complete(self):self.run_case('angular','STOPPED')
    def test_creeping_below_speed_limit_cannot_complete(self):self.run_case('creeping','STOPPED')
    def test_slow_turn_below_speed_limit_cannot_complete(self):self.run_case('slow_turn','STOPPED')
    def test_nonzero_final_commands_cannot_complete(self):self.run_case('nonzero_commands','STOPPED')
    def test_stale_zero_commands_cannot_complete(self):self.run_case('stale_commands','STOPPED')
    def test_odom_gaps_cannot_complete(self):self.run_case('odom_gaps','STOPPED')
    def test_invalid_odometry_pose_cannot_complete(self):self.run_case('invalid_pose','STOPPED')
    def test_operator_stop_during_verification_stays_stopped(self):self.run_case('operator_stop','STOPPED')


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6'):raise SystemExit('Use an isolated ROS domain, e.g.100')
    unittest.main(verbosity=2)
