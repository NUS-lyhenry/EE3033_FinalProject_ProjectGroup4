"""Native ROS transport for per-exposure coverage, without motor/control actions."""
import json
import math
import os
import threading
import time
import unittest
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from ee3033_project.bottom_range import BottomCamera,CALIBRATION_K,CALIBRATION_D
from ee3033_project.coverage import CoveragePlanner
from ee3033_project.mission import Mission


def until(check,seconds=3.):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if check():return
        time.sleep(.02)
    raise AssertionError('Native exposure coverage fixture timed out')


class CoverageTransport(unittest.TestCase):
    def setUp(self):
        self.assertNotIn(os.environ.get('ROS_DOMAIN_ID'),(None,'0','6'))
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'),'1')
        rclpy.init()
        self.io=Node('exposure_coverage_fixture')
        self.m=Mission(parameter_overrides=[Parameter('target_class',value='bottle')])
        # Test only sensor transport/callbacks: no Mission control timer, Nav2
        # server, velocity gate, driver, or enabled motion publisher is present.
        for timer in list(self.m.timers):self.m.destroy_timer(timer)
        model=BottomCamera(640,480,CALIBRATION_K,CALIBRATION_D)
        self.m.planner=CoveragePlanner(np.zeros((60,60),int),.05,(0,0,0),camera_model=model)
        self.m.planner.prepare(1.2,1.2)
        self.m.state='SEARCHING';self.m.started_stamp=time.time()-.5
        self.m.epoch_started_stamp=self.m.started_stamp;self.m.current_option=0
        self.m.pose=lambda:(_ for _ in ()).throw(AssertionError('Current pose cannot replace exposure pose'))
        self.pub=self.io.create_publisher(String,'/project/perception_status',10)
        self.executor=MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.io);self.executor.add_node(self.m)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        until(lambda:self.pub.get_subscription_count()>0)
        self.model=model

    def tearDown(self):
        self.executor.shutdown(timeout_sec=3);self.thread.join(timeout=3)
        self.m.destroy_node();self.io.destroy_node();rclpy.shutdown()

    def data(self,stamp=None,pose=(1.23,1.27,0.)):
        stamp=time.time() if stamp is None else stamp
        return dict(stamp=stamp,ready=True,target_class='bottle',geometry_ready=True,
            observation_ready=False,detected_candidates=1,
            motion_check=dict(verified=False,reason='Normal balancing or translation'),
            camera_model=self.model.metadata(),
            capture_pose=dict(stamp=stamp,frame_id='map',x=pose[0],y=pose[1],yaw=pose[2]))

    def send(self,data):self.pub.publish(String(data=json.dumps(data)))

    def test_native_moving_frame_target_and_duplicate_evidence(self):
        data=self.data();self.send(data);until(lambda:bool(self.m.seen))
        pose=(1.23,1.27,0.)
        expected=self.m.planner.visible(*self.m.planner.world_to_cell(*pose[:2]),pose[2],
            camera_model=self.model,position=pose[:2])
        self.assertEqual(self.m.seen,expected)
        count=len(self.m.observation_events)
        self.send(data);time.sleep(.12)
        self.assertEqual(len(self.m.observation_events),count)
        self.assertEqual(self.m.station_attempts,0)
        self.assertEqual(self.m.observation_events[-1]['pose'],list(pose))

    def test_missing_capture_pose_and_old_epoch_do_not_fallback_or_refill(self):
        data=self.data();data['capture_pose']=None
        before=self.m.perception_time
        self.send(data);until(lambda:self.m.perception_time>before)
        self.assertFalse(self.m.seen)
        old=self.data();self.send(old);until(lambda:bool(self.m.seen))
        self.m.search_epoch=1;self.m.epoch_started_stamp=time.time();self.m.seen.clear()
        self.send(old);time.sleep(.12)
        self.assertFalse(self.m.seen)
        self.send(self.data(pose=(1.2,1.2,math.pi)));until(lambda:bool(self.m.seen))


if __name__=='__main__':unittest.main()
