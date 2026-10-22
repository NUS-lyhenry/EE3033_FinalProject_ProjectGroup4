"""Native waypoint HTTP -> mapper -> NavToPose/Spin -> pose verification.

Sensors/actions are fixtures on an isolated domain; no physical drivers.
"""
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request,urlopen

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import Spin
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger

from ee3033_project.console import Console
from ee3033_project.mapper import Mapper
from ee3033_project.ros_utils import LATCHED
from integration_ros import FakeRobot,overrides,wait


class WaypointRobot(FakeRobot):
    def __init__(self,scenario):
        self.fixture_map=None
        super().__init__(scenario)
        self.spin_goals=0
        self.nav_positions=[]
        self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'fixture_map',m),LATCHED)
        self.create_timer(.5,self.refresh_map)
        self.spin_server=ActionServer(self,Spin,'/spin',execute_callback=self.execute_spin,
            goal_callback=self.accept_spin,cancel_callback=self.cancel,callback_group=ReentrantCallbackGroup())

    def refresh_map(self):
        if self.fixture_map is not None:
            self.fixture_map.header.stamp=self.get_clock().now().to_msg()
            self.map_pub.publish(self.fixture_map)

    def accept_spin(self,request):
        if self.scenario=='stop_spin':time.sleep(.4)
        return self.accept(request)

    async def execute(self,handle):
        self.active_goals+=1;self.max_active_goals=max(self.max_active_goals,self.active_goals)
        p=handle.request.pose.pose.position
        self.nav_positions.append((p.x,p.y))
        # Mapping navigation deliberately ignores final yaw; Spin must honor it.
        self.xy_yaw[:2]=[p.x+(.4 if self.scenario=='false_position' else 0),p.y]
        deadline=time.monotonic()+.15
        while time.monotonic()<deadline:
            if handle.is_cancel_requested:
                handle.canceled();self.active_goals-=1
                from nav2_msgs.action import NavigateToPose
                return NavigateToPose.Result()
            time.sleep(.01)
        handle.succeed();self.active_goals-=1
        from nav2_msgs.action import NavigateToPose
        return NavigateToPose.Result()

    async def execute_spin(self,handle):
        self.spin_goals+=1;self.active_goals+=1
        self.max_active_goals=max(self.max_active_goals,self.active_goals)
        deadline=time.monotonic()+(.6 if self.scenario=='stop_spin' else .15)
        while time.monotonic()<deadline:
            if handle.is_cancel_requested:
                handle.canceled();self.active_goals-=1;return Spin.Result()
            time.sleep(.01)
        if self.scenario!='false_heading':self.xy_yaw[2]+=handle.request.target_yaw
        handle.succeed();self.active_goals-=1;return Spin.Result()

    def destroy_node(self):
        self.spin_server.destroy();super().destroy_node()


class WaypointTests(unittest.TestCase):
    def setUp(self):
        rclpy.init();self.root=tempfile.TemporaryDirectory(prefix='group4_waypoint_')
        scenario=getattr(self,'scenario','normal')
        self.robot=WaypointRobot(scenario)
        self.mapper=Mapper(parameter_overrides=overrides(allow_motion=True,settle_seconds=.01,
            frontier_settle_seconds=.1,waypoint_hold_seconds=.1,waypoint_verify_timeout=.6,
            goal_timeout=2.,map_directory=self.root.name+'/maps'))
        self.console=Console(parameter_overrides=overrides(port=0,phase='mapping',allow_motion=True,
            map_directory=self.root.name+'/maps',session_directory=self.root.name+'/records'))
        self.executor=MultiThreadedExecutor(num_threads=6)
        for node in (self.robot,self.mapper,self.console):self.executor.add_node(node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        self.base=f'http://127.0.0.1:{self.console.p["port"]}'
        self.assertTrue(wait(lambda:self.mapper.error() is None and self.mapper.nav.server_is_ready() and self.mapper.spin.server_is_ready()))
        self.assertTrue(wait(lambda:'Map' in self.console.freshness and 'Mission' in self.console.freshness))

    def tearDown(self):
        self.mapper.stop(None,Trigger.Response())
        self.executor.shutdown(timeout_sec=3);self.thread.join(timeout=3)
        for node in (self.console,self.mapper,self.robot):node.destroy_node()
        rclpy.shutdown();self.root.cleanup()

    def post(self,path,body=None):
        req=Request(self.base+'/api/'+path,json.dumps(body or {}).encode(),
            {'Content-Type':'application/json','X-Project-Token':self.console.token},method='POST')
        return json.load(urlopen(req,timeout=10))

    def test_position_facing_and_repeated_waypoint(self):
        self.assertTrue(self.post('map_waypoint',dict(x=.5,y=0.,yaw=1.2))['success'])
        self.assertTrue(wait(lambda:self.mapper.state in ('WAYPOINT_REACHED','FAILED')))
        self.assertEqual(self.mapper.state,'WAYPOINT_REACHED',self.mapper.reason)
        self.assertEqual(self.robot.spin_goals,1)
        self.assertAlmostEqual(self.robot.xy_yaw[2],1.2,places=5)
        self.assertEqual(self.robot.max_active_goals,1)
        self.assertTrue(wait(lambda:self.console.state['state']=='WAYPOINT_REACHED'))
        self.assertTrue(self.post('map_waypoint',dict(x=.5,y=0.,yaw=-1.))['success'])
        self.assertTrue(wait(lambda:self.mapper.state in ('WAYPOINT_REACHED','FAILED')))
        self.assertEqual(self.mapper.state,'WAYPOINT_REACHED',self.mapper.reason)
        self.assertAlmostEqual(self.robot.xy_yaw[2],-1.,places=5)
        self.assertEqual(self.robot.nav_positions,[(.5,0.),(.5,0.)])

    def test_invalid_waypoints_and_stage2_are_rejected(self):
        for goal in [dict(x=5.,y=0.,yaw=0.),dict(x=1.15,y=0.,yaw=0.),dict(x=.5,y=0.,yaw=math.nan)]:
            with self.assertRaises(HTTPError):self.post('map_waypoint',goal)
        self.console.p['phase']='mission'
        with self.assertRaises(HTTPError):self.post('map_waypoint',dict(x=.5,y=0.,yaw=1.))
        self.assertEqual(self.robot.nav_positions,[])

    def test_false_heading_success_is_not_verified(self):
        self.robot.scenario='false_heading'
        self.post('map_waypoint',dict(x=.5,y=0.,yaw=1.))
        self.assertTrue(wait(lambda:self.mapper.state=='FAILED'))
        self.assertIn('could not be verified',self.mapper.reason)

    def test_false_position_success_never_starts_final_turn(self):
        self.robot.scenario='false_position'
        self.post('map_waypoint',dict(x=.5,y=0.,yaw=1.))
        self.assertTrue(wait(lambda:self.mapper.state=='FAILED'))
        self.assertEqual(self.robot.spin_goals,0)

    def test_stop_during_late_spin_acceptance(self):
        self.robot.scenario='stop_spin'
        self.post('map_waypoint',dict(x=.5,y=0.,yaw=1.))
        self.assertTrue(wait(lambda:self.mapper.state=='TURNING' and self.mapper.pending))
        self.post('stop')
        self.assertTrue(wait(lambda:self.mapper.state=='STOPPED' and not self.mapper.pending and self.mapper.nav_handle is None))
        time.sleep(.2);self.assertEqual(self.mapper.state,'STOPPED')
        self.assertNotEqual(self.robot.xy_yaw[2],1.)

    def test_request_preceding_stop_cannot_start_late(self):
        self.console.write_trial_node('project_mapper',dict(waypoint_x=.5,waypoint_y=0.,waypoint_yaw=1.,
            waypoint_request_time=self.console.get_clock().now().nanoseconds*1e-9))
        self.post('stop')
        client=self.robot.create_client(Trigger,'/project/start_waypoint')
        self.assertTrue(client.wait_for_service(timeout_sec=2));future=client.call_async(Trigger.Request())
        self.assertTrue(wait(future.done));self.assertFalse(future.result().success)
        self.assertEqual(self.robot.nav_positions,[])


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6'):raise SystemExit('Use an isolated ROS_DOMAIN_ID, e.g. 98')
    unittest.main(verbosity=2)
