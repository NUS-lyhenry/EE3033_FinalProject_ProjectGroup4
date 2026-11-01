"""Native planner proxy tests with raw-path and costmap fixtures, no motion."""
import os
import math
import threading
import time
import unittest
from pathlib import Path as FilePath

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid,Path
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rclpy.action import ActionClient,ActionServer,CancelResponse,GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from ee3033_project.path_planner import PathPlanner
from ee3033_project.path_validation import validate_path
from ee3033_project.ros_utils import LATCHED, set_yaw
from ee3033_project.core import quaternion_yaw
from ee3033_project.robot_geometry import GEOMETRY_PROFILE


def until(predicate,seconds=8):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.01)
    raise AssertionError('Planner fixture timed out')


def result(future,seconds=8):
    until(future.done,seconds);return future.result()


class Fixture(Node):
    def __init__(self):
        super().__init__('guarded_planner_fixture')
        self.group=ReentrantCallbackGroup()
        self.mode='normal';self.active=self.maximum=self.calls=self.cancels=0
        self.grid=np.zeros((60,60),dtype=int)
        self.resolution=.05;self.origin=(0.,0.,0.);self.native_costs=None
        self.grid[[0,-1],:]=100;self.grid[:,[0,-1]]=100
        self.maps=self.create_publisher(OccupancyGrid,'/map',LATCHED)
        self.paths=[]
        self.create_subscription(Path,'/plan',lambda msg:self.paths.append(msg),10)
        self.cost=self.create_service(GetCostmap,'/global_costmap/get_costmap',self.costmap,callback_group=self.group)
        self.server=ActionServer(self,ComputePathToPose,'/project/nav2_compute_path_to_pose',
            execute_callback=self.plan,goal_callback=self.accept,cancel_callback=self.cancel,callback_group=self.group)
        self.publish_map()
        self.create_timer(.01,lambda:self.publish_map() if self.mode=='heartbeat' else None,
                          callback_group=self.group)

    def publish_map(self):
        m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
        m.info.height,m.info.width=self.grid.shape;m.info.resolution=self.resolution;m.info.origin.orientation.w=1.
        m.info.origin.position.x,m.info.origin.position.y=self.origin[:2]
        m.data=self.grid.ravel().tolist();self.maps.publish(m)

    def costmap(self,request,response):
        m=response.map;m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
        if self.mode=='stale':m.header.stamp.sec-=5
        m.metadata.size_x=m.metadata.size_y=60;m.metadata.resolution=.05;m.metadata.origin.orientation.w=1.
        m.data=np.where(self.grid.ravel()<0,255,np.where(self.grid.ravel()>=25,253,0)).tolist()
        if self.native_costs is not None:
            costs,resolution,origin=self.native_costs
            m.metadata.size_y,m.metadata.size_x=costs.shape;m.metadata.resolution=resolution
            m.metadata.origin.position.x,m.metadata.origin.position.y=origin[:2]
            m.data=costs.ravel().tolist()
        if self.mode in ('fine_parallel','fine_barrier'):
            m.metadata.size_x=m.metadata.size_y=120;m.metadata.resolution=.025
            costs=np.zeros((120,120),dtype=int)
            if self.mode=='fine_parallel':costs[61,:]=253
            else:costs[:,60]=253
            m.data=costs.ravel().tolist()
        if self.mode=='changing':
            self.grid[:,30]=100;self.publish_map();time.sleep(.08)
        elif self.mode=='changing_far':
            self.grid[2,2]=100;self.publish_map();time.sleep(.08)
        elif self.mode=='heartbeat':
            time.sleep(.12)
        return response

    def accept(self,request):
        if self.mode=='late':time.sleep(.3)
        return GoalResponse.ACCEPT

    def cancel(self,handle):
        self.cancels+=1;return CancelResponse.ACCEPT

    async def plan(self,handle):
        self.last_planner_id=handle.request.planner_id
        self.calls+=1;self.active+=1;self.maximum=max(self.maximum,self.active)
        try:
            if self.mode in ('wait','late'):
                deadline=time.monotonic()+4
                while not handle.is_cancel_requested and time.monotonic()<deadline:time.sleep(.01)
                if handle.is_cancel_requested:handle.canceled()
                else:handle.abort()
                return ComputePathToPose.Result()
            time.sleep(.04)
            r=ComputePathToPose.Result();r.path.header.frame_id='map'
            for source in (handle.request.start,handle.request.goal):
                p=PoseStamped();p.header=r.path.header;p.pose=source.pose;r.path.poses.append(p)
            if self.mode=='bend':
                corner=PoseStamped();corner.header=r.path.header
                corner.pose.position.x=handle.request.goal.pose.position.x
                corner.pose.position.y=handle.request.start.pose.position.y-.5
                corner.pose.orientation.w=1.
                r.path.poses.insert(1,corner)
            handle.succeed();return r
        finally:self.active-=1

    def destroy_node(self):
        self.server.destroy();super().destroy_node()


class PlannerTests(unittest.TestCase):
    def setUp(self):
        rclpy.init();self.fixture=Fixture()
        self.node=PathPlanner(parameter_overrides=[Parameter('planning_timeout',value=4.)])
        self.executor=MultiThreadedExecutor(num_threads=8)
        for n in (self.fixture,self.node):self.executor.add_node(n)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        self.client=ActionClient(self.fixture,ComputePathToPose,'/compute_path_to_pose')
        self.assertTrue(self.client.wait_for_server(timeout_sec=3))
        until(lambda:self.node.map is not None and self.node.raw.server_is_ready() and self.node.cost.service_is_ready())

    def tearDown(self):
        for c in list(self.node.contexts.values()):c.stop.set();self.node.cancel_raw(c)
        self.executor.shutdown(timeout_sec=3);self.thread.join(timeout=3)
        self.node.destroy_node();self.fixture.destroy_node();rclpy.shutdown()

    def begin(self,start=(.5,1.5),goal=(2.5,1.5),planner_id='',yaw=0.):
        g=ComputePathToPose.Goal();g.use_start=True;g.planner_id=planner_id
        g.goal.header.stamp=self.fixture.get_clock().now().to_msg()
        g.start.header.frame_id=g.goal.header.frame_id='map'
        g.start.pose.position.x,g.start.pose.position.y=map(float,start)
        g.goal.pose.position.x,g.goal.pose.position.y=map(float,goal)
        g.start.pose.orientation.w=1.;set_yaw(g.goal.pose.orientation,yaw)
        h=result(self.client.send_goal_async(g));self.assertTrue(h.accepted);return h

    def test_mapping_straight_path_ignores_the_frontier_quaternion(self):
        h=self.begin(planner_id='MappingTangent',yaw=math.pi/2)
        r=result(h.get_result_async());self.assertEqual(r.status,GoalStatus.STATUS_SUCCEEDED)
        self.assertAlmostEqual(quaternion_yaw(r.result.path.poses[-1].pose.orientation),0.)
        self.assertEqual(self.fixture.last_planner_id,'GridBased')

    def test_mapping_bent_path_uses_the_last_segment_not_the_net_displacement(self):
        self.fixture.mode='bend';h=self.begin(planner_id='MappingTangent',yaw=0.)
        r=result(h.get_result_async());self.assertEqual(r.status,GoalStatus.STATUS_SUCCEEDED)
        self.assertAlmostEqual(quaternion_yaw(r.result.path.poses[-1].pose.orientation),math.pi/2)

    def test_nonmapping_goal_keeps_its_assigned_heading(self):
        h=self.begin(planner_id='GridBased',yaw=math.pi/2)
        r=result(h.get_result_async());self.assertEqual(r.status,GoalStatus.STATUS_SUCCEEDED)
        self.assertAlmostEqual(quaternion_yaw(r.result.path.poses[-1].pose.orientation),math.pi/2)

    def test_zero_length_first_mapping_path_cannot_invent_a_heading(self):
        h=self.begin(start=(1.,1.),goal=(1.,1.),planner_id='MappingTangent',yaw=math.pi/2)
        r=result(h.get_result_async());self.assertEqual(r.status,GoalStatus.STATUS_ABORTED)
        self.assertFalse(self.fixture.paths)

    def test_six_free_5cm_cells_remain_passable_with_preferred_margin(self):
        self.fixture.grid[:,30]=100;self.fixture.grid[27:33,30]=0;self.fixture.publish_map()
        until(lambda:self.node.map.data[15*60+30]==100)
        h=self.begin(start=(.5,1.5),goal=(2.5,1.5));r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(validate_path(self.fixture.grid,.05,(0.,0.,0.),
            [(p.pose.position.x,p.pose.position.y) for p in r.result.path.poses]).allowed)

    def test_safe_native_route_is_forwarded(self):
        h=self.begin();r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(validate_path(self.fixture.grid,.05,(0.,0.,0.),
            [(p.pose.position.x,p.pose.position.y) for p in r.result.path.poses]).allowed)

    def test_thin_wall_cut_is_replaced_by_complete_safe_route(self):
        self.fixture.grid[10:50,30]=100;self.fixture.publish_map()
        until(lambda:self.node.map.data[30*60+30]==100)
        h=self.begin();r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_SUCCEEDED)
        pts=[(p.pose.position.x,p.pose.position.y) for p in r.result.path.poses]
        self.assertTrue(validate_path(self.fixture.grid,.05,(0.,0.,0.),pts).allowed)
        self.assertGreater(len(pts),4)
        self.assertEqual(pts[-1],(2.5,1.5))

    def test_disconnected_map_aborts_without_publishing_unsafe_plan(self):
        self.fixture.grid[:,30]=-1;self.fixture.publish_map()
        until(lambda:self.node.map.data[30]==-1)
        h=self.begin();r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_ABORTED)
        self.assertFalse(self.fixture.paths)

    def test_stale_costmap_is_not_used(self):
        self.fixture.mode='stale'
        h=self.begin();r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_ABORTED)
        self.assertFalse(self.fixture.paths)

    def test_fine_native_costmap_does_not_expand_back_to_coarse_cells(self):
        self.fixture.mode='fine_parallel'
        h=self.begin();r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_SUCCEEDED)
        points=[(p.pose.position.x,p.pose.position.y) for p in r.result.path.poses]
        self.assertTrue(validate_path(self.fixture.grid,.05,(0.,0.,0.),points).allowed)
        self.assertTrue(all(p[1]==1.5 for p in points))

    def test_fine_native_costmap_barrier_still_rejects_route(self):
        self.fixture.mode='fine_barrier'
        h=self.begin();r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_ABORTED)
        self.assertFalse(self.fixture.paths)

    @unittest.skipUnless(GEOMETRY_PROFILE=='measured','Recorded measured robot')
    def test_recorded_route_passes_original_map_recheck_after_fine_search(self):
        with np.load(FilePath(__file__).parent/'fixtures/original_map_sampling_20261030.npz') as d:
            self.fixture.grid=d['grid'];self.fixture.resolution=float(d['resolution'])
            self.fixture.origin=d['origin']
            self.fixture.native_costs=(d['costs'],float(d['cost_resolution']),d['cost_origin'])
            self.fixture.publish_map()
            until(lambda:self.node.map.info.width==self.fixture.grid.shape[1])
            h=self.begin(d['start'],d['goal']);r=result(h.get_result_async())
            self.assertEqual(r.status,GoalStatus.STATUS_SUCCEEDED)
            points=[(p.pose.position.x,p.pose.position.y) for p in r.result.path.poses]
            self.assertTrue(validate_path(self.fixture.grid,self.fixture.resolution,self.fixture.origin,points).allowed)

    def test_map_generation_change_invalidates_previous_route(self):
        self.fixture.mode='changing'
        h=self.begin();r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_ABORTED)
        self.assertFalse(self.fixture.paths)

    def test_duplicate_map_heartbeats_do_not_invalidate_inflight_plan(self):
        revision=self.node.revision
        self.fixture.mode='heartbeat'
        h=self.begin();r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(self.node.revision,revision)

    def test_unrelated_map_change_revalidates_the_same_route(self):
        self.fixture.mode='changing_far'
        h=self.begin();r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_SUCCEEDED)
        self.assertGreater(self.node.revision,1)
        points=[(p.pose.position.x,p.pose.position.y) for p in r.result.path.poses]
        self.assertTrue(validate_path(self.fixture.grid,.05,(0.,0.,0.),points).allowed)

    def test_six_cost_probes_queue_without_starving_client_callbacks(self):
        handles=[self.begin() for _ in range(6)]
        answers=[result(h.get_result_async()) for h in handles]
        self.assertTrue(all(r.status==GoalStatus.STATUS_SUCCEEDED for r in answers))
        self.assertEqual(self.fixture.maximum,1)

    def test_cancel_and_late_acceptance_never_publish_route(self):
        self.fixture.mode='late';h=self.begin()
        until(lambda:any(c.future is not None for c in self.node.contexts.values()))
        result(h.cancel_goal_async());r=result(h.get_result_async())
        self.assertEqual(r.status,GoalStatus.STATUS_CANCELED)
        until(lambda:self.fixture.active==0 and self.fixture.calls==1)
        self.assertFalse(self.fixture.paths)
        self.assertFalse(self.node.blocked)


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6'):
        raise SystemExit('Run in an isolated ROS_DOMAIN_ID')
    unittest.main(verbosity=2)
