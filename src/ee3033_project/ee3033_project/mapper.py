"""Autonomous frontier mapping, return to mapping start, and unique map save."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
import time
import numpy as np
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid,Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool,String
from std_srvs.srv import Trigger
from tf2_ros import Buffer,TransformListener,TransformException
from .core import quaternion_yaw,stamp_seconds
from .frontiers import frontier_goal
from .map_io import save_new_map
from .ros_utils import LATCHED,parameters,run,set_yaw
from .trial import guarded_parameters
from .gap import guard_clearance


class Mapper(Node):
    ACTIVE={'MAPPING','RETURNING','CANCELING','SAVING'}

    def __init__(self,**kwargs):
        super().__init__('project_mapper',**kwargs)
        self.p=parameters(self,{'allow_motion':False,'clearance':0.22,
                                'max_mapping_seconds':600.0,'goal_timeout':60.0,
                                'sensor_timeout':1.0,'settle_seconds':2.0,
                                'frontier_settle_seconds':4.0,
                                'map_directory':'~/ee3033_maps'})
        self.state,self.reason='WAITING','Ready for on-site autonomous mapping'
        guarded_parameters(self, ('goal_timeout',), lambda: self.state not in self.ACTIVE)
        self.map=None
        self.map_time=self.scan_time=self.odom_time=-math.inf
        self.home=None
        self.goal=None
        self.nav_handle=None
        self.pending=False
        self.started=self.goal_time=self.last_decision=self.empty_since=0.0
        self.finished_at=0.0
        self.cancel_deadline=0.0
        self.save_after_cancel=False
        self.resume_after_cancel=False
        self.failed=[]
        self.deferred=[]
        self.retry_round=0
        self.visited=[]
        self.goals=0
        self.partial=False
        self.saved_map=None
        guard_clearance(self, lambda: self.state not in self.ACTIVE and not self.pending and self.nav_handle is None)
        self.tf=Buffer()
        self.listener=TransformListener(self.tf,self)
        self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.worker=ThreadPoolExecutor(max_workers=1)
        self.job=None
        self.auto_pub=self.create_publisher(Bool,'/project/mapping_active',LATCHED)
        self.drive_pub=self.create_publisher(Bool,'/project/drive_enabled',LATCHED)
        self.status_pub=self.create_publisher(String,'/project/mission_status',LATCHED)
        self.home_pub=self.create_publisher(PoseStamped,'/project/home',LATCHED)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,LATCHED)
        self.create_subscription(LaserScan,'/scan_fixed',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_service(Trigger,'/project/start',self.start)
        self.create_service(Trigger,'/project/stop',self.stop)
        self.create_timer(0.1,self.tick)

    def on_map(self,msg):
        self.map,self.map_time=msg,time.monotonic()

    def on_scan(self,msg):
        age=self.get_clock().now().nanoseconds*1e-9-stamp_seconds(msg.header.stamp)
        if msg.ranges and 0<=age<self.p['sensor_timeout']:
            self.scan_time=time.monotonic()

    def on_odom(self,msg):
        age=self.get_clock().now().nanoseconds*1e-9-stamp_seconds(msg.header.stamp)
        if 0<=age<self.p['sensor_timeout']:
            self.odom_time=time.monotonic()

    def pose(self):
        try:
            t=self.tf.lookup_transform('map','base_footprint',Time())
            age=self.get_clock().now().nanoseconds*1e-9-stamp_seconds(t.header.stamp)
            if not -1.0<=age<1.0:
                return None
            return (t.transform.translation.x,t.transform.translation.y,
                    quaternion_yaw(t.transform.rotation))
        except TransformException:
            return None

    def error(self):
        now=time.monotonic()
        if self.map is None or now-self.map_time>8:
            return 'SLAM map missing or stale'
        if now-min(self.scan_time,self.odom_time)>self.p['sensor_timeout']:
            return 'Lidar or odometry missing/stale'
        if self.pose() is None:
            return 'SLAM map-to-base transform missing/stale'
        return None

    def start(self,request,response):
        del request
        error=self.error()
        if not self.p['allow_motion']:
            error='Motion disabled; launch with allow_motion:=true for robot mapping'
        if self.state in self.ACTIVE or self.pending or self.nav_handle:
            error='Mapping or cancellation is already active'
        if not self.nav.server_is_ready():
            error='Nav2 not ready'
        if error:
            response.message=error
            return response
        self.home,self.started,self.failed,self.partial=self.pose(),time.monotonic(),[],False
        self.finished_at=0.0
        self.save_after_cancel=False
        self.resume_after_cancel=False
        self.deferred=[]
        self.retry_round=0
        self.visited=[]
        self.goals=0
        self.job=None
        self.saved_map=None
        self.last_decision=time.monotonic()
        self.empty_since=0.0
        self.state,self.reason='MAPPING','Selecting reachable frontiers'
        self.home_pub.publish(self.make_pose(self.home))
        response.success,response.message=True,'Automatic mapping started'
        return response

    def stop(self,request,response):
        del request
        self.save_after_cancel=False
        self.resume_after_cancel=False
        self.drive_pub.publish(Bool(data=False))
        if self.nav_handle or self.pending:
            self.state='CANCELING'
            self.cancel_deadline=time.monotonic()+5
            if self.nav_handle:
                self.nav_handle.cancel_goal_async()
        else:
            self.state='STOPPED'
            self.finished_at=time.monotonic()
        self.reason='Mapping stopped by operator; current map remains available for saving'
        response.success,response.message=True,self.reason
        return response

    def make_pose(self,pose):
        msg=PoseStamped()
        msg.header.frame_id,msg.header.stamp='map',self.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y=map(float,pose[:2])
        set_yaw(msg.pose.orientation,pose[2])
        return msg

    def send(self,pose):
        self.goal=pose
        self.pending=True
        self.goal_time=time.monotonic()
        goal=NavigateToPose.Goal()
        goal.pose=self.make_pose(pose)
        self.nav.send_goal_async(goal).add_done_callback(self.accepted)

    def accepted(self,future):
        self.pending=False
        try:
            handle=future.result()
            if not handle.accepted:
                self.done(False)
                return
            self.nav_handle=handle
            handle.get_result_async().add_done_callback(self.result)
            if self.state not in ('MAPPING','RETURNING'):
                handle.cancel_goal_async()
        except Exception as exc:
            self.fail('Navigation request failed: '+str(exc))

    def result(self,future):
        self.nav_handle=None
        try:
            success=future.result().status==GoalStatus.STATUS_SUCCEEDED
        except Exception:
            success=False
        self.done(success)

    def done(self,success):
        if self.state=='CANCELING':
            if self.resume_after_cancel:
                self.resume_after_cancel=False
                self.state='MAPPING'
                self.finish_frontier(False)
            elif self.save_after_cancel:
                self.save_partial()
            else:
                self.state='STOPPED'
                self.finished_at=time.monotonic()
        elif self.state=='RETURNING':
            pose=self.pose()
            if not success or pose is None or math.hypot(pose[0]-self.home[0],pose[1]-self.home[1])>0.25:
                self.partial=True
            self.state,self.reason='SAVING','Saving a separate on-site map'
            self.job=self.worker.submit(save_new_map,self.p['map_directory'])
        elif self.state=='MAPPING':
            self.finish_frontier(success)

    def finish_frontier(self,success):
        if self.goal:
            if success:
                self.visited.append(self.goal[:2])
                self.reason='Frontier reached; selecting the next unexplored area'
            else:
                self.failed.append(self.goal[:2])
                self.deferred.append(self.goal[:2])
                self.reason=('Frontier retry failed; checking other remaining areas' if self.retry_round
                             else 'Frontier deferred after navigation failure; exploring other areas first')
        self.goal=None
        self.empty_since=0.0
        self.last_decision=time.monotonic()

    def save_partial(self):
        # Called only once navigation has finished/canceled. Never restart a node
        # or issue recovery motion; preserve a separate, honestly partial map.
        self.save_after_cancel=False
        self.resume_after_cancel=False
        self.partial=True
        self.drive_pub.publish(Bool(data=False))
        self.state,self.reason='SAVING','Navigation timed out; stopped and saving a partial map'
        self.job=self.worker.submit(save_new_map,self.p['map_directory'])

    def fail(self,reason):
        self.save_after_cancel=self.resume_after_cancel=False
        self.state,self.reason='FAILED',reason
        self.finished_at=time.monotonic()
        self.drive_pub.publish(Bool(data=False))
        if self.nav_handle:
            self.nav_handle.cancel_goal_async()
        self.get_logger().error(reason)

    def tick(self):
        self.p['goal_timeout']=self.get_parameter('goal_timeout').value
        self.p['clearance']=self.get_parameter('clearance').value
        now=time.monotonic()
        automatic=self.state in self.ACTIVE
        drive=self.state in ('MAPPING','RETURNING') and self.nav_handle is not None
        self.auto_pub.publish(Bool(data=automatic))
        self.drive_pub.publish(Bool(data=drive))
        if self.state=='CANCELING':
            if now>self.cancel_deadline:
                self.fail('Mapping goal cancellation timed out')
        elif self.state in ('MAPPING','RETURNING'):
            error=self.error()
            if error:
                self.fail(error)
            elif (self.nav_handle or self.pending) and now-self.goal_time>self.p['goal_timeout']:
                mapping_timeout=self.state=='MAPPING'
                self.stop(None,Trigger.Response())
                if mapping_timeout:
                    # A successor is allowed only after the old action's
                    # terminal result, including a late goal acceptance.
                    if self.state=='CANCELING':
                        self.resume_after_cancel=True
                        self.reason='Frontier timed out; canceling before exploring another area'
                    else:
                        self.state='MAPPING'
                        self.finish_frontier(False)
                elif self.state=='CANCELING':
                    self.partial=True
                    self.save_after_cancel=True
                    self.reason='Navigation timed out; canceling before saving a partial map'
                else:
                    self.save_partial()
            elif self.state=='MAPPING' and not self.nav_handle and not self.pending:
                if self.job is not None:
                    if self.job.done():
                        try:
                            goal,groups=self.job.result()
                            self.job=None
                            if goal is not None:
                                self.empty_since=0.0
                                self.goals+=1
                                self.send(goal)
                            else:
                                self.empty_since=self.empty_since or now
                                if now-self.empty_since>self.p['frontier_settle_seconds']:
                                    if (self.deferred and self.retry_round==0
                                            and now-self.started<self.p['max_mapping_seconds']):
                                        # Replan using the latest map. Release
                                        # failed-area exclusions once, retaining
                                        # successfully visited areas and the
                                        # full failure history for reporting.
                                        self.deferred=[]
                                        self.retry_round=1
                                        self.empty_since=0.0
                                        self.reason='Other reachable areas explored; retrying deferred frontiers once'
                                        self.get_logger().info(self.reason)
                                    else:
                                        self.partial=self.partial or groups>0 or bool(self.deferred)
                                        self.state,self.reason='RETURNING','Frontier search finished; returning to mapping start'
                                        self.send(self.home)
                        except Exception as exc:
                            self.fail(str(exc))
                elif now-self.started>self.p['max_mapping_seconds']:
                    self.partial=True
                    self.state,self.reason='RETURNING','Mapping time limit; returning before saving'
                    self.send(self.home)
                elif now-self.last_decision>self.p['settle_seconds']:
                    m=self.map
                    origin=m.info.origin
                    data=np.array(m.data).reshape(m.info.height,m.info.width)
                    self.job=self.worker.submit(frontier_goal,data,m.info.resolution,
                        (origin.position.x,origin.position.y,quaternion_yaw(origin.orientation)),
                        self.pose(),tuple(self.deferred+self.visited),self.p['clearance'])
                    self.last_decision=now
        elif self.state=='SAVING' and self.job and self.job.done():
            try:
                self.saved_map=self.job.result()
                self.job=None
                self.state='MAPPED_PARTIAL' if self.partial else 'MAPPED'
                self.finished_at=time.monotonic()
                self.reason='Saved new map: '+self.saved_map
                self.get_logger().info(self.reason)
            except Exception as exc:
                self.fail(str(exc))
        self.status_pub.publish(String(data=json.dumps({
            'phase':'mapping','stamp':self.get_clock().now().nanoseconds*1e-9,'state':self.state,'reason':self.reason,'home':self.home,'target':None,
            'ready':self.error() is None and self.nav.server_is_ready(),
            'readiness_reason':self.error() or ('Ready' if self.nav.server_is_ready() else 'Nav2 not ready'),
            'map_partial':self.partial,
            'saved_map':self.saved_map,'elapsed_seconds':(self.finished_at or now)-self.started if self.started else 0,
            'viewpoints_completed':len(self.visited),'viewpoints_total':0,
            'frontier_goals_sent':self.goals,'deferred_frontiers':len(self.deferred),
            'frontier_retry_round':self.retry_round,
            'failed_viewpoints':self.failed,'observed_fraction':0.0})))

    def destroy_node(self):
        if self.context.ok():
            self.drive_pub.publish(Bool(data=False))
            self.auto_pub.publish(Bool(data=False))
        if self.nav_handle and self.context.ok():
            self.nav_handle.cancel_goal_async()
        self.worker.shutdown(wait=False,cancel_futures=True)
        super().destroy_node()


def main():
    run(Mapper)
