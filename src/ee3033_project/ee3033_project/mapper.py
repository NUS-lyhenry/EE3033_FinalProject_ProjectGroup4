"""Autonomous frontier mapping, return to mapping start, and unique map save."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
import time
import numpy as np
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped,Twist
from nav2_msgs.action import NavigateToPose,Spin
from rcl_interfaces.msg import SetParametersResult
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
from .gap import DEFAULT_CLEARANCE, guard_clearance
from .waypoint import validate_waypoint,waypoint_goal
from .core import angle_difference
from .navigation_progress import NavigationProgress
from .navigation_costmap import NavigationCostmap
from .stationary import StationaryWindow
from .mapping_escape import mapping_escape_goal


class Mapper(Node):
    ACTIVE={'MAPPING','RETURNING','CANCELING','SAVING','TURNING','VERIFYING','VERIFYING_HOME'}

    def __init__(self,**kwargs):
        super().__init__('project_mapper',**kwargs)
        self.p=parameters(self,{'allow_motion':False,'clearance':DEFAULT_CLEARANCE,
                                'max_mapping_seconds':600.0,'goal_timeout':60.0,
                                'sensor_timeout':1.0,'settle_seconds':2.0,
                                'frontier_settle_seconds':4.0,
                                'waypoint_x':0.0,'waypoint_y':0.0,'waypoint_yaw':0.0,
                                'waypoint_request_time':0.0,
                                'waypoint_position_tolerance':0.20,'waypoint_heading_tolerance':0.15,
                                'waypoint_hold_seconds':0.5,'waypoint_verify_timeout':5.0,
                                'map_directory':'~/ee3033_maps'})
        self.state,self.reason='WAITING','Ready for on-site autonomous mapping'
        guarded_parameters(self, ('goal_timeout',), lambda: self.state not in self.ACTIVE)
        self.map=None
        self.map_time=self.scan_time=self.odom_time=-math.inf
        self.home=None
        self.goal=None
        self.nav_handle=None
        self.pending=False
        self.goal_serial=0
        self.odom_pose=None
        self.nav_progress=NavigationProgress()
        self.navigation_recoveries=[]
        self.escape_pending=False
        self.escape_active=False
        self.escape_attempts=0
        self.escape_target=None
        self.return_escape_pending=False
        self.return_verified=False
        self.return_reason=None
        self.started=self.goal_time=self.last_decision=self.empty_since=0.0
        self.finished_at=0.0
        self.cancel_deadline=0.0
        self.save_after_cancel=False
        self.resume_after_cancel=False
        self.failed=[]
        self.deferred=[]
        self.retry_round=0
        self.visited=[]
        self.frontier_views=[]
        self.goals=0
        self.partial=False
        self.saved_map=None
        self.mode='automatic'
        self.waypoint=None
        self.final_leg=False
        self.last_stop_stamp=0.
        self.verify_started=self.stationary_since=0.
        self.odom_twist=(math.inf,math.inf)
        self.stop_window=StationaryWindow()
        self.stationary_check=None
        self.command_zero=False
        self.command_time=-math.inf
        self.verify_after=0.
        self.add_on_set_parameters_callback(self.guard_waypoint)
        guard_clearance(self, lambda: self.state not in self.ACTIVE and not self.pending and self.nav_handle is None)
        self.tf=Buffer()
        self.listener=TransformListener(self.tf,self)
        self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.navigation_map=NavigationCostmap(self)
        self.spin=ActionClient(self,Spin,'/spin')
        self.worker=ThreadPoolExecutor(max_workers=1)
        self.job=None
        self.auto_pub=self.create_publisher(Bool,'/project/mapping_active',LATCHED)
        self.drive_pub=self.create_publisher(Bool,'/project/drive_enabled',LATCHED)
        self.status_pub=self.create_publisher(String,'/project/mission_status',LATCHED)
        self.home_pub=self.create_publisher(PoseStamped,'/project/home',LATCHED)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,LATCHED)
        self.create_subscription(LaserScan,'/scan_fixed',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(Twist,'/cmd_vel',self.on_command,10)
        self.create_service(Trigger,'/project/start',self.start)
        self.create_service(Trigger,'/project/start_waypoint',self.start_waypoint)
        self.create_service(Trigger,'/project/stop',self.stop)
        self.create_timer(0.1,self.tick)

    def on_map(self,msg):
        self.map,self.map_time=msg,time.monotonic()

    def on_scan(self,msg):
        age=self.get_clock().now().nanoseconds*1e-9-stamp_seconds(msg.header.stamp)
        if msg.ranges and 0<=age<self.p['sensor_timeout']:
            self.scan_time=time.monotonic()

    def on_odom(self,msg):
        stamp=stamp_seconds(msg.header.stamp)
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        if 0<=age<self.p['sensor_timeout']:
            self.odom_time=time.monotonic()
            self.odom_twist=(msg.twist.twist.linear.x,msg.twist.twist.angular.z)
            p=msg.pose.pose.position
            self.odom_pose=(p.x,p.y,quaternion_yaw(msg.pose.pose.orientation))
            if self.state=='VERIFYING_HOME':
                q=msg.pose.pose.orientation
                valid=(stamp>=self.verify_after and self.command_zero
                    and time.monotonic()-self.command_time<=.3
                    and all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w))
                    and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<.05
                    and bool(msg.header.frame_id) and bool(msg.child_frame_id))
                if valid:
                    v=msg.twist.twist.linear
                    self.stop_window.add(stamp,p.x,p.y,self.odom_pose[2],math.hypot(v.x,v.y),
                        msg.twist.twist.angular.z,(msg.header.frame_id,msg.child_frame_id))
                else:self.stop_window.reset()
        else:self.stop_window.reset()

    def on_command(self,msg):
        values=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        self.command_zero=all(math.isfinite(v) and abs(v)<=1e-5 for v in values)
        self.command_time=time.monotonic()
        if not self.command_zero:self.stop_window.reset()

    def guard_waypoint(self,changes):
        for p in changes:
            if p.name not in ('waypoint_x','waypoint_y','waypoint_yaw','waypoint_request_time'):continue
            if self.state in self.ACTIVE or self.pending or self.nav_handle:
                return SetParametersResult(successful=False,reason='Stop before selecting a mapping waypoint')
            if type(p.value) is not float or not math.isfinite(p.value):
                return SetParametersResult(successful=False,reason='Waypoint parameters must be finite numbers')
        return SetParametersResult(successful=True)

    def map_geometry(self):
        m=self.map;o=m.info.origin
        return (np.array(m.data).reshape(m.info.height,m.info.width),m.info.resolution,
                (o.position.x,o.position.y,quaternion_yaw(o.orientation)))

    def start_waypoint(self,request,response):
        if not self.spin.server_is_ready():
            response.message='Nav2 turning server is not ready';return response
        if self.get_parameter('waypoint_request_time').value<=self.last_stop_stamp:
            response.message='Waypoint request preceded Stop; select the goal again';return response
        if self.map is None:
            response.message='Wait for a live map';return response
        try:
            target=validate_waypoint(*self.map_geometry(),tuple(self.get_parameter(k).value for k in ('waypoint_x','waypoint_y','waypoint_yaw')))
        except ValueError as exc:
            response.message=str(exc);return response
        response=self.start(request,response)
        if response.success:
            self.mode,self.waypoint='waypoint',target
            self.reason='Mapping toward the selected waypoint'
            response.message='Waypoint mapping started; final position and facing will be verified'
        return response

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
        if self.job is not None and not self.job.done():
            error='Previous mapping planning or save is still stopping'
        if not self.nav.server_is_ready():
            error='Nav2 not ready'
        if error:
            response.message=error
            return response
        self.home,self.started,self.failed,self.partial=self.pose(),time.monotonic(),[],False
        self.mode,self.waypoint,self.final_leg='automatic',None,False
        self.finished_at=0.0
        self.save_after_cancel=False
        self.resume_after_cancel=False
        self.deferred=[]
        self.retry_round=0
        self.visited=[]
        self.frontier_views=[]
        self.goals=0
        self.return_verified=False
        self.return_reason=None
        self.navigation_recoveries=[]
        self.escape_pending=False
        self.escape_active=False
        self.escape_attempts=0
        self.escape_target=None
        self.return_escape_pending=False
        self.stop_window.reset()
        self.stationary_check=None
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
        self.last_stop_stamp=self.get_clock().now().nanoseconds*1e-9
        self.save_after_cancel=False
        self.resume_after_cancel=False
        self.escape_pending=False
        self.escape_target=None
        self.return_escape_pending=False
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
        if self.pending or self.nav_handle is not None:
            self.fail('Refused overlapping mapping navigation goals');return
        if self.navigation_map.allowed(*pose[:2]) is False:
            # Costmap can change after worker selection; do not command motion
            # toward a newly inflated obstacle or a stale snapshot.
            self.goal=pose
            self.done(False)
            return
        self.goal_serial+=1
        serial=self.goal_serial
        self.goal=pose
        self.nav_progress.reset()
        self.pending=True
        self.goal_time=time.monotonic()
        goal=NavigateToPose.Goal()
        goal.pose=self.make_pose(pose)
        self.nav.send_goal_async(goal).add_done_callback(lambda f:self.accepted(f,serial))

    def accepted(self,future,serial=None):
        if serial is not None and serial!=self.goal_serial:
            try:
                handle=future.result()
                if handle.accepted:handle.cancel_goal_async()
            except Exception:pass
            return
        self.pending=False
        try:
            handle=future.result()
            if not handle.accepted:
                self.done(False)
                return
            self.nav_handle=handle
            self.nav_progress.reset(time.monotonic())
            handle.get_result_async().add_done_callback(lambda f:self.result(f,serial))
            if self.state not in ('MAPPING','RETURNING','TURNING'):
                handle.cancel_goal_async()
        except Exception as exc:
            self.fail('Navigation request failed: '+str(exc))

    def result(self,future,serial=None):
        if serial is not None and serial!=self.goal_serial:return
        self.nav_handle=None
        try:
            success=future.result().status==GoalStatus.STATUS_SUCCEEDED
        except Exception:
            success=False
        self.done(success)

    def done(self,success):
        if self.state=='CANCELING':
            if self.return_escape_pending:
                self.return_escape_pending=False
                self.state='RETURNING'
                if not self.start_mapping_escape(self.home):
                    self.return_reason='Return stalled and no safe recovery position is available'
                    self.save_partial()
            elif self.resume_after_cancel:
                self.resume_after_cancel=False
                self.state='MAPPING'
                if self.escape_active:
                    self.escape_active=False
                    self.goal=None
                    self.last_decision=time.monotonic()
                    self.reason='Clearance recovery stalled; exploring other areas'
                else:
                    self.finish_frontier(False)
                if self.escape_pending:
                    self.escape_pending=False
                    target,self.escape_target=self.escape_target,None
                    self.start_mapping_escape(target)
            elif self.save_after_cancel:
                self.save_partial()
            else:
                self.state='STOPPED'
                self.finished_at=time.monotonic()
        elif self.state=='RETURNING':
            if self.escape_active:
                self.escape_active=False
                self.goal=None
                if success:
                    self.reason='Recovery position reached; retrying return to mapping start'
                    self.send(self.home)
                else:
                    self.return_reason='Return clearance recovery failed'
                    self.save_partial()
                return
            pose=self.pose()
            if not success or pose is None or math.hypot(pose[0]-self.home[0],pose[1]-self.home[1])>0.25:
                self.partial=True
                self.return_reason='Return navigation failed; home was not verified'
            else:
                self.state,self.reason='VERIFYING_HOME','Verifying mapping home and stationary hold'
                self.verify_started=time.monotonic();self.stationary_since=0.
                self.verify_after=self.get_clock().now().nanoseconds*1e-9
                self.stop_window.reset();self.stationary_check=None
                self.drive_pub.publish(Bool(data=False))
                return
            self.state,self.reason='SAVING','Saving a separate on-site map'
            self.job=self.worker.submit(save_new_map,self.p['map_directory'])
        elif self.state=='TURNING':
            if success:self.begin_verify()
            else:self.fail('Waypoint turn failed; mapping remains live and the robot is stopped')
        elif self.state=='MAPPING':
            if self.escape_active:
                self.escape_active=False
                self.goal=None
                self.last_decision=time.monotonic()
                self.reason=('Recovery position reached; resuming exploration' if success
                             else 'Clearance recovery failed; exploring other areas')
            elif success and self.mode=='waypoint' and self.final_leg:
                pose=self.pose()
                if pose is None or math.hypot(pose[0]-self.waypoint[0],pose[1]-self.waypoint[1])>self.p['waypoint_position_tolerance']:
                    self.fail('Navigation reported success outside the waypoint position tolerance')
                else:self.turn_to_waypoint()
            else:self.finish_frontier(success)

    def start_mapping_escape(self, stalled_goal=None):
        """At most six map-checked, collision-controlled repositioning attempts."""
        if self.mode!='automatic' or self.escape_attempts>=6 or self.map is None:
            return False
        pose=self.pose()
        if pose is None or not self.navigation_map.fresh():
            return False
        data,resolution,origin=self.map_geometry()
        mask=self.navigation_map.mask(data.shape,resolution,origin)
        goal=mapping_escape_goal(data,resolution,origin,pose,mask,self.p['clearance'],stalled_goal)
        if goal is None:
            self.reason='No known-free clearance recovery; exploring other areas'
            return False
        self.escape_attempts+=1
        self.escape_active=True
        self.reason='Navigation stalled; moving to a known-free recovery position'
        self.send(goal)
        return True

    def turn_to_waypoint(self):
        pose=self.pose()
        if pose is None:self.fail('Waypoint heading unavailable');return
        delta=angle_difference(self.waypoint[2],pose[2])
        if abs(delta)<=self.p['waypoint_heading_tolerance']:
            self.begin_verify();return
        if not self.spin.server_is_ready():self.fail('Waypoint turning server unavailable');return
        self.state,self.reason='TURNING','At waypoint; turning to the selected final facing'
        goal=Spin.Goal();goal.target_yaw=float(delta)
        goal.time_allowance.sec=max(1,int(self.p['goal_timeout']))
        self.pending=True;self.goal_time=time.monotonic()
        self.goal_serial+=1
        serial=self.goal_serial
        self.spin.send_goal_async(goal).add_done_callback(lambda f:self.accepted(f,serial))

    def begin_verify(self):
        self.state,self.reason='VERIFYING','Checking waypoint position, heading and stationary hold'
        self.verify_started=time.monotonic();self.stationary_since=0.
        self.drive_pub.publish(Bool(data=False))

    def finish_frontier(self,success):
        if self.goal:
            if success:
                self.visited.append(self.goal[:2])
                self.frontier_views.append(tuple(self.goal))
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
        self.state,self.reason='SAVING',(self.return_reason or 'Navigation timed out')+'; stopped and saving a partial map'
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
        self.navigation_map.poll(now)
        stalled=None
        if self.state in ('MAPPING','RETURNING') and self.nav_handle and self.goal and self.odom_pose:
            pose=self.pose()
            if pose:
                distance=math.dist(pose[:2],self.goal[:2])
                heading=0. if self.state=='RETURNING' else angle_difference(self.goal[2],pose[2])
                stalled=self.nav_progress.check(now,self.odom_pose,distance,heading,
                    xy_tolerance=.12,heading_required=self.state!='RETURNING')
        automatic=self.state in self.ACTIVE
        drive=self.state in ('MAPPING','RETURNING','TURNING') and self.nav_handle is not None
        self.auto_pub.publish(Bool(data=automatic))
        self.drive_pub.publish(Bool(data=drive))
        if self.state=='CANCELING':
            if now>self.cancel_deadline:
                self.fail('Mapping goal cancellation timed out')
        elif self.state in ('MAPPING','RETURNING','TURNING'):
            error=self.error()
            if error:
                self.fail(error)
            elif (self.nav_handle or self.pending) and (stalled or now-self.goal_time>self.p['goal_timeout']):
                self.navigation_recoveries.append(dict(reason=stalled or 'Navigation timed out',goal=self.goal))
                mapping_timeout=self.state=='MAPPING'
                escape_stalled=self.escape_active
                stalled_goal=self.goal
                turning_timeout=self.state=='TURNING'
                self.stop(None,Trigger.Response())
                if turning_timeout:
                    self.reason='Waypoint turn timed out; stopped without starting another goal'
                elif mapping_timeout:
                    # A successor is allowed only after the old action's
                    # terminal result, including a late goal acceptance.
                    if self.state=='CANCELING':
                        self.resume_after_cancel=True
                        self.escape_pending=bool(stalled and not escape_stalled and self.mode=='automatic')
                        self.escape_target=stalled_goal if self.escape_pending else None
                        self.reason='Frontier timed out; canceling before exploring another area'
                    else:
                        self.state='MAPPING'
                        self.finish_frontier(False)
                elif self.state=='CANCELING':
                    if stalled and not escape_stalled and self.escape_attempts<6:
                        self.return_escape_pending=True
                        self.reason='Return stalled; canceling before a safe recovery attempt'
                    else:
                        self.partial=True
                        self.save_after_cancel=True
                        self.reason='Navigation timed out; canceling before saving a partial map'
                else:
                    self.save_partial()
            elif self.state=='MAPPING' and not self.nav_handle and not self.pending:
                if self.job is not None:
                    if self.job.done():
                        try:
                            if self.mode=='waypoint':goal,groups,self.final_leg=self.job.result()
                            else:goal,groups=self.job.result()
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
                                        if self.mode=='waypoint':
                                            self.fail('No reachable mapped approach to the waypoint remains; choose another point or map manually')
                                        else:
                                            self.partial=self.partial or groups>0 or bool(self.deferred)
                                            self.state,self.reason='RETURNING','Frontier search finished; returning to mapping start'
                                            self.send(self.home)
                        except Exception as exc:
                            self.fail(str(exc))
                elif now-self.started>self.p['max_mapping_seconds']:
                    if self.mode=='waypoint':self.fail('Waypoint mapping time limit reached; stopped')
                    else:
                        self.partial=True
                        self.state,self.reason='RETURNING','Mapping time limit; returning before saving'
                        self.send(self.home)
                elif now-self.last_decision>self.p['settle_seconds']:
                    if self.mode=='waypoint':
                        self.job=self.worker.submit(waypoint_goal,*self.map_geometry(),self.pose(),self.waypoint,
                            tuple(self.deferred),tuple(self.visited),self.p['clearance'])
                    else:
                        data,resolution,origin=self.map_geometry()
                        if (self.navigation_map.client.service_is_ready()
                                and (self.navigation_map.map is None or now-self.navigation_map.stamp>3.)):
                            self.reason='Waiting for a fresh Nav2 costmap before selecting a frontier'
                        else:
                            mask=self.navigation_map.mask(data.shape,resolution,origin)
                            self.job=self.worker.submit(frontier_goal,data,resolution,origin,
                                self.pose(),tuple(self.deferred),self.p['clearance'],
                                safe_mask=mask,visited=tuple(self.frontier_views))
                    self.last_decision=now
        elif self.state=='VERIFYING_HOME':
            pose=self.pose()
            valid=(self.error() is None and pose is not None
                and math.dist(pose[:2],self.home[:2])<=.25 and self.command_zero
                and now-self.command_time<=.3)
            if not valid:self.stop_window.reset()
            self.stationary_check=self.stop_window.evaluate(self.get_clock().now().nanoseconds*1e-9)
            if valid and self.stationary_check['verified']:
                self.return_verified=True;self.return_reason='Home and stationary hold verified'
                self.state,self.reason='SAVING','Saving a separate on-site map'
                self.job=self.worker.submit(save_new_map,self.p['map_directory'])
            elif now-self.verify_started>12.:
                self.return_reason='Mapping home stationary verification failed'
                self.save_partial()
        elif self.state=='VERIFYING':
            error=self.error();pose=self.pose()
            valid=(not error and pose is not None
                   and math.hypot(pose[0]-self.waypoint[0],pose[1]-self.waypoint[1])<=self.p['waypoint_position_tolerance']
                   and abs(angle_difference(self.waypoint[2],pose[2]))<=self.p['waypoint_heading_tolerance']
                   and all(math.isfinite(v) for v in self.odom_twist)
                   and abs(self.odom_twist[0])<=.03 and abs(self.odom_twist[1])<=.08)
            self.stationary_since=(self.stationary_since or now) if valid else 0.
            if error:self.fail(error)
            elif valid and now-self.stationary_since>=self.p['waypoint_hold_seconds']:
                self.state,self.reason='WAYPOINT_REACHED','Waypoint position and final facing verified; mapping remains live'
                self.finished_at=now;self.visited.append(self.waypoint[:2]);self.goal=None
            elif now-self.verify_started>self.p['waypoint_verify_timeout']:
                self.fail('Waypoint position/heading or stationary hold could not be verified')
        elif self.state=='SAVING' and self.job and self.job.done():
            try:
                self.saved_map=self.job.result()
                self.job=None
                self.state='MAPPED_PARTIAL' if self.partial else 'MAPPED'
                self.finished_at=time.monotonic()
                self.reason='Saved new map: '+self.saved_map+'; '+(self.return_reason or 'Home not verified')
                self.get_logger().info(self.reason)
            except Exception as exc:
                self.fail(str(exc))
        self.status_pub.publish(String(data=json.dumps({
            'phase':'mapping','stamp':self.get_clock().now().nanoseconds*1e-9,'state':self.state,'reason':self.reason,'home':self.home,'target':None,
            'ready':self.error() is None and self.nav.server_is_ready(),
            'readiness_reason':self.error() or ('Ready' if self.nav.server_is_ready() else 'Nav2 not ready'),
            'map_partial':self.partial,
            'return_verified':self.return_verified,'return_reason':self.return_reason,
            'stationary_check':self.stationary_check,
            'navigation_recoveries':self.navigation_recoveries,
            'mapping_mode':self.mode,'waypoint':self.waypoint,
            'waypoint_verified':self.state=='WAYPOINT_REACHED',
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
