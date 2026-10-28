"""ComputePathToPose proxy that never returns an unchecked continuous route."""
from dataclasses import dataclass, field
import json
import hashlib
import math
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from .core import quaternion_yaw, stamp_seconds
from .path_validation import guarded_plan, costmap_mask
from .ros_utils import LATCHED, parameters, set_yaw


@dataclass
class PlanningRequest:
    publish_plan: bool = True
    created: float = field(default_factory=time.monotonic)
    stop: threading.Event = field(default_factory=threading.Event)
    accepted: threading.Event = field(default_factory=threading.Event)
    future: object = None
    raw: object = None
    result: object = None
    canceled: bool = False
    abandoned: bool = False
    finish_deadline: object = None


class PathPlanner(Node):
    def __init__(self, **kwargs):
        super().__init__('project_path_planner', **kwargs)
        self.p=parameters(self,dict(planning_timeout=5.,fallback_timeout=3.,
            costmap_timeout=2.,cancel_timeout=2.,map_frame='map',
            base_frame='base_footprint',maximum_queue=16))
        self.group=ReentrantCallbackGroup()
        self.guard=threading.RLock()
        self.serial=threading.Lock()
        self.contexts={}
        self.queue=[]
        self.active_key=None
        self.count=0
        self.blocked=False
        self.map=None
        self.revision=0
        self.map_signature=None
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.raw=ActionClient(self,ComputePathToPose,'/project/nav2_compute_path_to_pose',
                              callback_group=self.group)
        self.cost=self.create_client(GetCostmap,'/global_costmap/get_costmap',
                                      callback_group=self.group)
        self.publisher=self.create_publisher(Path,'/plan',10)
        self.status_pub=self.create_publisher(String,'/project/planning_status',10)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,LATCHED,
                                 callback_group=self.group)
        self.server=ActionServer(self,ComputePathToPose,'/compute_path_to_pose',
            execute_callback=self.execute,goal_callback=self.goal,
            cancel_callback=self.cancel,handle_accepted_callback=self.accepted,
            callback_group=self.group)
        # Candidate scoring must use identical safety checks without repainting
        # the route the robot is actually following.
        self.evaluation_server=ActionServer(self,ComputePathToPose,'/project/evaluate_path_to_pose',
            execute_callback=self.execute,goal_callback=self.goal,
            cancel_callback=self.cancel,
            handle_accepted_callback=lambda handle:self.accepted(handle,publish_plan=False),
            callback_group=self.group)
        self.create_timer(.02,self.start_next,callback_group=self.group)

    def on_map(self,msg):
        if (msg.header.frame_id==self.p['map_frame'] and msg.info.resolution>0
                and len(msg.data)==msg.info.width*msg.info.height):
            o=msg.info.origin
            geometry=(msg.info.width,msg.info.height,msg.info.resolution,
                o.position.x,o.position.y,o.position.z,
                o.orientation.x,o.orientation.y,o.orientation.z,o.orientation.w)
            signature=(geometry,hashlib.sha256(np.asarray(msg.data,dtype=np.int16).tobytes()).digest())
            with self.guard:
                self.map=msg
                # A timestamp heartbeat is not a new planning geometry epoch.
                if signature!=self.map_signature:
                    self.map_signature=signature
                    self.revision+=1

    def goal(self,request):
        pose=request.goal.pose
        q=pose.orientation
        if (request.goal.header.frame_id!=self.p['map_frame']
                or not all(math.isfinite(v) for v in (pose.position.x,pose.position.y,q.x,q.y,q.z,q.w))
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.05):
            self.get_logger().warning('Planning goal rejected: invalid map-frame pose or quaternion')
            return GoalResponse.REJECT
        with self.guard:
            if (self.blocked or self.map is None or self.count>=self.p['maximum_queue']
                    or not self.raw.server_is_ready() or not self.cost.service_is_ready()):
                self.get_logger().warning('Planning goal rejected: '+str(dict(
                    cancellation_locked=self.blocked,map_received=self.map is not None,
                    queue_size=self.count,raw_server_ready=self.raw.server_is_ready(),
                    costmap_service_ready=self.cost.service_is_ready())))
                return GoalResponse.REJECT
            self.count+=1
        return GoalResponse.ACCEPT

    def accepted(self,handle,publish_plan=True):
        with self.guard:
            self.contexts[bytes(handle.goal_id.uuid)]=PlanningRequest(publish_plan=publish_plan)
            self.queue.append(handle)

    def start_next(self):
        canceled=[]
        selected=None
        with self.guard:
            for h in list(self.queue):
                key=bytes(h.goal_id.uuid);context=self.contexts[key]
                if context.stop.is_set() or time.monotonic()-context.created>=self.p['planning_timeout']:
                    self.queue.remove(h);canceled.append(h)
            if self.active_key is None and self.queue:
                selected=self.queue.pop(0)
                self.active_key=bytes(selected.goal_id.uuid)
        for h in canceled:h.execute()
        if selected is not None:selected.execute()

    def cancel(self,handle):
        with self.guard:
            context=self.contexts.get(bytes(handle.goal_id.uuid))
            if context is None or not handle.is_active:
                return CancelResponse.REJECT
            context.stop.set()
        self.cancel_raw(context)
        return CancelResponse.ACCEPT

    def cancel_raw(self,context):
        with self.guard:
            terminal_known=False
            if context.result is not None and context.result.done():
                try:
                    terminal_known=context.result.result().status in (
                        GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_ABORTED)
                except Exception:
                    pass
            if context.raw is not None and context.raw.accepted and not context.canceled and not terminal_known:
                context.canceled=True
                try:
                    context.raw.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().error('Planner cancellation failed: '+str(exc))

    def raw_accepted(self,future,context):
        try:
            handle=future.result()
            with self.guard:
                context.raw=handle
                if handle.accepted:context.result=handle.get_result_async()
        except Exception as exc:
            self.get_logger().error('Planner acceptance failed: '+str(exc))
        finally:
            if context.stop.is_set() or context.abandoned:
                self.cancel_raw(context)
            context.accepted.set()

    def wait(self,future,deadline,context=None):
        while rclpy.ok() and time.monotonic()<deadline:
            if future.done():return True
            if context is not None and context.stop.is_set():return False
            time.sleep(.01)
        return future.done()

    def finish_raw(self,context):
        with self.guard:
            context.abandoned=True
            if context.finish_deadline is None:
                context.finish_deadline=time.monotonic()+self.p['cancel_timeout']
            deadline=context.finish_deadline
        self.cancel_raw(context)
        def failed():
            with self.guard:self.blocked=True
            return False
        if context.future is not None:
            if not self.wait(context.future,deadline) or not context.accepted.wait(
                    timeout=max(0.,deadline-time.monotonic())):
                return failed()
            if context.raw is None:return failed()
        if context.raw is not None and context.raw.accepted:
            if context.result is None or not self.wait(context.result,deadline):
                return failed()
            try:
                terminal=context.result.result()
                if terminal.status not in (GoalStatus.STATUS_SUCCEEDED,
                        GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_ABORTED):
                    return failed()
            except Exception:
                return failed()
        return True

    def status(self,handle,state,reason,**extra):
        self.status_pub.publish(String(data=json.dumps(dict(stage=state,reason=reason,
            goal_id=bytes(handle.goal_id.uuid).hex(),stamp=self.get_clock().now().nanoseconds*1e-9,**extra))))

    def terminal(self,handle,reason,path=None,replaced=False,elapsed=0.,
                 validated_revision=None,cost_stamp=None):
        key=bytes(handle.goal_id.uuid)
        with self.guard:context=self.contexts.get(key)
        if context is not None and not self.finish_raw(context):
            path=None
            reason='Previous planner cancellation was not confirmed; planning locked'
        # The cancel callback can set its irreversible token before rclpy has
        # transitioned the public handle to CANCELING. Let that callback return
        # before marking the terminal state; never publish a route in between.
        if context is not None and context.stop.is_set() and not handle.is_cancel_requested:
            until=time.monotonic()+self.p['cancel_timeout']
            while handle.is_active and not handle.is_cancel_requested and time.monotonic()<until:
                time.sleep(.005)
        result=ComputePathToPose.Result()
        with self.guard:
            canceled=handle.is_cancel_requested or (context is not None and context.stop.is_set())
            expired=context is not None and time.monotonic()>=context.created+self.p['planning_timeout']
            if path is not None and expired:
                path=None;reason='Planning absolute time limit exceeded before publishing route'
            if path is not None and self.blocked:
                path=None;reason='Planning is locked after an unconfirmed cancellation'
            if path is not None and validated_revision is not None and self.revision!=validated_revision:
                path=None;reason='Map changed after final route validation'
            if path is not None and cost_stamp is not None:
                clock=self.get_clock().now().nanoseconds*1e-9
                if not -.5<=clock-cost_stamp<=3.:
                    path=None;reason='Inflated costmap aged while validating route'
            if canceled:
                if handle.is_cancel_requested:handle.canceled()
                else:handle.abort()
                self.status(handle,'CANCELED',reason)
            elif path is None:
                handle.abort();self.status(handle,'FAILED',reason)
            else:
                result.path=path
                result.planning_time.sec=int(elapsed)
                result.planning_time.nanosec=int((elapsed-int(elapsed))*1e9)
                visible=context is None or getattr(context,'publish_plan',True)
                if visible:self.publisher.publish(path)
                handle.succeed();self.status(handle,'VALIDATED',reason,replaced=replaced,
                    purpose='navigation' if visible else 'candidate_evaluation')
        return result

    def execute(self,handle):
        key=bytes(handle.goal_id.uuid)
        with self.guard:context=self.contexts[key]
        started=context.created;deadline=started+self.p['planning_timeout']
        acquired=False
        try:
            while rclpy.ok() and not context.stop.is_set() and time.monotonic()<deadline:
                if self.serial.acquire(timeout=.03):acquired=True;break
            if not acquired:
                return self.terminal(handle,'Planning canceled or queue time limit exceeded')
            if self.blocked:
                return self.terminal(handle,'Previous planner cancellation was not confirmed')
            self.status(handle,'PLANNING','Requesting native path')
            forwarded=ComputePathToPose.Goal()
            forwarded.goal=handle.request.goal
            forwarded.start=handle.request.start
            forwarded.use_start=handle.request.use_start
            forwarded.planner_id=handle.request.planner_id
            context.future=self.raw.send_goal_async(forwarded)
            context.future.add_done_callback(lambda f:self.raw_accepted(f,context))
            if (not self.wait(context.future,deadline,context)
                    or not context.accepted.wait(timeout=max(0.,deadline-time.monotonic()))):
                return self.terminal(handle,'Native planner acceptance timed out or canceled')
            if context.raw is None or not context.raw.accepted:
                return self.terminal(handle,'Native planner rejected goal')
            if not self.wait(context.result,deadline,context):
                return self.terminal(handle,'Native planner timed out or canceled')
            raw=context.result.result()
            candidate=[]
            if (raw.status==GoalStatus.STATUS_SUCCEEDED
                    and raw.result.path.header.frame_id==self.p['map_frame']):
                candidate=[(p.pose.position.x,p.pose.position.y) for p in raw.result.path.poses]
            # Even a native no-path result can be repaired if its particle path
            # representation fails beside an otherwise connected footprint map.
            with self.guard:
                msg,revision=self.map,self.revision
            if msg is None:return self.terminal(handle,'Raw map is unavailable')
            if handle.request.use_start:
                source=handle.request.start
                if source.header.frame_id!=self.p['map_frame']:
                    return self.terminal(handle,'Explicit start must be in map frame')
                start=(source.pose.position.x,source.pose.position.y)
            else:
                tr=self.tf.lookup_transform(self.p['map_frame'],self.p['base_frame'],Time())
                clock=self.get_clock().now().nanoseconds*1e-9
                if not -.5<=clock-stamp_seconds(tr.header.stamp)<=.5:
                    return self.terminal(handle,'Robot localization transform is stale')
                start=(tr.transform.translation.x,tr.transform.translation.y)
            target=handle.request.goal.pose
            goal=(target.position.x,target.position.y)
            self.status(handle,'VALIDATING','Checking the complete footprint along route segments')
            cost_future=self.cost.call_async(GetCostmap.Request())
            if not self.wait(cost_future,min(deadline,time.monotonic()+self.p['costmap_timeout']),context):
                return self.terminal(handle,'Fresh inflated costmap is unavailable')
            cost=cost_future.result().map
            clock=self.get_clock().now().nanoseconds*1e-9
            if (cost.header.frame_id!=self.p['map_frame'] or cost.metadata.resolution<=0
                    or not -.5<=clock-stamp_seconds(cost.header.stamp)<=3.):
                return self.terminal(handle,'Inflated costmap is stale or invalid')
            grid=np.asarray(msg.data).reshape(msg.info.height,msg.info.width)
            origin=(msg.info.origin.position.x,msg.info.origin.position.y,
                    quaternion_yaw(msg.info.origin.orientation))
            co=cost.metadata.origin
            mask=costmap_mask(grid.shape,msg.info.resolution,origin,
                np.asarray(cost.data).reshape(cost.metadata.size_y,cost.metadata.size_x),
                cost.metadata.resolution,(co.position.x,co.position.y,quaternion_yaw(co.orientation)))
            checked=guarded_plan(grid,msg.info.resolution,origin,start,goal,candidate,mask=mask,
                deadline=min(deadline,time.monotonic()+self.p['fallback_timeout']),cancel=context.stop.is_set)
            if not checked.allowed:return self.terminal(handle,checked.reason)
            if context.stop.is_set():return self.terminal(handle,'Planning canceled')
            with self.guard:
                latest,latest_revision=self.map,self.revision
            validated_revision=revision
            if latest_revision!=revision:
                # A changing SLAM map cannot invalidate safety after validation.
                from .path_validation import validate_path
                lo=latest.info.origin
                latest_origin=(lo.position.x,lo.position.y,quaternion_yaw(lo.orientation))
                latest_grid=np.asarray(latest.data).reshape(latest.info.height,latest.info.width)
                latest_mask=costmap_mask(latest_grid.shape,latest.info.resolution,latest_origin,
                    np.asarray(cost.data).reshape(cost.metadata.size_y,cost.metadata.size_x),
                    cost.metadata.resolution,(co.position.x,co.position.y,quaternion_yaw(co.orientation)))
                if not validate_path(latest_grid,latest.info.resolution,latest_origin,
                                     checked.path,mask=latest_mask).allowed:
                    return self.terminal(handle,'Map changed and invalidated the route')
                validated_revision=latest_revision
            path=Path();path.header.frame_id=self.p['map_frame'];path.header.stamp=self.get_clock().now().to_msg()
            for i,xy in enumerate(checked.path):
                p=PoseStamped();p.header=path.header;p.pose.position.x,p.pose.position.y=xy
                if i+1<len(checked.path):
                    nxt=checked.path[i+1]
                    set_yaw(p.pose.orientation,math.atan2(nxt[1]-xy[1],nxt[0]-xy[0]))
                else:p.pose.orientation=target.orientation
                path.poses.append(p)
            return self.terminal(handle,checked.reason,path,checked.replaced,time.monotonic()-started,
                validated_revision=validated_revision,cost_stamp=stamp_seconds(cost.header.stamp))
        except (Exception,TransformException) as exc:
            self.get_logger().error('Path validation failed: '+str(exc))
            return self.terminal(handle,'Path validation error: '+str(exc))
        finally:
            confirmed=self.finish_raw(context)
            with self.guard:
                self.contexts.pop(key,None);self.count-=1
                if self.active_key==key:self.active_key=None
                if not confirmed:self.blocked=True
            if acquired:self.serial.release()

    def destroy_node(self):
        with self.guard:contexts=list(self.contexts.values())
        for context in contexts:
            context.stop.set();self.cancel_raw(context)
        self.server.destroy()
        self.evaluation_server.destroy()
        super().destroy_node()


def main():
    rclpy.init();node=PathPlanner()
    executor=MultiThreadedExecutor(num_threads=8);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node();executor.shutdown(timeout_sec=3)
        if rclpy.ok():rclpy.shutdown()
