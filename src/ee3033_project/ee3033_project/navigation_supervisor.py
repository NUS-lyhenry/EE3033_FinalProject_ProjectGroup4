"""Single owner of navigation retries, cancellation and bounded recovery.

External clients retain NavigateToPose at /navigate_to_pose. Nav2's raw server
is remapped to /project/nav2_navigate_to_pose, so there is no competing recovery
loop. This node never writes motor commands or overrides the owner's heartbeat.
"""
import json
import math
import threading
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateToPose, BackUp
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException

from .core import quaternion_yaw, stamp_seconds
from .navigation_supervision import ProgressWatch, RecoveryBudget, wrap, path_reference
from .ros_utils import LATCHED, parameters
from .trial import guarded_parameters
from .stationary import StationaryWindow


class NavigationSupervisor(Node):
    def __init__(self, **kwargs):
        super().__init__('project_navigation_supervisor', **kwargs)
        self.p = parameters(self, dict(goal_timeout=60., maximum_recoveries=2,
            translation_window=12., translation_distance=.08, heading_window=3.,
            sensor_timeout=1., gate_timeout=1.5, cancel_timeout=3.,
            enable_grace=2., stationary_seconds=.5, stationary_timeout=2.,
            xy_tolerance=.12, command_topic='/cmd_vel', base_frame='base_footprint',
            odom_frame='odom_combined', map_frame='map'))
        self.group = ReentrantCallbackGroup()
        self.guard = threading.RLock()
        self.busy = False
        self.motion_dispatched = False
        self.blocked = False
        self.outer = None
        self.stop = threading.Event()
        self.raw_handle = None
        self.raw_future = None
        self.raw_result = None
        self.raw_cancel = None
        self.raw_canceled = False
        self.raw_abandoned = False
        self.raw_generation = 0
        self.raw_finish_deadline = None
        self.acceptance_event = threading.Event()
        self.drive = False
        self.drive_time = self.odom_time = self.command_time = self.gate_time = -math.inf
        self.odom = None
        self.odom_stamp = -math.inf
        self.odom_frame = None
        self.final_command = (math.inf, math.inf)
        self.gate = {}
        self.path = []
        self.path_received = -math.inf
        self.sequence = 0
        self.status = dict(stage='IDLE', reason='Ready', recovery_count=0,
                           goal_id=None, failure_reason=None)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, '/project/nav2_navigate_to_pose',
                                callback_group=self.group)
        self.backup = ActionClient(self, BackUp, '/backup', callback_group=self.group)
        self.clear_clients = [self.create_client(ClearEntireCostmap, path,
            callback_group=self.group) for path in (
                '/local_costmap/clear_entirely_local_costmap',
                '/global_costmap/clear_entirely_global_costmap')]
        self.status_pub = self.create_publisher(String, '/project/navigation_status', 10)
        self.create_subscription(Bool, '/project/drive_enabled', self.on_drive, LATCHED,
                                 callback_group=self.group)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(Twist, self.p['command_topic'], self.on_command, 10, callback_group=self.group)
        self.create_subscription(String, '/project/velocity_gate_status', self.on_gate, 10,
                                 callback_group=self.group)
        self.create_subscription(Path, '/plan', self.on_path, 10, callback_group=self.group)
        self.server = ActionServer(self, NavigateToPose, '/navigate_to_pose',
            execute_callback=self.execute, goal_callback=self.accept_goal,
            cancel_callback=self.cancel_goal, handle_accepted_callback=self.accepted,
            callback_group=self.group)
        self.create_timer(.2, self.publish_status, callback_group=self.group)

    def publish_status(self):
        with self.guard:
            value = dict(self.status)
        value['stamp'] = self.get_clock().now().nanoseconds*1e-9
        self.status_pub.publish(String(data=json.dumps(value)))

    def set_stage(self, stage, reason, failure=None):
        with self.guard:
            self.status.update(stage=stage, reason=reason)
            if failure is not None:
                self.status['failure_reason'] = failure
                self.status['last_failure_reason'] = failure
            elif stage=='SUCCEEDED':
                self.status['failure_reason'] = None
        self.publish_status()

    def on_drive(self, msg):
        with self.guard:
            self.drive, self.drive_time = msg.data, time.monotonic()
            # The owner sends False before it has accepted our outer handle.
            # Once armed, a False is an explicit, irreversible stop for this goal.
            if self.busy and self.status.get('armed') and not msg.data:
                self.stop.set()
        if self.stop.is_set():
            self.cancel_raw()

    def on_odom(self, msg):
        clock = self.get_clock().now().nanoseconds*1e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w,
                  msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if (msg.header.frame_id==self.p['odom_frame'] and msg.child_frame_id==self.p['base_frame']
                and 0 <= clock-stamp_seconds(msg.header.stamp) <= self.p['sensor_timeout']
                and all(math.isfinite(v) for v in values)
                and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<.05):
            with self.guard:
                self.odom = (p.x, p.y, quaternion_yaw(q),
                             msg.twist.twist.linear.x, msg.twist.twist.angular.z)
                self.odom_time = time.monotonic()
                self.odom_stamp = stamp_seconds(msg.header.stamp)
                self.odom_frame = (msg.header.frame_id,msg.child_frame_id)

    def on_command(self, msg):
        with self.guard:
            self.final_command = (msg.linear.x, msg.angular.z)
            self.command_time = time.monotonic()

    def on_gate(self, msg):
        try:
            data = json.loads(msg.data)
            with self.guard:
                self.gate, self.gate_time = data, time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_path(self, msg):
        if msg.header.frame_id != self.p['map_frame']:
            return
        with self.guard:
            self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.path_received = time.monotonic()

    def accept_goal(self, request):
        p, q = request.pose.pose.position, request.pose.pose.orientation
        if (request.pose.header.frame_id != self.p['map_frame']
                or not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w))
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.05):
            return GoalResponse.REJECT
        with self.guard:
            if self.busy or self.blocked or not self.nav.server_is_ready():
                return GoalResponse.REJECT
            self.busy = True
            self.motion_dispatched = False
            self.stop.clear()
            self.sequence += 1
            self.status = dict(stage='ACCEPTED', reason='Waiting for owner motion enable',
                recovery_count=0, goal_id=str(uuid.uuid4()), failure_reason=None,
                armed=False, sequence=self.sequence)
        return GoalResponse.ACCEPT

    def accepted(self, handle):
        with self.guard:
            self.outer = handle
            self.status['goal_id'] = bytes(handle.goal_id.uuid).hex()
            self.status['goal_started_stamp'] = self.get_clock().now().nanoseconds*1e-9
        handle.execute()

    def cancel_goal(self, handle):
        self.stop.set()
        self.cancel_raw()
        return CancelResponse.ACCEPT

    def interrupted(self, budget=None):
        if self.stop.is_set():
            return 'Navigation stopped by owner'
        if budget is not None and budget.expired(time.monotonic()):
            return 'Navigation absolute time limit exceeded'
        return None

    def cancel_raw(self):
        with self.guard:
            handle = self.raw_handle
            terminal_known = False
            if self.raw_result is not None and self.raw_result.done():
                try:
                    terminal_known = self.raw_result.result().status in (
                        GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_ABORTED)
                except Exception:
                    pass
            if handle is not None and handle.accepted and not self.raw_canceled and not terminal_known:
                self.raw_canceled = True
                try:
                    self.raw_cancel = handle.cancel_goal_async()
                except Exception as exc:
                    # An exception or cancel acknowledgement is not a terminal
                    # action result. finish_raw must still fail closed.
                    self.get_logger().error('Raw cancellation failed: '+str(exc))

    def wait_future(self, future, seconds, budget=None, ignore_stop=False):
        deadline = time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<deadline:
            if future.done():
                return True
            if not ignore_stop and self.interrupted(budget):
                return False
            time.sleep(.02)
        return future.done()

    def finish_raw(self):
        """Keep a cancellation tombstone until an explicit terminal result."""
        with self.guard:
            self.raw_abandoned = True
            if self.raw_finish_deadline is None:
                self.raw_finish_deadline = time.monotonic()+self.p['cancel_timeout']
            deadline = self.raw_finish_deadline
            future = self.raw_future
        self.cancel_raw()
        def failed():
            with self.guard:
                self.blocked = True
            return False
        if future is not None:
            if not self.wait_future(future,max(0.,deadline-time.monotonic()),ignore_stop=True):
                return failed()
            if not self.acceptance_event.wait(timeout=max(0.,deadline-time.monotonic())):
                return failed()
        with self.guard:
            handle,result = self.raw_handle,self.raw_result
        if future is not None and handle is None:
            return failed()
        if handle is not None and handle.accepted:
            # A rejected goal has no result future; an accepted one MUST have
            # a successful get-result response with a terminal GoalStatus.
            if result is None or not self.wait_future(
                    result,max(0.,deadline-time.monotonic()),ignore_stop=True):
                return failed()
            try:
                terminal = result.result()
                if terminal.status not in (GoalStatus.STATUS_SUCCEEDED,
                        GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_ABORTED):
                    return failed()
            except Exception:
                return failed()
        with self.guard:
            self.raw_handle = self.raw_future = self.raw_result = self.raw_cancel = None
            self.raw_canceled = False
            self.raw_finish_deadline = None
            # Do not clear raw_abandoned here: a late callback from this
            # generation must remain canceled even after a lockout/timeout.
        return True

    def pose(self):
        try:
            tr = self.tf.lookup_transform(self.p['map_frame'], self.p['base_frame'], Time())
            clock = self.get_clock().now().nanoseconds*1e-9
            if not -.5 <= clock-stamp_seconds(tr.header.stamp) <= .5:
                return None
            p, q = tr.transform.translation, tr.transform.rotation
            return p.x, p.y, quaternion_yaw(q)
        except TransformException:
            return None

    def stationary(self, budget=None, ignore_stop=False):
        self.set_stage('STOPPING', 'Verifying final command and actual stationary odometry pose')
        window = StationaryWindow(seconds=max(.5,self.p['stationary_seconds']),
            linear_limit=.035,angular_limit=.10,position_limit=.01,heading_limit=.02,max_gap=.25)
        start = time.monotonic()
        while rclpy.ok() and time.monotonic()-start < self.p['stationary_timeout']:
            if not ignore_stop and self.interrupted(budget):
                return False
            now = time.monotonic()
            with self.guard:
                odom,stamp,frame = self.odom,self.odom_stamp,self.odom_frame
                command = self.final_command
                fresh = now-self.odom_time<=self.p['sensor_timeout'] and now-self.command_time<=.3
            valid = (fresh and odom and frame
                     and all(math.isfinite(v) and abs(v)<=1e-5 for v in command))
            if valid:
                # Duplicate stamps cannot extend the hold. Position and yaw
                # spans independently reject falsely reported zero twist.
                window.add(stamp,odom[0],odom[1],odom[2],odom[3],odom[4],frame)
            else:
                window.reset()
            check = window.evaluate(self.get_clock().now().nanoseconds*1e-9)
            with self.guard:
                self.status['stationary_check'] = check
            if valid and check['verified']:
                return True
            time.sleep(.02)
        return False

    def clear(self, budget):
        self.set_stage('REFRESHING', 'Refreshing navigation costmaps before retry')
        for client in self.clear_clients:
            if not client.service_is_ready():
                return False
            future = client.call_async(ClearEntireCostmap.Request())
            if not self.wait_future(future, 2., budget):
                return False
            try:
                future.result()
            except Exception:
                return False
        # Permit fresh scans to refill the obstacle layer; do not drive blind.
        until = time.monotonic()+.3
        while time.monotonic()<until:
            if self.interrupted(budget):
                return False
            time.sleep(.02)
        return True

    def raw_accepted(self, future, generation=None):
        current = generation is None or generation==self.raw_generation
        try:
            handle = future.result()
            if not current:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            with self.guard:
                self.raw_handle = handle
                if handle.accepted:
                    self.raw_result = handle.get_result_async()
        except Exception as exc:
            self.get_logger().error('Raw action acceptance failed: '+str(exc))
        finally:
            if current:
                # This also runs when get_result_async raises, because an
                # accepted orphan must still be canceled and permanently lock.
                if self.stop.is_set() or self.raw_abandoned:
                    self.cancel_raw()
                self.acceptance_event.set()

    def dispatch(self, client, request, budget, feedback=None):
        with self.guard:
            if self.interrupted(budget) or self.blocked:
                return False
            if self.raw_future is not None or self.raw_handle is not None:
                self.blocked = True
                return False
            self.raw_handle = self.raw_result = None
            self.raw_canceled = False
            self.raw_abandoned = False
            self.raw_finish_deadline = None
            self.raw_generation += 1
            self.motion_dispatched = True
            generation = self.raw_generation
            self.acceptance_event.clear()
            self.raw_future = client.send_goal_async(request, feedback_callback=feedback)
            future = self.raw_future
            future.add_done_callback(lambda f:self.raw_accepted(f,generation))
        if not self.wait_future(future, 3., budget):
            return False
        # Completion callbacks may be scheduled after Future.done().
        deadline = time.monotonic()+1.
        while self.raw_handle is None and time.monotonic()<deadline:
            if self.interrupted(budget):
                return False
            time.sleep(.01)
        return self.raw_handle is not None and self.raw_handle.accepted

    def reached_goal(self, request):
        pose = self.pose()
        goal_pose = request.pose.pose
        observation = 'navigate_observation' in request.behavior_tree
        approach = 'navigate_approach' in request.behavior_tree
        home = 'navigate_home' in request.behavior_tree
        xy = .025 if approach else .10 if observation else self.p['xy_tolerance']
        yaw = .10 if approach else .03 if observation else .20
        return (pose is not None and math.hypot(
            pose[0]-goal_pose.position.x,pose[1]-goal_pose.position.y)<=xy
            and (home or abs(wrap(quaternion_yaw(goal_pose.orientation)-pose[2]))<=yaw))

    def navigation_health(self, now, accepted_at):
        """A fresh diagnostic heartbeat cannot make failed sensors healthy.

        Ordinary obstacle/unknown-space refusals remain motion decisions. Lost
        sensor, frame, map or localization evidence terminates the attempt and
        must never become a justification for a recovery motion.
        """
        with self.guard:
            odom = self.odom
            gate = dict(self.gate)
            fresh = (now-self.odom_time<=self.p['sensor_timeout']
                     and now-self.gate_time<=self.p['gate_timeout'])
            seen = odom is not None and self.gate_time > -math.inf
        reason = gate.get('reason')
        faults = {
            'scan_stale', 'scan_unusable', 'scan_transform_unavailable',
            'swept_map_missing', 'swept_transform_stale', 'swept_pose_unavailable',
            'reverse_map_or_localization_missing', 'reverse_localization_stale',
            'reverse_transform_stale', 'reverse_pose_unavailable',
            'Invalid reverse geometry', 'Invalid localization covariance',
            'Localization uncertainty too high for reverse',
            'invalid_map_geometry', 'invalid_motion_geometry', 'invalid_laser_geometry',
        }
        age = gate.get('scan_age')
        bad_scan_age = ('scan_age' in gate and
            (not isinstance(age,(int,float)) or not math.isfinite(age) or not 0<=age<=self.p['sensor_timeout']))
        if reason in faults or bad_scan_age:
            return 'Navigation sensors, localization or velocity gate are stale: '+str(reason or 'scan age invalid')
        if not fresh or self.pose() is None:
            if seen or now-accepted_at>self.p['enable_grace']:
                return 'Navigation sensors, localization or velocity gate are stale'
        return None

    def progress(self, watch, request, now, accepted_at):
        unhealthy = self.navigation_health(now, accepted_at)
        if unhealthy:
            return unhealthy
        with self.guard:
            odom = self.odom
            path, received, command = list(self.path), self.path_received, self.final_command
        pose = self.pose()
        if odom is None or pose is None:
            return None  # The bounded initial readiness grace is still running.
        goal = request.pose.pose
        distance = math.hypot(goal.position.x-pose[0], goal.position.y-pose[1])
        observation = 'navigate_observation' in request.behavior_tree
        approach = 'navigate_approach' in request.behavior_tree
        xy_tolerance = .025 if approach else .10 if observation else self.p['xy_tolerance']
        arrived = distance<=xy_tolerance
        # Explicit trees use the same tolerances as their controller checker.
        home = 'navigate_home' in request.behavior_tree
        yaw_tolerance = .10 if approach else .03 if observation else .20
        if arrived and home:
            heading = 0.
            reference = pose[2]
        elif arrived:
            reference = quaternion_yaw(goal.orientation)
            heading = wrap(reference-pose[2])
            if abs(heading)<=yaw_tolerance:
                heading = 0.
        elif path and received>=accepted_at and now-received<=3.:
            reference = path_reference(path,pose)
            if reference is None:
                reference = math.atan2(goal.position.y-pose[1],goal.position.x-pose[0])
            heading = wrap(reference-pose[2])
        else:
            reference = math.atan2(goal.position.y-pose[1],goal.position.x-pose[0])
            heading = wrap(reference-pose[2])
        aligning = abs(command[0])<.005 and abs(command[1])>.005 and abs(heading)>.05
        phase = 'FINAL_ALIGN' if arrived else 'ALIGNING' if aligning else 'TRACKING'
        with self.guard:
            self.status.update(stage=phase, distance=distance, heading_error=heading,
                velocity_gate_reason=self.gate.get('reason'), odom_age=now-self.odom_time,
                gate_age=now-self.gate_time)
        return watch.update(now, odom, heading, aligning=aligning, arrived=arrived,
                            reference=reference)

    def execute(self, outer):
        started = time.monotonic()
        budget = RecoveryBudget(started, self.get_parameter('goal_timeout').value, self.p['maximum_recoveries'])
        status, reason = GoalStatus.STATUS_ABORTED, 'Navigation failed'
        try:
            # The owner only enables driving after its action was accepted.
            while rclpy.ok() and not self.drive and time.monotonic()-started<self.p['enable_grace']:
                if self.interrupted(budget):
                    break
                time.sleep(.02)
            if not self.drive:
                reason = self.interrupted(budget) or 'Owner did not enable navigation'
                return self.terminal(outer, status, reason)
            with self.guard:
                self.status['armed'] = True
            while rclpy.ok():
                problem = self.interrupted(budget)
                if problem:
                    reason = problem
                    break
                self.set_stage('DISPATCHING', 'Following a collision-checked route')
                accepted_at = time.monotonic()
                forwarded = NavigateToPose.Goal()
                forwarded.pose = outer.request.pose
                forwarded.behavior_tree = outer.request.behavior_tree
                def feedback(msg):
                    if outer.is_active:
                        outer.publish_feedback(msg.feedback)
                reason = None
                accepted = self.dispatch(self.nav, forwarded, budget, feedback)
                if not accepted:
                    reason = self.interrupted(budget) or 'Nav2 rejected navigation goal'
                else:
                    watch = ProgressWatch(self.p['translation_window'], self.p['translation_distance'],
                                          self.p['heading_window'])
                    while rclpy.ok() and self.raw_result is not None and not self.raw_result.done():
                        reason = self.interrupted(budget)
                        if reason:
                            break
                        if time.monotonic()-self.drive_time>self.p['sensor_timeout']:
                            reason = 'Owner motion heartbeat is stale'
                            break
                        reason = self.progress(watch, outer.request, time.monotonic(), accepted_at)
                        if reason:
                            break
                        time.sleep(.05)
                    if not reason and self.raw_result is not None and self.raw_result.done():
                        result = self.raw_result.result()
                        if result.status==GoalStatus.STATUS_SUCCEEDED:
                            reached = self.reached_goal(outer.request)
                            if reached:
                                status = GoalStatus.STATUS_SUCCEEDED
                                reason = 'Navigation goal reached'
                                break
                            reason = 'Nav2 reported success outside verified goal tolerance'
                            self.set_stage('FAILED',reason,failure=reason)
                            break
                        reason = 'Nav2 could not follow the collision-free route'
                self.set_stage('CANCELING', 'Canceling Nav2 before recovery', failure=reason)
                if not self.finish_raw():
                    reason = 'Nav2 cancellation was not confirmed; navigation locked'
                    break
                if self.interrupted(budget):
                    reason = self.interrupted(budget)
                    break
                if (reason.startswith('Navigation sensors') or reason.startswith('Owner motion')
                        or reason.startswith('Nav2 reported success')):
                    break
                unhealthy = self.navigation_health(time.monotonic(), accepted_at)
                if unhealthy:
                    reason = unhealthy
                    break
                with self.guard:
                    had_path = self.path_received>=started and bool(self.path)
                if not self.stationary(budget):
                    reason = self.interrupted(budget) or 'Robot did not stop before recovery'
                    break
                unhealthy = self.navigation_health(time.monotonic(), accepted_at)
                if unhealthy:
                    reason = unhealthy
                    break
                recovery = budget.next(time.monotonic(), had_path)
                if recovery is None:
                    reason = ('No collision-free route; no reverse attempted' if not had_path
                              else 'Navigation recovery budget exhausted')
                    break
                with self.guard:
                    self.status['recovery_count'] = budget.count
                if recovery=='reverse':
                    if not self.backup.server_is_ready():
                        reason = 'Recovery backup server unavailable'
                        break
                    self.set_stage('REVERSING', 'Attempting map-checked bounded recovery reverse')
                    reverse = BackUp.Goal()
                    reverse.target.x, reverse.speed = .20, .04
                    reverse.time_allowance.sec = 8
                    if not self.dispatch(self.backup, reverse, budget):
                        reason = self.interrupted(budget) or 'Backup request failed'
                        break
                    reverse_started = time.monotonic()
                    refused = None
                    while (self.raw_result is not None and not self.raw_result.done()
                           and time.monotonic()-reverse_started<8.5 and not self.interrupted(budget)):
                        unhealthy = self.navigation_health(time.monotonic(), reverse_started)
                        if unhealthy:
                            refused = unhealthy
                            break
                        if time.monotonic()-self.drive_time>self.p['sensor_timeout']:
                            refused = 'Owner motion heartbeat is stale'
                            break
                        with self.guard:
                            refusal = self.gate.get('reason','')
                            fresh_gate = time.monotonic()-self.gate_time<=self.p['gate_timeout']
                        if fresh_gate and time.monotonic()-reverse_started>.6 and (
                                refusal.startswith('reverse_') or refusal in (
                                'Unknown space behind robot', 'Obstacle in reverse envelope',
                                'Localization uncertainty too high for reverse',
                                'Reverse envelope outside mapped area')):
                            refused = refusal
                            break
                        time.sleep(.05)
                    if refused or self.raw_result is None or not self.raw_result.done():
                        reason = self.interrupted(budget) or ('Safe reverse refused: '+refused if refused
                                  else 'Backup did not finish in time')
                        break
                    result = self.raw_result.result()
                    if result.status!=GoalStatus.STATUS_SUCCEEDED:
                        reason = 'Safe reverse unavailable or unsuccessful'
                        break
                    if not self.finish_raw() or not self.stationary(budget):
                        reason = 'Backup stop was not confirmed'
                        break
                if not self.clear(budget):
                    reason = self.interrupted(budget) or 'Could not refresh navigation costmaps'
                    break
            return self.terminal(outer, status, reason)
        except Exception as exc:
            self.get_logger().error('Navigation supervision failed: '+str(exc))
            return self.terminal(outer, GoalStatus.STATUS_ABORTED,
                                 'Navigation supervision error: '+str(exc))
        finally:
            # An unconfirmed action keeps the public server locked until process
            # restart. Clearing the lock without a terminal result is unsafe.
            confirmed = self.finish_raw()
            with self.guard:
                if not confirmed:
                    self.blocked = True
                self.busy = self.blocked
                if not self.blocked:
                    self.outer = None

    def terminal(self, outer, status, reason):
        # Do not notify clients while a raw action could still drive or while
        # physical slew is settling. Clients may immediately request a Spin or
        # another navigation action after receiving an outer terminal result.
        with self.guard:
            had_raw = (self.raw_future is not None or self.raw_handle is not None
                       or getattr(self,'motion_dispatched',False))
        confirmed = self.finish_raw()
        settled = confirmed and (not had_raw or self.stationary(ignore_stop=True))
        if not confirmed or not settled:
            with self.guard:
                self.blocked = self.busy = True
                self.status['locked'] = True
            status = GoalStatus.STATUS_ABORTED
            detail = 'Raw navigation terminal was not confirmed' if not confirmed else 'Actual stationary stop was not verified'
            reason = (reason or 'Navigation failed')+'; '+detail+'; navigation locked'
        elif status==GoalStatus.STATUS_SUCCEEDED and not self.reached_goal(outer.request):
            status = GoalStatus.STATUS_ABORTED
            reason = 'Final settled pose is outside verified goal tolerance'
        if self.stop.is_set() or outer.is_cancel_requested:
            if outer.is_cancel_requested:
                outer.canceled()
            else:
                outer.abort()
            self.set_stage('CANCELED', reason, failure=reason)
        elif status==GoalStatus.STATUS_SUCCEEDED:
            outer.succeed()
            self.set_stage('SUCCEEDED', reason)
        else:
            outer.abort()
            self.set_stage('FAILED', reason, failure=reason)
        return NavigateToPose.Result()

    def destroy_node(self):
        self.stop.set()
        self.cancel_raw()
        self.server.destroy()
        super().destroy_node()


def main():
    rclpy.init()
    node = NavigationSupervisor()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown(timeout_sec=3)
        if rclpy.ok():
            rclpy.shutdown()
