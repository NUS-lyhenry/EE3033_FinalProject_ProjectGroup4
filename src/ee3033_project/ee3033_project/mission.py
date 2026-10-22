"""Autonomous search -> report target -> return -> verify and stop.

No motor commands are generated here: Nav2 owns motion and a separate velocity
gate requires this node's fresh heartbeat. Canceled/late goals cannot reopen it.
"""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import tempfile
import time

import numpy as np
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from .core import angle_difference, quaternion_yaw, stamp_seconds
from .coverage import CoveragePlanner, returned_home
from .gap import DEFAULT_CLEARANCE, guard_clearance
from .ros_utils import LATCHED, parameters, run, set_yaw


class Mission(Node):
    ACTIVE = {'PLANNING', 'SEARCHING', 'OBSERVING', 'CANCELING', 'RETURNING', 'VERIFYING'}

    def __init__(self, **kwargs):
        super().__init__('project_mission', **kwargs)
        self.p = parameters(self, {
            'map_frame': 'map', 'base_frame': 'base_footprint', 'allow_motion': False,
            'target_class': 'cup', 'clearance': DEFAULT_CLEARANCE, 'viewpoint_spacing': 0.65,
            'camera_range': 2.0, 'camera_fov': 1.35, 'viewpoint_headings': 6,
            'max_candidates': 250, 'max_goals': 120, 'coverage_fraction': 0.98,
            'observe_seconds': 3.0, 'goal_timeout': 75.0, 'mission_timeout': 900.0,
            'cancel_timeout': 5.0, 'accept_timeout': 8.0, 'planning_timeout': 45.0,
            'return_tolerance': 0.20, 'return_heading_required': False,
            'verify_seconds': 2.0, 'max_stationary_speed': 0.035,
            'verify_timeout': 12.0,
            'max_xy_variance': 0.16, 'max_yaw_variance': 0.36,
            'sensor_timeout': 1.0, 'perception_timeout': 3.0, 'tf_timeout': 1.0,
            'max_target_variance': 0.04, 'max_target_age': 2.0,
            'max_map_cells': 1000000, 'results_directory': '~/ee3033_results',
        })
        self.state, self.reason = 'WAITING', 'Call /project/start after localization'
        self.home = self.target = self.map = self.planner = None
        self.return_verified = False
        self.target_details = None
        self.route, self.seen, self.failures = [], set(), []
        self.index = 0
        self.scan_time = self.odom_time = self.perception_time = -math.inf
        self.amcl = None
        self.speed = math.inf
        self.goal_handle = None
        self.goal_pending = False
        self.goal_serial = 0
        guard_clearance(self, lambda: self.state == 'WAITING' and not self.goal_pending and self.goal_handle is None)
        self.goal_kind = None
        self.goal_started = self.started = 0.0
        self.finished_at = 0.0
        self.cancel_next = None
        self.cancel_deadline = 0.0
        self.observe_started = self.verify_started = 0.0
        self.last_status = 0.0
        self.run_dir = None
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.plan_future = None
        self.tf = Buffer(cache_time=Duration(seconds=20))
        self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.enable_pub = self.create_publisher(Bool, '/project/drive_enabled', LATCHED)
        self.search_pub = self.create_publisher(Bool, '/project/search_active', LATCHED)
        self.status_pub = self.create_publisher(String, '/project/mission_status', LATCHED)
        self.result_pub = self.create_publisher(String, '/project/mission_result', LATCHED)
        self.markers_pub = self.create_publisher(MarkerArray, '/project/markers', LATCHED)
        self.route_pub = self.create_publisher(NavPath, '/project/search_path', LATCHED)
        self.home_pub = self.create_publisher(PoseStamped, '/project/home', LATCHED)
        self.target_pub = self.create_publisher(PoseStamped, '/project/reported_target', LATCHED)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_subscription(LaserScan, '/scan_fixed', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/project/perception_status', self.on_perception, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/project/confirmed_target',
                                 self.on_target, 10)
        self.create_subscription(String, '/project/target_details', self.on_details, 10)
        self.create_service(Trigger, '/project/start', self.start)
        self.create_service(Trigger, '/project/stop', self.stop)
        self.create_timer(0.1, self.tick)
        self.heartbeat(False, False)

    def pose(self):
        try:
            tr = self.tf.lookup_transform(self.p['map_frame'], self.p['base_frame'], Time())
            age = self.get_clock().now().nanoseconds*1e-9 - stamp_seconds(tr.header.stamp)
            if not -1.0 <= age <= self.p['tf_timeout']:
                return None
            t, q = tr.transform.translation, tr.transform.rotation
            values = (t.x, t.y, quaternion_yaw(q))
            return values if all(math.isfinite(v) for v in values) else None
        except TransformException:
            return None

    def on_map(self, msg):
        # The mission uses one frozen map; a changing map during Step 2 is rejected.
        if self.state not in self.ACTIVE:
            self.map = msg

    def on_amcl(self, msg):
        if msg.header.frame_id == self.p['map_frame']:
            self.amcl = msg

    def on_scan(self, msg):
        age = self.get_clock().now().nanoseconds*1e-9 - stamp_seconds(msg.header.stamp)
        if msg.ranges and 0 <= age < self.p['sensor_timeout']:
            self.scan_time = time.monotonic()

    def on_odom(self, msg):
        age = self.get_clock().now().nanoseconds*1e-9 - stamp_seconds(msg.header.stamp)
        if 0 <= age < self.p['sensor_timeout']:
            self.odom_time = time.monotonic()
            self.speed = max(abs(msg.twist.twist.linear.x), abs(msg.twist.twist.angular.z))

    def on_perception(self, msg):
        try:
            data = json.loads(msg.data)
            age = self.get_clock().now().nanoseconds*1e-9 - float(data['stamp'])
            if (data['ready'] and data['target_class'] == self.p['target_class']
                    and 0 <= age < self.p['perception_timeout']):
                self.perception_time = time.monotonic()
        except (ValueError, TypeError, KeyError):
            pass

    def readiness_error(self, require_perception=True):
        now = time.monotonic()
        if self.map is None:
            return 'No saved map received'
        if now-self.scan_time > self.p['sensor_timeout']:
            return 'Lidar stream missing or stale'
        if now-self.odom_time > self.p['sensor_timeout']:
            return 'Odometry missing or stale'
        if require_perception and now-self.perception_time > self.p['perception_timeout']:
            return 'YOLO stream missing, stale, or target class mismatched'
        if self.amcl is None:
            return 'No AMCL pose; set the assigned initial pose before starting'
        cov = self.amcl.pose.covariance
        if (not all(math.isfinite(v) for v in cov)
                or min(cov[0], cov[7], cov[35]) < 0
                or max(cov[0], cov[7]) > self.p['max_xy_variance']
                or cov[35] > self.p['max_yaw_variance']):
            return 'Localization uncertainty is too high'
        if self.pose() is None:
            return 'Map-to-base TF missing or stale'
        return None

    def heartbeat(self, drive, search):
        self.enable_pub.publish(Bool(data=drive))
        self.search_pub.publish(Bool(data=search))

    def set_state(self, state, reason):
        self.state, self.reason = state, reason
        self.get_logger().info(state + ': ' + reason)
        self.publish_status()

    def start(self, request, response):
        del request
        if self.state in self.ACTIVE or self.goal_pending or self.goal_handle is not None:
            response.message = 'A mission or navigation cancellation is still active'
            return response
        error = self.readiness_error()
        if not self.p['allow_motion']:
            error = 'Motion disabled; launch with allow_motion:=true after bench tests'
        if not self.nav.server_is_ready():
            error = 'Nav2 NavigateToPose action server is not ready'
        if error:
            response.message = error
            return response
        self.p['clearance'] = self.get_parameter('clearance').value
        self.home, self.target = self.pose(), None
        self.target_details = None
        self.return_verified = False
        self.index, self.route, self.seen, self.failures = 0, [], set(), []
        self.started = time.monotonic()
        self.finished_at = 0.0
        self.verify_started = 0.0
        self.goal_serial += 1
        try:
            root = Path(self.p['results_directory']).expanduser()
            root.mkdir(parents=True, exist_ok=True)
            self.run_dir = Path(tempfile.mkdtemp(prefix='mission_', dir=str(root)))
            msg = self.map
            if msg.info.width*msg.info.height > self.p['max_map_cells']:
                raise ValueError('Map exceeds configured cell limit')
            data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
            origin = msg.info.origin
            self.planner = CoveragePlanner(
                data, msg.info.resolution,
                (origin.position.x, origin.position.y, quaternion_yaw(origin.orientation)),
                self.p['clearance'], self.p['viewpoint_spacing'], self.p['camera_range'],
                self.p['camera_fov'], self.p['viewpoint_headings'],
                self.p['max_candidates'], self.p['max_goals'], self.p['coverage_fraction'])
            self.plan_future = self.worker.submit(self.planner.plan, *self.home[:2])
            self.home_pub.publish(self.make_pose(*self.home))
            self.set_state('PLANNING', 'Generating camera viewpoints from the saved map')
            self.persist()
            response.success, response.message = True, 'Mission started; home pose recorded'
        except (ValueError, OSError) as exc:
            self.finish('FAILED', str(exc))
            response.message = str(exc)
        return response

    def stop(self, request, response):
        del request
        self.cancel('STOPPED', 'Stop requested')
        response.success = True
        response.message = 'Motion disabled; outstanding navigation is being canceled'
        return response

    def make_pose(self, x, y, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = self.p['map_frame']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        set_yaw(pose.pose.orientation, yaw)
        return pose

    def send_goal(self, pose, kind):
        if self.goal_pending or self.goal_handle is not None:
            self.finish('FAILED', 'Refused overlapping navigation goals')
            return
        if not self.nav.server_is_ready():
            self.finish('FAILED', 'Nav2 action server disappeared')
            return
        self.goal_serial += 1
        serial = self.goal_serial
        self.goal_pending = True
        self.goal_kind = kind
        self.goal_started = time.monotonic()
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.set_state('RETURNING' if kind == 'home' else 'SEARCHING',
                       'Returning to recorded start' if kind == 'home'
                       else f'Viewpoint {self.index+1}/{len(self.route)}')
        # Keep drive disabled until Nav2 explicitly accepts this goal.
        self.heartbeat(False, kind == 'search')
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(lambda f: self.goal_response(f, serial))

    def goal_response(self, future, serial):
        try:
            handle = future.result()
        except Exception as exc:
            self.goal_pending = False
            self.finish('FAILED', 'Nav2 goal request failed: ' + str(exc))
            return
        self.goal_pending = False
        if not handle.accepted:
            if self.state == 'CANCELING':
                self.after_cancel()
            elif serial == self.goal_serial:
                self.goal_finished(False)
            return
        self.goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(lambda f: self.goal_result(f, serial))
        if serial != self.goal_serial or self.state not in ('SEARCHING', 'RETURNING'):
            handle.cancel_goal_async()

    def goal_result(self, future, serial):
        self.goal_handle = None
        self.goal_pending = False
        if self.state == 'CANCELING':
            self.after_cancel()
            return
        if serial != self.goal_serial or self.state not in ('SEARCHING', 'RETURNING'):
            return
        try:
            success = future.result().status == GoalStatus.STATUS_SUCCEEDED
        except Exception:
            success = False
        self.goal_finished(success)

    def goal_finished(self, success):
        self.heartbeat(False, self.goal_kind == 'search')
        if self.goal_kind == 'home':
            if not success:
                self.finish('FAILED', 'Return navigation failed')
            else:
                self.verify_started = 0.0
                self.goal_started = time.monotonic()
                self.set_state('VERIFYING', 'Checking home distance and stationary odometry')
        elif success:
            self.observe_started = time.monotonic()
            self.set_state('OBSERVING', 'Holding at viewpoint for target confirmation')
        else:
            self.failures.append(self.index)
            self.index += 1
            self.next_viewpoint()

    def next_viewpoint(self):
        if self.index >= len(self.route):
            self.reason = 'Search route exhausted without a confirmed target'
            self.begin_return()
            return
        v = self.route[self.index]
        self.send_goal(self.make_pose(v.x, v.y, v.yaw), 'search')

    def on_target(self, msg):
        if self.state not in ('SEARCHING', 'OBSERVING') or self.target is not None:
            return
        p = msg.pose.pose.position
        age = self.get_clock().now().nanoseconds*1e-9 - stamp_seconds(msg.header.stamp)
        variance = max(msg.pose.covariance[0], msg.pose.covariance[7])
        if (msg.header.frame_id != self.p['map_frame']
                or not all(math.isfinite(v) for v in (p.x, p.y, variance))
                or not 0 <= age <= self.p['max_target_age']
                or not 0 < variance <= self.p['max_target_variance']
                or self.readiness_error()):
            return
        # Coordinates must be inside the saved assessment map.
        r, c = self.planner.world_to_cell(p.x, p.y)
        if not self.planner.in_bounds(r, c):
            return
        self.target = {'x': p.x, 'y': p.y, 'variance': variance,
                       'class': self.p['target_class'], 'frame': self.p['map_frame']}
        self.attach_details()
        self.target_pub.publish(self.make_pose(p.x, p.y))
        self.get_logger().info(f'TARGET ({p.x:.3f}, {p.y:.3f}) m in map frame')
        self.persist()
        self.result_pub.publish(String(data=json.dumps({'event': 'target_reported',
                                                         'target': self.target})))
        self.cancel('RETURN', 'Target confirmed and reported')

    def on_details(self, msg):
        try:
            data = json.loads(msg.data)
            if (data['label'] == self.p['target_class']
                    and all(math.isfinite(float(data[k])) for k in
                            ('x','y','confidence','frames','stamp'))):
                self.target_details = data
                if self.target:
                    self.attach_details()
                    self.persist()
        except (ValueError, TypeError, KeyError):
            pass

    def attach_details(self):
        if self.target and self.target_details:
            d = self.target_details
            age = self.get_clock().now().nanoseconds*1e-9-float(d['stamp'])
            if (0 <= age <= self.p['max_target_age']
                    and math.hypot(d['x']-self.target['x'],d['y']-self.target['y']) < 0.05):
                self.target['confidence'] = d['confidence']
                self.target['confirmation_frames'] = d['frames']

    def cancel(self, next_state, reason):
        self.heartbeat(False, False)
        self.cancel_next = next_state
        if self.goal_pending or self.goal_handle is not None:
            self.cancel_deadline = time.monotonic()+self.p['cancel_timeout']
            self.set_state('CANCELING', reason)
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
        elif next_state == 'RETURN':
            self.begin_return()
        else:
            self.finish(next_state, reason)

    def after_cancel(self):
        next_state, self.cancel_next = self.cancel_next, None
        if next_state == 'RETURN':
            self.begin_return()
        else:
            self.finish(next_state or 'FAILED', self.reason)

    def begin_return(self):
        self.heartbeat(False, False)
        if self.home is None:
            self.finish('FAILED', 'No recorded home pose')
            return
        error = self.readiness_error(require_perception=False)
        if error:
            self.finish('FAILED', error)
            return
        current = self.pose()
        # Orientation is optional in the assessment. Face current direction at home.
        yaw = self.home[2] if self.p['return_heading_required'] else current[2]
        self.send_goal(self.make_pose(self.home[0], self.home[1], yaw), 'home')

    def finish(self, state, reason):
        self.heartbeat(False, False)
        self.finished_at = time.monotonic()
        self.goal_serial += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.set_state(state, reason)
        self.persist()
        self.result_pub.publish(String(data=json.dumps(self.summary())))

    def summary(self):
        pose=self.pose() if self.home is not None else None
        distance=math.hypot(pose[0]-self.home[0],pose[1]-self.home[1]) if pose and self.home else None
        readiness=self.readiness_error()
        return {'phase':'mission','stamp':self.get_clock().now().nanoseconds*1e-9,'state': self.state, 'reason': self.reason,
                'ready':readiness is None and self.nav.server_is_ready(),
                'readiness_reason':readiness or ('Ready' if self.nav.server_is_ready() else 'Nav2 not ready'),
                'return_verified':self.return_verified,'return_distance':distance,
                'return_tolerance':self.p['return_tolerance'],
                'stationary_speed':self.speed if math.isfinite(self.speed) else None,
                'stationary_required_seconds':self.p['verify_seconds'],
                'target_class': self.p['target_class'],
                'target': self.target, 'home': self.home,
                'elapsed_seconds': (self.finished_at or time.monotonic())-self.started if self.started else 0,
                'viewpoints_completed': self.index, 'viewpoints_total': len(self.route),
                'failed_viewpoints': self.failures,
                'observed_fraction': len(self.seen)/max(1, len(self.planner.target_cells))
                    if self.planner is not None else 0.0,
                'coverable_fraction': len(self.planner.coverable)/max(1,len(self.planner.target_cells))
                    if self.planner is not None else 0.0,
                'route_truncated': bool(self.planner.truncated) if self.planner else False}

    def persist(self):
        if self.run_dir is None:
            return
        try:
            payload = self.summary()
            payload['utc'] = datetime.now(timezone.utc).isoformat()
            temporary = self.run_dir/'result.tmp'
            temporary.write_text(json.dumps(payload, indent=2)+'\n')
            temporary.replace(self.run_dir/'result.json')
        except OSError as exc:
            self.get_logger().error('Could not save mission result: ' + str(exc))

    def tick(self):
        now = time.monotonic()
        active_search = self.state in ('SEARCHING', 'OBSERVING')
        moving = self.state in ('SEARCHING', 'RETURNING') and self.goal_handle is not None
        self.heartbeat(moving, active_search)
        if self.state in self.ACTIVE:
            if self.state == 'CANCELING':
                if now > self.cancel_deadline:
                    self.finish('FAILED', 'Nav2 cancellation timed out; motion remains disabled')
            elif now-self.started > self.p['mission_timeout']:
                self.cancel('FAILED', 'Mission time limit exceeded')
            else:
                error = self.readiness_error(require_perception=self.state != 'RETURNING'
                                               and self.state != 'VERIFYING')
                if error:
                    self.cancel('FAILED', error)
                elif self.state == 'PLANNING':
                    if self.plan_future.done():
                        try:
                            self.route = self.plan_future.result()
                            if not self.route:
                                raise ValueError('Empty search plan')
                            self.next_viewpoint()
                        except Exception as exc:
                            self.finish('FAILED', 'Search planning failed: ' + str(exc))
                    elif now-self.started > self.p['planning_timeout']:
                        self.finish('FAILED', 'Search planning time limit exceeded')
                elif self.state in ('SEARCHING', 'RETURNING'):
                    if self.goal_pending and now-self.goal_started > self.p['accept_timeout']:
                        self.cancel('FAILED', 'Nav2 goal acceptance timed out')
                    elif now-self.goal_started > self.p['goal_timeout']:
                        self.cancel('FAILED', 'Navigation goal time limit exceeded')
                elif self.state == 'OBSERVING' and now-self.observe_started >= self.p['observe_seconds']:
                    current = self.pose()
                    v = self.route[self.index]
                    if (math.hypot(current[0]-v.x, current[1]-v.y) < 0.25
                            and abs(angle_difference(current[2], v.yaw)) < 0.35):
                        row, col = self.planner.world_to_cell(*current[:2])
                        self.seen.update(self.planner.visible(row, col, current[2]))
                    self.index += 1
                    self.next_viewpoint()
                elif self.state == 'VERIFYING':
                    current = self.pose()
                    correct = returned_home(current, self.home, self.p['return_tolerance'])
                    if self.p['return_heading_required']:
                        correct = correct and abs(angle_difference(current[2],self.home[2])) < 0.3
                    if correct and self.speed < self.p['max_stationary_speed']:
                        self.verify_started = self.verify_started or now
                        if now-self.verify_started >= self.p['verify_seconds']:
                            self.return_verified=True
                            self.finish('COMPLETE' if self.target else 'NOT_FOUND',
                                        'Home verified; target reported' if self.target
                                        else 'Home verified; no target was confirmed')
                    else:
                        self.verify_started = 0.0
                    if self.state == 'VERIFYING' and now-self.goal_started > self.p['verify_timeout']:
                        self.finish('FAILED', 'Return distance/stationary verification failed')
        if now-self.last_status > 0.5:
            self.publish_status()
            self.last_status = now

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self.summary())))
        path = NavPath()
        path.header.frame_id = self.p['map_frame']
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self.make_pose(v.x, v.y, v.yaw) for v in self.route]
        self.route_pub.publish(path)
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, v in enumerate(self.route):
            m = Marker()
            m.header = path.header
            m.ns, m.id, m.type, m.action = 'viewpoints', i, Marker.ARROW, Marker.ADD
            m.pose = self.make_pose(v.x, v.y, v.yaw).pose
            m.scale.x, m.scale.y, m.scale.z = 0.22, 0.05, 0.05
            m.color.a = 0.9
            m.color.g = 1.0 if i < self.index and i not in self.failures else 0.2
            m.color.r = 1.0 if i in self.failures else 0.1
            m.color.b = 0.9 if i >= self.index else 0.1
            markers.markers.append(m)
        text = Marker()
        text.header = path.header
        text.ns, text.id, text.type = 'status', 0, Marker.TEXT_VIEW_FACING
        text.pose.orientation.w = 1.0
        if self.home:
            text.pose.position.x, text.pose.position.y = self.home[:2]
        text.pose.position.z, text.scale.z = 0.8, 0.16
        text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
        text.text = f'{self.state}\n{self.reason}'
        if self.target:
            text.text += f'\nTarget ({self.target["x"]:.2f}, {self.target["y"]:.2f}) m'
            if 'confidence' in self.target:
                text.text += f' | confidence {self.target["confidence"]:.0%}'
            target = Marker()
            target.header = path.header
            target.ns, target.id, target.type = 'target', 0, Marker.SPHERE
            target.pose = self.make_pose(self.target['x'], self.target['y']).pose
            target.pose.position.z = 0.15
            target.scale.x = target.scale.y = target.scale.z = 0.15
            target.color.r, target.color.g, target.color.a = 1.0, 0.5, 1.0
            markers.markers.append(target)
        markers.markers.append(text)
        self.markers_pub.publish(markers)

    def destroy_node(self):
        if self.context.ok():
            self.heartbeat(False, False)
        if self.goal_handle and self.context.ok():
            self.goal_handle.cancel_goal_async()
        self.worker.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()


def main():
    run(Mission)
