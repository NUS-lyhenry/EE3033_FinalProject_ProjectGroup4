"""Fault injection through complete native Nav2, supervisor and final safety gate.

Only this fixture supplies world/odometry/sensors. Physical truth is recorded for
evaluation; navigation receives ordinary map, laser, odometry and TF messages.
Run each case in an isolated process/domain with ROS_LOCALHOST_ONLY=1.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from ee3033_project.robot_geometry import geometry_metadata, BODY_RADIUS
from ee3033_project.ros_utils import LATCHED, set_yaw
from ee3033_project.velocity_gate import VelocityGate
from simulation_geometry import raycast
from simulation_motion import advance_motion


def wait(future, timeout=10.):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.02)
    if not future.done():
        raise TimeoutError('Native navigation fixture request timed out')
    return future.result()


class FaultWorld(Node):
    def __init__(self, fault):
        super().__init__('navigation_fault_fixture')
        self.fault = fault
        self.xyz = [0., 0., 0.]
        self.command = (0., 0.)
        self.actual = (0., 0.)
        self.running = False
        self.enabled = True
        self.first_command = self.fault_at = None
        self.last = time.monotonic()
        self.collisions = 0
        self.trace = []
        self.gate_status = {}
        self.nav_status = {}
        self.recoveries = 0
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        tr = TransformStamped()
        tr.header.frame_id, tr.child_frame_id = 'map', 'odom_combined'
        tr.header.stamp = self.get_clock().now().to_msg()
        tr.transform.rotation.w = 1.
        self.static.sendTransform(tr)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_fixed', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.drive = self.create_publisher(Bool, '/project/drive_enabled', LATCHED)
        self.maps = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_subscription(String, '/project/velocity_gate_status', self.on_gate, 10)
        self.create_subscription(String, '/project/navigation_status', self.on_navigation, 10)
        self.walls = np.array([[-2., -2., 2., -2.], [2., -2., 2., 2.],
                               [2., 2., -2., 2.], [-2., 2., -2., -2.]]).reshape(-1, 2, 2)
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.info.width = m.info.height = 80
        m.info.resolution = .05
        m.info.origin.position.x = m.info.origin.position.y = -2.
        m.info.origin.orientation.w = 1.
        cells = np.zeros((80, 80), np.int8)
        cells[[0, -1], :] = 100
        cells[:, [0, -1]] = 100
        # Rear unknown begins outside both supported protected footprints.
        # Forward route is unaffected; a frozen chassis later asks to reverse.
        if fault == 'unknown_rear':
            cells[36:44, 28:36] = -1
        m.data = cells.ravel().tolist()
        m.header.stamp = self.get_clock().now().to_msg()
        self.maps.publish(m)
        self.map_message = m
        self.last_map = -math.inf
        self.create_timer(.02, self.tick)

    def on_command(self, msg):
        self.command = (msg.linear.x, msg.angular.z)
        if self.running and self.first_command is None and any(abs(v) > .005 for v in self.command):
            self.first_command = time.monotonic()

    def on_gate(self, msg):
        self.gate_status = json.loads(msg.data)

    def on_navigation(self, msg):
        self.nav_status = json.loads(msg.data)
        self.recoveries = max(self.recoveries, self.nav_status.get('recovery_count', 0))

    def tick(self):
        now = time.monotonic()
        dt, self.last = min(.05, now-self.last), now
        trigger = (self.nav_status.get('stage') == 'REVERSING' if self.fault == 'reverse_laser'
                   else self.first_command is not None and now-self.first_command >= .5)
        if self.fault_at is None and trigger:
            self.fault_at = now
        injected = self.fault_at is not None
        if injected and self.fault == 'stop':
            self.enabled = False  # This fixture is the sole motion owner.
        old = list(self.xyz)
        v, w = self.command if self.running and self.fault not in ('frozen', 'unknown_rear', 'reverse_laser') else (0., 0.)
        self.xyz, blocked = advance_motion(self.xyz, (v, w), dt, self.walls, BODY_RADIUS)
        self.collisions += bool(blocked)
        self.actual = (math.dist(self.xyz[:2], old[:2])/dt, (self.xyz[2]-old[2])/dt)
        stamp = self.get_clock().now().to_msg()
        if now-self.last_map >= .2:
            self.last_map = now
            self.map_message.header.stamp = stamp
            self.maps.publish(self.map_message)
        tr = TransformStamped()
        tr.header.stamp, tr.header.frame_id, tr.child_frame_id = stamp, 'odom_combined', 'base_footprint'
        tr.transform.translation.x, tr.transform.translation.y = self.xyz[:2]
        set_yaw(tr.transform.rotation, self.xyz[2])
        if not (injected and self.fault == 'localization'):
            self.tf.sendTransform(tr)
        odom = Odometry()
        odom.header = tr.header
        odom.child_frame_id = tr.child_frame_id
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.xyz[:2]
        set_yaw(odom.pose.pose.orientation, self.xyz[2])
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.actual
        self.odom_pub.publish(odom)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp, pose.header.frame_id = stamp, 'map'
        pose.pose.pose.orientation.w = 1.
        pose.pose.covariance[0] = pose.pose.covariance[7] = .000025
        pose.pose.covariance[35] = .0001
        self.pose_pub.publish(pose)
        if not (injected and self.fault in ('laser', 'reverse_laser')):
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'base_footprint'
            scan.angle_min, scan.angle_increment = -math.pi, math.tau/360
            scan.angle_max = scan.angle_min+359*scan.angle_increment
            scan.range_min, scan.range_max = .08, 8.
            angles = scan.angle_min+np.arange(360)*scan.angle_increment+self.xyz[2]
            scan.ranges = raycast(self.xyz[:2], angles, self.walls).astype(float).tolist()
            self.scan_pub.publish(scan)
        self.drive.publish(Bool(data=self.running and self.enabled))
        if self.running:
            self.trace.append(dict(elapsed=now-(self.first_command or now), fault=injected,
                pose=list(self.xyz), actual=list(self.actual), command=list(self.command),
                gate=self.gate_status.get('reason'), stage=self.nav_status.get('stage')))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--fault', choices=['laser', 'reverse_laser', 'localization', 'frozen', 'unknown_rear', 'stop'], required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '6')
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    directory = Path(args.output)
    directory.mkdir(parents=True, exist_ok=True)
    report = dict(fault=args.fault, geometry=geometry_metadata(), passed=False)
    sources = ['src/ee3033_navigation/src/regulated_path_follower.cpp',
        *['src/ee3033_project/ee3033_project/'+s for s in ('navigation_supervisor.py', 'path_planner.py', 'velocity_gate.py')]]
    report['source_sha256'] = {s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources}
    from ament_index_python.packages import get_package_prefix
    library = Path(get_package_prefix('ee3033_navigation'))/'lib/libee3033_regulated_path_follower.so'
    report['controller_library_sha256'] = hashlib.sha256(library.read_bytes()).hexdigest()
    process = None
    rclpy.init()
    world = FaultWorld(args.fault)
    gate = VelocityGate(parameter_overrides=[Parameter('allow_motion', value=True)])
    executor = MultiThreadedExecutor(num_threads=5)
    for node in (world, gate):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        with (directory/'launch.log').open('w') as stream:
            process = subprocess.Popen(['ros2', 'launch', 'ee3033_project', 'navigation.launch.py'],
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        lifecycle = world.create_client(GetState, '/bt_navigator/get_state')
        assert lifecycle.wait_for_service(timeout_sec=30), 'Nav2 lifecycle unavailable'
        deadline = time.monotonic()+30
        while wait(lifecycle.call_async(GetState.Request())).current_state.id != 3:
            assert time.monotonic() < deadline
            time.sleep(.1)
        client = ActionClient(world, NavigateToPose, '/navigate_to_pose')
        assert client.wait_for_server(timeout_sec=10)
        time.sleep(2.)
        world.running = True
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = 1.2
        goal.pose.pose.orientation.w = 1.
        handle = wait(client.send_goal_async(goal))
        assert handle.accepted
        result = wait(handle.get_result_async(), 65.)
        report.update(action_status=result.status, recoveries=world.recoveries,
            reason=world.nav_status.get('reason'), collision_rejections=world.collisions,
            pose=list(world.xyz), injected=world.fault_at is not None,
            terminal_after_fault=time.monotonic()-world.fault_at if world.fault_at else None)
        time.sleep(.7)
        report['terminal_command'] = list(world.command)
        report['actual_velocity'] = list(world.actual)
        tail = world.trace[-15:]
        safe = (result.status != 4 and world.collisions == 0 and world.fault_at is not None
            and all(abs(v) <= .005 for row in tail for v in row['command']))
        if args.fault in ('laser', 'localization', 'stop'):
            safe &= world.recoveries == 0 and report['terminal_after_fault'] < 5.
        else:
            safe &= world.recoveries <= 2 and math.dist(world.xyz[:2], (0., 0.)) < .005
        if args.fault == 'reverse_laser':
            safe &= report['terminal_after_fault'] < 5.
        report['passed'] = bool(safe)
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        world.running = False
        time.sleep(.1)
        if process:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        gate.destroy_node()
        world.destroy_node()
        rclpy.shutdown()
        (directory/'trace.json').write_text(json.dumps(world.trace))
        (directory/'result.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
