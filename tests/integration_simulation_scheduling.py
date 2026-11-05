"""Real executor proves a slow renderer cannot serialize physics and commands.

Synthetic camera pixels deliberately take 240 ms. No navigation, perception,
physical drivers or nonzero hardware output is involved. Isolated domain 156.
"""
import json
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
import simulation_robot


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '156'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    def slow_renderer(*args):
        time.sleep(.24)
        return np.zeros((480, 640, 3), np.uint8)
    simulation_robot.render_camera = slow_renderer
    rclpy.init()
    robot = simulation_robot.SimulatedRobot('/tmp/ee3033_sim/assets/bottle.png')
    io = Node('simulation_scheduling_probe')
    odometry, images, infos = [], [], []
    io.create_subscription(Odometry, '/odom', lambda m: odometry.append(time.monotonic()), qos_profile_sensor_data)
    io.create_subscription(CameraInfo, '/camera_info', lambda m: infos.append(m.header.stamp), qos_profile_sensor_data)
    io.create_subscription(Image, '/image_raw', lambda m: images.append((m.header.stamp,time.time())), qos_profile_sensor_data)
    command = io.create_publisher(Twist, '/cmd_vel', 10)
    io.create_timer(.03, lambda: command.publish(Twist()))
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(robot)
    executor.add_node(io)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        time.sleep(1.8)
        gaps = np.diff(odometry)
        ages = [received-stamp.sec-stamp.nanosec*1e-9 for stamp,received in images]
        result = dict(odometry_samples=len(odometry), image_samples=len(images),
            max_odometry_gap_seconds=float(max(gaps)) if len(gaps) else None,
            median_image_capture_age_seconds=float(np.median(ages)) if ages else None,
            command_age_seconds=time.monotonic()-robot.command_time,
            same_camera_info_stamps=all(any(a.sec==b.sec and a.nanosec==b.nanosec for b in infos)
                                        for a,_ in images))
        assert len(odometry) >= 35, result
        assert len(images) >= 4, result
        assert max(gaps) < .18, result
        assert min(ages) >= .20, result
        assert result['command_age_seconds'] < .15, result
        assert result['same_camera_info_stamps'], result
        print(json.dumps(result, indent=2),flush=True)
    finally:
        executor.shutdown(timeout_sec=3.)
        thread.join(timeout=3.)
        robot.destroy_node()
        io.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
