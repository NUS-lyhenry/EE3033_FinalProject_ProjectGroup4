"""Separate preparation session. Refuse to overlap mapping or mission stacks."""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node as GraphNode
    graph_context = Context()
    # Launch's argv contains launch arguments, not ROS remaps. Use this
    # context's executor explicitly: Humble's global executor uses the default
    # (uninitialized) context and cannot spin a node from this private context.
    rclpy.init(args=[], context=graph_context)
    probe = GraphNode('project_calibration_preflight', context=graph_context)
    executor = SingleThreadedExecutor(context=graph_context)
    executor.add_node(probe)
    try:
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        conflicts = [topic for topic in ('/cmd_vel', '/project/nav_cmd_vel',
                     '/project/drive_enabled', '/project/mission_status') if probe.count_publishers(topic)]
        names = probe.get_node_names()
        for flag, name in [('base', 'balance_robot_node'), ('lidar', 'lslidar_driver_node'),
                           ('camera', 'usb_cam')]:
            if LaunchConfiguration(flag).perform(context).lower() == 'true' and name in names:
                conflicts.append(name)
        if 'project_scan_resampler' in names or 'project_camera_calibration' in names:
            conflicts.append('existing project sensor/calibration node')
        if conflicts:
            raise RuntimeError('Stop the existing robot stack before calibration: '+', '.join(conflicts))
    finally:
        executor.shutdown()
        probe.destroy_node()
        rclpy.shutdown(context=graph_context)
    share = get_package_share_directory('ee3033_project')
    motion = ParameterValue(LaunchConfiguration('allow_motion'), value_type=bool)
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hardware.launch.py')),
                                 launch_arguments={k: LaunchConfiguration(k) for k in ('base', 'lidar', 'camera')}.items()),
        Node(package='ee3033_project', executable='velocity_gate', output='screen', parameters=[
            os.path.join(share, 'config', 'project.yaml'), dict(allow_motion=motion,
                max_linear=.04, max_angular=.15, rotation_clearance=.16)]),
        Node(package='ee3033_project', executable='camera_calibration', output='screen', parameters=[{
            'allow_motion': motion, 'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            **{key: ParameterValue(LaunchConfiguration(key), value_type=str)
               for key in ('model', 'target_class', 'output_directory')},
        }]),
    ]


def generate_launch_description():
    defaults = dict(allow_motion='false', base='true', lidar='true', camera='true', port='8780',
                    model='/home/ros2_ws/weights/yolo26n.pt', target_class='bottle',
                    output_directory='~/ee3033_calibrations')
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
                              OpaqueFunction(function=start)])
