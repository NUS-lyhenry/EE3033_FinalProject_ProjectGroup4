"""Use installed lab drivers without modifying their code or configuration."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    actions = []
    for arg, package, filename in [
        ('base', 'balance_robot_base', 'balance_robot_base.launch.py'),
        ('lidar', 'lslidar_driver_n10p', 'lslidar_launch.py'),
        ('camera', 'balance_usb_cam', 'usb_cam.launch.py'),
    ]:
        if LaunchConfiguration(arg).perform(context).lower() == 'true':
            share = get_package_share_directory(package)
            path = os.path.join(share, 'launch', filename)
            kwargs = {'mode': 'detector'} if arg == 'camera' else {}
            if arg == 'base':
                # Mission's generic `config` is the project YAML. Passing the lab
                # config explicitly keeps its frame IDs and validated settings.
                kwargs['config'] = os.path.join(share, 'config', 'base_params.yaml')
            # Included launch arguments otherwise leak back into the parent:
            # the base's `config` must not replace mission/perception's YAML.
            actions.append(GroupAction(scoped=True, actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(path), launch_arguments=kwargs.items())]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('base', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        OpaqueFunction(function=setup),
        Node(package='ee3033_project', executable='scan_resampler', output='screen'),
        # REP-103 body frame (x forward/y left/z up) -> optical frame (z forward).
        # Existing base launch owns camera_link's measured mounting position/pitch.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='project_camera_optical_tf', arguments=[
                 '--x', '0', '--y', '0', '--z', '0',
                 '--roll', '-1.5707963267948966', '--pitch', '0',
                 '--yaw', '-1.5707963267948966',
                 '--frame-id', 'camera_link', '--child-frame-id', 'project_camera_optical']),
    ])
