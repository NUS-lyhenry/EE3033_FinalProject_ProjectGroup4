"""Use installed lab drivers and an optional, explicitly built camera timebase fix."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
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
            driver_actions = []
            if arg == 'camera':
                fixed = os.environ.get('GROUP4_USB_CAM_LIBRARY_DIR',
                    '/home/ros2_ws/group4_driver_fixes/usb_cam-0.8.1/fixed/lib')
                if os.path.isfile(os.path.join(fixed, 'libusb_cam.so')):
                    driver_actions.append(SetEnvironmentVariable('LD_LIBRARY_PATH',
                        fixed+':'+context.environment.get('LD_LIBRARY_PATH', '')))
            driver_actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(path), launch_arguments=kwargs.items()))
            actions.append(GroupAction(scoped=True, actions=driver_actions))
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
