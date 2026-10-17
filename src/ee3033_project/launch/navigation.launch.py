"""Humble Nav2 nodes with one explicit, gated velocity output.

The chassis already performs acceleration/jerk smoothing; a second Nav2 velocity
smoother is unnecessary. Explicit nodes avoid the lab launch's cmd_vel remaps.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('ee3033_project'),
                           'config', 'navigation.yaml')
    config = LaunchConfiguration('nav_config')
    mapping = LaunchConfiguration('mapping_mode')
    servers = [
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
    ]
    def start_servers(context):
        overrides = ({'goal_checker.yaw_goal_tolerance': 3.14159,
                      'FollowPath.rotate_to_goal_heading': False}
                     if mapping.perform(context)=='true' else {})
        return [Node(package=package, executable=name, name=name,
                     output='screen', parameters=[config, {'use_sim_time': False},
                                                  overrides if name=='controller_server' else {}],
                     remappings=[('cmd_vel', '/project/nav_cmd_vel')])
                for package,name in servers]

    return LaunchDescription([
        DeclareLaunchArgument('nav_config', default_value=default),
        DeclareLaunchArgument('mapping_mode', default_value='false'),
        OpaqueFunction(function=start_servers),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='project_navigation_lifecycle', output='screen', parameters=[{
                 'autostart': True, 'use_sim_time': False,
                 'node_names': [name for _, name in servers],
                 'attempt_respawn_reconnection': False,
             }]),
    ])
