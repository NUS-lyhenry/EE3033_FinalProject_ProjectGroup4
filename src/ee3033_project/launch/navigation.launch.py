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
    share = get_package_share_directory('ee3033_project')
    default = os.path.join(share, 'config', 'navigation.yaml')
    recovery_tree = os.path.join(share, 'behavior_trees', 'navigate_with_bounded_recovery.xml')
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
        fast = LaunchConfiguration('navigation_profile').perform(context) == 'fast'
        overrides = {
            'controller_server': ({'goal_checker.yaw_goal_tolerance': 3.14159,
                                   'FollowPath.rotate_to_goal_heading': False}
                                  if mapping.perform(context)=='true' else {}),
            'bt_navigator': {'default_nav_to_pose_bt_xml': recovery_tree},
        }
        if fast:
            overrides['controller_server'].update({
                'FollowPath.max_vel_x': .16, 'FollowPath.max_speed_xy': .16,
                'FollowPath.max_vel_theta': .60, 'FollowPath.rotate_to_heading_angular_vel': .55})
            overrides['behavior_server'] = {'max_rotational_vel': .60}
        return [Node(package=package, executable=name, name=name,
                     output='screen', parameters=[config, {'use_sim_time': False},
                                                  overrides.get(name,{})],
                     remappings=[('cmd_vel', '/project/nav_cmd_vel')])
                for package,name in servers]

    return LaunchDescription([
        DeclareLaunchArgument('nav_config', default_value=default),
        DeclareLaunchArgument('mapping_mode', default_value='false'),
        DeclareLaunchArgument('navigation_profile', default_value='original', choices=['original', 'fast']),
        OpaqueFunction(function=start_servers),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='project_navigation_lifecycle', output='screen', parameters=[{
                 'autostart': True, 'use_sim_time': False,
                 'node_names': [name for _, name in servers],
                 'attempt_respawn_reconnection': False,
             }]),
    ])
