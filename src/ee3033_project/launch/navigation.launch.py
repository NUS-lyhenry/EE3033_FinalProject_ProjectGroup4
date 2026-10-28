"""Humble Nav2 nodes with one explicit, gated velocity output.

The chassis already performs acceleration/jerk smoothing; a second Nav2 velocity
smoother is unnecessary. Explicit nodes avoid the lab launch's cmd_vel remaps.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from nav2_common.launch import RewrittenYaml
from ee3033_project.robot_geometry import GEOMETRY_PROFILE, PHYSICAL_RADIUS


def action_remaps(source, target):
    # Humble action names expand into five endpoints before ROS remapping.
    return [(f'/{source}/_action/{suffix}',f'{target}/_action/{suffix}')
            for suffix in ('send_goal','get_result','cancel_goal','feedback','status')]


def generate_launch_description():
    share = get_package_share_directory('ee3033_project')
    default = os.path.join(share, 'config', 'navigation.yaml')
    config = LaunchConfiguration('nav_config')
    if GEOMETRY_PROFILE=='legacy':
        config=ParameterFile(RewrittenYaml(source_file=config,
            param_rewrites={'robot_radius':str(PHYSICAL_RADIUS)},convert_types=True),allow_substs=True)
    servers = [
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
    ]
    def start_servers(context):
        fast = LaunchConfiguration('navigation_profile').perform(context) == 'fast'
        supervised = LaunchConfiguration('supervised_navigation').perform(context)=='true'
        overrides = {'controller_server': {}}
        if supervised:
            overrides['bt_navigator']={'default_nav_to_pose_bt_xml':
                os.path.join(share, 'behavior_trees', 'navigate_supervised.xml')}
        if fast:
            overrides['controller_server'].update({
                'FollowPath.desired_linear_vel': .16, 'FollowPath.max_linear_vel': .16,
                'FollowPath.rotate_to_heading_angular_vel': .60, 'FollowPath.max_angular_vel': .60})
            overrides['behavior_server'] = {'max_rotational_vel': .60}
        return [Node(package=package, executable=name, name=name,
                     output='screen', parameters=[config, {'use_sim_time': False},
                                                  overrides.get(name,{})],
                     remappings=[('cmd_vel', '/project/nav_cmd_vel')]+(
                         action_remaps('navigate_to_pose','/project/nav2_navigate_to_pose')
                         if name=='bt_navigator' and supervised else
                         action_remaps('compute_path_to_pose','/project/nav2_compute_path_to_pose')+
                         [('plan','/project/native_plan')]
                         if name=='planner_server' else []))
                for package,name in servers]

    return LaunchDescription([
        DeclareLaunchArgument('nav_config', default_value=default),
        DeclareLaunchArgument('mapping_mode', default_value='false'),
        DeclareLaunchArgument('supervised_navigation', default_value='true'),
        DeclareLaunchArgument('final_command_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('navigation_profile', default_value='original', choices=['original', 'fast']),
        OpaqueFunction(function=start_servers),
        Node(package='ee3033_project', executable='path_planner', output='screen'),
        Node(package='ee3033_project', executable='navigation_supervisor',
             condition=IfCondition(LaunchConfiguration('supervised_navigation')), output='screen', parameters=[{'command_topic': LaunchConfiguration('final_command_topic'),
                 'goal_timeout': ParameterValue(PythonExpression(["60.0 if '",LaunchConfiguration('mapping_mode'),"' == 'true' else 75.0"]),value_type=float)}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='project_navigation_lifecycle', output='screen', parameters=[{
                 'autostart': True, 'use_sim_time': False,
                 'node_names': [name for _, name in servers],
                 'attempt_respawn_reconnection': False,
             }]),
    ])
