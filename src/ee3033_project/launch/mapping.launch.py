import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ee3033_project.robot_geometry import GEOMETRY_PROFILE, FOOTPRINT_RADIUS, DEFAULT_STOP_DISTANCE
from ee3033_project.gap import TURNING, DEFAULT_AUDACITY


def generate_launch_description():
    share = get_package_share_directory('ee3033_project')
    geometry_overrides=({'footprint_radius':FOOTPRINT_RADIUS,'reverse_radius':FOOTPRINT_RADIUS,
        'stop_distance':DEFAULT_STOP_DISTANCE,'rotation_clearance':TURNING[DEFAULT_AUDACITY]['stop']}
        if GEOMETRY_PROFILE=='legacy' else {})
    return LaunchDescription([
        DeclareLaunchArgument('allow_motion', default_value='false'),
        DeclareLaunchArgument('base', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('self_mask_radius', default_value='.08'),
        DeclareLaunchArgument('mapping_resolution', default_value='.05'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('console', default_value='true'),
        DeclareLaunchArgument('map_directory', default_value='~/ee3033_maps'),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('navigation_profile', default_value='original', choices=['original', 'fast']),
        DeclareLaunchArgument('nav_config', default_value=os.path.join(share,'config','navigation.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'hardware.launch.py')), launch_arguments={
                'base': LaunchConfiguration('base'), 'lidar': LaunchConfiguration('lidar'),
                'camera': 'false','self_mask_radius':LaunchConfiguration('self_mask_radius')}.items()),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', parameters=[os.path.join(share, 'config', 'slam.yaml'),
                 {'resolution':ParameterValue(LaunchConfiguration('mapping_resolution'),value_type=float)}],
             output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share,'launch','navigation.launch.py')),launch_arguments={
                'nav_config':LaunchConfiguration('nav_config'),'mapping_mode':'true',
                'navigation_profile':LaunchConfiguration('navigation_profile')}.items()),
        Node(package='ee3033_project',executable='mapper',output='screen',parameters=[{
            'allow_motion':ParameterValue(LaunchConfiguration('allow_motion'),value_type=bool),
            'map_directory':LaunchConfiguration('map_directory')}]),
        Node(package='ee3033_project', executable='velocity_gate', parameters=[
            os.path.join(share, 'config', 'project.yaml'), {'mapping_mode': True,
            'navigation_profile': LaunchConfiguration('navigation_profile'),
            'allow_motion': ParameterValue(LaunchConfiguration('allow_motion'), value_type=bool)},geometry_overrides],
             output='screen'),
        Node(package='ee3033_project',executable='console',output='screen',
             condition=IfCondition(LaunchConfiguration('console')),parameters=[{
                 'phase':'mapping','map_directory':LaunchConfiguration('map_directory'),
                 'port':ParameterValue(LaunchConfiguration('port'),value_type=int),
                 'allow_motion':ParameterValue(LaunchConfiguration('allow_motion'),value_type=bool)}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'rviz', 'mission.rviz')]),
    ])
