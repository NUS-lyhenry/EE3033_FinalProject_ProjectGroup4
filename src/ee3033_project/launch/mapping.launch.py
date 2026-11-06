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
from ee3033_project.operator_defaults import DEFAULT_REVERSE_RECOVERY, DEFAULT_SCAN_RADIUS
from ee3033_project.trial import DEFAULT_PROFILE, PROFILES
from ee3033_project.grid_settings import GRID_DEFAULTS
from ee3033_project.mapping_settings import DEFAULT_SLAM_SETTINGS


def generate_launch_description():
    share = get_package_share_directory('ee3033_project')
    geometry_overrides=({'footprint_radius':FOOTPRINT_RADIUS,'reverse_radius':FOOTPRINT_RADIUS,
        'stop_distance':DEFAULT_STOP_DISTANCE,'rotation_clearance':TURNING[DEFAULT_AUDACITY]['stop']}
        if GEOMETRY_PROFILE=='legacy' else {})
    return LaunchDescription([
        DeclareLaunchArgument('allow_motion', default_value='false'),
        DeclareLaunchArgument('base', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('self_mask_radius', default_value=str(DEFAULT_SCAN_RADIUS)),
        DeclareLaunchArgument('mapping_resolution', default_value=str(GRID_DEFAULTS['mapping_resolution'])),
        DeclareLaunchArgument('occupancy_threshold', default_value=str(DEFAULT_SLAM_SETTINGS['occupancy_threshold'])),
        DeclareLaunchArgument('minimum_pass_through', default_value=str(DEFAULT_SLAM_SETTINGS['minimum_pass_through'])),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('console', default_value='true'),
        DeclareLaunchArgument('map_directory', default_value='~/ee3033_maps'),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('allow_reverse_recovery', default_value=str(DEFAULT_REVERSE_RECOVERY).lower()),
        DeclareLaunchArgument('navigation_profile', default_value=DEFAULT_PROFILE, choices=list(PROFILES)),
        DeclareLaunchArgument('nav_config', default_value=os.path.join(share,'config','navigation.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'hardware.launch.py')), launch_arguments={
                'base': LaunchConfiguration('base'), 'lidar': LaunchConfiguration('lidar'),
                'camera': 'false','self_mask_radius':LaunchConfiguration('self_mask_radius')}.items()),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', parameters=[os.path.join(share, 'config', 'slam.yaml'),
                 {'resolution':ParameterValue(LaunchConfiguration('mapping_resolution'),value_type=float),
                  'occupancy_threshold':ParameterValue(LaunchConfiguration('occupancy_threshold'),value_type=float),
                  'min_pass_through':ParameterValue(LaunchConfiguration('minimum_pass_through'),value_type=int)}],
             output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share,'launch','navigation.launch.py')),launch_arguments={
                'nav_config':LaunchConfiguration('nav_config'),'mapping_mode':'true',
                'navigation_profile':LaunchConfiguration('navigation_profile'),
                'allow_reverse_recovery':LaunchConfiguration('allow_reverse_recovery')}.items()),
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
                 'mapping_resolution':ParameterValue(LaunchConfiguration('mapping_resolution'),value_type=float),
                 'port':ParameterValue(LaunchConfiguration('port'),value_type=int),
                 'allow_motion':ParameterValue(LaunchConfiguration('allow_motion'),value_type=bool)}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'rviz', 'mission.rviz')]),
    ])
