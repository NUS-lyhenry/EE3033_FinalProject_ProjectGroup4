import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def check_map(context):
    path = LaunchConfiguration('map').perform(context)
    if not os.path.isfile(path):
        raise RuntimeError('Provide the map YAML saved on-site: map:=/absolute/path.yaml')
    return []


def generate_launch_description():
    share = get_package_share_directory('ee3033_project')
    config = LaunchConfiguration('config')
    target = ParameterValue(LaunchConfiguration('target_class'), value_type=str)
    motion = ParameterValue(LaunchConfiguration('allow_motion'), value_type=bool)
    include = lambda path, args: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=args.items())
    nav_share = get_package_share_directory('nav2_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='On-site map YAML (required)'),
        DeclareLaunchArgument('target_class', default_value='cup'),
        DeclareLaunchArgument('allow_motion', default_value='false'),
        DeclareLaunchArgument('base', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('console', default_value='true'),
        DeclareLaunchArgument('map_directory', default_value='~/ee3033_maps'),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'project.yaml')),
        DeclareLaunchArgument('nav_config', default_value=os.path.join(share, 'config', 'navigation.yaml')),
        DeclareLaunchArgument('model', default_value='/home/ros2_ws/weights/yolo26n.pt'),
        DeclareLaunchArgument('localization_method', default_value='lidar'),
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('return_tolerance', default_value='0.20'),
        OpaqueFunction(function=check_map),
        include(os.path.join(share, 'launch', 'hardware.launch.py'), {
            k: LaunchConfiguration(k) for k in ('base', 'lidar', 'camera')}),
        include(os.path.join(nav_share, 'launch', 'localization_launch.py'), {
            'map': LaunchConfiguration('map'), 'params_file': LaunchConfiguration('nav_config'),
            'use_sim_time': 'false', 'use_composition': 'False', 'use_respawn': 'False'}),
        include(os.path.join(share, 'launch', 'navigation.launch.py'), {
            'nav_config': LaunchConfiguration('nav_config')}),
        Node(package='ee3033_project', executable='perception', output='screen', parameters=[
            config, {'target_class': target,
                     'calibration_file': ParameterValue(LaunchConfiguration('calibration_file'), value_type=str),
                     'model': ParameterValue(LaunchConfiguration('model'), value_type=str),
                     'localization_method': ParameterValue(LaunchConfiguration('localization_method'),
                                                          value_type=str)}]),
        Node(package='ee3033_project', executable='target_filter', output='screen', parameters=[config]),
        Node(package='ee3033_project', executable='velocity_gate', output='screen', parameters=[
            config, {'allow_motion': motion}]),
        Node(package='ee3033_project', executable='mission', output='screen', parameters=[
            config, {'target_class': target, 'allow_motion': motion,
                     'return_tolerance':ParameterValue(LaunchConfiguration('return_tolerance'),value_type=float)}]),
        Node(package='ee3033_project',executable='console',output='screen',
             condition=IfCondition(LaunchConfiguration('console')),parameters=[{
                 'phase':'mission','allow_motion':motion,'map_directory':LaunchConfiguration('map_directory'),
                 'port':ParameterValue(LaunchConfiguration('port'),value_type=int)}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'rviz', 'mission.rviz')]),
    ])
