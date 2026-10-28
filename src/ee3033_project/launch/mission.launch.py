import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ee3033_project.robot_geometry import GEOMETRY_PROFILE, FOOTPRINT_RADIUS, DEFAULT_STOP_DISTANCE
from ee3033_project.gap import TURNING, DEFAULT_AUDACITY


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
    geometry_overrides=({'footprint_radius':FOOTPRINT_RADIUS,'reverse_radius':FOOTPRINT_RADIUS,
        'stop_distance':DEFAULT_STOP_DISTANCE,'rotation_clearance':TURNING[DEFAULT_AUDACITY]['stop']}
        if GEOMETRY_PROFILE=='legacy' else {})
    return LaunchDescription([
        DeclareLaunchArgument('map', description='On-site map YAML (required)'),
        DeclareLaunchArgument('map_partial', default_value='false'),
        DeclareLaunchArgument('target_class', default_value='cup'),
        DeclareLaunchArgument('allow_motion', default_value='false'),
        DeclareLaunchArgument('base', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('self_mask_radius', default_value='.08'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('console', default_value='true'),
        DeclareLaunchArgument('map_directory', default_value='~/ee3033_maps'),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'project.yaml')),
        DeclareLaunchArgument('nav_config', default_value=os.path.join(share, 'config', 'navigation.yaml')),
        DeclareLaunchArgument('model', default_value='/home/ros2_ws/weights/yolo26n.pt'),
        DeclareLaunchArgument('localization_method', default_value='hybrid'),
        DeclareLaunchArgument('vision_config', default_value=os.path.join(share, 'config', 'vision.yaml')),
        DeclareLaunchArgument('navigation_profile', default_value='original', choices=['original', 'fast']),
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('return_tolerance', default_value='0.20'),
        DeclareLaunchArgument('preferred_target_sigma', default_value='0.10'),
        DeclareLaunchArgument('enable_ab_maneuver', default_value='false'),
        DeclareLaunchArgument('acquisition_seconds', default_value='90.0'),
        DeclareLaunchArgument('attitude_config', default_value=''),
        DeclareLaunchArgument('results_directory', default_value='~/ee3033_results'),
        OpaqueFunction(function=check_map),
        include(os.path.join(share, 'launch', 'hardware.launch.py'), {
            k: LaunchConfiguration(k) for k in ('base', 'lidar', 'camera', 'self_mask_radius')}),
        include(os.path.join(nav_share, 'launch', 'localization_launch.py'), {
            'map': LaunchConfiguration('map'), 'params_file': LaunchConfiguration('nav_config'),
            'use_sim_time': 'false', 'use_composition': 'False', 'use_respawn': 'False'}),
        include(os.path.join(share, 'launch', 'navigation.launch.py'), {
            'nav_config': LaunchConfiguration('nav_config'),
            'navigation_profile': LaunchConfiguration('navigation_profile')}),
        Node(package='ee3033_project', executable='perception', output='screen', parameters=[
            config, LaunchConfiguration('vision_config'), {'target_class': target,
                     'max_report_sigma': ParameterValue(LaunchConfiguration('preferred_target_sigma'),value_type=float),
                     'calibration_file': ParameterValue(LaunchConfiguration('calibration_file'), value_type=str),
                     'model': ParameterValue(LaunchConfiguration('model'), value_type=str),
                     'localization_method': ParameterValue(LaunchConfiguration('localization_method'),
                                                          value_type=str),
                     'attitude_config': ParameterValue(LaunchConfiguration('attitude_config'),
                                                      value_type=str)}]),
        Node(package='ee3033_project', executable='target_filter', output='screen', parameters=[config, {'target_class':target}]),
        Node(package='ee3033_project', executable='velocity_gate', output='screen', parameters=[
            config, {'allow_motion': motion, 'navigation_profile': LaunchConfiguration('navigation_profile')},geometry_overrides]),
        Node(package='ee3033_project', executable='mission', output='screen', parameters=[
            config, {'target_class': target, 'allow_motion': motion,
                     'results_directory':LaunchConfiguration('results_directory'),
                     'map_partial':ParameterValue(LaunchConfiguration('map_partial'),value_type=bool),
                     'max_target_variance':ParameterValue(PythonExpression(['float(', LaunchConfiguration('preferred_target_sigma'), ') ** 2']),value_type=float),
                     'enable_ab_maneuver':ParameterValue(LaunchConfiguration('enable_ab_maneuver'),value_type=bool),
                     'acquisition_seconds':ParameterValue(LaunchConfiguration('acquisition_seconds'),value_type=float),
                     'navigation_profile': LaunchConfiguration('navigation_profile'),
                     'return_tolerance':ParameterValue(LaunchConfiguration('return_tolerance'),value_type=float)}]),
        Node(package='ee3033_project',executable='console',output='screen',
             condition=IfCondition(LaunchConfiguration('console')),parameters=[{
                 'phase':'mission','allow_motion':motion,'map_directory':LaunchConfiguration('map_directory'),
                 'port':ParameterValue(LaunchConfiguration('port'),value_type=int)}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'rviz', 'mission.rviz')]),
    ])
