import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('ee3033_project')
    return LaunchDescription([
        DeclareLaunchArgument('allow_motion', default_value='false'),
        DeclareLaunchArgument('base', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('console', default_value='true'),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('nav_config', default_value=os.path.join(share,'config','navigation.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'hardware.launch.py')), launch_arguments={
                'base': LaunchConfiguration('base'), 'lidar': LaunchConfiguration('lidar'),
                'camera': 'false'}.items()),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', parameters=[os.path.join(share, 'config', 'slam.yaml')],
             output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share,'launch','navigation.launch.py')),launch_arguments={
                'nav_config':LaunchConfiguration('nav_config')}.items()),
        Node(package='ee3033_project',executable='mapper',output='screen',parameters=[{
            'allow_motion':ParameterValue(LaunchConfiguration('allow_motion'),value_type=bool)}]),
        Node(package='ee3033_project', executable='velocity_gate', parameters=[
            os.path.join(share, 'config', 'project.yaml'), {'mapping_mode': True,
            'allow_motion': ParameterValue(LaunchConfiguration('allow_motion'), value_type=bool)}],
             output='screen'),
        Node(package='ee3033_project',executable='console',output='screen',
             condition=IfCondition(LaunchConfiguration('console')),parameters=[{
                 'phase':'mapping',
                 'port':ParameterValue(LaunchConfiguration('port'),value_type=int),
                 'allow_motion':ParameterValue(LaunchConfiguration('allow_motion'),value_type=bool)}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'rviz', 'mission.rviz')]),
    ])
