from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('phase',default_value='mission'),
        DeclareLaunchArgument('allow_motion',default_value='false'),
        DeclareLaunchArgument('map_directory', default_value='~/ee3033_maps'),
        DeclareLaunchArgument('port',default_value='8765'),
        Node(package='ee3033_project',executable='console',output='screen',parameters=[{
            'map_directory':LaunchConfiguration('map_directory'),
            'phase':ParameterValue(LaunchConfiguration('phase'),value_type=str),
            'allow_motion':ParameterValue(LaunchConfiguration('allow_motion'),value_type=bool),
            'port':ParameterValue(LaunchConfiguration('port'),value_type=int)}]),
    ])
