"""One persistent operator console; it owns sequential mapping/mission launches.

No hardware is started until the operator loads a stage. Both stages wait for
Start. A failed process is never automatically restarted.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults={'allow_motion':'false','base':'true','lidar':'true','camera':'true',
              'port':'8765','map_directory':'~/ee3033_maps',
              'session_directory':'~/ee3033_results/assessment',
              'model':'/home/ros2_ws/weights/yolo26n.pt','software_version':'development',
              'calibration_file':'','calibration_directory':'~/ee3033_calibrations',
              'calibration_port':'8780','navigation_profile':'original','vision_config':''}
    params={'manage_stages':True,'phase':'mapping'}
    for name in defaults:
        key='stage_'+name if name in ('base','lidar','camera') else name
        kind=bool if name in ('allow_motion','base','lidar','camera') else int if name in ('port','calibration_port') else str
        params[key]=ParameterValue(LaunchConfiguration(name),value_type=kind)
    return LaunchDescription([
        *[DeclareLaunchArgument(name,default_value=value) for name,value in defaults.items()],
        Node(package='ee3033_project',executable='console',name='project_console',
             # The console gives its owned ROS stage up to 12 s to shut down.
             # Let it finish that cleanup before launch escalates the signal.
             sigterm_timeout='20',output='screen',parameters=[params]),
    ])
