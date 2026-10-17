import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ee3033_project')
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'mission.rviz')]),
        Node(package='rqt_image_view', executable='rqt_image_view',
             arguments=['/project/detection_image']),
    ])
