#!/usr/bin/env bash
set -eo pipefail
for package in balance_robot_base balance_usb_cam lslidar_driver_n10p nav2_bringup \
               nav2_msgs nav2_map_server slam_toolbox cv_bridge tf2_ros; do
  ros2 pkg prefix "$package" >/dev/null
done
python3 -c 'import rclpy, cv2, scipy, numpy, torch, ultralytics; print("Python dependencies available")'
test -r "${GROUP4_MODEL:-/home/ros2_ws/weights/yolo26n.pt}"
printf 'ROS packages and YOLO weights available. No motor commands were sent.\n'
