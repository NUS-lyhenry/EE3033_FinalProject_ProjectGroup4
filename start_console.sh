#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
source /home/ros2_ws/group4_workspaces/attitude_source_20261028_v1/install/setup.bash
source /home/ros2_ws/group4_workspaces/merged_single_frame_wall_tip_20261101_513edd1751/install/setup.bash
export ROS_DOMAIN_ID=6 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export EE3033_ROBOT_GEOMETRY=measured OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
merged_session=/home/ros2_ws/group4_results/assessment_merged_single_frame_wall_tip_20261101_513edd1751
mkdir -p "$merged_session"
exec >>"$merged_session/console.log" 2>&1
printf '%s\n' "$$" > "$merged_session/console.pid"
exec ros2 launch ee3033_project assessment.launch.py allow_motion:=true navigation_profile:=original mapping_resolution:=.05 map_directory:=/home/ros2_ws/group4_maps session_directory:="$merged_session" vision_config:="$merged_session/vision.yaml" software_version:=merged_single_frame_wall_tip_20261101_513edd1751
