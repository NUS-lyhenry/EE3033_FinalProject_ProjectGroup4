#!/usr/bin/env bash
# Synthetic sensors/actions only; never use the robot's domain.
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ee3033_project
source install/setup.bash
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
python3 -m pytest -q
ROS_DOMAIN_ID=110 python3 tests/integration_visual_perception.py
ROS_DOMAIN_ID=109 python3 tests/integration_visual_search.py
ROS_DOMAIN_ID=99 python3 tests/integration_hardware_launch.py
ROS_DOMAIN_ID=94 python3 tests/integration_console.py
ROS_DOMAIN_ID=88 python3 tests/integration_ros.py
ROS_DOMAIN_ID=107 python3 tests/integration_adaptive_search.py
ROS_DOMAIN_ID=100 python3 tests/integration_stationary.py
ROS_DOMAIN_ID=106 python3 tests/integration_calibration.py
ROS_DOMAIN_ID=108 python3 tests/integration_automatic_calibration.py
ROS_DOMAIN_ID=96 python3 tests/integration_mapping_retry.py
ROS_DOMAIN_ID=93 python3 tests/integration_workflow.py
ROS_DOMAIN_ID=98 python3 tests/integration_waypoint.py
ROS_DOMAIN_ID=97 python3 tests/integration_turning_nav2.py
ROS_DOMAIN_ID=101 python3 tests/integration_recovery_nav2.py
ROS_DOMAIN_ID=92 GROUP4_GAP_RESOLUTION=.05 python3 tests/integration_gap_nav2.py
ROS_DOMAIN_ID=91 GROUP4_GAP_RESOLUTION=.01 python3 tests/integration_gap_nav2.py
