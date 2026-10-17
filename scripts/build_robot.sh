#!/usr/bin/env bash
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
project_ws="${GROUP4_WORKSPACE:-/home/ros2_ws/group4_ws}"
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
mkdir -p "$project_ws"
cd "$project_ws"
colcon build --base-paths "$project_root/src" --symlink-install --packages-select ee3033_project
printf '\nSource this overlay before launching:\nsource %s/install/setup.bash\n' "$project_ws"
