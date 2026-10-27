#!/usr/bin/env bash
# A new directory is allocated for every save. Existing lab maps are never targets.
set -euo pipefail
map_root="${1:-$PWD/maps}"
mkdir -p "$map_root"
map_root="$(cd -- "$map_root" && pwd)"
map_dir="$(mktemp -d "$map_root/onsite_$(date -u +%Y%m%dT%H%M%SZ)_XXXXXX")"
ros2 run nav2_map_server map_saver_cli -f "$map_dir/map" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true \
  -p free_thresh_default:=0.196 -p occupied_thresh_default:=0.65
test -s "$map_dir/map.yaml"
test -s "$map_dir/map.pgm"
printf '\nSaved new map: %s/map.yaml\n' "$map_dir"
