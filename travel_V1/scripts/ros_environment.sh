#!/usr/bin/env bash
# Source this file, do not execute it to configure a parent shell.
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "${RADAR_ROS_SETUP:-/opt/ros/humble/setup.bash}"
driver_setup="${WHEELTEC_SETUP:-$HOME/xiaofang/install/setup.bash}"
if [[ -f "$driver_setup" ]]; then source "$driver_setup"; fi
own_setup="$task_root/ros2_ws/v8-install/setup.bash"
if [[ ! -f "$own_setup" ]]; then printf 'Missing %s; run build_ros2.sh in this project.\n' "$own_setup" >&2; return 1; fi
source "$own_setup"
export RADAR_PROJECT_ROOT="$task_root"
export RADAR_BASELINE_ROOT="$task_root"
export RADAR_TRACK_DIR="$task_root/track_txt"
export RADAR_ODOM_SHM_KEY="${RADAR_ODOM_SHM_KEY:-123458}"
export PYTHONDONTWRITEBYTECODE=1
cd "$task_root"
