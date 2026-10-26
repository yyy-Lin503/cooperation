#!/usr/bin/env bash
# Receive odom without starting a motion publisher or chassis driver.
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$task_root/scripts/ros_environment.sh"
/usr/bin/python3 -B "$task_root/tools/verify_ros_overlay.py"
exec ros2 launch my_radar_follower radar_sensing.launch.py "$@"
