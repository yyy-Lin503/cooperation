#!/usr/bin/env bash
set -eo pipefail
baseline_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_setup="${RADAR_ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "$ros_setup" ]]; then printf 'Missing ROS2 setup: %s\n' "$ros_setup" >&2; exit 1; fi
source "$ros_setup"
mkdir -p "$baseline_root/ros2_ws"
cd "$baseline_root/ros2_ws"
colcon --log-base log build --base-paths "$baseline_root/ros2_ws/src" --packages-select my_radar_follower \
    --symlink-install --build-base build --install-base install
source "$baseline_root/ros2_ws/install/setup.bash"
export RADAR_PROJECT_ROOT="$baseline_root"
/usr/bin/python3 -B "$baseline_root/scripts/runtime/verify_ros_overlay.py"
