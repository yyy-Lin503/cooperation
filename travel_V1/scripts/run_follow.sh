#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$task_root/scripts/ros_environment.sh"
if [[ "${1:-}" == '--chassis' ]]; then
    printf '%s\n' 'Start the chassis driver separately, then run_sensing.sh, then run_follow.sh.' \
        'This script does not start a second driver or odom writer.' >&2
    exit 2
fi
/usr/bin/python3 -B "$task_root/tools/verify_ros_overlay.py"
printf '%s\n' 'Motion controller starts now. V4 policy unchanged; no additional odom-stop gate.'
exec ros2 launch my_radar_follower radar_follow.launch.py "$@"
