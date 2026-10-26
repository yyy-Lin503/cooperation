#!/usr/bin/env bash
# Private GUI helper: keep argv structured and keep stdout for JSON events.
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$task_root/scripts/ros_environment.sh"
/usr/bin/python3 -B "$task_root/tools/verify_ros_overlay.py" >&2
exec /usr/bin/python3 -B -m my_radar_follower.gui_runtime "$@"
