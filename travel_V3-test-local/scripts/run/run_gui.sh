#!/usr/bin/env bash
set -euo pipefail
baseline_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$baseline_root"
export RADAR_BASELINE_ROOT="$baseline_root"
export RADAR_PROJECT_ROOT="$baseline_root"
export RADAR_ODOM_SHM_KEY="${RADAR_ODOM_SHM_KEY:-123458}"
if [[ ! -x "$baseline_root/build/jetson/radar_gui" ]]; then
    printf '%s\n' 'Run bash scripts/build/build.sh on the Jetson before starting the GUI.' >&2
    exit 1
fi
printf '%s\n' 'V8 GUI: use Start tracking to manage chassis, odometry and following in the interface.'
exec "$baseline_root/build/jetson/radar_gui" "$@"
