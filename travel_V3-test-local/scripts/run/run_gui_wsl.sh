#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
wslg_mode() {
    sed -n 's/.*RDP backend: use_gfxredir = //p' /mnt/wslg/weston.log | tail -n 1
}
if [[ "${1:-}" == "--repair-wslg" ]]; then
    shift
    # Keep this Linux shell alive across the restart. Otherwise WSL may stop
    # when the last GUI disconnects, undoing the repaired graphics session.
    wsl.exe -d "${WSL_DISTRO_NAME:?Run this script inside WSL2}" --system -- sh -lc 'pkill -TERM -x weston'
    display_ready=false
    for ((attempt=0; attempt<30; attempt++)); do
        sleep 1
        if [[ "$(wslg_mode)" == "1" ]] && xdpyinfo >/dev/null 2>&1; then
            display_ready=true
            break
        fi
    done
    if [[ "$display_ready" != true ]]; then
        printf '%s\n' 'WSLg display did not recover within 30 seconds.' >&2
        exit 1
    fi
fi
if [[ -r /mnt/wslg/weston.log ]] && [[ "$(wslg_mode)" == "0" ]]; then
    printf '%s\n' 'WSLg is in COPY MODE: windows may be invisible. Close other Linux GUI apps, then rerun run_gui_wsl.ps1 with -RepairWslg.' >&2
    exit 1
fi
# Keep WSL binaries separate from the ARM/Jetson build shipped with the project.
build_dir="${RADAR_WSL_BUILD_DIR:-$project_root/build/wsl}"
cmake -S "$project_root" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_RADAR_APP=ON -DRADAR_ENABLE_CUDA=OFF -DRADAR_ENABLE_CAMERA=ON
cmake --build "$build_dir" --target radar_gui --parallel "${RADAR_BUILD_JOBS:-2}"
cd "$project_root"
export RADAR_PROJECT_ROOT="$project_root"
export RADAR_BASELINE_ROOT="$project_root"
exec "$build_dir/radar_gui" "$@"
