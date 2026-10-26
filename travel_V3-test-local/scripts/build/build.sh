#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
profile="${1:-jetson}"
if [[ $# -gt 0 && "$1" != -D* ]]; then shift; else profile=jetson; fi
case "$profile" in
  cpu) gui=OFF; cuda=OFF; camera=OFF ;;
  wsl) gui=ON; cuda=OFF; camera=ON ;;
  jetson) gui=ON; cuda=ON; camera=ON ;;
  *) printf 'Unknown build profile: %s (cpu/wsl/jetson)\n' "$profile" >&2; exit 2 ;;
esac
if [[ -z "${CUDACXX:-}" && -x /usr/local/cuda/bin/nvcc ]]; then export CUDACXX=/usr/local/cuda/bin/nvcc; fi
build_dir="$project_root/build/$profile"
cmake -S "$project_root" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release -DBUILD_RADAR_APP="$gui" -DRADAR_ENABLE_CUDA="$cuda" -DRADAR_ENABLE_CAMERA="$camera" "$@"
cmake --build "$build_dir" --parallel "${RADAR_BUILD_JOBS:-2}"
ctest --test-dir "$build_dir" --output-on-failure
