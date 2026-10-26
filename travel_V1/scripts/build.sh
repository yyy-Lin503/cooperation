#!/usr/bin/env bash
set -euo pipefail
baseline_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${CUDACXX:-}" && -x /usr/local/cuda/bin/nvcc ]]; then
    export CUDACXX=/usr/local/cuda/bin/nvcc
fi
cmake -S "$baseline_root" -B "$baseline_root/build-v8-jetson" -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build "$baseline_root/build-v8-jetson" --parallel "${RADAR_BUILD_JOBS:-2}"
ctest --test-dir "$baseline_root/build-v8-jetson" --output-on-failure
