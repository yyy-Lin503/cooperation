param([string]$Cxx = 'g++')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    New-Item -ItemType Directory -Force -Path 'build/replay' | Out-Null
    & $Cxx -std=c++17 -O3 -DRADAR_ENABLE_CUDA=0 -I. tools/replay_radar.cpp signal_processing.cpp fft.cpp CFAR.cpp DoA.cpp iwr6843param.cpp offline_odom_replay.cpp target_tracking.cpp trajectory_association.cpp -lstdc++fs -o build/replay/replay_radar.exe
    if ($LASTEXITCODE -ne 0) { throw 'CPU replay build failed' }
    & $Cxx -std=c++17 -O3 -DRADAR_ENABLE_CUDA=0 -DRADAR_REPLAY_USE_BASELINE_TRACKER=1 -include signal_processing.hpp -I. tools/replay_radar.cpp signal_processing.cpp fft.cpp CFAR.cpp DoA.cpp iwr6843param.cpp offline_odom_replay.cpp reference/YDS-V12/target_tracking.cpp reference/YDS-V12/trajectory_association.cpp -lstdc++fs -o build/replay/replay_v1.exe
    if ($LASTEXITCODE -ne 0) { throw 'Baseline replay build failed' }
    & $Cxx -std=c++17 -O3 -DRADAR_ENABLE_CUDA=0 -I. bin2pointcloud.cpp signal_processing.cpp fft.cpp CFAR.cpp DoA.cpp iwr6843param.cpp offline_odom_replay.cpp -lstdc++fs -o build/replay/bin2pointcloud.exe
    if ($LASTEXITCODE -ne 0) { throw 'CPU converter build failed' }
} finally { Pop-Location }
