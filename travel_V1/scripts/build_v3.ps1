param([string]$Cxx='g++')
$ErrorActionPreference='Stop'
$taskRoot=Split-Path -Parent $PSScriptRoot
Push-Location $taskRoot
try {
    New-Item -ItemType Directory -Force -Path build | Out-Null
    $shared=@('signal_processing.cpp','fft.cpp','CFAR.cpp','DoA.cpp','iwr6843param.cpp','offline_odom_replay.cpp')
    & $Cxx -std=c++17 -O3 -DRADAR_ENABLE_CUDA=0 -I. tools/replay_radar.cpp @shared target_tracking.cpp trajectory_association.cpp -lstdc++fs -o build/replay_v3.exe
    if($LASTEXITCODE -ne 0){throw 'V3 build failed'}
    & $Cxx -std=c++17 -O3 -DRADAR_ENABLE_CUDA=0 -DRADAR_REPLAY_USE_V2_TRACKER=1 -include signal_processing.hpp -I. tools/replay_radar.cpp @shared reference/V2-current/target_tracking.cpp reference/V2-current/trajectory_association.cpp -lstdc++fs -o build/replay_v2.exe
    if($LASTEXITCODE -ne 0){throw 'V2 comparator build failed'}
    & $Cxx -std=c++17 -O3 -DRADAR_ENABLE_CUDA=0 -DRADAR_REPLAY_USE_BASELINE_TRACKER=1 -include signal_processing.hpp -I. tools/replay_radar.cpp @shared reference/YDS-V12/target_tracking.cpp reference/YDS-V12/trajectory_association.cpp -lstdc++fs -o build/replay_v1.exe
    if($LASTEXITCODE -ne 0){throw 'V1 comparator build failed'}
    & $Cxx -std=c++17 -O3 -DRADAR_ENABLE_CUDA=0 -I. bin2pointcloud.cpp @shared -lstdc++fs -o build/bin2pointcloud.exe
    if($LASTEXITCODE -ne 0){throw 'Converter build failed'}
} finally {Pop-Location}
