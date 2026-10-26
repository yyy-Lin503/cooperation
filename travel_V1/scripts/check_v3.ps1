param([string]$Cxx='g++',[string]$Python='python')
$ErrorActionPreference='Stop'
Push-Location (Split-Path -Parent $PSScriptRoot)
try {
    New-Item -ItemType Directory -Force -Path build/native-check | Out-Null
    foreach($name in @('target_tracking_match_result_test','v3_tracking_stability_test')) {
        & $Cxx -std=c++17 -O2 -DTARGET_TRACKING_ENABLE_TEST_ACCESS -I. "tests/$name.cpp" target_tracking.cpp trajectory_association.cpp -o "build/native-check/$name.exe"
        if($LASTEXITCODE -ne 0){throw "Build failed: $name"}
        & "build/native-check/$name.exe"
        if($LASTEXITCODE -ne 0){throw "Test failed: $name"}
    }
    & $Cxx -std=c++17 -O2 -I. tests/offline_odom_replay_test.cpp offline_odom_replay.cpp -lstdc++fs -o build/native-check/odom.exe
    if($LASTEXITCODE -ne 0){throw 'Odom build failed'}
    & '.\build\native-check\odom.exe' fixture_capture.bin tests/fixtures/odom
    if($LASTEXITCODE -ne 0){throw 'Odom test failed'}
    & $Cxx -std=c++17 -O2 -I. tests/offline_replay_boundary_test.cpp offline_odom_replay.cpp -lstdc++fs -o build/native-check/boundary.exe
    if($LASTEXITCODE -ne 0){throw 'Boundary build failed'}
    & '.\build\native-check\boundary.exe' tests/fixtures/odom/boundary.csv
    if($LASTEXITCODE -ne 0){throw 'Boundary test failed'}
    & $Cxx -std=c++17 -O3 -DRADAR_ENABLE_CUDA=0 -I. tests/frame_layout_doppler_test.cpp signal_processing.cpp fft.cpp CFAR.cpp DoA.cpp -lstdc++fs -o build/native-check/doppler.exe
    if($LASTEXITCODE -ne 0){throw 'IQ/Doppler build failed'}
    & '.\build\native-check\doppler.exe'
    if($LASTEXITCODE -ne 0){throw 'IQ/Doppler test failed'}
    & $Python -X utf8 -B tests/test_txt_tracker_bridge_protocol.py
    if($LASTEXITCODE -ne 0){throw 'Protocol test failed'}
} finally {Pop-Location}
