param(
    [string]$Distribution = 'TravelMate-Ubuntu',
    [switch]$RepairWslg,
    [switch]$LocalCamera,
    [ValidateRange(0, 99)][int]$CameraIndex = 0
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$guiArguments = @()
if ($RepairWslg) { $guiArguments += '--repair-wslg' }
$bridgeProcess = $null
$sessionDir = $null
$exitCode = 1
try {
    $linuxCommand = @('bash', 'scripts/run/run_gui_wsl.sh') + $guiArguments
    if ($LocalCamera) {
        $python = (Get-Command python.exe -ErrorAction Stop).Source
        $bridgeHost = '127.0.0.1'
        $networkMode = (& wsl.exe -d $Distribution --exec wslinfo --networking-mode 2>$null | Out-String).Trim()
        if ($networkMode -ne 'mirrored') {
            # /proc is available even in minimal distributions without iproute2.
            $routes = & wsl.exe -d $Distribution --exec cat /proc/net/route
            if ($LASTEXITCODE -ne 0) { throw 'Cannot read WSL network routes.' }
            $defaultRoute = $routes | Where-Object { ($_ -split '\s+')[1] -eq '00000000' } | Select-Object -First 1
            if (-not $defaultRoute) { throw 'Cannot determine the Windows host address from WSL.' }
            $gateway = ($defaultRoute -split '\s+')[2]
            $bridgeHost = ((6, 4, 2, 0 | ForEach-Object { [Convert]::ToInt32($gateway.Substring($_, 2), 16) }) -join '.')
        }
        $sessionDir = Join-Path ([IO.Path]::GetTempPath()) ('travelmate-camera-' + [guid]::NewGuid().ToString('N'))
        [IO.Directory]::CreateDirectory($sessionDir) | Out-Null
        $readyFile = Join-Path $sessionDir 'ready.json'
        $stopFile = Join-Path $sessionDir 'stop'
        $stdoutFile = Join-Path $sessionDir 'stdout.log'
        $stderrFile = Join-Path $sessionDir 'stderr.log'
        $bridgeScript = Join-Path $projectRoot 'tools/diagnostics/windows_camera_bridge.py'
        $bridgeArgs = @('-B', ('"' + $bridgeScript + '"'), '--camera-index', $CameraIndex,
            '--host', $bridgeHost, '--ready-file', ('"' + $readyFile + '"'), '--stop-file', ('"' + $stopFile + '"'))
        $bridgeProcess = Start-Process -FilePath $python -ArgumentList $bridgeArgs -WindowStyle Hidden -PassThru `
            -RedirectStandardOutput $stdoutFile -RedirectStandardError $stderrFile
        $deadline = [DateTime]::UtcNow.AddSeconds(15)
        while (-not (Test-Path -LiteralPath $readyFile)) {
            $bridgeProcess.Refresh()
            if ($bridgeProcess.HasExited) {
                $details = Get-Content -LiteralPath $stderrFile -Raw -ErrorAction SilentlyContinue
                throw "Local camera startup failed. $details"
            }
            if ([DateTime]::UtcNow -gt $deadline) { throw 'Local camera startup timed out. Check that the camera is enabled and accessible.' }
            Start-Sleep -Milliseconds 100
        }
        $ready = Get-Content -LiteralPath $readyFile -Raw | ConvertFrom-Json
        $device = 'tcp://{0}:{1}' -f $bridgeHost, $ready.port
        # Confirm host/WSL reachability before spending time building the GUI.
        & wsl.exe -d $Distribution --exec python3 -B -c 'import socket,sys; socket.create_connection((sys.argv[1],int(sys.argv[2])),3).close()' $bridgeHost ([string]$ready.port)
        if ($LASTEXITCODE -ne 0) { throw 'WSL cannot reach the camera bridge. Check Windows firewall access for Python on the WSL interface.' }
        Write-Host ("Local camera {0}: {1}x{2}, {3}, {4}" -f $CameraIndex, $ready.width, $ready.height, $ready.backend, $device)
        # env applies only to this Linux launch; no persistent environment changes.
        $linuxCommand = @('env', "RADAR_CAMERA_DEVICE=$device") + $linuxCommand
        # A copied checkout can contain a cache from another project location.
        # Keep that build intact and use a fresh directory for this launch.
        $cacheFile = Join-Path $projectRoot 'build/wsl/CMakeCache.txt'
        if (Test-Path -LiteralPath $cacheFile) {
            $linuxRoot = (& wsl.exe -d $Distribution --cd $projectRoot --exec pwd | Out-String).Trim()
            $cachedRoot = Get-Content -LiteralPath $cacheFile | Where-Object { $_.StartsWith('CMAKE_HOME_DIRECTORY:INTERNAL=') }
            if ($cachedRoot -and $cachedRoot.Substring('CMAKE_HOME_DIRECTORY:INTERNAL='.Length) -ne $linuxRoot) {
                $linuxCommand = @('env', "RADAR_WSL_BUILD_DIR=$linuxRoot/build/local-camera") + $linuxCommand
            }
        }
    }
    & wsl.exe -d $Distribution --cd $projectRoot --exec @linuxCommand
    $exitCode = $LASTEXITCODE
    if ($LocalCamera -and (Test-Path -LiteralPath $stderrFile)) {
        $details = Get-Content -LiteralPath $stderrFile -Raw
        if ($details) { Write-Warning $details.Trim() }
    }
}
finally {
    if ($bridgeProcess) {
        [IO.File]::WriteAllText($stopFile, '')
        if (-not $bridgeProcess.WaitForExit(3000)) { Stop-Process -Id $bridgeProcess.Id -Force -ErrorAction SilentlyContinue }
        $bridgeProcess.Dispose()
    }
    if ($sessionDir) {
        # Only known files in our unique temporary directory; no recursive delete.
        foreach ($name in @('ready.json', 'ready.tmp', 'stop', 'stdout.log', 'stderr.log')) {
            Remove-Item -LiteralPath (Join-Path $sessionDir $name) -Force -ErrorAction SilentlyContinue
        }
        [IO.Directory]::Delete($sessionDir)
    }
}
exit $exitCode
