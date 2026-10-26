# Exercise launcher argument forwarding and owned-process cleanup without WSL,
# administrator access, a camera, or opening a GUI.
$ErrorActionPreference = 'Stop'
$script:projectRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$script:launch = Join-Path $script:projectRoot 'scripts/run/run_gui_wsl.ps1'
$global:cameraLauncherCommands = @()
$global:cameraLauncherProcesses = @()
$global:cameraLauncherFailStartup = $false
function Check($condition, $message) { if (-not $condition) { throw $message } }
function wsl.exe {
    $global:cameraLauncherCommands += ,@($args)
    $global:LASTEXITCODE = 0
    if ($args -contains 'wslinfo') { return 'nat' }
    if ($args -contains '/proc/net/route') { return "eth0`t00000000`t010010AC`t0003" }
    if ($args -contains 'pwd') { return '/mnt/c/current-project' }
}
function Start-Process {
    param($FilePath, $ArgumentList, $WindowStyle, [switch]$PassThru, $RedirectStandardOutput, $RedirectStandardError)
    Check ($WindowStyle -eq 'Hidden') 'Bridge opened a visible console'
    Check ($ArgumentList -contains '--camera-index') 'Camera index was not forwarded'
    Check ($ArgumentList -contains 2) 'Requested camera index was lost'
    $position = [Array]::IndexOf($ArgumentList, '--ready-file')
    $readyPath = $ArgumentList[$position + 1].Trim('"')
    $stopPosition = [Array]::IndexOf($ArgumentList, '--stop-file')
    $stopPath = $ArgumentList[$stopPosition + 1].Trim('"')
    [IO.File]::WriteAllText($RedirectStandardOutput, '')
    [IO.File]::WriteAllText($RedirectStandardError, $(if ($global:cameraLauncherFailStartup) { 'disabled camera fixture' } else { '' }))
    if (-not $global:cameraLauncherFailStartup) {
        [IO.File]::WriteAllText($readyPath, '{"port":32123,"width":640,"height":480,"backend":"FAKE"}')
    }
    $process = [pscustomobject]@{ HasExited = $global:cameraLauncherFailStartup; Id = -1; Disposed = $false; StopPath = $stopPath; Stopped = $false }
    $process | Add-Member ScriptMethod Refresh { }
    $process | Add-Member ScriptMethod WaitForExit { param($timeout) $this.Stopped = Test-Path -LiteralPath $this.StopPath; return $true }
    $process | Add-Member ScriptMethod Dispose { $this.Disposed = $true }
    $global:cameraLauncherProcesses += $process
    return $process
}

& $script:launch -LocalCamera -CameraIndex 2 -Distribution 'Fixture-Ubuntu' -RepairWslg
Check ($LASTEXITCODE -eq 0) 'Successful launcher did not preserve exit status'
$invocation = $global:cameraLauncherCommands[-1]
Check ($invocation -contains 'RADAR_CAMERA_DEVICE=tcp://172.16.0.1:32123') ("NAT bridge address was not forwarded: " + ($invocation -join '|'))
Check ($invocation -contains '--repair-wslg') 'WSLg argument was lost'
Check ($invocation -contains 'Fixture-Ubuntu') 'Distribution argument was lost'
Check ($invocation -contains '--exec') 'Launcher used shell reparsing'
Check ($global:cameraLauncherProcesses[-1].Disposed -and $global:cameraLauncherProcesses[-1].Stopped) 'Bridge was not stopped and disposed'
Check (-not (Test-Path -LiteralPath (Split-Path -Parent $global:cameraLauncherProcesses[-1].StopPath))) 'Temporary bridge files remain'

$before = $global:cameraLauncherProcesses.Count
& $script:launch -Distribution 'Fixture-Ubuntu'
Check ($global:cameraLauncherProcesses.Count -eq $before) 'Normal launch unexpectedly opened Windows camera'
Check (-not ($global:cameraLauncherCommands[-1] -match '^RADAR_CAMERA_DEVICE=')) 'Normal launch forced bridge input'

$global:cameraLauncherFailStartup = $true
try {
    & $script:launch -LocalCamera -CameraIndex 2
    throw 'Expected camera startup to fail'
}
catch {
    Check ($_.ToString() -like '*disabled camera fixture*') 'Camera failure diagnostics were lost'
}
Check ($global:cameraLauncherProcesses[-1].Disposed -and $global:cameraLauncherProcesses[-1].Stopped) 'Failed startup leaked its bridge process'
Check (-not (Test-Path -LiteralPath (Split-Path -Parent $global:cameraLauncherProcesses[-1].StopPath))) 'Failed startup leaked temporary files'
Write-Output 'PASS: launcher NAT address, arguments, normal launch, startup failure and owned cleanup'
