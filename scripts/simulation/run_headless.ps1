# SPDX-License-Identifier: Apache-2.0
# Controlled native MP API cohort; does not start the Mission Planner application.
param([string]$LinuxWorkspace = '/home/youssef/nomad-simulation', [switch]$Rendered, [switch]$LastLetter,
    [switch]$Visible, [switch]$Preview,
    [ValidateRange(0,1800)][int]$DurationSeconds=0,
    [ValidateSet('ground','photo-board','natural')][string]$Scene='ground',
    [ValidateSet('Area','Detect','Occlusion','Object')][string]$VideoCase='Area',
    [ValidateSet(400,1000)][int]$PhysicsHz=1000,
    [ValidateSet('orbit','flyby','direct','approach')][string]$ObjectMotion='orbit',
    [ValidateSet(5,15)][int]$ObjectAltitude=5, [ValidateSet(5,10)][int]$ObjectSpeed=5,
    [ValidateRange(100,50000)][int]$EncoderKbps=800)
$ErrorActionPreference = 'Stop'
if ($Rendered -and $LastLetter) { throw 'Only one flight dynamics backend may own a run.' }
if (!$Rendered -and $PhysicsHz -ne 1000) { throw 'A nondefault physics rate requires rendered Gazebo.' }
if (!$Rendered -and $VideoCase -ne 'Area') { throw 'Video cases need rendered video.' }
if ($VideoCase -eq 'Object' -and ($Scene -eq 'photo-board' -or $DurationSeconds -or $PhysicsHz -ne 1000)) {
    throw 'Object motion requires ground/natural scenery,1000 Hz physics and no endurance mode.'
}
if ($Preview -and !$Visible) { throw 'Preview requires visible windows.' }
if ($DurationSeconds -ne 0 -and (!$Rendered -or $VideoCase -ne 'Area' -or $Preview -or $DurationSeconds -lt 600)) {
    throw 'Endurance requires rendered Area,600..1800 seconds,without Preview.'
}
$wholeClock = [Diagnostics.Stopwatch]::StartNew()
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
Add-Type -Path (Join-Path $root 'mission_planner/src/Simulation/OwnedProcessJob.cs')
if (Get-Process MissionPlanner -ErrorAction SilentlyContinue) { throw 'An existing MP process is running.' }
if (Get-NetTCPConnection -LocalPort $(if ($Rendered) { @(15760,15761) } else { 15760 }) -State Listen -ErrorAction SilentlyContinue) {
    throw 'The reserved Windows loopback port is already owned.'
}
if ($Rendered -and (Get-NetUDPEndpoint -LocalPort 15600 -ErrorAction SilentlyContinue)) {
    throw 'The reserved Windows video port is already owned.'
}
$preserved = @{
    host_config='C:\Program Files (x86)\Mission Planner\MissionPlanner.exe.config';
    network_trace='C:\ProgramData\Mission Planner\network.log';
    host_log='C:\ProgramData\Mission Planner\MissionPlanner.log';
    operator_config=(Join-Path $env:USERPROFILE 'Documents/Mission Planner/config.xml');
    operator_profiles=(Join-Path $env:LOCALAPPDATA 'Mission Planner/plugins/nomad_aircraft_profiles.json');
    operator_backup=(Join-Path $env:LOCALAPPDATA 'Mission Planner/plugins/nomad_aircraft_profiles.json.bak')
}
$before = @{}
foreach ($name in $preserved.Keys) {
    $before[$name] = if (Test-Path -LiteralPath $preserved[$name]) {
        (Get-FileHash -LiteralPath $preserved[$name]).Hash
    } else { 'missing' }
}
$run = & (Join-Path $PSScriptRoot 'prepare_host.ps1') -Media:$Rendered
$job = [NOMAD.Simulation.OwnedProcessJob]::new()
$supervisor = $null
$fixture = $null
$result = @{schema=1; passed=$false; stage='linux_start'; preservation=@{}}
if ($Rendered) { $result.physics_hz = $PhysicsHz }
try {
    $linuxRun = & wsl -d Ubuntu-24.04 --exec wslpath -a $run
    if ($LASTEXITCODE -ne 0) { throw 'Run directory is not reachable through WSL.' }
    $linuxScript = & wsl -d Ubuntu-24.04 --exec wslpath -a (Join-Path $PSScriptRoot 'supervise.py')
    $start = New-Object Diagnostics.ProcessStartInfo
    $start.FileName = 'wsl.exe'
    $start.Arguments = '-d Ubuntu-24.04 --exec python3 "' + $linuxScript + '" --workspace "' +
        $LinuxWorkspace + '" --run "' + $linuxRun + '"'
    if ($Rendered) {
        $start.Arguments += ' --rendered --scene ' + $Scene + ' --physics-hz ' + $PhysicsHz +
            ' --encoder-kbps ' + $EncoderKbps
    }
    if ($Visible) { $start.Arguments += ' --visible' }
    if ($VideoCase -eq 'Occlusion') { $start.Arguments += ' --occlusion' }
    if ($VideoCase -eq 'Object') {
        $start.Arguments += ' --object-motion ' + $ObjectMotion + ' --object-altitude ' + $ObjectAltitude +
            ' --object-speed ' + $ObjectSpeed
    }
    if ($DurationSeconds) { $start.Arguments += ' --endurance-seconds ' + $DurationSeconds }
    if ($LastLetter) { $start.Arguments += ' --last-letter' }
    $start.UseShellExecute = $false
    $start.CreateNoWindow = $true
    $start.RedirectStandardInput = $true
    $start.RedirectStandardOutput = $true
    $start.RedirectStandardError = $true
    $supervisor = [Diagnostics.Process]::Start($start)
    $job.Attach($supervisor)
    $errors = $supervisor.StandardError.ReadToEndAsync()
    $ready = $supervisor.StandardOutput.ReadLineAsync()
    if (!$ready.Wait($(if ($Rendered) { 95000 } else { 25000 })) -or $ready.Result -ne 'READY') {
        throw 'Owned SITL/renderer readiness failed.'
    }
    $result.stage = 'native_fixture'
    $native = New-Object Diagnostics.ProcessStartInfo
    $native.FileName = Join-Path $run 'host/VideoLifetimeTests.exe'
    $mode = if ($VideoCase -eq 'Occlusion') { '--occlusion' } elseif ($VideoCase -eq 'Detect') { '--detect' } `
        elseif ($Rendered) { '--rendered' } else { '--native' }
    $native.Arguments = $mode + ' "' + $run + '"'
    $native.WorkingDirectory = Join-Path $run 'host'
    $native.UseShellExecute = $false
    $native.CreateNoWindow = $true
    $native.RedirectStandardOutput = $true
    $native.RedirectStandardError = $true
    $native.EnvironmentVariables['NOMAD_SIMULATION_VISIBLE'] = $(if ($Visible) { '1' } else { '0' })
    $native.EnvironmentVariables['NOMAD_SIMULATION_PREVIEW'] = $(if ($Preview) { '1' } else { '0' })
    $native.EnvironmentVariables['NOMAD_SIMULATION_DURATION'] = $DurationSeconds.ToString()
    if ($Rendered) {
        $prefix = Join-Path $run 'host/media/gstreamer/1.0/x86_64'
        $temp = Join-Path $run 'temp'
        New-Item -ItemType Directory -Path $temp | Out-Null
        foreach ($entry in @{
            GST_PLUGIN_PATH_1_0=(Join-Path $prefix 'lib/gstreamer-1.0'); GST_PLUGIN_SYSTEM_PATH_1_0='';
            GST_REGISTRY_1_0=(Join-Path $run 'gst-registry.bin');
            GST_PLUGIN_SCANNER_1_0=(Join-Path $prefix 'libexec/gstreamer-1.0/gst-plugin-scanner.exe');
            TEMP=$temp; TMP=$temp
            GST_DEBUG='2'; GST_DEBUG_FILE=(Join-Path $run 'gst-debug.log')
        }.GetEnumerator()) { $native.EnvironmentVariables[$entry.Key] = $entry.Value }
    }
    $fixture = [Diagnostics.Process]::Start($native)
    $job.Attach($fixture)
    $nativeOutput = $fixture.StandardOutput.ReadToEndAsync()
    $nativeErrors = $fixture.StandardError.ReadToEndAsync()
    $nativeDeadline = if ($DurationSeconds) { ($DurationSeconds + 120) * 1000 } elseif ($Preview) {270000} `
        elseif ($VideoCase -eq 'Object') {150000} elseif ($Rendered) {90000} else {60000}
    $nativeDeadline = [Math]::Min($nativeDeadline, [Math]::Max(1, 2400000 - $wholeClock.ElapsedMilliseconds))
    if (!$fixture.WaitForExit([int]$nativeDeadline)) {
        throw 'Controlled native API deadline exceeded.'
    }
    if ($fixture.ExitCode -ne 0) { throw 'Controlled native API fixture failed.' }
    $result.passed = $true
} catch {
    $result.failure = $_.Exception.Message
} finally {
    try {
    $cleanupClock = [Diagnostics.Stopwatch]::StartNew()
    if ($fixture -and !$fixture.HasExited) { $fixture.Kill(); $fixture.WaitForExit() }
    if ($nativeOutput -and $nativeOutput.Wait(3000)) {
        $nativeOutput.Result | Set-Content -LiteralPath (Join-Path $run 'native-console.log')
    }
    if ($nativeErrors -and $nativeErrors.Wait(3000)) {
        $nativeErrors.Result | Set-Content -LiteralPath (Join-Path $run 'native-error.log')
    }
    if ($supervisor -and !$supervisor.HasExited) {
        try {
            $supervisor.StandardInput.WriteLine('STOP')
            $supervisor.StandardInput.Close()
        } catch {
            $result.passed = $false; $result.cleanup_error = 'Supervisor STOP pipe failed: ' + $_.Exception.Message
        }
        if (!$supervisor.WaitForExit(10000)) {
            $result.passed = $false; $result.cleanup_error = 'Linux supervisor deadline exceeded.'
        }
    }
    if ($errors -and $errors.Wait(3000) -and $errors.Result) {
        $errors.Result | Set-Content -LiteralPath (Join-Path $run 'supervisor-error.log')
    }
    foreach ($name in $preserved.Keys) {
        $after = if (Test-Path -LiteralPath $preserved[$name]) {
            (Get-FileHash -LiteralPath $preserved[$name]).Hash
        } else { 'missing' }
        $result.preservation[$name] = $after -eq $before[$name]
        if (!$result.preservation[$name]) { $result.passed = $false }
    }
    $linuxPath = Join-Path $run 'linux.json'
    $linuxResult = if (Test-Path -LiteralPath $linuxPath) { Get-Content -LiteralPath $linuxPath -Raw | ConvertFrom-Json }
    $result.simulator_passed = $linuxResult -and $linuxResult.passed
    $result.simulator_cleanup = $linuxResult -and $linuxResult.owned_process_exited -and
        @($linuxResult.cleanup_errors).Count -eq 0
    if ($Rendered -and !$linuxResult.renderer_exited) { $result.simulator_cleanup = $false }
    if ($Rendered -and $Visible -and !$linuxResult.gui_exited) { $result.simulator_cleanup = $false }
    if ($LastLetter -and !$linuxResult.last_letter_exited) { $result.simulator_cleanup = $false }
    if (!$result.simulator_cleanup -or !$result.simulator_passed) { $result.passed = $false }
    $result.cleanup_seconds = $cleanupClock.Elapsed.TotalSeconds
    if ($result.cleanup_seconds -gt 10) { $result.passed = $false }
    if ($Rendered) {
        $result.oracle_passed = $false
        if (Test-Path -LiteralPath (Join-Path $run 'video-frames.jsonl')) {
            $oracleScript = & wsl -d Ubuntu-24.04 --exec wslpath -a (Join-Path $PSScriptRoot 'analyze_rendered.py')
            $oracleStart = New-Object Diagnostics.ProcessStartInfo
            $oracleStart.FileName = 'wsl.exe'
            $oracleStart.Arguments = '-d Ubuntu-24.04 --exec timeout --signal=TERM --kill-after=2 55 python3 "' +
                $oracleScript + '" "' + $linuxRun + '"'
            $oracleStart.UseShellExecute = $false; $oracleStart.CreateNoWindow = $true
            $oracleStart.RedirectStandardOutput = $true; $oracleStart.RedirectStandardError = $true
            $oracle = [Diagnostics.Process]::Start($oracleStart)
            $job.Attach($oracle)
            $oracleOutput = $oracle.StandardOutput.ReadToEndAsync()
            $oracleErrors = $oracle.StandardError.ReadToEndAsync()
            if ($oracle.WaitForExit(70000)) { $result.oracle_passed = $oracle.ExitCode -eq 0 }
            else { $oracle.Kill(); $oracle.WaitForExit(); $result.oracle_error = 'Independent oracle deadline exceeded.' }
            if ($oracleOutput.Wait(3000)) {
                $oracleOutput.Result | Set-Content -LiteralPath (Join-Path $run 'oracle-console.log')
            }
            if ($oracleErrors.Wait(3000) -and $oracleErrors.Result) {
                $oracleErrors.Result | Set-Content -LiteralPath (Join-Path $run 'oracle-error.log')
            }
        }
        if (!$result.oracle_passed) { $result.passed = $false }
    }
    if ($DurationSeconds) {
        $resourceScript = & wsl -d Ubuntu-24.04 --exec wslpath -a (Join-Path $PSScriptRoot 'resources.py')
        & wsl -d Ubuntu-24.04 --exec timeout 20 python3 $resourceScript --endurance $linuxRun $DurationSeconds `
            *> (Join-Path $run 'endurance-console.log')
        $result.endurance_passed = $LASTEXITCODE -eq 0
        $result.endurance_analysis_exit_code = $LASTEXITCODE
        if (!$result.endurance_passed) { $result.passed = $false }
    }
    $result.duration_seconds = $DurationSeconds
    $result.total_seconds = $wholeClock.Elapsed.TotalSeconds
    if ($wholeClock.Elapsed.TotalMinutes -gt 40) { $result.passed = $false }
    $result | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $run 'result.json') -Encoding UTF8
    } finally { $job.Dispose() }
}
if (!$result.passed) { throw "Controlled native cohort failed: $run/result.json" }
Write-Output "PASS controlled native MP/SITL cohort: $run"
