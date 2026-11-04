# SPDX-License-Identifier: Apache-2.0
# No MP application startup, network connection, simulator, or device access.
$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$run = & (Join-Path $root 'scripts/simulation/prepare_host.ps1')
if ($run -isnot [string]) { throw 'Owned host preparation returned an unexpected result.' }
$exe = Join-Path $run 'host/VideoLifetimeTests.exe'
& $exe --probe $run
if ($LASTEXITCODE -ne 0) { throw 'Owned settings probe failed.' }
$probe = Get-Content (Join-Path $run 'probe.json') -Raw | ConvertFrom-Json
if (!$probe.passed -or $probe.host_started -or $probe.native_connection_opened) {
    throw 'Probe unexpectedly started a runtime owner.'
}
foreach ($bad in @('relative-run', $root)) {
    & $exe --probe $bad
    if ($LASTEXITCODE -eq 0) { throw 'Unowned run path was accepted.' }
}
$marker = Join-Path $run 'run.id'
$original = [IO.File]::ReadAllText($marker)
try {
    [IO.File]::WriteAllText($marker, 'wrong-owner')
    & $exe --probe $run
    if ($LASTEXITCODE -eq 0) { throw 'Wrong run ownership marker was accepted.' }
} finally { [IO.File]::WriteAllText($marker, $original) }
$configPath = Join-Path $run 'host/MissionPlanner.exe.config'
$originalConfig = [IO.File]::ReadAllText($configPath)
try {
    $config = New-Object Xml.XmlDocument
    $config.Load($configPath)
    $listener = $config.SelectSingleNode('//system.diagnostics//*[@initializeData]')
    if (!$listener) { throw 'The reviewed shared network trace fixture is missing.' }
    $listener.SetAttribute('initializeData', 'C:\ProgramData\Mission Planner\network.log')
    $config.Save($configPath)
    & $exe --probe $run
    if ($LASTEXITCODE -eq 0) { throw 'An operator trace sink was accepted.' }
} finally { [IO.File]::WriteAllText($configPath, $originalConfig) }
Add-Type -Path (Join-Path $root 'mission_planner/src/Simulation/OwnedProcessJob.cs')
$job = [NOMAD.Simulation.OwnedProcessJob]::new()
$start = New-Object Diagnostics.ProcessStartInfo
$start.FileName = $exe
$start.Arguments = '--job-child "' + $run + '"'
$start.UseShellExecute = $false
$start.CreateNoWindow = $true
$child = [Diagnostics.Process]::Start($start)
try {
    $job.Attach($child)
    $job.Dispose()
    if (!$child.WaitForExit(2000)) { throw 'Owned job child survived handle closure.' }
} finally {
    $job.Dispose()
    if (!$child.HasExited) { $child.Kill(); $child.WaitForExit() }
}
Write-Output "PASS simulation bootstrap probe and invalid path/ownership checks: $run"
