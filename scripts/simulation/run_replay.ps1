# SPDX-License-Identifier: Apache-2.0
param([Parameter(Mandatory=$true)][string]$Dataset,
    [ValidateSet('frames','decoded')][string]$Folder='decoded', [ValidateRange(0,100000)][int]$First=0,
    [ValidateSet('Area','Object')][string]$Mode='Area', [string]$Selection='',
    [switch]$Visible)
$ErrorActionPreference = 'Stop'
$datasetPath = (Resolve-Path -LiteralPath $Dataset).Path
$options = @{dataset=$datasetPath; folder=$Folder; first=$First; mode=$Mode}
if ($Selection) {
    $values = @($Selection.Split(',') | ForEach-Object {
        $value = [float]::Parse($_, [Globalization.CultureInfo]::InvariantCulture)
        if ([float]::IsNaN($value) -or [float]::IsInfinity($value)) { throw 'Selection must contain finite pixels.' }
        $value
    })
    if ($values.Count -ne 4) { throw 'Selection needs x,y,width,height.' }
    $options.selection = $values
}
if ($Mode -eq 'Object' -and !$Selection) { throw 'Object replay needs an explicit -Selection x,y,width,height.' }
$run = & (Join-Path $PSScriptRoot 'prepare_host.ps1')
$options | ConvertTo-Json |
    Set-Content -LiteralPath (Join-Path $run 'replay.json') -Encoding UTF8
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
Add-Type -Path (Join-Path $root 'mission_planner/src/Simulation/OwnedProcessJob.cs')
$job = [NOMAD.Simulation.OwnedProcessJob]::new()
$process = $null
try {
    $start = New-Object Diagnostics.ProcessStartInfo
    $start.FileName = Join-Path $run 'host/VideoLifetimeTests.exe'
    $start.Arguments = '--replay "' + $run + '"'
    $start.WorkingDirectory = Join-Path $run 'host'
    $start.UseShellExecute = $false
    $start.CreateNoWindow = $true
    $start.RedirectStandardOutput = $true
    $start.RedirectStandardError = $true
    $start.EnvironmentVariables['NOMAD_SIMULATION_VISIBLE'] = $(if ($Visible) { '1' } else { '0' })
    $process = [Diagnostics.Process]::Start($start)
    $job.Attach($process)
    $output = $process.StandardOutput.ReadToEndAsync()
    $errors = $process.StandardError.ReadToEndAsync()
    if (!$process.WaitForExit(600000)) { throw "Recorded replay exceeded its ten-minute deadline: $run" }
    if ($process.ExitCode -ne 0) { throw "Recorded replay failed: $run" }
} finally {
    try {
        if ($process -and !$process.HasExited) { $process.Kill(); $process.WaitForExit() }
        if ($output -and $output.Wait(3000)) {
            $output.Result | Set-Content -LiteralPath (Join-Path $run 'replay-console.log')
        }
        if ($errors -and $errors.Wait(3000)) {
            $errors.Result | Set-Content -LiteralPath (Join-Path $run 'replay-error.log')
        }
    } finally { $job.Dispose() }
}
Write-Output "PASS recorded replay: $run"
