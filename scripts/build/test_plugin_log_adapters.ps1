# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors

$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$mpDir = "${env:ProgramFiles(x86)}\Mission Planner"
$pluginPath = Join-Path $repoRoot 'mission_planner\src\bin\Release\NOMADPlugin.dll'
$fixturePath = Join-Path $repoRoot 'mission_planner\tests\logs\fixtures\sample-dataflash.log'
$threadingSource = Join-Path $repoRoot 'mission_planner\tests\logs\LogViewThreadingSmoke.cs'

if (-not (Test-Path $pluginPath)) {
    throw "Build the Release plugin before running the log adapter smoke test."
}
if (-not (Test-Path (Join-Path $mpDir 'MissionPlanner.exe'))) {
    throw "Mission Planner is not installed or staged at $mpDir."
}

Get-ChildItem $mpDir -Filter '*.dll' | ForEach-Object {
    try { [void][Reflection.Assembly]::LoadFrom($_.FullName) } catch { }
}
[void][Reflection.Assembly]::LoadFrom((Join-Path $mpDir 'MissionPlanner.exe'))
$plugin = [Reflection.Assembly]::LoadFrom($pluginPath)
$modelType = $plugin.GetType('NOMAD.MissionPlanner.DFLogModel', $true)
$constructor = $modelType.GetConstructors(
    [Reflection.BindingFlags]'Instance,Public,NonPublic') | Select-Object -First 1
$model = $constructor.Invoke([object[]]@([string]$fixturePath))

try {
    $messageTypes = $modelType.GetProperty('MessageTypes').GetValue($model)
    if ($messageTypes -notcontains 'GPS' -or $messageTypes -notcontains 'BAT') {
        throw "DFLogBuffer did not discover the expected fixture message types."
    }

    $gpsRows = @($modelType.GetMethod('Records').Invoke($model, @('GPS')))
    if ($gpsRows.Count -ne 3) {
        throw "Expected 3 GPS rows, got $($gpsRows.Count)."
    }
    if ([Math]::Abs($gpsRows[0].TimeSeconds - 1) -gt 0.001) {
        throw "DFLogBuffer timestamp scaling changed: expected 1 second."
    }
    $getDouble = $gpsRows[0].GetType().GetMethod('GetDouble')
    $altitude = $getDouble.Invoke($gpsRows[0], [object[]]@(0.0, [string[]]@('Alt')))
    if ([Math]::Abs($altitude - 100) -gt 0.001) {
        throw "DFLogBuffer numeric field extraction changed: expected Alt=100."
    }
    $capacity = $modelType.GetMethod('Parameter').Invoke(
        $model,
        [object[]]@('BATT_CAPACITY', 0.0))
    if ([Math]::Abs($capacity - 5000) -gt 0.001) {
        throw "DFLogBuffer string field extraction changed: expected BATT_CAPACITY=5000."
    }

    $analysis = $plugin.GetType('NOMAD.MissionPlanner.LogAnalysis').GetMethod('Analyze')
    # Reflection does not fill optional parameters, so pass the cancellation token explicitly.
    $summary = $analysis.Invoke($null, @($model, $null, [System.Threading.CancellationToken]::None))
    if ([Math]::Abs($summary.ArmedDurationSeconds - 21) -gt 0.001) {
        throw "Expected a 21-second armed span, got $($summary.ArmedDurationSeconds)."
    }
    if ($summary.Metrics.Count -lt 6 -or $summary.Anomalies.Count -lt 3) {
        throw "The fixture did not produce the expected analysis coverage."
    }

    Write-Host "Flight log adapter smoke test passed." -ForegroundColor Green
}
finally {
    $model.Dispose()
}

$msbuild = (Get-Command msbuild -ErrorAction SilentlyContinue).Source
if (-not $msbuild) {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $vsPath = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -property installationPath
        if ($vsPath) { $msbuild = Join-Path $vsPath 'MSBuild\Current\Bin\MSBuild.exe' }
    }
}
if (-not $msbuild -or -not (Test-Path $msbuild)) {
    throw "MSBuild was not found."
}
$csc = Join-Path (Split-Path $msbuild) 'Roslyn\csc.exe'
$threadingExe = Join-Path (Split-Path $pluginPath) 'LogViewThreadingSmoke.exe'
& $csc /nologo /target:exe /langversion:latest "/out:$threadingExe" `
    /r:System.dll /r:System.Core.dll /r:System.Drawing.dll /r:System.Windows.Forms.dll `
    $threadingSource
if ($LASTEXITCODE -ne 0) {
    throw "The WinForms threading smoke test did not compile."
}

$stdout = Join-Path $env:TEMP "nomad-log-threading-$PID.out"
$stderr = Join-Path $env:TEMP "nomad-log-threading-$PID.err"
$arguments = "`"$pluginPath`" `"$fixturePath`" `"$mpDir`""
$process = Start-Process -FilePath $threadingExe -ArgumentList $arguments -PassThru `
    -WindowStyle Hidden -RedirectStandardOutput $stdout -RedirectStandardError $stderr
try {
    if (-not $process.WaitForExit(30000)) {
        $process.Kill()
        throw "The WinForms threading smoke test timed out."
    }
    $process.WaitForExit()
    $process.Refresh()
    $exitCode = $process.ExitCode
    $output = Get-Content $stdout -Raw -ErrorAction SilentlyContinue
    $errorOutput = Get-Content $stderr -Raw -ErrorAction SilentlyContinue
    $reportedSuccess = $output -match 'Flight log WinForms threading and responsive layout smoke test passed\.'
    if (($null -ne $exitCode -and $exitCode -ne 0) -or
        -not $reportedSuccess -or
        -not [string]::IsNullOrWhiteSpace($errorOutput)) {
        throw "The WinForms threading smoke test failed with exit code $exitCode. " +
            "stdout: $output stderr: $errorOutput"
    }
    Write-Host $output.Trim() -ForegroundColor Green
}
finally {
    Remove-Item $stdout, $stderr -Force -ErrorAction SilentlyContinue
    Remove-Item $threadingExe -Force -ErrorAction SilentlyContinue
}
