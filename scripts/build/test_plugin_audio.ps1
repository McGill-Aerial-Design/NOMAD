# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# ============================================================
# Altitude-callout unit tests for the Mission Planner plugin
# ============================================================
# Compiles the Mission Planner-free AltitudeCallout.cs together
# with the test runner using the Roslyn csc bundled with Visual
# Studio's MSBuild — no .NET SDK or test-framework packages.
#
# Usage: pixi run test-plugin-audio
# Exits non-zero on compile error or test failure.
# ============================================================

$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..\..')

# ---- Locate csc (next to MSBuild: PATH, then Visual Studio via vswhere) ----
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$csc = Find-RoslynCompiler

# ---- Compile ----
$sources = @(
    (Join-Path $repoRoot 'mission_planner\src\Notifications\AltitudeCallout.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\audio\AltitudeCalloutTests.cs')
)
$outDir = Join-Path $repoRoot 'mission_planner\tests\audio\bin'
New-Item -ItemType Directory -Force $outDir | Out-Null
$exe = Join-Path $outDir 'AltitudeCalloutTests.exe'

Write-Host "Compiling altitude-callout tests..." -ForegroundColor Yellow
& $csc /nologo /target:exe /langversion:latest "/out:$exe" @sources
if ($LASTEXITCODE -ne 0) {
    Write-Host "Compile FAILED." -ForegroundColor Red
    exit 1
}

# ---- Run ----
Write-Host "Running altitude-callout tests..." -ForegroundColor Yellow
& $exe
$result = $LASTEXITCODE
if ($result -ne 0) {
    Write-Host "Altitude-callout tests FAILED." -ForegroundColor Red
}
if ($result -ne 0) { exit $result }
$batterySources = @(
    (Join-Path $repoRoot 'mission_planner/src/Telemetry/BatteryHealth.cs'),
    (Join-Path $repoRoot 'mission_planner/tests/audio/BatteryHealthTests.cs')
)
$batteryExe = Join-Path $outDir 'BatteryHealthTests.exe'
& $csc /nologo /target:exe /langversion:latest /r:Microsoft.CSharp.dll "/out:$batteryExe" @batterySources
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $batteryExe
exit $LASTEXITCODE
