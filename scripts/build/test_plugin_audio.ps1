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
$msbuild = (Get-Command msbuild -ErrorAction SilentlyContinue).Source
if (-not $msbuild) {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $vsPath = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -property installationPath
        if ($vsPath) { $msbuild = Join-Path $vsPath 'MSBuild\Current\Bin\MSBuild.exe' }
    }
}
if (-not $msbuild -or -not (Test-Path $msbuild)) {
    Write-Host "ERROR: MSBuild not found. Install Visual Studio 2022 (with .NET desktop)." -ForegroundColor Red
    exit 1
}
$csc = Join-Path (Split-Path $msbuild) 'Roslyn\csc.exe'
if (-not (Test-Path $csc)) {
    Write-Host "ERROR: csc.exe not found at $csc" -ForegroundColor Red
    exit 1
}

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
exit $result
