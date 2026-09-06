# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# ============================================================
# Dual-link router stress tests for the Mission Planner plugin
# ============================================================
# Compiles the Mission Planner-free dual-link stack
# (GroundLinkRouter*.cs + MAVLinkConnectionManager.cs + Log.cs)
# together with the test runner using the Roslyn csc bundled
# with Visual Studio's MSBuild — no .NET SDK or test-framework
# packages required. The tests run the router against real
# loopback UDP sockets: dedup, failover, manual override,
# param-transaction pinning, watchdog reopen, and high-rate
# mirrored stress with strict no-loss/no-duplication checks.
#
# Usage: pixi run test-plugin-duallink
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
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\GroundLinkRouter.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\GroundLinkRouter.Rx.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\GroundLinkRouter.Tx.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\MAVLinkConnectionManager.cs'),
    (Join-Path $repoRoot 'mission_planner\src\UI\Log.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\duallink\DualLinkStressTests.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\duallink\DualLinkStressTests.Harness.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\duallink\DualLinkStressTests.Router.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\duallink\DualLinkStressTests.Stress.cs')
)
$outDir = Join-Path $repoRoot 'mission_planner\tests\duallink\bin'
New-Item -ItemType Directory -Force $outDir | Out-Null
$exe = Join-Path $outDir 'DualLinkStressTests.exe'

Write-Host "Compiling dual-link tests..." -ForegroundColor Yellow
& $csc /nologo /target:exe /langversion:latest "/out:$exe" @sources
if ($LASTEXITCODE -ne 0) {
    Write-Host "Compile FAILED." -ForegroundColor Red
    exit 1
}

# ---- Run ----
Write-Host "Running dual-link tests (uses loopback UDP ports 28600+)..." -ForegroundColor Yellow
& $exe
$result = $LASTEXITCODE
if ($result -ne 0) {
    Write-Host "Dual-link tests FAILED." -ForegroundColor Red
}
exit $result
