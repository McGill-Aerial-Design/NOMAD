# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# ============================================================
# NomadCoreClient boundary tests for the Mission Planner plugin
# ============================================================
# Compiles the Mission Planner-free NomadCoreClient (the plugin's client for
# the C++ core CLI boundary) together with the test runner using the Roslyn
# csc bundled with Visual Studio's MSBuild — no .NET SDK or test-framework
# packages required.
#
# The pure checks (argument vector, invariant formatting, fail-closed input
# validation, unavailable-core fail-closed) always run. When the C++ core
# binary has been built (pixi run build-core), the harness also exercises the
# live CLI authentication gate: an actuation verb without NOMAD_API_KEY must
# be refused before any socket work. The gate check is skipped otherwise.
#
# Usage: pixi run test-plugin-core-client
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
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\NomadCoreClient.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\coreclient\NomadCoreClientTests.cs')
)
$outDir = Join-Path $repoRoot 'mission_planner\tests\coreclient\bin'
New-Item -ItemType Directory -Force $outDir | Out-Null
$exe = Join-Path $outDir 'NomadCoreClientTests.exe'

Write-Host "Compiling core-client tests..." -ForegroundColor Yellow
& $csc /nologo /target:exe /langversion:latest "/out:$exe" @sources
if ($LASTEXITCODE -ne 0) {
    Write-Host "Compile FAILED." -ForegroundColor Red
    exit 1
}

# ---- Run ----
# The auth-gate check inside the test refuses to run while NOMAD_API_KEY is
# set (the gate needs an unset key to observe the refusal), so clear it here.
Remove-Item Env:NOMAD_API_KEY -ErrorAction SilentlyContinue

Write-Host "Running core-client tests..." -ForegroundColor Yellow
& $exe
$result = $LASTEXITCODE
if ($result -ne 0) {
    Write-Host "Core-client tests FAILED." -ForegroundColor Red
}
exit $result
