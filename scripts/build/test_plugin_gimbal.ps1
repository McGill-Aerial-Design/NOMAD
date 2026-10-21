# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# ============================================================
# Gimbal command unit tests for the Mission Planner plugin
# ============================================================
# Compiles the Mission Planner-free gimbal command core (GimbalCommand.cs)
# together with the test runner using the Roslyn csc bundled with Visual
# Studio's MSBuild — no .NET SDK or test-framework packages required.
#
# Usage: pixi run test-plugin-gimbal
# Exits non-zero on compile error or test failure.
# ============================================================

$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..\..')

# ---- Locate csc (next to MSBuild: PATH, then Visual Studio via vswhere) ----
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$csc = Find-RoslynCompiler

# ---- Compile ----
$sources = @(
    (Join-Path $repoRoot 'mission_planner\src\Control\GimbalCommand.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\gimbal\GimbalCommandTests.cs')
)
$outDir = Join-Path $repoRoot 'mission_planner\tests\gimbal\bin'
New-Item -ItemType Directory -Force $outDir | Out-Null
$exe = Join-Path $outDir 'GimbalCommandTests.exe'

Write-Host "Compiling gimbal-command tests..." -ForegroundColor Yellow
& $csc /nologo /target:exe /langversion:latest "/out:$exe" @sources
if ($LASTEXITCODE -ne 0) {
    Write-Host "Compile FAILED." -ForegroundColor Red
    exit 1
}

# ---- Run ----
Write-Host "Running gimbal-command tests..." -ForegroundColor Yellow
& $exe
$result = $LASTEXITCODE
if ($result -ne 0) {
    Write-Host "Gimbal-command tests FAILED." -ForegroundColor Red
}
if ($result -ne 0) { exit $result }
$nativeSources = @(
    (Join-Path $repoRoot 'mission_planner/src/Control/GimbalCommand.cs'),
    (Join-Path $repoRoot 'mission_planner/src/Control/GimbalController.cs'),
    (Join-Path $repoRoot 'mission_planner/src/Input/GimbalInputState.cs'),
    (Join-Path $repoRoot 'mission_planner/src/Control/GimbalArrowKeyFilter.cs'),
    (Join-Path $repoRoot 'mission_planner/src/Control/OutputController.cs'),
    (Join-Path $repoRoot 'mission_planner/tests/gimbal/NativeOutputTests.cs')
)
$nativeExe = Join-Path $outDir 'NativeOutputTests.exe'
& $csc /nologo /target:exe /langversion:latest /r:System.Windows.Forms.dll /r:System.Drawing.dll "/out:$nativeExe" @nativeSources
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $nativeExe
exit $LASTEXITCODE
