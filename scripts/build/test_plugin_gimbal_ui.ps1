# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
#
# Runs the joystick pad against an isolated off-screen STA WinForms host.
# Build the plugin first with `pixi run build-plugin-only`.

$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$plugin = Join-Path $repoRoot 'mission_planner\src\bin\Release\NOMADPlugin.dll'
if (-not (Test-Path -LiteralPath $plugin -PathType Leaf)) {
    Write-Host 'ERROR: Build NOMADPlugin.dll with pixi run build-plugin-only first.' -ForegroundColor Red
    exit 1
}

. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$csc = Find-RoslynCompiler

$testSource = Join-Path $repoRoot 'mission_planner\tests\gimbal\JoystickPadUiTests.cs'
$outDir = Join-Path $repoRoot 'mission_planner\tests\gimbal\bin'
New-Item -ItemType Directory -Force $outDir | Out-Null
$exe = Join-Path $outDir 'JoystickPadUiTests.exe'

Write-Host 'Compiling joystick-pad UI harness...' -ForegroundColor Yellow
& $csc /nologo /target:exe /langversion:latest /r:System.Drawing.dll /r:System.Windows.Forms.dll "/out:$exe" $testSource
if ($LASTEXITCODE -ne 0) {
    Write-Host 'Compile FAILED.' -ForegroundColor Red
    exit 1
}

Write-Host 'Running isolated joystick-pad UI harness...' -ForegroundColor Yellow
& $exe $plugin
exit $LASTEXITCODE
