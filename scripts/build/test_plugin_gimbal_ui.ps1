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

$msbuild = (Get-Command msbuild -ErrorAction SilentlyContinue).Source
if (-not $msbuild) {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $vsPath = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -property installationPath
        if ($vsPath) {
            $msbuild = Join-Path $vsPath 'MSBuild\Current\Bin\MSBuild.exe'
        }
    }
}
if (-not $msbuild -or -not (Test-Path $msbuild)) {
    Write-Host 'ERROR: MSBuild not found. Install Visual Studio 2022 with .NET desktop.' -ForegroundColor Red
    exit 1
}

$csc = Join-Path (Split-Path $msbuild) 'Roslyn\csc.exe'
if (-not (Test-Path $csc)) {
    Write-Host "ERROR: csc.exe not found at $csc" -ForegroundColor Red
    exit 1
}

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
