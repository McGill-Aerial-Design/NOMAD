# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
#
# Runs the video lifetime against an isolated off-screen STA WinForms host.
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

$mpDir = 'C:\Program Files (x86)\Mission Planner'
$sources = Get-ChildItem (Join-Path $repoRoot 'mission_planner\tests\video') -Filter '*.cs' | ForEach-Object FullName
$outDir = Join-Path $repoRoot 'mission_planner\tests\video\bin'
New-Item -ItemType Directory -Force $outDir | Out-Null
$exe = Join-Path $outDir 'VideoLifetimeTests.exe'
$references = @(
    "/r:$plugin",
    '/r:System.Drawing.dll',
    '/r:System.Windows.Forms.dll',
    "/r:$mpDir\MissionPlanner.exe",
    "/r:$mpDir\MissionPlanner.Controls.dll"
)
& $csc /nologo /target:exe /langversion:latest "/out:$exe" @references @sources
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Copy-Item -LiteralPath $plugin -Destination $outDir -Force
# The harness resolves dependencies using the supplied reference directory.
& $exe $mpDir
exit $LASTEXITCODE
