# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '../..')
$msbuild = (Get-Command msbuild -ErrorAction SilentlyContinue).Source
if (-not $msbuild) {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    $vsPath = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -property installationPath
    $msbuild = Join-Path $vsPath 'MSBuild\Current\Bin\MSBuild.exe'
}
$csc = Join-Path (Split-Path $msbuild) 'Roslyn/csc.exe'
$outDir = Join-Path $repoRoot 'mission_planner/tests/version/bin'
New-Item -ItemType Directory -Force $outDir | Out-Null
$exe = Join-Path $outDir 'MissionPlannerVersionTests.exe'
$sources = @(
    (Join-Path $repoRoot 'mission_planner/src/Plugin/MissionPlannerVersion.cs'),
    (Join-Path $repoRoot 'mission_planner/src/Plugin/ReleaseIdentity.cs'),
    (Join-Path $repoRoot 'mission_planner/tests/version/MissionPlannerVersionTests.cs')
)
& $csc /nologo /target:exe /langversion:latest "/out:$exe" @sources
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
& $exe
exit $LASTEXITCODE
