# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '../..')
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$csc = Find-RoslynCompiler
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
