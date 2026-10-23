# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '../..')
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$csc = Find-RoslynCompiler
$testRoot = Join-Path $repoRoot 'mission_planner/tests/installer'
$outDir = Join-Path $testRoot 'bin'
New-Item -ItemType Directory -Force $outDir | Out-Null
$fakeHost = Join-Path $testRoot 'FakeHost.cs'
& $csc /nologo /target:exe /langversion:latest "/out:$outDir/MissionPlanner.exe" $fakeHost
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $csc /nologo /target:exe /langversion:latest /define:WRONG_HOST "/out:$outDir/WrongHost.exe" $fakeHost
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$exe = Join-Path $outDir 'InstallerTests.exe'
$sources = @(
    (Join-Path $repoRoot 'mission_planner/packaging/installer/InstallEngine.cs'),
    (Join-Path $repoRoot 'mission_planner/src/Plugin/MissionPlannerVersion.cs'),
    (Join-Path $testRoot 'InstallerTests.cs')
)
& $csc /nologo /target:exe /langversion:latest "/out:$exe" @sources
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& powershell -STA -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'test_plugin_installer_package.ps1')
exit $LASTEXITCODE
