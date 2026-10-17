# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Runs router status presentation against the built plugin in an isolated STA host.
$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '../..')
$plugin = Join-Path $repoRoot 'mission_planner/src/bin/Release/NOMADPlugin.dll'
if (-not (Test-Path -LiteralPath $plugin -PathType Leaf)) {
    throw 'Build NOMADPlugin.dll with pixi run build-plugin-only first.'
}
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$csc = Find-RoslynCompiler
$mpDir = Join-Path ${env:ProgramFiles(x86)} 'Mission Planner'
$outDir = Join-Path $repoRoot 'mission_planner/tests/router/bin'
New-Item -ItemType Directory -Force $outDir | Out-Null
$exe = Join-Path $outDir 'RouterStatusUiTests.exe'
$references = @(
    "/r:$plugin",
    '/r:System.Drawing.dll',
    '/r:System.Windows.Forms.dll',
    "/r:$mpDir/MissionPlanner.exe",
    "/r:$mpDir/MissionPlanner.Controls.dll"
)
& $csc /nologo /target:exe /langversion:latest "/out:$exe" @references `
    (Join-Path $repoRoot 'mission_planner/tests/router/RouterStatusUiTests.cs')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Copy-Item -LiteralPath $plugin -Destination $outDir -Force
& $exe $mpDir
exit $LASTEXITCODE
