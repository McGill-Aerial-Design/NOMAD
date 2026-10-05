# SPDX-License-Identifier: Apache-2.0
$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '../..')
$msbuild = (Get-Command msbuild -ErrorAction SilentlyContinue).Source
if (-not $msbuild) {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    $vsPath = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -property installationPath
    $msbuild = Join-Path $vsPath 'MSBuild\Current\Bin\MSBuild.exe'
}
$csc = Join-Path (Split-Path $msbuild) 'Roslyn/csc.exe'
$outDir = Join-Path $repoRoot 'build/plugin-tests/local-messages'
New-Item -ItemType Directory -Force $outDir | Out-Null
$exe = Join-Path $outDir 'LocalMessageTests.exe'
$sources = @(
    (Join-Path $repoRoot 'mission_planner/src/Telemetry/TelemetryInjector.cs'),
    (Join-Path $repoRoot 'mission_planner/src/UI/UiAsync.cs'),
    (Join-Path $repoRoot 'mission_planner/tests/telemetry/LocalMessageTests.cs')
)
& $csc /nologo /target:exe /langversion:latest /r:System.Windows.Forms.dll "/out:$exe" @sources
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe
exit $LASTEXITCODE
