# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors

$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..\..')

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

$sources = @(
    (Join-Path $repoRoot 'mission_planner\src\Logs\LogSummary.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Logs\IssueRules.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Logs\LogAnalysis.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\logs\LogAnalysisTests.cs')
)
$outDir = Join-Path $repoRoot 'mission_planner\tests\logs\bin'
New-Item -ItemType Directory -Force $outDir | Out-Null
$exe = Join-Path $outDir 'LogAnalysisTests.exe'

Write-Host "Compiling log analysis tests..." -ForegroundColor Yellow
& $csc /nologo /target:exe /langversion:latest "/out:$exe" @sources
if ($LASTEXITCODE -ne 0) {
    Write-Host "Compile FAILED." -ForegroundColor Red
    exit 1
}

Write-Host "Running log analysis tests..." -ForegroundColor Yellow
& $exe
$result = $LASTEXITCODE
if ($result -ne 0) {
    Write-Host "Log analysis tests FAILED." -ForegroundColor Red
}
exit $result
