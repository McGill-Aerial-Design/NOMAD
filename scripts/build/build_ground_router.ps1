# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '../..')
$msbuild = (Get-Command msbuild -ErrorAction SilentlyContinue).Source
if (-not $msbuild) {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $vsPath = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -property installationPath
        if ($vsPath) { $msbuild = Join-Path $vsPath 'MSBuild\Current\Bin\MSBuild.exe' }
    }
}
if (-not $msbuild) { throw 'MSBuild with Roslyn is required' }
$csc = Join-Path (Split-Path $msbuild) 'Roslyn/csc.exe'
$outDir = Join-Path $repoRoot 'build/ground-router'
New-Item -ItemType Directory -Force $outDir | Out-Null
$sources = Get-ChildItem (Join-Path $repoRoot 'infra/transport/ground_router/*.cs') | ForEach-Object FullName
& $csc /nologo /target:library /langversion:latest /r:System.Web.Extensions.dll "/out:$outDir/Nomad.LinkRouter.dll" @sources
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $csc /nologo /target:exe /langversion:latest /r:System.Web.Extensions.dll `
    "/r:$outDir/Nomad.LinkRouter.dll" "/out:$outDir/nomad-link-router.exe" `
    (Join-Path $repoRoot 'infra/transport/ground_router/host/Program.cs')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Copy-Item (Join-Path $repoRoot 'infra/transport/ground_router/example.json') `
    (Join-Path $outDir 'router.example.json') -Force
Copy-Item (Join-Path $repoRoot 'infra/transport/ground_router/README.md') `
    (Join-Path $outDir 'README.md') -Force
exit 0
