# SPDX-License-Identifier: Apache-2.0
# Qualification-only binaries; this script never installs or publishes them.
$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '../..')
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$csc = Find-RoslynCompiler
$sources = Get-ChildItem (Join-Path $repoRoot 'infra/transport/ground_router/*.cs') | ForEach-Object FullName
foreach ($label in @('A', 'B')) {
    $outDir = Join-Path $repoRoot "build/release-fixtures/router-$label"
    New-Item -ItemType Directory -Force $outDir | Out-Null
    $identity = Join-Path $outDir 'FixtureIdentity.cs'
    $code = "namespace NOMAD.MissionPlanner { public static class NomadRelease { public const string Version = `"0.0.0-fixture.$label`"; } }"
    [IO.File]::WriteAllText($identity, $code)
    & $csc /nologo /target:library /langversion:latest /r:System.Web.Extensions.dll "/out:$outDir/Nomad.LinkRouter.dll" @sources $identity
    if ($LASTEXITCODE -ne 0) { throw 'router fixture library compilation failed' }
    & $csc /nologo /target:exe /langversion:latest /r:System.Web.Extensions.dll `
        "/r:$outDir/Nomad.LinkRouter.dll" "/out:$outDir/nomad-link-router.exe" `
        (Join-Path $repoRoot 'infra/transport/ground_router/host/Program.cs')
    if ($LASTEXITCODE -ne 0) { throw 'router fixture host compilation failed' }
    Remove-Item -LiteralPath $identity
}
