# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
param([string]$PluginPath, [string]$OutputDirectory)
$ErrorActionPreference = 'Stop'
# Pixi can retain PowerShell 7 module paths when launching Windows PowerShell 5.1.
Import-Module (Join-Path $PSHOME 'Modules/Microsoft.PowerShell.Utility/Microsoft.PowerShell.Utility.psd1') -ErrorAction Stop
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
if (!$PluginPath) { $PluginPath = Join-Path $root 'mission_planner/src/bin/Release/NOMADPlugin.dll' }
if (!$OutputDirectory) { $OutputDirectory = Join-Path $root 'build/installer' }
$directory = Join-Path $root 'mission_planner/packaging/installer'
$sources = @((Join-Path $directory 'Program.cs'), (Join-Path $directory 'InstallEngine.cs'),
    (Join-Path $root 'mission_planner/src/Plugin/MissionPlannerVersion.cs'))
$inputs = $sources + @((Join-Path $directory 'app.manifest'), $PSCommandPath,
    (Join-Path $PSScriptRoot 'windows_build_tools.ps1'),
    (Join-Path $root 'scripts/release/mission-planner-target.json'), (Join-Path $root 'LICENSE'), (Join-Path $root 'NOTICE'))
$inputHashes = @{}
foreach ($path in $inputs) {
    $relative = $path.Substring($root.Length + 1).Replace('\','/')
    $inputHashes[$relative] = (Get-FileHash -LiteralPath $path).Hash.ToLowerInvariant()
}
$PluginPath = (Resolve-Path -LiteralPath $PluginPath).Path
$hash = (Get-FileHash -LiteralPath $PluginPath -Algorithm SHA256).Hash.ToLowerInvariant()
$info = [Diagnostics.FileVersionInfo]::GetVersionInfo($PluginPath)
if ($info.ProductVersion -notmatch '^(?<version>\d+\.\d+\.\d+)\+(?<source>[a-f0-9]{40})$') {
    throw 'Build the reviewed plugin first; release version/source metadata is required.'
}
$version = $Matches.version
$source = $Matches.source
if ([Reflection.AssemblyName]::GetAssemblyName($PluginPath).Name -ne 'NOMADPlugin' -or
    $info.FileVersion -ne ($version + '.0')) { throw 'Unexpected plugin assembly identity.' }
$target = (Get-Content (Join-Path $root 'scripts/release/mission-planner-target.json') -Raw | ConvertFrom-Json).version
if ($target -notmatch '^\d+\.\d+\.\d+$') { throw 'Invalid reviewed Mission Planner version.' }
$installerSource = (git -C $root rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or $installerSource -notmatch '^[a-f0-9]{40}$') { throw 'Installer source identity unavailable.' }
if ($source -ne $installerSource) { throw 'Build the plugin from this checkout before packaging setup.' }
$metadata = [Reflection.Assembly]::ReflectionOnlyLoad([IO.File]::ReadAllBytes($PluginPath))
$identity = $metadata.GetType('NOMAD.MissionPlanner.NomadRelease', $true)
$flags = [Reflection.BindingFlags]'Public,NonPublic,Static'
if ($identity.GetField('MissionPlannerTarget', $flags).GetRawConstantValue() -ne $target -or
    $identity.GetField('Version', $flags).GetRawConstantValue() -ne $version -or
    $identity.GetField('SourceSha', $flags).GetRawConstantValue() -ne $source) {
    throw 'Plugin embedded identity differs from the packaging inputs.'
}
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$null = Get-CurrentPluginBuild -RepoRoot $root -PluginPath $PluginPath
$compiler = Find-RoslynCompiler
$output = [IO.Path]::GetFullPath($OutputDirectory)
New-Item -ItemType Directory -Path $output -Force | Out-Null
$generated = Join-Path $output 'SetupInfo.cs'
@"
// Generated installer identity.
using System.Reflection;
[assembly: AssemblyVersion("$version.0")]
[assembly: AssemblyFileVersion("$version.0")]
[assembly: AssemblyInformationalVersion("$version+installer.$installerSource")]
namespace NOMAD.Installer {
    internal static class SetupInfo {
        internal const string Version = "$version";
        internal const string ReviewedVersion = "$target";
        internal const string PayloadSha256 = "$hash";
    }
}
"@ | Set-Content -LiteralPath $generated -Encoding UTF8
[xml]$nativeManifest = Get-Content -LiteralPath (Join-Path $directory 'app.manifest') -Raw
$nativeManifest.assembly.assemblyIdentity.version = $version + '.0'
$nativeManifestPath = Join-Path $output 'Setup.manifest'
$nativeManifest.Save($nativeManifestPath)
$exe = Join-Path $output "NOMAD-$version-Setup.exe"
& $compiler /nologo /target:winexe /platform:anycpu /langversion:latest /optimize+ /deterministic+ `
    /r:System.Windows.Forms.dll /r:System.Drawing.dll "/out:$exe" `
    "/win32manifest:$nativeManifestPath" "/resource:$PluginPath,NOMAD.Payload.dll" `
    "/resource:$root/LICENSE,NOMAD.LICENSE" "/resource:$root/NOTICE,NOMAD.NOTICE" @sources $generated
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
foreach ($relative in $inputHashes.Keys) {
    if ((Get-FileHash -LiteralPath (Join-Path $root $relative)).Hash -ne $inputHashes[$relative]) {
        throw 'Installer inputs changed during compilation; rebuild before packaging setup.'
    }
}
if ((Get-FileHash -LiteralPath $PluginPath).Hash -ne $hash) {
    throw 'Plugin payload changed during compilation; rebuild before packaging setup.'
}
$null = Get-CurrentPluginBuild -RepoRoot $root -PluginPath $PluginPath
$manifest = [ordered]@{
    version = $version
    plugin_source_sha = $source
    plugin_sha256 = $hash
    reviewed_mission_planner = $target
    installer_source_sha = $installerSource
    installer_sha256 = (Get-FileHash -LiteralPath $exe).Hash.ToLowerInvariant()
    inputs_sha256 = $inputHashes
}
$manifest | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $output 'installer-manifest.json')
$checksums = @(
    ($manifest.installer_sha256 + '  ' + [IO.Path]::GetFileName($exe))
    ((Get-FileHash -LiteralPath (Join-Path $output 'installer-manifest.json')).Hash.ToLowerInvariant() + '  installer-manifest.json')
)
$checksums |
    Set-Content -LiteralPath (Join-Path $output 'SHA256SUMS') -Encoding ascii
Write-Output "Built $exe (plugin $version, source $source); nothing installed."
