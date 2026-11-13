# NOMAD Mission Planner plugin build (installation is separate).
$ErrorActionPreference = 'Stop'
Write-Host "======================================" -ForegroundColor Cyan
Write-Host " NOMAD Mission Planner Plugin Build" -ForegroundColor Cyan
Write-Host "======================================" -ForegroundColor Cyan
Write-Host ""

# Find project directory (relative to this script)
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent (Split-Path -Parent $ScriptDir)
$ProjectDir = Join-Path $RepoRoot "mission_planner\src"
Set-Location $ProjectDir

# Configuration
$ProjectFile = "NOMADPlugin.csproj"
$Configuration = "Release"
$version = '0.5.0'

# Step 1: Find MSBuild
Write-Host "[1/4] Locating MSBuild..." -ForegroundColor Yellow

. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$msbuild = Find-MSBuild
if (-not $msbuild) {
    Write-Host 'ERROR: MSBuild not found. Install Visual Studio or Visual Studio Build Tools.' -ForegroundColor Red
    exit 1
}

Write-Host "  Found: $msbuild" -ForegroundColor Green
Write-Host ""

$target = (Get-Content (Join-Path $RepoRoot 'scripts/release/mission-planner-target.json') -Raw | ConvertFrom-Json).version
if ($target -notmatch '^\d+\.\d+\.\d+$') { throw 'Invalid reviewed Mission Planner target' }
$hostDirectory = Join-Path ${env:ProgramFiles(x86)} 'Mission Planner'
$hostPath = Join-Path $hostDirectory 'MissionPlanner.exe'
if (!(Test-Path -LiteralPath $hostPath -PathType Leaf)) { throw 'Mission Planner reference host is missing.' }
$hostVersion = [Diagnostics.FileVersionInfo]::GetVersionInfo($hostPath)
$actualVersion = '{0}.{1}.{2}' -f $hostVersion.FileMajorPart, $hostVersion.FileMinorPart, $hostVersion.FileBuildPart
if ($actualVersion -ne $target) {
    throw "Mission Planner reference version mismatch: expected $target, found $actualVersion."
}

# Step 2: Clean previous build
& (Join-Path $PSScriptRoot 'prepare_vision.ps1')
if (-not $?) { throw 'Pinned vision input preparation failed.' }
Write-Host "[2/4] Cleaning previous build..." -ForegroundColor Yellow
& $msbuild $ProjectFile /t:Clean /p:Configuration=$Configuration "/p:MissionPlannerDir=$hostDirectory" /v:minimal /nologo
$cleanExitCode = $LASTEXITCODE
if ($cleanExitCode -ne 0) {
    Write-Host "ERROR: Clean failed!" -ForegroundColor Red
    exit $cleanExitCode
}
Write-Host "  Clean complete" -ForegroundColor Green
Write-Host ""

# Embed the version, source revision and reviewed host identity.
$sourceSha = & git -C $RepoRoot rev-parse --verify HEAD
if ($LASTEXITCODE -ne 0 -or $sourceSha -notmatch '^[0-9a-f]{40}$') { throw 'Plugin source identity is unavailable' }
$csharpPath = Join-Path $RepoRoot 'build/release/ReleaseIdentity.cs'
New-Item -ItemType Directory -Path (Split-Path $csharpPath) -Force | Out-Null
@"
// Generated NOMAD plugin identity.
using System.Reflection;
[assembly: AssemblyVersion("$version.0")]
[assembly: AssemblyFileVersion("$version.0")]
[assembly: AssemblyInformationalVersion("$version+$sourceSha")]
namespace NOMAD.MissionPlanner {
    internal static class NomadRelease {
        public const string Version = "$version";
        public const string SourceSha = "$sourceSha";
        public const string MissionPlannerTarget = "$target";
    }
}
"@ | Set-Content -LiteralPath $csharpPath -Encoding UTF8

$sourceHashes = Get-PluginSourceHashes $RepoRoot
& (Join-Path $PSScriptRoot 'build_media_worker.ps1') -MSBuild $msbuild -RepoRoot $RepoRoot -HostDirectory $hostDirectory
if (-not $?) { throw 'Private media worker build failed.' }

# Step 3: Build project
Write-Host "[3/4] Building plugin..." -ForegroundColor Yellow
& $msbuild $ProjectFile /t:Build /p:Configuration=$Configuration "/p:MissionPlannerDir=$hostDirectory" /v:minimal /nologo
$buildExitCode = $LASTEXITCODE
if ($buildExitCode -ne 0) {
    Write-Host "ERROR: Build failed!" -ForegroundColor Red
    exit $buildExitCode
}
Write-Host "  Build successful" -ForegroundColor Green
Write-Host ""

# Confirm the expected output before entering the deployment branch.
$BuiltDll = Join-Path $ProjectDir "bin\$Configuration\NOMADPlugin.dll"
if (-not (Test-Path -LiteralPath $BuiltDll -PathType Leaf)) {
    Write-Host "ERROR: Built DLL not found at $BuiltDll" -ForegroundColor Red
    exit 1
}

$FileInfo = Get-Item -LiteralPath $BuiltDll
$completedHashes = Get-PluginSourceHashes $RepoRoot
if ($completedHashes.Count -ne $sourceHashes.Count) {
    Write-Host 'ERROR: Product source changed during compilation; the artifact is not qualified.' -ForegroundColor Red
    exit 1
}
foreach ($relative in $sourceHashes.Keys) {
    if ($completedHashes[$relative] -ne $sourceHashes[$relative]) {
        Write-Host 'ERROR: Product source changed during compilation; the artifact is not qualified.' -ForegroundColor Red
        exit 1
    }
}
@{ schema=1; source=$sourceSha; host_target=$target; plugin_sha256=(Get-FileHash -LiteralPath $BuiltDll).Hash;
    source_files=$sourceHashes; media_bundle=(Get-Content -LiteralPath (Join-Path $RepoRoot 'build/media_worker/bundle.json') -Raw | ConvertFrom-Json) } | ConvertTo-Json -Depth 4 |
    Set-Content -LiteralPath (Join-Path $RepoRoot 'build/release/plugin-build.json') -Encoding UTF8
Write-Host "  Plugin size: $($FileInfo.Length / 1KB) KB" -ForegroundColor Gray

Write-Host "[4/4] Deployment skipped; install the reviewed artifact separately."
Write-Host "  Output artifact: $BuiltDll" -ForegroundColor Green
