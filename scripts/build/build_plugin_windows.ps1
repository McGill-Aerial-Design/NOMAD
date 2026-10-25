# NOMAD Mission Planner plugin build (installation is separate).
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
$version = '0.4.1'

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

# Step 2: Clean previous build
& (Join-Path $PSScriptRoot 'prepare_vision.ps1')
if (-not $?) { throw 'Pinned vision input preparation failed.' }
Write-Host "[2/4] Cleaning previous build..." -ForegroundColor Yellow
& $msbuild $ProjectFile /t:Clean /p:Configuration=$Configuration /v:minimal /nologo
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
$target = (Get-Content (Join-Path $RepoRoot 'scripts/release/mission-planner-target.json') -Raw | ConvertFrom-Json).version
if ($target -notmatch '^\d+\.\d+\.\d+$') { throw 'Invalid reviewed Mission Planner target' }
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

# Step 3: Build project
Write-Host "[3/4] Building plugin..." -ForegroundColor Yellow
& $msbuild $ProjectFile /t:Build /p:Configuration=$Configuration /v:minimal /nologo
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
Write-Host "  Plugin size: $($FileInfo.Length / 1KB) KB" -ForegroundColor Gray

Write-Host "[4/4] Deployment skipped; install the reviewed artifact separately."
Write-Host "  Output artifact: $BuiltDll" -ForegroundColor Green
