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

# Step 1: Find MSBuild
Write-Host "[1/4] Locating MSBuild..." -ForegroundColor Yellow

$msbuild = (Get-Command msbuild -ErrorAction SilentlyContinue).Source

if (-not $msbuild) {
    Write-Host "  MSBuild not in PATH, checking Visual Studio..." -ForegroundColor Gray
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"

    if (Test-Path $vswhere) {
        $vsPath = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -property installationPath
        if ($vsPath) {
            $msbuild = Join-Path $vsPath 'MSBuild\Current\Bin\MSBuild.exe'
            if (-not (Test-Path $msbuild)) {
                Write-Host "ERROR: MSBuild not found!" -ForegroundColor Red
                Write-Host "Please install Visual Studio 2022 or Visual Studio Build Tools" -ForegroundColor Red
                exit 1
            }
        }
    } else {
        Write-Host "ERROR: Visual Studio not found!" -ForegroundColor Red
        Write-Host "Please install Visual Studio 2022 or Visual Studio Build Tools" -ForegroundColor Red
        exit 1
    }
}

Write-Host "  Found: $msbuild" -ForegroundColor Green
Write-Host ""

# Step 2: Clean previous build
Write-Host "[2/4] Cleaning previous build..." -ForegroundColor Yellow
& $msbuild $ProjectFile /t:Clean /p:Configuration=$Configuration /v:minimal /nologo
$cleanExitCode = $LASTEXITCODE
if ($cleanExitCode -ne 0) {
    Write-Host "ERROR: Clean failed!" -ForegroundColor Red
    exit $cleanExitCode
}
Write-Host "  Clean complete" -ForegroundColor Green
Write-Host ""

# Generate deterministic source identity before compiling the plugin.
$identityPath = Join-Path $RepoRoot 'build/release/package-identity.json'
$csharpPath = Join-Path $RepoRoot 'build/release/ReleaseIdentity.cs'
& python (Join-Path $RepoRoot 'scripts/release/identity.py') component --component plugin `
    --platform windows --architecture any --output $identityPath --csharp $csharpPath --required NOMADPlugin.dll
if ($LASTEXITCODE -ne 0) { throw 'Plugin release identity generation failed' }

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
