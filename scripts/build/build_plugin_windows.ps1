# NOMAD Mission Planner Plugin - Build and Deploy Script
# Location: scripts/build/build_plugin_windows.ps1
# Usage: .\scripts\build\build_plugin_windows.ps1 (from repo root)
#        or Run from anywhere - it will auto-locate the project

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
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Clean failed!" -ForegroundColor Red
    exit 1
}
Write-Host "  Clean complete" -ForegroundColor Green
Write-Host ""

# Step 3: Build project
Write-Host "[3/4] Building plugin..." -ForegroundColor Yellow
& $msbuild $ProjectFile /t:Build /p:Configuration=$Configuration /v:minimal /nologo
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Build failed!" -ForegroundColor Red
    exit 1
}
Write-Host "  Build successful" -ForegroundColor Green
Write-Host ""

# Step 4: Deploy plugin
Write-Host "[4/4] Deploying plugin..." -ForegroundColor Yellow

# Try to include libVLC redistributables if present in packaging folder
try {
    $PackagingDir = Join-Path $RepoRoot "mission_planner\packaging"
    $copyManaged = Join-Path $PackagingDir 'copy-managed-libs.ps1'
    if (Test-Path $copyManaged) {
        Write-Host "  Copying managed LibVLC assemblies (if present)..." -ForegroundColor Gray
        & $copyManaged | Out-Null
    }

    $copyScript = Join-Path $PackagingDir 'copy-libvlc.ps1'
    if (Test-Path $copyScript) {
        Write-Host "  Found libVLC packaging helper, copying redistributables..." -ForegroundColor Gray
        & $copyScript | Out-Null
    } else {
        Write-Host "  No libVLC packaging helper found (skipping)" -ForegroundColor Gray
    }
} catch {
    Write-Host "  Warning: failed to copy libVLC files: $_" -ForegroundColor Yellow
}

$BuiltDll = "bin\$Configuration\NOMADPlugin.dll"

# Get all DLLs from the bin folder for copying dependencies (Helix Toolkit, etc.)
$BuiltDlls = Get-ChildItem "bin\$Configuration\*.dll" -ErrorAction SilentlyContinue

if (-not (Test-Path $BuiltDll)) {
    Write-Host "ERROR: Built DLL not found at $BuiltDll" -ForegroundColor Red
    exit 1
}

# Get file info
$FileInfo = Get-Item $BuiltDll
Write-Host "  Plugin size: $($FileInfo.Length / 1KB) KB" -ForegroundColor Gray

# Deploy to AppData (user plugins folder)
$AppDataPluginsDir = "$env:LOCALAPPDATA\Mission Planner\plugins"
if (-not (Test-Path $AppDataPluginsDir)) {
    New-Item -ItemType Directory -Path $AppDataPluginsDir -Force | Out-Null
}

Copy-Item $BuiltDll $AppDataPluginsDir -Force
Write-Host "  Copied to: $AppDataPluginsDir" -ForegroundColor Green

# Copy HelixToolkit dependencies
$HelixDlls = @(
    "$env:USERPROFILE\.nuget\packages\helixtoolkit.wpf\2.20.2\lib\net45\HelixToolkit.Wpf.dll",
    "$env:USERPROFILE\.nuget\packages\helixtoolkit\2.20.2\lib\netstandard1.1\HelixToolkit.dll"
)
foreach ($dll in $HelixDlls) {
    if (Test-Path $dll) {
        Copy-Item $dll $AppDataPluginsDir -Force
        Write-Host "  Copied: $(Split-Path $dll -Leaf)" -ForegroundColor Gray
    }
}

    # Also copy any libVLC native files, plugins folder, and managed assemblies to the AppData plugin folder
    $BuildOutputDir = Join-Path $ProjectDir "bin\$Configuration"
    Get-ChildItem "$BuildOutputDir" -Filter "libvlc*.dll" -File -ErrorAction SilentlyContinue | ForEach-Object {
        Copy-Item $_.FullName $AppDataPluginsDir -Force
    }
    if (Test-Path "$BuildOutputDir\plugins") {
        Copy-Item "$BuildOutputDir\plugins\*" (Join-Path $AppDataPluginsDir 'plugins') -Recurse -Force -ErrorAction SilentlyContinue
    }
    Get-ChildItem "$BuildOutputDir" -Filter "LibVLCSharp*.dll" -File -ErrorAction SilentlyContinue | ForEach-Object {
        Copy-Item $_.FullName $AppDataPluginsDir -Force
    }

# Single canonical copy: Mission Planner scans BOTH its install plugins folder
# (Program Files) and %LOCALAPPDATA%\Mission Planner\plugins. If NOMADPlugin.dll
# exists in both, MP loads the plugin TWICE — two control panels, two configs —
# and a stale Program Files build can silently shadow settings (e.g. payloads
# reappearing after you cleared them, because the old build's defaults differ).
# AppData is the canonical location (no admin needed, and it's where
# nomad_config.json lives), so remove any Program Files duplicate here.
$ProgramFilesPluginDll = "${env:ProgramFiles(x86)}\Mission Planner\plugins\NOMADPlugin.dll"
if (Test-Path $ProgramFilesPluginDll) {
    try {
        Remove-Item $ProgramFilesPluginDll -Force -ErrorAction Stop
        Write-Host "  Removed stale duplicate: $ProgramFilesPluginDll" -ForegroundColor Green
    } catch {
        Write-Host "  WARNING: A duplicate NOMADPlugin.dll is present in Program Files and" -ForegroundColor Red
        Write-Host "           could not be removed (needs admin). Mission Planner will load" -ForegroundColor Red
        Write-Host "           the plugin TWICE and may restore old defaults." -ForegroundColor Red
        Write-Host "           Delete it manually (Run as Administrator):" -ForegroundColor Yellow
        Write-Host "             Remove-Item '$ProgramFilesPluginDll' -Force" -ForegroundColor Yellow
    }
} else {
    Write-Host "  No duplicate in Program Files (good — single canonical copy in AppData)" -ForegroundColor Gray
}

Write-Host ""
Write-Host "======================================" -ForegroundColor Cyan
Write-Host " Build and Deployment Complete!" -ForegroundColor Green
Write-Host "======================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Next steps:" -ForegroundColor White
Write-Host "  1. Launch Mission Planner" -ForegroundColor Gray
Write-Host "  2. Click Tools -> NOMAD Settings to configure" -ForegroundColor Gray
Write-Host "  3. Access NOMAD Control Panel from top menu" -ForegroundColor Gray
Write-Host ""
Write-Host "New features in this build:" -ForegroundColor White
Write-Host "  - Telemetry Injection (STATUSTEXT to HUD)" -ForegroundColor Gray
Write-Host "  - WASD Indoor Nudge Control" -ForegroundColor Gray
Write-Host "  - Jetson Health Monitor Tab" -ForegroundColor Gray
Write-Host ""
