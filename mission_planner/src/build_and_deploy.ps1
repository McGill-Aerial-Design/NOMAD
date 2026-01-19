# NOMAD Mission Planner Plugin - Build and Deploy Script
# Usage: .\build_and_deploy.ps1

Write-Host "======================================" -ForegroundColor Cyan
Write-Host " NOMAD Mission Planner Plugin Build" -ForegroundColor Cyan
Write-Host "======================================" -ForegroundColor Cyan
Write-Host ""

# Set working directory to script location
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $ScriptDir

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
    $copyManaged = Join-Path $ScriptDir '..\packaging\copy-managed-libs.ps1'
    if (Test-Path $copyManaged) {
        Write-Host "  Copying managed LibVLC assemblies (if present)..." -ForegroundColor Gray
        & $copyManaged | Out-Null
    }

    $copyScript = Join-Path $ScriptDir '..\packaging\copy-libvlc.ps1'
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

    # Also copy any libVLC native files, plugins folder, and managed assemblies to the AppData plugin folder
    Get-ChildItem "$ScriptDir\bin\Release" -Filter "libvlc*.dll" -File -ErrorAction SilentlyContinue | ForEach-Object {
        Copy-Item $_.FullName $AppDataPluginsDir -Force
    }
    if (Test-Path "$ScriptDir\bin\Release\plugins") {
        Copy-Item "$ScriptDir\bin\Release\plugins\*" (Join-Path $AppDataPluginsDir 'plugins') -Recurse -Force -ErrorAction SilentlyContinue
    }
    Get-ChildItem "$ScriptDir\bin\Release" -Filter "LibVLCSharp*.dll" -File -ErrorAction SilentlyContinue | ForEach-Object {
        Copy-Item $_.FullName $AppDataPluginsDir -Force
    }

# Try to deploy to Program Files (may require admin)
$ProgramFilesPluginsDir = "${env:ProgramFiles(x86)}\Mission Planner\plugins"
if (Test-Path "${env:ProgramFiles(x86)}\Mission Planner") {
    try {
        Copy-Item $BuiltDll $ProgramFilesPluginsDir -Force -ErrorAction Stop
        Write-Host "  Copied to: $ProgramFilesPluginsDir" -ForegroundColor Green
    } catch {
        Write-Host "  Could not copy to Program Files (may need admin rights)" -ForegroundColor Yellow
        Write-Host "  Run PowerShell as Administrator to deploy there" -ForegroundColor Yellow
    }
} else {
    Write-Host "  Mission Planner not found in Program Files" -ForegroundColor Gray
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
