#!/usr/bin/env pwsh
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# NOMAD development environment bootstrap (Windows / PowerShell)
#
# Usage:
#   pwsh scripts/setup/bootstrap.ps1
#   pixi run bootstrap
#
# This script:
#   1. Checks prerequisites (git, pixi, Visual Studio for C# build)
#   2. Runs `pixi install` to set up the reproducible dev environment
#   3. Installs pre-commit hooks
#   4. Builds the C++ core and lists the supported product profiles
# =============================================================================

param(
    [switch]$SkipHooks,
    [switch]$Quiet
)

$Host.UI.RawUI.WindowTitle = "NOMAD Bootstrap"

# ---- colors ----
$cInfo = 'Cyan'
$cOk   = 'Green'
$cWarn = 'Yellow'
$cErr  = 'Red'

function Log-Info  { Write-Host "[INFO]  $($args -join ' ')" -ForegroundColor $cInfo }
function Log-Ok    { Write-Host "[OK]    $($args -join ' ')" -ForegroundColor $cOk }
function Log-Warn  { Write-Host "[WARN]  $($args -join ' ')" -ForegroundColor $cWarn }
function Log-Error { Write-Host "[ERROR] $($args -join ' ')" -ForegroundColor $cErr }

# ---- Step 0: repo root ----
$RepoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
Set-Location $RepoRoot
Log-Info "NOMAD bootstrap - $RepoRoot"

# ---- Step 0a: suppress pixi/conda SSL cert dir warning on Windows ----
$env:SSL_CERT_DIR = $null

# ---- Step 0b: ensure pixi is in PATH ----
$pixiBin = "$env:USERPROFILE\.pixi\bin"
if (-not (Get-Command pixi -ErrorAction SilentlyContinue)) {
    if (Test-Path "$pixiBin\pixi.exe") {
        $env:Path += ";$pixiBin"
        Log-Info "Added pixi to PATH for this session"
    } else {
        Log-Error "Pixi not found. Install from https://pixi.sh"
        exit 1
    }
}
$pixiExe = (Get-Command pixi).Source
Log-Ok "Pixi found at $pixiExe"

# ---- Step 1: prerequisites ----
Log-Info "Step 1/5 - Checking prerequisites"

if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    Log-Error "Git not found. Install from https://git-scm.com/"
    exit 1
}
Log-Ok "Git found"

$msbuild = Get-Command msbuild -ErrorAction SilentlyContinue
if (-not $msbuild) {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $vsPath = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -property installationPath
        if ($vsPath) {
            $msbuild = Join-Path $vsPath 'MSBuild\Current\Bin\MSBuild.exe'
        }
    }
}
if ($msbuild -and (Test-Path $msbuild)) {
    Log-Ok "Visual Studio / MSBuild available (for Mission Planner plugin)"
} else {
    Log-Warn "Visual Studio not found - Mission Planner plugin build unavailable"
    Log-Warn "  Install Visual Studio 2022 with .NET desktop workload"
}

$mp = "${env:ProgramFiles(x86)}\Mission Planner"
if (Test-Path $mp) {
    Log-Ok "Mission Planner reference assemblies found at $mp"
} else {
    Log-Warn "Mission Planner not found - plugin build needs staged reference assemblies"
}

# ---- Step 2: pixi install ----
Log-Info "Step 2/5 - Installing pixi environment"
& $pixiExe install
if ($LASTEXITCODE -ne 0) {
    Log-Error "pixi install failed"
    exit 1
}
Log-Ok "Pixi environment installed"

# ---- Step 3: pre-commit hooks ----
if (-not $SkipHooks) {
    Log-Info "Step 3/5 - Installing pre-commit hooks"
    & $pixiExe run pre-commit install
    if ($LASTEXITCODE -ne 0) {
        Log-Warn "pre-commit had issues (may need 'git init' first)"
    }
    Log-Ok "Pre-commit hooks installed"
} else {
    Log-Info "Step 3/5 - Skipped pre-commit hooks"
}

# ---- Step 4: C++ core and product profile checks ----
Log-Info "Step 4/5 - Building the C++ core"
& $pixiExe run build-core
if ($LASTEXITCODE -ne 0) {
    Log-Error "C++ core build failed"
    exit 1
}
Log-Ok "C++ core build OK"

Log-Info "Checking supported product profiles"
& $pixiExe run profile-list
if ($LASTEXITCODE -ne 0) {
    Log-Error "Product profile manager check failed"
    exit 1
}
Log-Ok "Product profile manager OK"

# ---- Step 5: summary ----
Log-Info "Step 5/5 - Done"
Write-Host ""
Write-Host "======================================" -ForegroundColor $cInfo
Write-Host " NOMAD dev environment ready!" -ForegroundColor $cOk
Write-Host "======================================" -ForegroundColor $cInfo
Write-Host ""
Write-Host "Quick commands:" -ForegroundColor $cWarn
Write-Host "  pixi run build-core     Build the production C++ executables"
Write-Host "  pixi run dev-up         Start the hardware-free SITL stack"
Write-Host "  pixi run test           Run pytest"
Write-Host "  pixi run lint           Run ruff check"
Write-Host "  pixi run fmt            Auto-format all Python"
Write-Host "  pixi run docs           Serve the canonical docs site"
Write-Host "  pixi run build-plugin-only  Compile plugin without deployment (Windows)"
