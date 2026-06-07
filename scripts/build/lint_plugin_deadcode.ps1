# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# Dead-code lint for the NOMAD Mission Planner plugin.
#
# Rebuilds NOMADPlugin.csproj with the C# compiler's dead-code diagnostics
# promoted to errors, so the check fails when dead code is introduced:
#
#   CS0162  unreachable code detected
#   CS0168  variable declared but never used
#   CS0169  private field never used
#   CS0219  variable assigned but its value never used
#   CS0414  private field assigned but never used
#   CS0649  field never assigned (always default)
#   CS1717  assignment made to the same variable
#
# Usage:  pixi run lint-plugin   (or run this script directly)
#
# To intentionally keep flagged code, wrap it in:
#   #pragma warning disable CS0169
#   ...
#   #pragma warning restore CS0169
# =============================================================================
$ErrorActionPreference = 'Stop'

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot  = Split-Path -Parent (Split-Path -Parent $ScriptDir)
$Project   = Join-Path $RepoRoot 'mission_planner\src\NOMADPlugin.csproj'

# ---- Locate MSBuild (PATH, then Visual Studio via vswhere) ----
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

$deadCodeWarnings = 'CS0162;CS0168;CS0169;CS0219;CS0414;CS0649;CS1717'

Write-Host "Dead-code lint (plugin): $Project" -ForegroundColor Cyan
Write-Host "Promoting to errors: $deadCodeWarnings`n" -ForegroundColor Gray

# Rebuild (not Build) so every file is recompiled and all diagnostics surface.
& $msbuild $Project /t:Rebuild /p:Configuration=Release /warnaserror:$deadCodeWarnings /nologo /v:minimal
$code = $LASTEXITCODE

Write-Host ""
if ($code -ne 0) {
    Write-Host "Dead-code check FAILED. Remove the flagged code, or justify it with a" -ForegroundColor Red
    Write-Host "scoped '#pragma warning disable <code>' if it must stay." -ForegroundColor Red
    exit 1
}
Write-Host "No dead code detected." -ForegroundColor Green
