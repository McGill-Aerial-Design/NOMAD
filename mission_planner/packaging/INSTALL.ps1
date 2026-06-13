# NOMAD Mission Planner plugin installer.
#
# Copies NOMADPlugin.dll into Mission Planner's per-user plugins folder
# (%LOCALAPPDATA%\Mission Planner\plugins) — no admin required. Run from the
# folder this script lives in:
#
#   powershell -ExecutionPolicy Bypass -File INSTALL.ps1
#
# Close Mission Planner first, then restart it after installing.

$ErrorActionPreference = "Stop"

$dll = Join-Path $PSScriptRoot "NOMADPlugin.dll"
if (-not (Test-Path $dll)) {
    throw "NOMADPlugin.dll not found next to this script ($PSScriptRoot)."
}

$plugins = Join-Path $env:LOCALAPPDATA "Mission Planner\plugins"
New-Item -ItemType Directory -Force -Path $plugins | Out-Null
Copy-Item -Path $dll -Destination $plugins -Force
Write-Host "Installed NOMADPlugin.dll -> $plugins" -ForegroundColor Green

# Mission Planner also scans its install folder. A copy in BOTH places loads the
# plugin twice (duplicate panels/config), so warn about a Program Files copy.
$pfDll = "${env:ProgramFiles(x86)}\Mission Planner\plugins\NOMADPlugin.dll"
if (Test-Path $pfDll) {
    Write-Host "WARNING: another NOMADPlugin.dll exists at" -ForegroundColor Yellow
    Write-Host "  $pfDll" -ForegroundColor Yellow
    Write-Host "  Delete it (Run as Administrator) or Mission Planner loads the plugin twice." -ForegroundColor Yellow
}

Write-Host "Restart Mission Planner to load the plugin." -ForegroundColor Cyan
