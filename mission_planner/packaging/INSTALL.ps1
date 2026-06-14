# NOMAD Mission Planner plugin installer.
#
# Copies NOMADPlugin.dll into Mission Planner's installation plugins folder.
# Run from the folder this script lives in:
#
#   powershell -ExecutionPolicy Bypass -File INSTALL.ps1
#
# Close Mission Planner first, then restart it after installing.

$ErrorActionPreference = "Stop"

$dll = Join-Path $PSScriptRoot "NOMADPlugin.dll"
if (-not (Test-Path $dll)) {
    throw "NOMADPlugin.dll not found next to this script ($PSScriptRoot)."
}

$missionPlannerDir = "${env:ProgramFiles(x86)}\Mission Planner"
$missionPlannerExe = Join-Path $missionPlannerDir "MissionPlanner.exe"
if (-not (Test-Path $missionPlannerExe)) {
    throw "Mission Planner not found at $missionPlannerExe."
}

$plugins = Join-Path $missionPlannerDir "plugins"
New-Item -ItemType Directory -Force -Path $plugins | Out-Null
Copy-Item -Path $dll -Destination $plugins -Force
Write-Host "Installed NOMADPlugin.dll -> $plugins" -ForegroundColor Green

$legacyDll = Join-Path $env:LOCALAPPDATA "Mission Planner\plugins\NOMADPlugin.dll"
if (Test-Path $legacyDll) {
    Remove-Item $legacyDll -Force
    Write-Host "Removed legacy AppData copy: $legacyDll" -ForegroundColor Gray
}

Write-Host "Restart Mission Planner to load the plugin." -ForegroundColor Cyan
