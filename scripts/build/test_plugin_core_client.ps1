# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# ============================================================
# NomadCoreClient boundary tests for the Mission Planner plugin
# ============================================================
# Compiles the Mission Planner-free NomadCoreClient (the plugin's client for
# the C++ runtime IPC boundary) together with the test runner using the Roslyn
# csc bundled with Visual Studio's MSBuild â€” no .NET SDK or test-framework
# packages required.
#
# The harness exercises typed runtime requests, fail-closed validation,
# explicit GuidedGoto unavailability, authority controls, protocol negotiation,
# and unknown-outcome no-replay behavior without Mission Planner assemblies.
#
# Usage: pixi run test-plugin-core-client
# Exits non-zero on compile error or test failure.
# ============================================================

$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..\..')

# ---- Locate csc (next to MSBuild: PATH, then Visual Studio via vswhere) ----
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
$csc = Join-Path (Split-Path $msbuild) 'Roslyn\csc.exe'
if (-not (Test-Path $csc)) {
    Write-Host "ERROR: csc.exe not found at $csc" -ForegroundColor Red
    exit 1
}

# ---- Compile ----
$sources = @(
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\NomadCoreClient.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\NomadCoreRequestResult.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\NomadRuntimeClient.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\NomadRuntimeClient.Network.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\NomadRuntimeClient.Requests.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Control\FlightModeController.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Control\GimbalCommand.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Control\GimbalController.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\coreclient\TerminationRequestTests.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\coreclient\NomadCoreClientTests.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\coreclient\NomadCoreClientRuntimeTests.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\coreclient\NomadCoreClientAsyncTests.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\coreclient\NomadCoreClientGimbalTests.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\coreclient\AsyncRuntimeTestFixture.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\coreclient\NomadCoreClientOutcomeTests.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\coreclient\OutputOutcomeTests.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\coreclient\OutputTestFixtures.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\coreclient\JoystickMappingTests.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\coreclient\JoystickInputConfigurationTests.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Input\NomadJoystickService.Switches.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Input\NomadJoystickService.Axis.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Control\OutputController.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Config\NOMADConfig.Input.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\NomadActuatorModels.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\NomadCoreClient.Actuators.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\NomadRuntimeClient.Actuators.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Actuators\ActuatorControlPanel.cs')
)
$outDir = Join-Path $repoRoot 'mission_planner\tests\coreclient\bin'
New-Item -ItemType Directory -Force $outDir | Out-Null
$exe = Join-Path $outDir 'NomadCoreClientTests.exe'

Write-Host "Compiling core-client tests..." -ForegroundColor Yellow
$frameworkReferenceDirectory = Join-Path "${env:ProgramFiles(x86)}" 'Reference Assemblies\Microsoft\Framework\.NETFramework\v4.8'
$systemWebExtensions = Join-Path $frameworkReferenceDirectory 'System.Web.Extensions.dll'
if (-not (Test-Path $systemWebExtensions)) {
    Write-Host "ERROR: .NET Framework 4.8 reference assembly not found at $systemWebExtensions" -ForegroundColor Red
    exit 1
}
& $csc /nologo /target:exe /langversion:latest /reference:System.Drawing.dll /reference:System.Windows.Forms.dll "/reference:$systemWebExtensions" "/out:$exe" @sources
if ($LASTEXITCODE -ne 0) {
    Write-Host "Compile FAILED." -ForegroundColor Red
    exit 1
}

# ---- Run ----
Write-Host "Running core-client tests..." -ForegroundColor Yellow
& $exe
$result = $LASTEXITCODE
if ($result -ne 0) {
    Write-Host "Core-client tests FAILED." -ForegroundColor Red
}
exit $result
