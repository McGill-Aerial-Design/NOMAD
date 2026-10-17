# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# ============================================================
# Standalone router and Mission Planner management-client stress tests
# ============================================================
# Compiles the standalone routing stack, management client, and Log.cs
# together with the test runner using the Roslyn csc bundled
# with Visual Studio's MSBuild — no .NET SDK or test-framework
# packages required. The tests run the router against real
# loopback UDP sockets: dedup, failover, manual override,
# param-transaction pinning, watchdog reopen, and high-rate
# mirrored stress with strict no-loss/no-duplication checks.
#
# Usage: pixi run test-plugin-duallink
# Exits non-zero on compile error or test failure.
# ============================================================

$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..\..')

# ---- Locate csc (next to MSBuild: PATH, then Visual Studio via vswhere) ----
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$csc = Find-RoslynCompiler

# ---- Compile ----
$sources = @(
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\MAVLinkConnectionManager.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\MAVLinkConnectionManager.Standalone.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\StandaloneRouterClient.cs'),
    (Join-Path $repoRoot 'mission_planner\src\Connectivity\StandaloneRouterClient.Mapping.cs'),
    (Join-Path $repoRoot 'mission_planner\src\UI\Log.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\duallink\DualLinkStressTests.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\duallink\DualLinkStressTests.Harness.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\duallink\DualLinkStressTests.Router.cs'),
    (Join-Path $repoRoot 'mission_planner\tests\duallink\DualLinkStressTests.Stress.cs')
)
$sources += Get-ChildItem (Join-Path $repoRoot 'infra\transport\ground_router\*.cs') | ForEach-Object FullName
$sources += Join-Path $repoRoot 'mission_planner/tests/duallink/MultiLinkTests.cs'
$sources += Join-Path $repoRoot 'mission_planner/tests/duallink/RouterReviewTests.cs'
$sources += Join-Path $repoRoot 'mission_planner/tests/duallink/RouterTimingTests.cs'
$sources += Join-Path $repoRoot 'mission_planner/tests/duallink/RouterManagementTests.cs'
$sources += Join-Path $repoRoot 'mission_planner/src/Panels/LinkStatusDisplay.cs'
$outDir = Join-Path $repoRoot 'mission_planner\tests\duallink\bin'
New-Item -ItemType Directory -Force $outDir | Out-Null
$exe = Join-Path $outDir 'DualLinkStressTests.exe'

Write-Host "Compiling dual-link tests..." -ForegroundColor Yellow
& $csc /nologo /target:exe /langversion:latest /r:System.Web.Extensions.dll "/out:$exe" @sources
if ($LASTEXITCODE -ne 0) {
    Write-Host "Compile FAILED." -ForegroundColor Red
    exit 1
}

# ---- Run ----
Write-Host "Running dual-link tests (uses loopback UDP ports 28600+)..." -ForegroundColor Yellow
& $exe
$result = $LASTEXITCODE
if ($result -ne 0) {
    Write-Host "Dual-link tests FAILED." -ForegroundColor Red
}
exit $result
