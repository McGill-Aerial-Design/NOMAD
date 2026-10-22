# SPDX-License-Identifier: Apache-2.0
# Validate coordinates and stage native Fence rows without MP startup, files or a live connection.
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$csc = Find-RoslynCompiler
$mpDir = Join-Path ${env:ProgramFiles(x86)} 'Mission Planner'
$outDir = Join-Path $repoRoot 'mission_planner/tests/geometry/bin'
New-Item -ItemType Directory -Force $outDir | Out-Null
$exe = Join-Path $outDir 'NativeFenceDraftTests.exe'
$references = @('/r:System.Drawing.dll', '/r:System.Windows.Forms.dll')
foreach ($name in @('MissionPlanner.exe', 'MissionPlanner.ArduPilot.dll', 'MissionPlanner.Comms.dll',
    'MissionPlanner.Controls.dll', 'MissionPlanner.Utilities.dll', 'GMap.NET.Core.dll',
    'GMap.NET.Drawing.dll', 'GMap.NET.WindowsForms.dll', 'MAVLink.dll', 'Interfaces.dll')) {
    $references += '/r:' + (Join-Path $mpDir $name)
}
$references += '/r:' + (Join-Path ${env:ProgramFiles(x86)} 'Reference Assemblies/Microsoft/Framework/.NETFramework/v4.8/Facades/netstandard.dll')
& $csc /nologo /nowarn:0436 /target:exe /langversion:latest "/out:$exe" @references `
    (Join-Path $repoRoot 'mission_planner/src/Geofence/FenceCoordinates.cs') `
    (Join-Path $repoRoot 'mission_planner/src/Geofence/NativeFenceDraft.cs') `
    (Join-Path $repoRoot 'mission_planner/tests/geometry/NativeFenceDraftTests.cs')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe $mpDir
exit $LASTEXITCODE
