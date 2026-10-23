# SPDX-License-Identifier: Apache-2.0
# Actual host/editor/parameter APIs with isolated settings and a synthetic loopback peer. Never starts MP.
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$compiler = Find-RoslynCompiler
$hostRoot = Join-Path ${env:ProgramFiles(x86)} 'Mission Planner'
$output = Join-Path $root 'build/tests/fence'
New-Item -ItemType Directory -Path $output -Force | Out-Null
$plugin = Join-Path $root 'mission_planner/src/bin/Release/NOMADPlugin.dll'
$references = @('/r:System.Drawing.dll', '/r:System.Windows.Forms.dll', "/r:$plugin")
foreach ($name in @('MissionPlanner.exe', 'MissionPlanner.ArduPilot.dll', 'MissionPlanner.Comms.dll',
    'MissionPlanner.Controls.dll', 'MissionPlanner.Utilities.dll', 'MAVLink.dll', 'Interfaces.dll',
    'GMap.NET.Core.dll', 'GMap.NET.Drawing.dll', 'GMap.NET.WindowsForms.dll')) {
    $references += '/r:' + (Join-Path $hostRoot $name)
}
$references += '/r:' + (Join-Path ${env:ProgramFiles(x86)} 'Reference Assemblies/Microsoft/Framework/.NETFramework/v4.8/Facades/netstandard.dll')
$exe = Join-Path $output 'NativeFenceParametersTests.exe'
& $compiler /nologo /target:exe /langversion:latest "/out:$exe" @references `
    (Join-Path $root 'mission_planner/tests/geometry/NativeFenceParametersTests.cs')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Copy-Item -LiteralPath $plugin -Destination $output -Force
$attempt = Join-Path $output ('attempt-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $attempt | Out-Null
& $exe $hostRoot $attempt
exit $LASTEXITCODE
