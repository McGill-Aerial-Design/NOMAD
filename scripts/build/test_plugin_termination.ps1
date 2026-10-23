# SPDX-License-Identifier: Apache-2.0
# Actual installed MP APIs, built product DLL and one owned synthetic loopback peer. Never installs or starts MP.
param([switch]$CompileOnly)
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$compiler = Find-RoslynCompiler
$hostRoot = Join-Path ${env:ProgramFiles(x86)} 'Mission Planner'
$output = Join-Path $root 'build/tests/termination'
New-Item -ItemType Directory -Path $output -Force | Out-Null
$plugin = Join-Path $root 'mission_planner/src/bin/Release/NOMADPlugin.dll'
$references = @('/r:System.Drawing.dll', '/r:System.Windows.Forms.dll', '/r:System.Xml.Linq.dll', "/r:$plugin")
foreach ($name in @('MissionPlanner.exe', 'MissionPlanner.ArduPilot.dll', 'MissionPlanner.Comms.dll',
    'MissionPlanner.Utilities.dll', 'MAVLink.dll', 'Interfaces.dll')) {
    $references += '/r:' + (Join-Path $hostRoot $name)
}
$references += '/r:' + (Join-Path ${env:ProgramFiles(x86)} 'Reference Assemblies/Microsoft/Framework/.NETFramework/v4.8/Facades/netstandard.dll')
$exe = Join-Path $output 'NativeTerminationTests.exe'
& $compiler /nologo /target:exe /langversion:latest "/out:$exe" @references `
    (Join-Path $root 'mission_planner/tests/termination/NativeTerminationTests.cs')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Copy-Item -LiteralPath $plugin -Destination $output -Force
if ($CompileOnly) { exit 0 }
$attempt = Join-Path $output ('attempt-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $attempt | Out-Null
& $exe $hostRoot $attempt
exit $LASTEXITCODE
