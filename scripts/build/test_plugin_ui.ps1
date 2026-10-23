# SPDX-License-Identifier: Apache-2.0
# Tests owned inactive controls and sampled state; never starts or controls Mission Planner.
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$compiler = Find-RoslynCompiler
$hostRoot = Join-Path ${env:ProgramFiles(x86)} 'Mission Planner'
$output = Join-Path $root 'build/tests/ui'
New-Item -ItemType Directory -Path $output -Force | Out-Null
$plugin = Join-Path $root 'mission_planner/src/bin/Release/NOMADPlugin.dll'
$references = @('/r:System.Drawing.dll', '/r:System.Windows.Forms.dll', "/r:$plugin")
foreach ($name in @('MissionPlanner.exe', 'MissionPlanner.Controls.dll', 'MissionPlanner.Utilities.dll',
    'Newtonsoft.Json.dll', 'Interfaces.dll')) {
    $references += '/r:' + (Join-Path $hostRoot $name)
}
$exe = Join-Path $output 'PluginUiTests.exe'
& $compiler /nologo /target:exe /langversion:latest "/out:$exe" @references `
    (Join-Path $root 'mission_planner/tests/ui/PluginUiTests.cs')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Copy-Item -LiteralPath $plugin -Destination $output -Force
$attempt = Join-Path $output ('attempt-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $attempt | Out-Null
& $exe $hostRoot $attempt
exit $LASTEXITCODE
