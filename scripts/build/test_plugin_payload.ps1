# SPDX-License-Identifier: Apache-2.0
$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '../..')
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$csc = Find-RoslynCompiler
$mp = Join-Path ${env:ProgramFiles(x86)} 'Mission Planner'
$json = Join-Path $mp 'Newtonsoft.Json.dll'
$outDir = Join-Path $repoRoot 'build/tests/payload'
New-Item -ItemType Directory -Force $outDir | Out-Null
Copy-Item -LiteralPath $json -Destination $outDir -Force
$sources = @('Control/GimbalCommand.cs','Control/GimbalController.cs','Control/OutputController.cs',
    'Control/OutputController.Payload.cs','Control/PayloadDefinition.cs','Control/PayloadController.cs',
    'Input/InputBinding.cs','Input/InputControlState.cs','Input/InputListenState.cs','Input/GimbalInputState.cs','Input/NomadJoystickService.cs','Input/NomadJoystickService.Axis.cs',
    'Input/NomadJoystickService.Payload.cs','Config/NOMADConfig.Input.cs') | ForEach-Object { Join-Path $repoRoot "mission_planner/src/$_" }
$sources += Join-Path $repoRoot 'mission_planner/tests/gimbal/PayloadTests.cs'
$exe = Join-Path $outDir 'PayloadTests.exe'
& $csc /nologo /target:exe /langversion:latest /r:System.Windows.Forms.dll /r:System.Drawing.dll "/r:$json" "/out:$exe" @sources
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe
exit $LASTEXITCODE
