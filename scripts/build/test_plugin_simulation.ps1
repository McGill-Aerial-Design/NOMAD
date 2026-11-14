# SPDX-License-Identifier: Apache-2.0
# Owned controllers/loopback fixtures. Never launches Mission Planner or opens physical devices.
param([string]$NativeHost = '', [int]$NativePort = 0, [int]$VideoTcpPort = 0, [string]$DecoderRuntime = '',
    [string]$OutputDirectory = '', [string]$WslDistribution = '', [string]$LinuxWorkspace = '',
    [string]$LauncherScenario = '', [string]$LauncherToolsDirectory = '')
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$compiler = Find-RoslynCompiler
$hostRoot = Join-Path ${env:ProgramFiles(x86)} 'Mission Planner'
$output = Join-Path $root 'build/tests/simulation-controls'
New-Item -ItemType Directory -Path $output -Force | Out-Null
$plugin = Join-Path $root 'mission_planner/src/bin/Release/NOMADPlugin.dll'
$references = @('/r:System.Drawing.dll', '/r:System.Windows.Forms.dll', "/r:$plugin")
$references += '/r:' + (Join-Path ${env:ProgramFiles(x86)} 'Reference Assemblies/Microsoft/Framework/.NETFramework/v4.8/Facades/netstandard.dll')
foreach ($name in @('MissionPlanner.exe', 'MissionPlanner.Comms.dll', 'MissionPlanner.ArduPilot.dll',
    'MissionPlanner.Utilities.dll', 'MissionPlanner.Controls.dll', 'MAVLink.dll', 'Interfaces.dll', 'Newtonsoft.Json.dll', 'System.Memory.dll')) {
    $references += '/r:' + (Join-Path $hostRoot $name)
}
$exe = Join-Path $output 'SimulationControlTests.exe'
$references += Get-WpfTestReferences
& $compiler /nologo /target:exe /platform:x64 /langversion:latest "/out:$exe" `
    ("/win32manifest:" + (Join-Path $root 'mission_planner/tests/ui/host.manifest')) @references `
    (Join-Path $root 'mission_planner/tests/simulation/SimulationControlTests.cs')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Copy-Item -LiteralPath $plugin -Destination $output -Force
$attempt = if ($OutputDirectory) { [IO.Path]::GetFullPath($OutputDirectory) } else {
    Join-Path $output ('attempt-' + [Guid]::NewGuid().ToString('N')) }
New-Item -ItemType Directory -Path $attempt -Force | Out-Null
if ($VideoTcpPort -gt 0 -and -not $DecoderRuntime) { throw 'Choose an existing -DecoderRuntime prefix for the video probe.' }
if ($WslDistribution) {
    if (!$LinuxWorkspace -or !$LauncherScenario) { throw 'The opt-in WSL launch probe needs -LinuxWorkspace and -LauncherScenario.' }
    $tools = if ($LauncherToolsDirectory) { [IO.Path]::GetFullPath($LauncherToolsDirectory) } else { '' }
    & $exe $hostRoot $attempt wsl $WslDistribution $LinuxWorkspace ([IO.Path]::GetFullPath($LauncherScenario)) $DecoderRuntime $tools
}
elseif ($NativeHost) { & $exe $hostRoot $attempt native $NativeHost $NativePort $VideoTcpPort $DecoderRuntime }
else { & $exe $hostRoot $attempt }
exit $LASTEXITCODE
