# SPDX-License-Identifier: Apache-2.0
# Real native parameter callbacks, seeded in-memory metadata; no host startup or parameter writes.
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$csc = Find-RoslynCompiler
$mpDir = Join-Path ${env:ProgramFiles(x86)} 'Mission Planner'
$outDir = Join-Path $root 'build/tests/native-land-draft'
New-Item -ItemType Directory -Path $outDir -Force | Out-Null
$exe = Join-Path $outDir 'NativeLandParameterDraftTests.exe'
$references = @('/r:System.Drawing.dll','/r:System.Windows.Forms.dll')
foreach ($name in @('MissionPlanner.exe','MissionPlanner.ArduPilot.dll','MissionPlanner.Comms.dll',
    'MissionPlanner.Controls.dll','MissionPlanner.Utilities.dll','MAVLink.dll','Interfaces.dll',
    'Microsoft.Extensions.Caching.Memory.dll','Microsoft.Extensions.Caching.Abstractions.dll')) {
    $references += '/r:' + (Join-Path $mpDir $name)
}
$references += '/r:' + (Join-Path ${env:ProgramFiles(x86)} 'Reference Assemblies/Microsoft/Framework/.NETFramework/v4.8/Facades/netstandard.dll')
& $csc /nologo /target:exe /langversion:latest "/out:$exe" @references `
    (Join-Path $root 'mission_planner/src/Geofence/NativeLandParameterDraft.cs') `
    (Join-Path $root 'mission_planner/tests/parameters/NativeLandParameterDraftTests.cs')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$preserved = @(
    (Join-Path $env:USERPROFILE 'Documents/Mission Planner/config.xml'),
    (Join-Path $env:LOCALAPPDATA 'Mission Planner/plugins/nomad_aircraft_profiles.json'),
    (Join-Path $env:LOCALAPPDATA 'Mission Planner/plugins/nomad_aircraft_profiles.json.bak'),
    (Join-Path $env:LOCALAPPDATA 'Mission Planner/plugins/nomad_config.json'),
    (Join-Path $env:LOCALAPPDATA 'Mission Planner/plugins/nomad_config.json.bak'),
    (Join-Path $env:LOCALAPPDATA 'Mission Planner/plugins/NOMAD/geofence_config.json')
)
$before = @{}
function Protected-Hash([string]$path) {
    if (!(Test-Path -LiteralPath $path)) { return '' }
    $hash = [Security.Cryptography.SHA256]::Create(); $stream = $null
    try {
        $stream = [IO.File]::OpenRead($path)
        return [Convert]::ToBase64String($hash.ComputeHash($stream))
    } finally {
        if ($stream) { $stream.Dispose() }
        $hash.Dispose()
    }
}
foreach ($path in $preserved) {
    $before[$path] = Protected-Hash $path
}
& $exe $mpDir
$result = $LASTEXITCODE
foreach ($path in $preserved) {
    $after = Protected-Hash $path
    if ($after -ne $before[$path]) { throw 'Native draft fixture changed a protected operator configuration.' }
}
Write-Host 'Protected operator configuration files unchanged.'
exit $result
