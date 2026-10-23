# SPDX-License-Identifier: Apache-2.0
param(
    [Parameter(Mandatory = $true)][string]$SitlRuntimeDirectory
)

$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$output = Join-Path $root 'build/conference'
$runtime = (Resolve-Path -LiteralPath $SitlRuntimeDirectory).Path
$hostRoot = Join-Path ${env:ProgramFiles(x86)} 'Mission Planner'
$runtimeManifestHash = 'D7214B1626038936C56A773A0C6F031FE7A36ADCC0353534C0F0D535E6D4944F'
if ((Get-FileHash -LiteralPath (Join-Path $runtime 'manifest.json')).Hash -ne $runtimeManifestHash) {
    throw 'Supply the pinned conference Windows TERMINATE runtime, including its original manifest.'
}
$runtimeManifest = Get-Content -LiteralPath (Join-Path $runtime 'manifest.json') -Raw | ConvertFrom-Json
if ($runtimeManifest.target -ne 'Windows x86_64 Copter SITL' -or
    $runtimeManifest.source_identity.revision -ne 'fb773567896014da007bca58358704d4a0d8516f' -or
    $runtimeManifest.source_sha256 -ne 'f728fb7fc24260b3f85445e2a3f8ed2786ec642c256374a41db703fbb8b374ab' -or
    $runtimeManifest.source_identity.patch_sha256 -ne '4beb471e4b1fc769d6285c09c44fbd1618fab81caac0b13ca4c69a3a121d2be8' -or
    $runtimeManifest.verified_features.AP_ADVANCEDFAILSAFE_ENABLED -cne '1' -or
    $runtimeManifest.verified_features.AP_COPTER_ADVANCED_FAILSAFE_ENABLED -cne '1' -or
    $runtimeManifest.verified_features.AP_SCRIPTING_ENABLED -cne '0') {
    throw 'The runtime is not the reviewed native AFS/no-Lua Copter build.'
}
$runtimeFiles = @($runtimeManifest.files.PSObject.Properties)
foreach ($required in @('ArduCopter.exe', 'cygwin1.dll', 'copter.parm', 'COPYING.txt', 'SOURCE.txt',
    'source.json', 'controlled-termination.patch')) {
    if ($required -notin $runtimeFiles.Name) { throw "Runtime file missing: $required" }
}
foreach ($file in $runtimeFiles) {
    if ($file.Name -notmatch '^[A-Za-z0-9][A-Za-z0-9_.+-]*$' -or
        (Get-FileHash -LiteralPath (Join-Path $runtime $file.Name)).Hash -ne $file.Value) {
        throw "Runtime file changed or invalid: $($file.Name)"
    }
}
if (@(Get-ChildItem -LiteralPath $runtime -Force).Count -ne $runtimeFiles.Count + 1) {
    throw 'The runtime directory contains files outside the pinned cohort.'
}
$sitlHash = $runtimeManifest.files.'ArduCopter.exe'
if ([Diagnostics.FileVersionInfo]::GetVersionInfo((Join-Path $hostRoot 'MissionPlanner.exe')).FileVersion -ne '1.3.83') {
    throw 'Mission Planner 1.3.83 is required.'
}
New-Item -ItemType Directory -Path $output -Force | Out-Null

function Get-Package([string]$Url, [string]$Name, [string]$Hash, [string]$Destination) {
    $downloads = Join-Path $output 'simulators/downloads'
    New-Item -ItemType Directory -Path $downloads -Force | Out-Null
    $zip = Join-Path $downloads $Name
    if (!(Test-Path -LiteralPath $zip)) {
        Invoke-WebRequest -UseBasicParsing -Uri $Url -OutFile ($zip + '.partial')
        if ((Get-FileHash -LiteralPath ($zip + '.partial')).Hash -ne $Hash) {
            throw "Package hash mismatch: $Name"
        }
        Move-Item -LiteralPath ($zip + '.partial') -Destination $zip
    }
    if ((Get-FileHash -LiteralPath $zip).Hash -ne $Hash) {
        throw "Package hash mismatch: $Name"
    }
    if (!(Test-Path -LiteralPath $Destination)) {
        Expand-Archive -LiteralPath $zip -DestinationPath $Destination
    }
}

Get-Package 'https://github.com/iamaisim/ProjectAirSim/releases/download/v1.0.1/Neighborhood-Windows-1.0.1.zip' `
    'Neighborhood-Windows-1.0.1.zip' '55CFCB6BF2FFC3065A60396F471E41087AF5F59DD131896A9007DBF094B3F0AD' `
    (Join-Path $output 'simulators/neighborhood-1.0.1')
Get-Package 'https://github.com/bluenviron/mediamtx/releases/download/v1.21.2/mediamtx_v1.21.2_windows_amd64.zip' `
    'mediamtx_v1.21.2_windows_amd64.zip' 'A84224F5A490855684E62BA6FB71E80EA2A5E16AED61883BB58380290E93E467' `
    (Join-Path $output 'simulators/mediamtx-1.21.2')
$python = Join-Path $output 'python/Scripts/python.exe'
if (!(Test-Path -LiteralPath $python)) {
    & py -3.11 -m venv (Join-Path $output 'python')
    if ($LASTEXITCODE -ne 0) {
        throw 'Python 3.11 is required.'
    }
}
& $python -m pip install --disable-pip-version-check -r (Join-Path $PSScriptRoot 'requirements.txt') *> (Join-Path $output 'python-setup.log')
if ($LASTEXITCODE -ne 0) {
    throw 'Simulator client installation failed; see build/conference/python-setup.log.'
}

$prepared = Join-Path $output 'prepared'
$hostCopy = Join-Path $prepared 'host'
$sitl = Join-Path $prepared 'sitl'
$config = Join-Path $prepared 'config'
if (Test-Path -LiteralPath $sitl) {
    $previous = Join-Path $output ('sitl-before-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' +
        [Guid]::NewGuid().ToString('N').Substring(0, 6))
    foreach ($path in @($sitl, $previous)) {
        if (![IO.Path]::GetFullPath($path).StartsWith($output + [IO.Path]::DirectorySeparatorChar,
            [StringComparison]::OrdinalIgnoreCase)) { throw 'Runtime backup must stay under build/conference.' }
    }
    Move-Item -LiteralPath $sitl -Destination $previous
}
New-Item -ItemType Directory -Path $hostCopy, $sitl, $config -Force | Out-Null
foreach ($item in Get-ChildItem -LiteralPath $hostRoot) {
    if ($item.Name -notin @('plugins', 'Drivers')) {
        Copy-Item -LiteralPath $item.FullName -Destination $hostCopy -Recurse -Force
    }
}
$logs = Join-Path $prepared 'logs'
New-Item -ItemType Directory -Path $logs -Force | Out-Null
[xml]$hostConfiguration = Get-Content -LiteralPath (Join-Path $hostCopy 'MissionPlanner.exe.config') -Raw
foreach ($file in $hostConfiguration.SelectNodes('//log4net/appender/file')) {
    $file.SetAttribute('value', (Join-Path $logs 'mission-planner.log'))
}
$hostConfiguration.Save((Join-Path $hostCopy 'MissionPlanner.exe.config'))
$gst = 'C:/gstreamer/1.0/msvc_x86_64/bin/gst-launch-1.0.exe'
if (!(Test-Path -LiteralPath $gst)) {
    throw 'The 64-bit GStreamer runtime is required.'
}
$savedGstRegistry = $env:GST_REGISTRY
$savedGstPlugins = $env:GST_PLUGIN_PATH
try {
    $env:GST_REGISTRY = Join-Path $prepared 'gst-registry.bin'
    & $gst -q fakesrc num-buffers=1 '!' fakesink
    if ($LASTEXITCODE -ne 0) {
        throw 'GStreamer initialization failed.'
    }
    $nativeGst = Join-Path $env:ProgramData 'Mission Planner/gstreamer/1.0/x86_64'
    $env:GST_REGISTRY = Join-Path $prepared 'mp-gst-registry.bin'
    $env:GST_PLUGIN_PATH = Join-Path $nativeGst 'lib/gstreamer-1.0'
    & (Join-Path $nativeGst 'bin/gst-launch-1.0.exe') -q fakesrc num-buffers=1 '!' fakesink
    if ($LASTEXITCODE -ne 0) {
        throw 'Mission Planner GStreamer initialization failed.'
    }
} finally {
    $env:GST_REGISTRY = $savedGstRegistry
    $env:GST_PLUGIN_PATH = $savedGstPlugins
}
& powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $root 'scripts/build/build_plugin_windows.ps1')
if ($LASTEXITCODE -ne 0) {
    throw 'Plugin build failed.'
}
New-Item -ItemType Directory -Path (Join-Path $hostCopy 'plugins') -Force | Out-Null
$plugin = Join-Path $root 'mission_planner/src/bin/Release/NOMADPlugin.dll'
Copy-Item -LiteralPath $plugin -Destination (Join-Path $hostCopy 'plugins/NOMADPlugin.dll') -Force
Copy-Item -Path (Join-Path $runtime '*') -Destination $sitl
Copy-Item -LiteralPath (Join-Path $runtime 'copter.parm') -Destination $config -Force
Copy-Item -LiteralPath (Join-Path $root 'mission_planner/tests/native_sitl/outputs.parm') -Destination $config -Force
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'termination.parm') -Destination $config -Force
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'descent.py') -Destination $config -Force
Add-Content -LiteralPath (Join-Path $config 'outputs.parm') -Value "`nSERIAL1_PROTOCOL 2`nSERIAL2_PROTOCOL 2`nFS_CRASH_CHECK 1"
$pas = 'https://raw.githubusercontent.com/iamaisim/ProjectAirSim/v1.0.1/client/python/example_user_scripts/ardupilot/'
foreach ($file in @('scene_ardu_quadrotor.jsonc', 'robot_ardu_quadrotor.jsonc')) {
    Invoke-WebRequest -UseBasicParsing -Uri ($pas + 'sim_config/' + $file) -OutFile (Join-Path $config $file)
}
Invoke-WebRequest -UseBasicParsing -Uri ($pas + 'project-airsim-quad.param') -OutFile (Join-Path $config 'project-airsim-quad.param')
Invoke-WebRequest -UseBasicParsing -Uri 'https://raw.githubusercontent.com/ArduPilot/ardupilot/fb773567896014da007bca58358704d4a0d8516f/Tools/autotest/default_params/airsim-quadX.parm' -OutFile (Join-Path $config 'airsim-quadX.parm')
$adjust = @'
import commentjson,json,sys
from pathlib import Path
root=Path(sys.argv[1])
scene=commentjson.loads((root/'scene_ardu_quadrotor.jsonc').read_text())
scene['clock'].update({'step-ns':1000000,'real-time-update-rate':1000000})
(root/'scene_ardu_quadrotor.jsonc').write_text(json.dumps(scene,indent=2))
robot=commentjson.loads((root/'robot_ardu_quadrotor.jsonc').read_text())
for camera in robot['sensors']:
    if camera['type']!='camera':continue
    if camera['id']=='DownCamera':camera['origin']['xyz']='0.3 0 0.35'
    camera['capture-interval']=.1
    for capture in camera['capture-settings']:
        capture.update({'width':640,'height':360,'capture-enabled':capture['image-type']==0,'streaming-enabled':capture['image-type']==0})
(root/'robot_ardu_quadrotor.jsonc').write_text(json.dumps(robot,indent=2))
'@
& $python -c $adjust $config
if ($LASTEXITCODE -ne 0) {
    throw 'Simulator configuration failed.'
}
. (Join-Path $root 'scripts/build/windows_build_tools.ps1')
$compiler = Find-RoslynCompiler
& $compiler /nologo /target:exe /platform:x64 /langversion:latest "/out:$prepared/NOMADHost.exe" `
    /r:System.Drawing.dll /r:System.Windows.Forms.dll `
    ('/r:' + (Join-Path $hostRoot 'MissionPlanner.Utilities.dll')) `
    ('/r:' + (Join-Path ${env:ProgramFiles(x86)} 'Reference Assemblies/Microsoft/Framework/.NETFramework/v4.8/Facades/netstandard.dll')) `
    (Join-Path $PSScriptRoot 'host/Program.cs')
if ($LASTEXITCODE -ne 0) {
    throw 'Native host bootstrap build failed.'
}
$files = @(Get-ChildItem -LiteralPath $sitl, $config -File) + @(
    Get-Item -LiteralPath (Join-Path $prepared 'NOMADHost.exe'),
        (Join-Path $hostCopy 'MissionPlanner.exe'), (Join-Path $hostCopy 'MissionPlanner.exe.config'),
        (Join-Path $hostCopy 'plugins/NOMADPlugin.dll'),
        (Join-Path $output 'simulators/neighborhood-1.0.1/AirSimNH/Binaries/Win64/AirSimNH-Win64-Shipping.exe'),
        (Join-Path $output 'simulators/mediamtx-1.21.2/mediamtx.exe'))
$manifest = [ordered]@{
    mission_planner = '1.3.83'
    projectairsim = '1.0.1'
    python_client = '1.0.2'
    mediamtx = '1.21.2'
    sitl_revision = 'fb773567896014da007bca58358704d4a0d8516f'
    sitl_sha256 = $sitlHash
    sitl_manifest_sha256 = $runtimeManifestHash
    plugin_sha256 = (Get-FileHash -LiteralPath $plugin).Hash
    files = @($files | ForEach-Object {
        @{
            name = $_.FullName.Substring($output.Length + 1)
            sha256 = (Get-FileHash -LiteralPath $_.FullName).Hash
        }
    })
}
$manifest | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $prepared 'manifest.json') -Encoding utf8
Write-Output 'Prepared actual Mission Planner, NOMAD, Project AirSim and pinned SITL under build/conference/prepared.'
