# SPDX-License-Identifier: Apache-2.0
# Copies reviewed binaries only; never starts or changes the installed host.
param([string]$HostDirectory = 'C:\Program Files (x86)\Mission Planner', [switch]$Media)
$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
. (Join-Path $root 'scripts/build/windows_build_tools.ps1')
$version = (Get-Content (Join-Path $root 'scripts/release/mission-planner-target.json') -Raw | ConvertFrom-Json).version
$hostExe = Join-Path $HostDirectory 'MissionPlanner.exe'
if ([Diagnostics.FileVersionInfo]::GetVersionInfo($hostExe).FileVersion -ne $version) {
    throw 'Reviewed host version mismatch.'
}
$plugin = Join-Path $root 'mission_planner/src/bin/Release/NOMADPlugin.dll'
if (!(Test-Path -LiteralPath $plugin -PathType Leaf)) { throw 'Build the plugin before preparing the owned host.' }
$build = Get-CurrentPluginBuild -RepoRoot $root -PluginPath $plugin
$id = [Guid]::NewGuid().ToString('N')
$run = Join-Path $root ('build/tests/simulation/run-' + $id)
$hostCopy = Join-Path $run 'host'
New-Item -ItemType Directory -Path $hostCopy,(Join-Path $hostCopy 'plugins') -Force | Out-Null
[IO.File]::WriteAllText((Join-Path $run 'run.id'), $id)
$hashes = @{}
foreach ($file in Get-ChildItem -LiteralPath $HostDirectory -File) {
    if ($file.Extension -notin '.dll','.exe','.config') { continue }
    Copy-Item -LiteralPath $file.FullName -Destination $hostCopy
    $hashes[$file.Name] = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash
}
Copy-Item -LiteralPath $plugin -Destination (Join-Path $hostCopy 'plugins/NOMADPlugin.dll')
if ($Media) {
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    $archivePath = 'C:\ProgramData\Mission Planner\gstreamer-1.0-x86_64-1.14.4.zip'
    if ((Get-FileHash -LiteralPath $archivePath).Hash -ne '2CAF7F56963136B846ACC3EF078055D6989DE3A02879EB320164CF312D7208DE') {
        throw 'Existing media package checksum mismatch; no download is attempted.'
    }
    $mediaRoot = Join-Path $hostCopy 'media'
    New-Item -ItemType Directory -Path $mediaRoot | Out-Null
    $archive = [IO.Compression.ZipFile]::OpenRead($archivePath)
    try {
        foreach ($entry in $archive.Entries) {
            $destination = [IO.Path]::GetFullPath((Join-Path $mediaRoot $entry.FullName))
            if (!$destination.StartsWith($mediaRoot + '\', [StringComparison]::OrdinalIgnoreCase)) {
                throw 'Media archive entry escaped the owned directory.'
            }
            if ($entry.FullName.EndsWith('/')) { New-Item -ItemType Directory -Path $destination -Force | Out-Null; continue }
            New-Item -ItemType Directory -Path (Split-Path $destination) -Force | Out-Null
            [IO.Compression.ZipFileExtensions]::ExtractToFile($entry, $destination, $false)
        }
    } finally { $archive.Dispose() }
}
# MP's original config has shared log sinks independent of Settings.CustomUserDataDirectory.
$configPath = Join-Path $hostCopy 'MissionPlanner.exe.config'
$config = New-Object Xml.XmlDocument
$config.Load($configPath)
foreach ($sink in $config.SelectNodes('//log4net/appender/file')) {
    $sink.SetAttribute('value', (Join-Path $run 'host.log'))
}
foreach ($listener in $config.SelectNodes('//system.diagnostics//*[@initializeData]')) {
    if ($listener.HasAttribute('initializeData')) {
        $listener.SetAttribute('initializeData', (Join-Path $run 'network.log'))
    }
}
foreach ($sink in $config.SelectNodes('//log4net/appender/file[@value] | //system.diagnostics//*[@initializeData]')) {
    $value = if ($sink.HasAttribute('initializeData')) { $sink.GetAttribute('initializeData') } else { $sink.GetAttribute('value') }
    if (![IO.Path]::GetFullPath($value).StartsWith($run + '\', [StringComparison]::OrdinalIgnoreCase)) {
        throw 'Host file logging escaped the owned run.'
    }
}
$config.Save($configPath)
Copy-Item -LiteralPath $configPath -Destination (Join-Path $hostCopy 'VideoLifetimeTests.exe.config')
$settings = New-Object Xml.XmlDocument
$settings.AppendChild($settings.CreateElement('Config')) | Out-Null
foreach ($entry in @{
    AutoConnect='[]'; CHK_resetapmonconnect='False'; CHK_rtsresetesp32='False';
    mapCache='CacheOnly'; logdirectory=(Join-Path $run 'logs'); speechenable='False';
    beta_updates='False'; update_check=(Get-Date).ToShortDateString()
}.GetEnumerator()) {
    $node = $settings.CreateElement($entry.Key)
    $node.InnerText = $entry.Value
    $settings.DocumentElement.AppendChild($node) | Out-Null
}
$settings.Save((Join-Path $run 'mp.config.xml'))
$compiler = Find-RoslynCompiler
$visionRuntime = Join-Path $root 'build/vision/runtime'
$visionManaged = @{
    'Microsoft.ML.OnnxRuntime.dll'='5c3c531af36a6cb4baa01db20dcf94a0464ec36f8090aa42f35ff565b90a1ea6';
    'OpenCvSharp.dll'='7e591871b2eaaa045c22b7d720bde3adceab2e19d64047135ad39ade03f8c045'
}
foreach ($name in $visionManaged.Keys) {
    $path = Join-Path $visionRuntime $name
    if (!(Test-Path -LiteralPath $path -PathType Leaf) -or
        (Get-FileHash -LiteralPath $path).Hash -ne $visionManaged[$name]) {
        throw 'The test-only tracker needs the existing pinned managed vision build inputs.'
    }
    $target = Join-Path $hostCopy $name
    if ((Test-Path -LiteralPath $target) -and (Get-FileHash -LiteralPath $target).Hash -ne $visionManaged[$name]) {
        throw 'A reviewed host dependency conflicts with the pinned test vision runtime; preserved.'
    }
    Copy-Item -LiteralPath $path -Destination $target -Force
}
$references = @('MissionPlanner.exe','MissionPlanner.Utilities.dll','MissionPlanner.Comms.dll',
    'MissionPlanner.ArduPilot.dll','MissionPlanner.Controls.dll','MAVLink.dll','Interfaces.dll',
    'Newtonsoft.Json.dll','System.Memory.dll','Microsoft.ML.OnnxRuntime.dll','OpenCvSharp.dll') |
    ForEach-Object { '/r:' + (Join-Path $hostCopy $_) }
# Native trackers live in the media worker, not the plugin DLL; the fixtures build them in-process
# (through the plugin's InternalsVisibleTo("VideoLifetimeTests")), as test_plugin_vision.ps1 does.
# MEDIA_WORKER selects VisionRuntime's native implementation; this copy shadows the plugin's stub (CS0436).
$trackers = @('VisionRuntime', 'AreaVisualTracker', 'AreaVisualTracker.Matching', 'CsrtVisualTracker',
    'CsrtVisualTracker.Object', 'EfficientTamVisualTracker', 'EfficientTamCore', 'FrozenAppearance',
    'VisualTrackingImage', 'ObjectRecoveryReference', 'ObjectForeground', 'GpuAdapter') |
    ForEach-Object { Join-Path $root "mission_planner/src/Vision/$_.cs" }
& $compiler /nologo /target:exe /platform:x64 /define:MEDIA_WORKER /langversion:latest `
    "/out:$(Join-Path $hostCopy 'VideoLifetimeTests.exe')" /r:System.Web.Extensions.dll `
    /r:System.Windows.Forms.dll /r:System.Drawing.dll "/r:$plugin" @references `
    ('/r:' + (Join-Path ${env:ProgramFiles(x86)} 'Reference Assemblies/Microsoft/Framework/.NETFramework/v4.8/Facades/netstandard.dll')) `
    (Join-Path $root 'mission_planner/tests/simulation/SimulationHost.cs') `
    (Join-Path $root 'mission_planner/tests/simulation/NativeFixture.cs') `
    (Join-Path $root 'mission_planner/tests/simulation/NativeFixture.Video.cs') `
    (Join-Path $root 'mission_planner/tests/simulation/ReplayFixture.cs') @trackers | ForEach-Object { Write-Host $_ }
if ($LASTEXITCODE -ne 0) { throw 'Owned host compilation failed.' }
$manifest = @{
    schema=1; source=(& git -C $root rev-parse HEAD); host_version=$version;
    host_files=$hashes; plugin_sha256=(Get-FileHash -LiteralPath $plugin).Hash;
    config_sha256=(Get-FileHash -LiteralPath $configPath).Hash; run_id=$id;
    full_host_launch_enabled=$false; fixture_assembly='VideoLifetimeTests'
    product_build=$build
    working_tree=(@(& git -C $root status --porcelain)); fixture_files=@{}
    test_vision_managed_files=$visionManaged
}
foreach ($directory in @('scripts/simulation','mission_planner/tests/simulation')) {
    foreach ($file in Get-ChildItem -LiteralPath (Join-Path $root $directory) -File -Recurse) {
        if ($file.Extension -notin '.cs','.cc','.py','.ps1','.sh','.txt','.sdf','.json') { continue }
        $relative = $file.FullName.Substring($root.Length + 1).Replace('\','/')
        $manifest.fixture_files[$relative] = (Get-FileHash -LiteralPath $file.FullName).Hash
    }
}
$manifest | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $run 'manifest.json') -Encoding UTF8
Write-Output $run
