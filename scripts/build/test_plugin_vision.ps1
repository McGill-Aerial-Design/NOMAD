# SPDX-License-Identifier: Apache-2.0
param([string]$FixtureImage = (Join-Path $PSScriptRoot '../../mission_planner/tests/vision/vehicle.png'),
    [string]$TrackingFrames = '', [string]$TrackingGroundTruth = '')
$ErrorActionPreference = 'Stop'
$repoRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
& (Join-Path $PSScriptRoot 'prepare_vision.ps1')
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$compiler = Find-RoslynCompiler
$mpDir = 'C:\Program Files (x86)\Mission Planner'
$runtime = Join-Path $repoRoot 'build\vision\runtime'
$outDir = Join-Path $repoRoot 'build\tests\vision'
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$plugin = Join-Path $repoRoot 'mission_planner\src\bin\Release\NOMADPlugin.dll'
if (-not (Test-Path -LiteralPath $plugin -PathType Leaf)) {
    throw 'Build NOMADPlugin.dll before testing fresh host plugin type discovery.'
}
$loadDir = Join-Path $outDir ('plugin-load-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $loadDir | Out-Null
try {
    $loadExe = Join-Path $loadDir 'PluginTypeLoadTests.exe'
    & $compiler /nologo /target:exe /platform:x64 "/out:$loadExe" `
        (Join-Path $repoRoot 'mission_planner\tests\vision\PluginTypeLoadTests.cs')
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    Copy-Item -LiteralPath $plugin -Destination $loadDir
    & $loadExe (Join-Path $loadDir 'NOMADPlugin.dll') $mpDir
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
} finally {
    if (-not ([IO.Path]::GetFullPath($loadDir)).StartsWith($outDir + '\', [StringComparison]::OrdinalIgnoreCase)) {
        throw 'Plugin type-load fixture left the owned output directory.'
    }
    Remove-Item -LiteralPath $loadDir -Recurse -Force
}
$exe = Join-Path $outDir 'VisionDetectorTests.exe'
$sources = @('VideoDetection', 'VisionDetectorFactory', 'VisionModelProfile', 'VisionRuntime', 'YoloXVideoDetector',
    'IVisualTracker', 'VisualTrackerFactory', 'CsrtVisualTracker') |
    ForEach-Object { Join-Path $repoRoot "mission_planner\src\Vision\$_.cs" }
$sources += Join-Path $repoRoot 'mission_planner\tests\vision\VisionDetectorTests.cs'
$sources += Join-Path $repoRoot 'mission_planner\tests\vision\VisualTrackerTests.cs'
$resources = @(Get-ChildItem -LiteralPath $runtime -File | ForEach-Object {
    "/resource:$($_.FullName),NOMAD.Vision.$($_.Name)"
})
$references = @('/r:System.Drawing.dll', "/r:$mpDir\Newtonsoft.Json.dll", "/r:$mpDir\System.Memory.dll",
    "/r:$runtime\Microsoft.ML.OnnxRuntime.dll", "/r:$runtime\OpenCvSharp.dll",
    '/r:C:\Program Files (x86)\Reference Assemblies\Microsoft\Framework\.NETFramework\v4.8\Facades\netstandard.dll')
& $compiler /nologo /target:exe /platform:x64 /langversion:latest "/out:$exe" @references @resources @sources
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
foreach ($name in @('Newtonsoft.Json', 'System.Memory', 'System.Buffers', 'System.Numerics.Vectors',
    'System.Runtime.CompilerServices.Unsafe')) {
    Copy-Item -LiteralPath (Join-Path $mpDir "$name.dll") -Destination $outDir -Force
}
$redirects = @('System.Memory', 'System.Buffers', 'System.Numerics.Vectors', 'System.Runtime.CompilerServices.Unsafe') |
    ForEach-Object {
        $identity = [Reflection.AssemblyName]::GetAssemblyName((Join-Path $mpDir "$_.dll"))
        $key = ([BitConverter]::ToString($identity.GetPublicKeyToken())).Replace('-', '').ToLowerInvariant()
        '<dependentAssembly><assemblyIdentity name="' + $_ + '" publicKeyToken="' + $key +
            '" culture="neutral"/><bindingRedirect oldVersion="0.0.0.0-' + $identity.Version +
            '" newVersion="' + $identity.Version + '"/></dependentAssembly>'
    }
[IO.File]::WriteAllText("$exe.config", '<configuration><runtime><assemblyBinding xmlns="urn:schemas-microsoft-com:asm.v1">' +
    ($redirects -join '') + '</assemblyBinding></runtime></configuration>')
$testData = Join-Path $outDir ('data-' + [guid]::NewGuid().ToString('N'))
$previousData = $env:NOMAD_VISION_TEST_DATA
$previousFrames = $env:NOMAD_VISION_TRACKING_FRAMES
$previousTruth = $env:NOMAD_VISION_TRACKING_GT
try {
    $env:NOMAD_VISION_TEST_DATA = $testData
    $env:NOMAD_VISION_TRACKING_FRAMES = $TrackingFrames
    $env:NOMAD_VISION_TRACKING_GT = $TrackingGroundTruth
    if ($FixtureImage) { & $exe ([IO.Path]::GetFullPath($FixtureImage)) } else { & $exe }
    $result = $LASTEXITCODE
} finally {
    $env:NOMAD_VISION_TEST_DATA = $previousData
    $env:NOMAD_VISION_TRACKING_FRAMES = $previousFrames
    $env:NOMAD_VISION_TRACKING_GT = $previousTruth
    if (Test-Path -LiteralPath $testData) {
        if (-not ([IO.Path]::GetFullPath($testData)).StartsWith($outDir + '\', [StringComparison]::OrdinalIgnoreCase)) {
            throw 'Vision test data left the owned output directory.'
        }
        Remove-Item -LiteralPath $testData -Recurse -Force
    }
}
exit $result
