# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Shared Windows build tools and artifact identity checks.
param([string]$MediaBundlePlugin)

# Pixi may pass PowerShell 7 module paths to Windows PowerShell 5.1.
Import-Module (Join-Path $PSHOME 'Modules/Microsoft.PowerShell.Utility/Microsoft.PowerShell.Utility.psd1') -ErrorAction Stop

function Find-MSBuild {
    $command = Get-Command msbuild -ErrorAction SilentlyContinue
    if ($command -and (Test-Path -LiteralPath $command.Source -PathType Leaf)) {
        return $command.Source
    }

    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path -LiteralPath $vswhere -PathType Leaf)) {
        return $null
    }
    $installation = & $vswhere -latest -products '*' -requires Microsoft.Component.MSBuild -property installationPath
    if ($LASTEXITCODE -ne 0 -or $installation -isnot [string] -or [string]::IsNullOrWhiteSpace($installation)) {
        return $null
    }
    $candidate = Join-Path $installation.Trim() 'MSBuild\Current\Bin\MSBuild.exe'
    if (Test-Path -LiteralPath $candidate -PathType Leaf) {
        return $candidate
    }
    return $null
}

function Find-RoslynCompiler {
    $msbuild = Find-MSBuild
    if (-not $msbuild) {
        throw 'MSBuild not found. Install Visual Studio with .NET desktop build tools.'
    }
    $compiler = Join-Path (Split-Path $msbuild) 'Roslyn\csc.exe'
    if (-not (Test-Path -LiteralPath $compiler -PathType Leaf)) {
        throw 'The selected MSBuild installation has no Roslyn csc.exe.'
    }
    return $compiler
}

function Get-WpfTestReferences {
    foreach ($name in @('WindowsBase', 'PresentationCore', 'PresentationFramework', 'WindowsFormsIntegration',
        'System.Xaml', 'UIAutomationProvider', 'UIAutomationTypes')) {
        '/r:' + (Join-Path ${env:ProgramFiles(x86)} "Reference Assemblies/Microsoft/Framework/.NETFramework/v4.8/$name.dll")
    }
}

function Get-PluginSourceHashes {
    param([Parameter(Mandatory=$true)][string]$RepoRoot)
    $root = [IO.Path]::GetFullPath($RepoRoot)
    $hashes = @{}
    foreach ($directory in @('mission_planner/src','mission_planner/media_worker','infra/transport/ground_router')) {
        $path = Join-Path $root $directory
        if (!(Test-Path -LiteralPath $path)) { continue }
        foreach ($file in Get-ChildItem -LiteralPath $path -File -Recurse) {
            if ($file.FullName -match '[\\/](bin|obj)[\\/]' -or
                $file.Extension -notin '.cs','.csproj','.resx','.xaml') { continue }
            $hashes[$file.FullName.Substring($root.Length + 1)] = (Get-FileHash -LiteralPath $file.FullName).Hash
        }
    }
    foreach ($relative in @('scripts/build/build_media_worker.ps1','scripts/build/build_plugin_windows.ps1',
        'scripts/build/windows_build_tools.ps1','scripts/build/prepare_vision.ps1')) {
        $file = Join-Path $root $relative
        if (Test-Path -LiteralPath $file -PathType Leaf) { $hashes[$relative] = (Get-FileHash -LiteralPath $file).Hash }
    }
    return $hashes
}

function Get-CurrentPluginBuild {
    param([Parameter(Mandatory=$true)][string]$RepoRoot, [Parameter(Mandatory=$true)][string]$PluginPath)
    $build = Get-Content -LiteralPath (Join-Path $RepoRoot 'build/release/plugin-build.json') -Raw | ConvertFrom-Json
    $source = & git -C $RepoRoot rev-parse HEAD
    if ($LASTEXITCODE -ne 0) { throw 'Plugin source identity unavailable.' }
    $target = (Get-Content -LiteralPath (Join-Path $RepoRoot 'scripts/release/mission-planner-target.json') -Raw |
        ConvertFrom-Json).version
    if ($build.schema -ne 1 -or $build.source -ne $source -or $build.host_target -ne $target -or
        $build.plugin_sha256 -ne (Get-FileHash -LiteralPath $PluginPath).Hash) {
        throw 'Plugin build/source binding mismatch; rebuild before using this artifact.'
    }
    $current = Get-PluginSourceHashes $RepoRoot
    if ($current.Count -ne @($build.source_files.PSObject.Properties).Count) {
        throw 'Product source set differs from the built artifact; rebuild before using this artifact.'
    }
    foreach ($name in $current.Keys) {
        if ($build.source_files.PSObject.Properties[$name].Value -ne $current[$name]) {
            throw 'Product source differs from the built artifact; rebuild before using this artifact.'
        }
    }
    $bundle = Get-EmbeddedMediaBundle -PluginPath $PluginPath
    if (!$build.media_bundle -or @($build.media_bundle.PSObject.Properties).Count -ne 7) {
        throw 'Media bundle build proof is missing.'
    }
    foreach ($entry in $bundle.PSObject.Properties) {
        if ($build.media_bundle.PSObject.Properties[$entry.Name].Value -ne $entry.Value) {
            throw 'Media bundle differs from its build proof.'
        }
    }
    return $build
}

function Get-EmbeddedMediaBundle {
    param([Parameter(Mandatory=$true)][string]$PluginPath)
    # A fresh metadata process avoids assembly identity caching and file locks across proof checks.
    $powershell = Join-Path $env:WINDIR 'System32/WindowsPowerShell/v1.0/powershell.exe'
    $helper = Join-Path $PSScriptRoot 'windows_build_tools.ps1'
    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $output = & $powershell -NoProfile -ExecutionPolicy Bypass -File $helper `
            -MediaBundlePlugin ([IO.Path]::GetFullPath($PluginPath)) 2>&1
        $exitCode = $LASTEXITCODE
    } finally { $ErrorActionPreference = $previousPreference }
    if ($exitCode -ne 0) { throw ('Embedded media verification failed: ' + ($output -join [Environment]::NewLine)) }
    return ($output -join [Environment]::NewLine) | ConvertFrom-Json
}

function Read-EmbeddedMediaBundle {
    param([Parameter(Mandatory=$true)][string]$PluginPath)
    $assembly = [Reflection.Assembly]::ReflectionOnlyLoad([IO.File]::ReadAllBytes([IO.Path]::GetFullPath($PluginPath)))
    $stream = $assembly.GetManifestResourceStream('NOMAD.Media.bundle.json')
    if (!$stream -or $stream.Length -gt 65536) { throw 'Embedded media bundle manifest missing or oversized.' }
    $reader = New-Object IO.StreamReader($stream)
    try { $bundle = $reader.ReadToEnd() | ConvertFrom-Json } finally { $reader.Dispose() }
    $allowed = @('NOMADMediaWorker.exe', 'NOMADMediaWorker.exe.config', 'Newtonsoft.Json.dll',
        'System.Memory.dll', 'System.Buffers.dll', 'System.Numerics.Vectors.dll',
        'System.Runtime.CompilerServices.Unsafe.dll')
    if (@($bundle.PSObject.Properties).Count -ne $allowed.Count) { throw 'Unexpected media bundle entries.' }
    $names = @($assembly.GetManifestResourceNames() | Where-Object { $_.StartsWith('NOMAD.Media.') } | Sort-Object)
    $expectedNames = @(@('NOMAD.Media.bundle.json') + @($allowed | ForEach-Object { 'NOMAD.Media.' + $_ }) | Sort-Object)
    if (($names -join '|') -ne ($expectedNames -join '|')) { throw 'Unexpected embedded media resource.' }
    foreach ($name in $allowed) {
        $expected = $bundle.PSObject.Properties[$name].Value
        if ($expected -notmatch '^[0-9a-f]{64}$') { throw 'Invalid embedded media checksum.' }
        $resource = $assembly.GetManifestResourceStream('NOMAD.Media.' + $name)
        if (!$resource) { throw 'Embedded media bundle entry missing.' }
        $algorithm = [Security.Cryptography.SHA256]::Create()
        try { $actual = [BitConverter]::ToString($algorithm.ComputeHash($resource)).Replace('-', '').ToLowerInvariant() }
        finally { $resource.Dispose(); $algorithm.Dispose() }
        if ($actual -ne $expected) { throw 'Embedded media bundle checksum mismatch.' }
    }
    foreach ($reference in $assembly.GetReferencedAssemblies()) {
        if ($reference.Name -in @('Microsoft.ML.OnnxRuntime', 'OpenCvSharp')) {
            throw 'Native vision implementation leaked into the Mission Planner plugin.'
        }
    }
    return $bundle
}

if ($MediaBundlePlugin) {
    $ErrorActionPreference = 'Stop'
    try { Read-EmbeddedMediaBundle -PluginPath $MediaBundlePlugin | ConvertTo-Json -Depth 4 -Compress }
    catch { [Console]::Error.WriteLine($_.Exception.Message); exit 1 }
    exit 0
}
