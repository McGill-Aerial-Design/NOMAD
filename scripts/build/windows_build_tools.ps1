# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Shared Windows build tools and artifact identity checks.
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

function Get-PluginSourceHashes {
    param([Parameter(Mandatory=$true)][string]$RepoRoot)
    $root = [IO.Path]::GetFullPath($RepoRoot)
    $hashes = @{}
    foreach ($directory in @('mission_planner/src','infra/transport/ground_router')) {
        $path = Join-Path $root $directory
        if (!(Test-Path -LiteralPath $path)) { continue }
        foreach ($file in Get-ChildItem -LiteralPath $path -File -Recurse) {
            if ($file.FullName -match '[\\/](bin|obj)[\\/]' -or
                $file.Extension -notin '.cs','.csproj','.resx') { continue }
            $hashes[$file.FullName.Substring($root.Length + 1)] = (Get-FileHash -LiteralPath $file.FullName).Hash
        }
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
    return $build
}
