# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Shared Windows build-tool discovery. Keep test-double compiler bootstrap independent.

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
