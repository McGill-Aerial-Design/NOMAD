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
