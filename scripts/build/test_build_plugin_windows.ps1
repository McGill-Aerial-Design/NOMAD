# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# ============================================================
# Safe build/deploy dispatch tests for the Mission Planner plugin script
# ============================================================
# Runs the build script in a temporary repository with a small native MSBuild
# test double. The temporary Mission Planner plugins directory denies writes.
# Visual Studio's MSBuild installation supplies the C# compiler for the test double.
#
# Usage: pixi run test-plugin-build-only
# ============================================================

$ErrorActionPreference = 'Stop'

function Assert-Equal {
    param(
        [object]$Actual,
        [object]$Expected,
        [string]$Message
    )

    if ($Actual -ne $Expected) {
        throw "$Message (expected '$Expected', got '$Actual')"
    }
}

function Assert-Contains {
    param(
        [string]$Text,
        [string]$Expected,
        [string]$Message
    )

    if ($Text.IndexOf($Expected, [StringComparison]::OrdinalIgnoreCase) -lt 0) {
        throw "$Message (missing '$Expected')"
    }
}

function Get-Sha256 {
    param([string]$Path)

    $algorithm = [Security.Cryptography.SHA256]::Create()
    $stream = $null
    try {
        $stream = [IO.File]::OpenRead($Path)
        $bytes = $algorithm.ComputeHash($stream)
        return [BitConverter]::ToString($bytes).Replace('-', '')
    } finally {
        if ($stream) {
            $stream.Dispose()
        }
        $algorithm.Dispose()
    }
}

function Get-TreeSnapshot {
    param([string]$Path)

    $entries = @(
        Get-ChildItem -LiteralPath $Path -Force -Recurse |
            Sort-Object FullName |
            ForEach-Object {
                $relativePath = $_.FullName.Substring($Path.Length).TrimStart('\')
                $hash = if ($_.PSIsContainer) {
                    ''
                } else {
                    Get-Sha256 -Path $_.FullName
                }

                [pscustomobject]@{
                    Path = $relativePath
                    IsDirectory = $_.PSIsContainer
                    Length = if ($_.PSIsContainer) { 0 } else { $_.Length }
                    LastWriteUtc = if ($_.PSIsContainer) { 0 } else { $_.LastWriteTimeUtc.Ticks }
                    Hash = $hash
                }
            }
    )

    return ConvertTo-Json -InputObject $entries -Compress
}

function Get-MsBuildCompiler {
    $programFilesX86 = [Environment]::GetEnvironmentVariable('ProgramFiles(x86)', 'Process')
    $vswhere = Join-Path $programFilesX86 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path -LiteralPath $vswhere -PathType Leaf)) {
        throw "Visual Studio discovery tool not found at $vswhere"
    }

    $visualStudioPath = & $vswhere -latest -products '*' `
        -requires Microsoft.Component.MSBuild -property installationPath
    if (-not $visualStudioPath) {
        throw 'Visual Studio MSBuild was not found for the native test double.'
    }

    $compiler = Join-Path $visualStudioPath 'MSBuild\Current\Bin\Roslyn\csc.exe'
    if (-not (Test-Path -LiteralPath $compiler -PathType Leaf)) {
        throw "C# compiler not found at $compiler"
    }

    return $compiler
}

function New-TestFixture {
    param([string]$SourceBuildScript)

    $root = Join-Path ([IO.Path]::GetTempPath()) "NOMAD Plugin Build Test $([guid]::NewGuid().ToString('N'))"
    $fixture = [pscustomobject]@{
        Root = $root
        Repo = Join-Path $root 'Repository With Spaces'
        Script = Join-Path $root 'Repository With Spaces\scripts\build\build_plugin_windows.ps1'
        Tools = Join-Path $root 'Mock Build Tools'
        MockSource = Join-Path $root 'Mock Build Tools\MockMsbuild.cs'
        MockMsbuild = Join-Path $root 'Mock Build Tools\msbuild.exe'
        MockLog = Join-Path $root 'MSBuild Calls.log'
        Project = Join-Path $root 'Repository With Spaces\mission_planner\src'
        Artifact = Join-Path $root 'Repository With Spaces\mission_planner\src\bin\Release\NOMADPlugin.dll'
        ProgramFilesRoot = Join-Path $root 'Program Files (x86)'
        Install = Join-Path $root 'Program Files (x86)\Mission Planner'
        InstallPlugins = Join-Path $root 'Program Files (x86)\Mission Planner\plugins'
        LocalAppData = Join-Path $root 'Local App Data'
        LegacyPlugin = Join-Path $root 'Local App Data\Mission Planner\plugins\NOMADPlugin.dll'
        PowerShell = Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe'
        Icacls = (Get-Command icacls.exe -ErrorAction Stop).Source
        Compiler = (Get-MsBuildCompiler)
        CurrentUserSid = [System.Security.Principal.WindowsIdentity]::GetCurrent().User.Value
        DenyRule = ''
        DenyApplied = $false
        InstallBefore = ''
        LegacyBefore = ''
    }

    try {
        New-Item -ItemType Directory -Path (Split-Path $fixture.Script) -Force | Out-Null
        New-Item -ItemType Directory -Path $fixture.Project -Force | Out-Null
        New-Item -ItemType Directory -Path $fixture.Tools -Force | Out-Null
        New-Item -ItemType Directory -Path $fixture.InstallPlugins -Force | Out-Null
        New-Item -ItemType Directory -Path (Split-Path $fixture.LegacyPlugin) -Force | Out-Null
        Copy-Item -LiteralPath $SourceBuildScript -Destination $fixture.Script
        [IO.File]::WriteAllText((Join-Path $fixture.Project 'NOMADPlugin.csproj'), '<Project />')
        New-ReleaseIdentityFixture -Fixture $fixture -SourceBuildScript $SourceBuildScript
        [IO.File]::WriteAllText((Join-Path $fixture.Install 'MissionPlanner.exe'), 'installation sentinel')
        [IO.File]::WriteAllText((Join-Path $fixture.InstallPlugins 'existing-plugin.txt'), 'preserve this file')
        [IO.File]::WriteAllText($fixture.LegacyPlugin, 'preserve the legacy copy')
        New-NativeMsBuildTestDouble -Fixture $fixture

        $fixture.InstallBefore = Get-TreeSnapshot -Path $fixture.Install
        $legacyRoot = Split-Path (Split-Path $fixture.LegacyPlugin)
        $fixture.LegacyBefore = Get-TreeSnapshot -Path $legacyRoot
        $fixture.DenyRule = "*$($fixture.CurrentUserSid):(W,D)"
        & $fixture.Icacls $fixture.InstallPlugins /deny $fixture.DenyRule /C | Out-Null
        if ($LASTEXITCODE -ne 0) {
            throw 'Could not apply deny-write ACL to the temporary Mission Planner plugins directory.'
        }
        $fixture.DenyApplied = $true
        Assert-DenyWrite -Fixture $fixture

        return $fixture
    } catch {
        Remove-TestFixture -Fixture $fixture
        throw
    }
}

function Invoke-FixtureGit {
    param([pscustomobject]$Fixture, [string[]]$GitArguments)
    & git -C $Fixture.Repo @GitArguments
    if ($LASTEXITCODE -ne 0) {
        throw 'Synthetic release identity repository setup failed.'
    }
}

function New-ReleaseIdentityFixture {
    param([pscustomobject]$Fixture, [string]$SourceBuildScript)
    $sourceRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $SourceBuildScript))
    $destination = Join-Path $Fixture.Repo 'scripts\release'
    New-Item -ItemType Directory -Path $destination -Force | Out-Null
    foreach ($name in @('identity.py', 'manifest.py')) {
        Copy-Item -LiteralPath (Join-Path $sourceRoot "scripts\release\$name") -Destination $destination
    }
    Invoke-FixtureGit $Fixture @('init', '--quiet')
    Invoke-FixtureGit $Fixture @('add', 'scripts', 'mission_planner')
    $mavsdkFixtureSha = 'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa'
    Invoke-FixtureGit $Fixture @('update-index', '--add', '--cacheinfo', "160000,$mavsdkFixtureSha,third_party/MAVSDK")
    Invoke-FixtureGit $Fixture @('-c', 'user.name=NOMAD qualification fixture', '-c',
        'user.email=fixture@example.invalid', 'commit', '--quiet', '--no-gpg-sign', '-m', 'Synthetic fixture source')
}

function New-NativeMsBuildTestDouble {
    param([pscustomobject]$Fixture)

    $source = @'
using System;
using System.IO;

internal static class MockMsbuild
{
    private static int Main(string[] args)
    {
        string logPath = Environment.GetEnvironmentVariable("MOCK_MSBUILD_LOG");
        File.AppendAllText(logPath, "cwd=" + Environment.CurrentDirectory + Environment.NewLine);
        File.AppendAllText(logPath, String.Join(" ", args) + Environment.NewLine);

        if (HasTarget(args, "/t:Clean"))
        {
            return GetExitCode("MOCK_CLEAN_EXIT");
        }

        if (!HasTarget(args, "/t:Build"))
        {
            return 0;
        }

        int buildExitCode = GetExitCode("MOCK_BUILD_EXIT");
        if (buildExitCode != 0 || Environment.GetEnvironmentVariable("MOCK_SKIP_ARTIFACT") == "1")
        {
            return buildExitCode;
        }

        string outputDirectory = Path.Combine(Environment.CurrentDirectory, "bin", "Release");
        Directory.CreateDirectory(outputDirectory);
        File.WriteAllText(Path.Combine(outputDirectory, "NOMADPlugin.dll"), "test artifact");
        return 0;
    }

    private static bool HasTarget(string[] args, string target)
    {
        foreach (string argument in args)
        {
            if (String.Equals(argument, target, StringComparison.OrdinalIgnoreCase))
            {
                return true;
            }
        }

        return false;
    }

    private static int GetExitCode(string variableName)
    {
        int exitCode;
        return Int32.TryParse(Environment.GetEnvironmentVariable(variableName), out exitCode)
            ? exitCode
            : 0;
    }
}
'@
    [IO.File]::WriteAllText($Fixture.MockSource, $source, [Text.Encoding]::ASCII)
    & $Fixture.Compiler /nologo /target:exe "/out:$($Fixture.MockMsbuild)" $Fixture.MockSource
    if ($LASTEXITCODE -ne 0) {
        throw "Could not compile native MSBuild test double (exit $LASTEXITCODE)."
    }
}

function Assert-DenyWrite {
    param([pscustomobject]$Fixture)

    $probePath = Join-Path $Fixture.InstallPlugins 'write-probe.tmp'
    $writeWasDenied = $false
    try {
        [IO.File]::WriteAllText($probePath, 'probe')
    } catch [UnauthorizedAccessException] {
        $writeWasDenied = $true
    }

    if (-not $writeWasDenied) {
        Remove-Item -LiteralPath $probePath -Force -ErrorAction SilentlyContinue
        throw 'The temporary Mission Planner plugins ACL did not block writes.'
    }
}

function Set-TestEnvironment {
    param([pscustomobject]$Fixture)

    $env:PATH = "$($Fixture.Tools);$script:OriginalPath"
    [Environment]::SetEnvironmentVariable('ProgramFiles(x86)', $Fixture.ProgramFilesRoot, 'Process')
    $env:LOCALAPPDATA = $Fixture.LocalAppData
    $env:USERPROFILE = $Fixture.Root
    $env:MOCK_MSBUILD_LOG = $Fixture.MockLog

    $resolvedMsbuild = (Get-Command msbuild -ErrorAction Stop).Source
    Assert-Equal -Actual $resolvedMsbuild -Expected $Fixture.MockMsbuild `
        -Message 'Native MSBuild dispatch did not resolve the mock in a path with spaces'
}

function Invoke-BuildScript {
    param(
        [pscustomobject]$Fixture,
        [string]$CleanExitCode,
        [string]$BuildExitCode,
        [switch]$SkipArtifact
    )

    [IO.File]::WriteAllText($Fixture.MockLog, '')
    $env:MOCK_CLEAN_EXIT = $CleanExitCode
    $env:MOCK_BUILD_EXIT = $BuildExitCode
    $env:MOCK_SKIP_ARTIFACT = if ($SkipArtifact) { '1' } else { '0' }

    $output = & $Fixture.PowerShell -NoProfile -ExecutionPolicy Bypass -File $Fixture.Script 2>&1
    $exitCode = $LASTEXITCODE
    $log = Get-Content -LiteralPath $Fixture.MockLog -Raw

    return [pscustomobject]@{
        ExitCode = $exitCode
        Output = $output -join [Environment]::NewLine
        Log = $log
    }
}

function Assert-ExitCode {
    param(
        [pscustomobject]$Result,
        [int]$Expected,
        [string]$Scenario
    )

    if ($Result.ExitCode -ne $Expected) {
        throw "$Scenario returned $($Result.ExitCode), expected $Expected.`n$($Result.Output)`n$($Result.Log)"
    }
}

function Test-BuildOnlySuccess {
    param([pscustomobject]$Fixture)

    $result = Invoke-BuildScript -Fixture $Fixture
    Assert-ExitCode -Result $result -Expected 0 -Scenario 'Build-only success'
    Assert-Contains -Text $result.Output -Expected $Fixture.Artifact `
        -Message 'Build-only output omitted its artifact path'
    Assert-Contains -Text $result.Output -Expected 'Deployment skipped' `
        -Message 'Build-only mode did not skip deployment'
    Assert-Contains -Text $result.Log -Expected "cwd=$($Fixture.Project)" `
        -Message 'The script did not dispatch MSBuild from the project path with spaces'
    Assert-Contains -Text $result.Log -Expected '/t:Clean' -Message 'The clean target did not run'
    Assert-Contains -Text $result.Log -Expected '/t:Build' -Message 'The build target did not run'

    if (-not (Test-Path -LiteralPath $Fixture.Artifact -PathType Leaf)) {
        throw "Build-only did not produce the expected artifact at $($Fixture.Artifact)"
    }
    $identityPath = Join-Path $Fixture.Repo 'build\release\package-identity.json'
    $identity = Get-Content -LiteralPath $identityPath -Raw | ConvertFrom-Json
    Assert-Equal $identity.official $false 'Synthetic development build must not claim a tagged release'
    Assert-Equal $identity.release_version "dev-$($identity.source_sha)" 'Development identity must bind exact source'
    Assert-Equal $identity.mavsdk_sha 'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa' 'Identity omitted fixture MAVSDK pin'
    Assert-Equal $identity.name 'plugin' 'Build generated the wrong component identity'
    Assert-Equal $identity.mission_planner_target '1.3.83' 'Build generated an incompatible Mission Planner target'
    if (-not (Test-Path -LiteralPath (Join-Path $Fixture.Repo 'build\release\ReleaseIdentity.cs'))) {
        throw 'Plugin build did not generate its C# embedded identity input.'
    }
}

function Test-BuildFailure {
    param([pscustomobject]$Fixture)

    Remove-Item -LiteralPath $Fixture.Artifact -Force -ErrorAction SilentlyContinue
    $result = Invoke-BuildScript -Fixture $Fixture -BuildExitCode '23'
    Assert-ExitCode -Result $result -Expected 23 -Scenario 'Build failure'
    Assert-Contains -Text $result.Output -Expected 'Build failed' -Message 'Build failure was not reported'
}

function Test-CleanFailure {
    param([pscustomobject]$Fixture)

    $result = Invoke-BuildScript -Fixture $Fixture -CleanExitCode '17'
    Assert-ExitCode -Result $result -Expected 17 -Scenario 'Clean failure'
    if ($result.Log.Contains('/t:Build')) {
        throw 'The build target ran after the clean target failed.'
    }
}

function Test-MissingArtifact {
    param([pscustomobject]$Fixture)

    Remove-Item -LiteralPath $Fixture.Artifact -Force -ErrorAction SilentlyContinue
    $result = Invoke-BuildScript -Fixture $Fixture -SkipArtifact
    Assert-ExitCode -Result $result -Expected 1 -Scenario 'Missing artifact'
    Assert-Contains -Text $result.Output -Expected 'Built DLL not found' `
        -Message 'Missing artifact did not produce a clear failure'
}

function Test-InstallationUnchanged {
    param([pscustomobject]$Fixture)

    Remove-DenyWrite -Fixture $Fixture
    Assert-Equal -Actual (Get-TreeSnapshot -Path $Fixture.Install) -Expected $Fixture.InstallBefore `
        -Message 'Build-only changed the deny-write Mission Planner installation fixture'
    $legacyRoot = Split-Path (Split-Path $Fixture.LegacyPlugin)
    Assert-Equal -Actual (Get-TreeSnapshot -Path $legacyRoot) -Expected $Fixture.LegacyBefore `
        -Message 'Build-only changed the legacy AppData plugin fixture'
}

function Remove-DenyWrite {
    param([pscustomobject]$Fixture)

    if (-not $Fixture.DenyApplied) {
        return
    }

    & $Fixture.Icacls $Fixture.InstallPlugins /remove:d "*$($Fixture.CurrentUserSid)" /C | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw 'Could not remove the deny-write ACL from the temporary Mission Planner plugins directory.'
    }
    $Fixture.DenyApplied = $false
}

function Save-ProcessEnvironment {
    return [pscustomobject]@{
        Path = [Environment]::GetEnvironmentVariable('PATH', 'Process')
        ProgramFilesX86 = [Environment]::GetEnvironmentVariable('ProgramFiles(x86)', 'Process')
        LocalAppData = [Environment]::GetEnvironmentVariable('LOCALAPPDATA', 'Process')
        UserProfile = [Environment]::GetEnvironmentVariable('USERPROFILE', 'Process')
        MockLog = [Environment]::GetEnvironmentVariable('MOCK_MSBUILD_LOG', 'Process')
        CleanExit = [Environment]::GetEnvironmentVariable('MOCK_CLEAN_EXIT', 'Process')
        BuildExit = [Environment]::GetEnvironmentVariable('MOCK_BUILD_EXIT', 'Process')
        SkipArtifact = [Environment]::GetEnvironmentVariable('MOCK_SKIP_ARTIFACT', 'Process')
    }
}

function Restore-ProcessEnvironment {
    param([pscustomobject]$Original)

    [Environment]::SetEnvironmentVariable('PATH', $Original.Path, 'Process')
    [Environment]::SetEnvironmentVariable('ProgramFiles(x86)', $Original.ProgramFilesX86, 'Process')
    [Environment]::SetEnvironmentVariable('LOCALAPPDATA', $Original.LocalAppData, 'Process')
    [Environment]::SetEnvironmentVariable('USERPROFILE', $Original.UserProfile, 'Process')
    [Environment]::SetEnvironmentVariable('MOCK_MSBUILD_LOG', $Original.MockLog, 'Process')
    [Environment]::SetEnvironmentVariable('MOCK_CLEAN_EXIT', $Original.CleanExit, 'Process')
    [Environment]::SetEnvironmentVariable('MOCK_BUILD_EXIT', $Original.BuildExit, 'Process')
    [Environment]::SetEnvironmentVariable('MOCK_SKIP_ARTIFACT', $Original.SkipArtifact, 'Process')
}

function Remove-TestFixture {
    param([pscustomobject]$Fixture)

    Remove-DenyWrite -Fixture $Fixture
    if (-not (Test-Path -LiteralPath $Fixture.Root)) {
        return
    }

    $temporaryRoot = [IO.Path]::GetFullPath([IO.Path]::GetTempPath()).TrimEnd('\') + '\'
    $resolvedRoot = [IO.Path]::GetFullPath($Fixture.Root).TrimEnd('\')
    if (-not $resolvedRoot.StartsWith($temporaryRoot, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove test fixture outside the temporary directory: $resolvedRoot"
    }
    if ((Split-Path $resolvedRoot -Leaf) -notlike 'NOMAD Plugin Build Test *') {
        throw "Refusing to remove an unexpected test fixture directory: $resolvedRoot"
    }

    Remove-Item -LiteralPath $resolvedRoot -Force -Recurse
}

if ($env:OS -ne 'Windows_NT') {
    throw 'These dispatch tests require Windows PowerShell and NTFS ACLs.'
}

$sourceBuildScript = Join-Path $PSScriptRoot 'build_plugin_windows.ps1'
$originalEnvironment = Save-ProcessEnvironment
$fixture = $null

try {
    $fixture = New-TestFixture -SourceBuildScript $sourceBuildScript
    $script:OriginalPath = $originalEnvironment.Path
    Set-TestEnvironment -Fixture $fixture

    Test-BuildOnlySuccess -Fixture $fixture
    Test-BuildFailure -Fixture $fixture
    Test-CleanFailure -Fixture $fixture
    Test-MissingArtifact -Fixture $fixture
    Test-InstallationUnchanged -Fixture $fixture

    $summary = 'Build-only dispatch tests passed: artifact, paths with spaces, '
    $summary += 'exact failures, and unchanged installation.'
    Write-Host $summary -ForegroundColor Green
} finally {
    Restore-ProcessEnvironment -Original $originalEnvironment
    if ($fixture) {
        Remove-TestFixture -Fixture $fixture
    }
}
