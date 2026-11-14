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
        UnrelatedPlugin = Join-Path $root 'Local App Data\Mission Planner\plugins\NOMADPlugin.dll'
        PowerShell = Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe'
        Icacls = (Get-Command icacls.exe -ErrorAction Stop).Source
        Compiler = (Get-MsBuildCompiler)
        CurrentUserSid = [System.Security.Principal.WindowsIdentity]::GetCurrent().User.Value
        DenyRule = ''
        DenyApplied = $false
        InstallBefore = ''
        UnrelatedBefore = ''
    }

    try {
        New-Item -ItemType Directory -Path (Split-Path $fixture.Script) -Force | Out-Null
        New-Item -ItemType Directory -Path $fixture.Project -Force | Out-Null
        New-Item -ItemType Directory -Path $fixture.Tools -Force | Out-Null
        New-Item -ItemType Directory -Path $fixture.InstallPlugins -Force | Out-Null
        New-Item -ItemType Directory -Path (Split-Path $fixture.UnrelatedPlugin) -Force | Out-Null
        Copy-Item -LiteralPath $SourceBuildScript -Destination $fixture.Script
        Copy-Item -LiteralPath (Join-Path (Split-Path $SourceBuildScript) 'windows_build_tools.ps1') `
            -Destination (Split-Path $fixture.Script)
        # This fixture tests dispatch and installation isolation; detector tests prepare real pinned inputs.
        [IO.File]::WriteAllText((Join-Path (Split-Path $fixture.Script) 'prepare_vision.ps1'),
            "Write-Host 'Synthetic pinned vision preparation'" + [Environment]::NewLine)
        New-SyntheticMediaStep -Fixture $fixture
        [IO.File]::WriteAllText((Join-Path $fixture.Project 'NOMADPlugin.csproj'), '<Project />')
        New-ReleaseIdentityFixture -Fixture $fixture -SourceBuildScript $SourceBuildScript
        $hostSource = Join-Path $fixture.Tools 'MissionPlanner.cs'
        $target = (Get-Content (Join-Path $fixture.Repo 'scripts/release/mission-planner-target.json') -Raw |
            ConvertFrom-Json).version
        [IO.File]::WriteAllText($hostSource,
            "[assembly: System.Reflection.AssemblyFileVersion(`"$target.0`")] public class Host { public static void Main() {} }")
        & $fixture.Compiler /nologo /target:exe "/out:$(Join-Path $fixture.Install 'MissionPlanner.exe')" $hostSource
        if ($LASTEXITCODE -ne 0) { throw 'Could not compile versioned Mission Planner reference fixture.' }
        [IO.File]::WriteAllText((Join-Path $fixture.InstallPlugins 'existing-plugin.txt'), 'preserve this file')
        [IO.File]::WriteAllText($fixture.UnrelatedPlugin, 'preserve the unrelated copy')
        New-NativeMsBuildTestDouble -Fixture $fixture

        $fixture.InstallBefore = Get-TreeSnapshot -Path $fixture.Install
        $unrelatedRoot = Split-Path (Split-Path $fixture.UnrelatedPlugin)
        $fixture.UnrelatedBefore = Get-TreeSnapshot -Path $unrelatedRoot
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

function New-SyntheticMediaStep {
    param([pscustomobject]$Fixture)
    # Dispatch/proof fixtures use synthetic payloads; native vision has its own executable gate.
    $source = @'
param([string]$MSBuild, [string]$RepoRoot, [string]$HostDirectory)
$ErrorActionPreference = 'Stop'
[IO.File]::AppendAllText($env:MOCK_MSBUILD_LOG, "media-worker=$RepoRoot" + [Environment]::NewLine)
[IO.File]::AppendAllText($env:MOCK_MSBUILD_LOG, "media-host=$HostDirectory" + [Environment]::NewLine)
$output = Join-Path $RepoRoot 'build/media_worker'
New-Item -ItemType Directory -Path $output -Force | Out-Null
$bundle = [ordered]@{}
foreach ($name in @('NOMADMediaWorker.exe', 'NOMADMediaWorker.exe.config', 'Newtonsoft.Json.dll',
    'System.Memory.dll', 'System.Buffers.dll', 'System.Numerics.Vectors.dll',
    'System.Runtime.CompilerServices.Unsafe.dll')) {
    $path = Join-Path $output $name
    [IO.File]::WriteAllText($path, 'Synthetic media fixture: ' + $name)
    $bundle[$name] = (Get-FileHash -LiteralPath $path).Hash.ToLowerInvariant()
}
$bundle | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $output 'bundle.json') -Encoding UTF8
'@
    [IO.File]::WriteAllText((Join-Path (Split-Path $Fixture.Script) 'build_media_worker.ps1'), $source)
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
    foreach ($name in @('mission-planner-target.json')) {
        Copy-Item -LiteralPath (Join-Path $sourceRoot "scripts\release\$name") -Destination $destination
    }
    Invoke-FixtureGit $Fixture @('init', '--quiet')
    Invoke-FixtureGit $Fixture @('add', 'scripts', 'mission_planner')
    Invoke-FixtureGit $Fixture @('-c', 'user.name=NOMAD qualification fixture', '-c',
        'user.email=fixture@example.invalid', 'commit', '--quiet', '--no-gpg-sign', '-m', 'Synthetic fixture source')
}

function New-NativeMsBuildTestDouble {
    param([pscustomobject]$Fixture)

    $source = @'
using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Reflection.Emit;
using System.Text;

internal static class MockMsbuild
{
    private static int Main(string[] args)
    {
        if (String.Equals(Path.GetFileNameWithoutExtension(Environment.GetCommandLineArgs()[0]),
            "vswhere", StringComparison.OrdinalIgnoreCase))
        {
            string installation = Environment.GetEnvironmentVariable("MOCK_VS_PATH");
            if (installation != null) Console.WriteLine(installation);
            return GetExitCode("MOCK_VS_EXIT");
        }

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
        string repository = Directory.GetParent(Directory.GetParent(Environment.CurrentDirectory).FullName).FullName;
        WritePayload(outputDirectory, Path.Combine(repository, "build", "media_worker"));
        string mutation = Environment.GetEnvironmentVariable("MOCK_SOURCE_MUTATION");
        if (mutation == "change") File.AppendAllText("NOMADPlugin.csproj", "changed during build");
        if (mutation == "add") File.WriteAllText("Unexpected.cs", "class Unexpected {}");
        return 0;
    }

    private static void WritePayload(string outputDirectory, string bundleDirectory)
    {
        string mutation = Environment.GetEnvironmentVariable("MOCK_BUNDLE_MUTATION");
        var assembly = AppDomain.CurrentDomain.DefineDynamicAssembly(
            new AssemblyName("SyntheticPlugin" + Guid.NewGuid().ToString("N")), AssemblyBuilderAccess.Save, outputDirectory);
        var module = assembly.DefineDynamicModule("NOMADPlugin", "NOMADPlugin.dll");
        module.DefineType("FixtureMarker", TypeAttributes.Public).CreateType();
        var streams = new List<MemoryStream>();
        try
        {
            foreach (string name in new[] { "NOMADMediaWorker.exe", "NOMADMediaWorker.exe.config", "Newtonsoft.Json.dll",
                "System.Memory.dll", "System.Buffers.dll", "System.Numerics.Vectors.dll",
                "System.Runtime.CompilerServices.Unsafe.dll", "bundle.json" })
            {
                byte[] bytes = File.ReadAllBytes(Path.Combine(bundleDirectory, name));
                if (mutation == "payload" && name == "NOMADMediaWorker.exe")
                    bytes = Encoding.UTF8.GetBytes("Corrupted synthetic media payload");
                if (mutation == "worker-name" && name == "bundle.json")
                    bytes = Encoding.UTF8.GetBytes(Encoding.UTF8.GetString(bytes)
                        .Replace("\"NOMADMediaWorker.exe\"", "\"OtherWorker.exe\""));
                AddResource(module, streams, "NOMAD.Media." + name, bytes);
            }
            if (mutation == "unlisted")
                AddResource(module, streams, "NOMAD.Media.unlisted.bin", new byte[] { 1 });
            assembly.Save("NOMADPlugin.dll");
        }
        finally { foreach (var stream in streams) stream.Dispose(); }
    }

    private static void AddResource(ModuleBuilder module, List<MemoryStream> streams, string name, byte[] bytes)
    {
        var stream = new MemoryStream(bytes); streams.Add(stream);
        module.DefineManifestResource(name, stream, ResourceAttributes.Public);
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

function New-DiscoveryTools {
    param([pscustomobject]$Fixture)

    $installation = Join-Path $Fixture.Root 'Visual Studio With Spaces'
    $msbuild = Join-Path $installation 'MSBuild\Current\Bin\MSBuild.exe'
    $vswhere = Join-Path $Fixture.ProgramFilesRoot 'Microsoft Visual Studio\Installer\vswhere.exe'
    $compiler = Join-Path $Fixture.Tools 'Roslyn\csc.exe'
    $emptyPath = Join-Path $Fixture.Root 'Empty Tool Path'
    foreach ($path in @((Split-Path $msbuild), (Split-Path $vswhere), (Split-Path $compiler), $emptyPath)) {
        New-Item -ItemType Directory -Path $path -Force | Out-Null
    }
    Copy-Item -LiteralPath $Fixture.MockMsbuild -Destination $msbuild
    Copy-Item -LiteralPath $Fixture.MockMsbuild -Destination $vswhere
    [IO.File]::WriteAllText($compiler, 'compiler lookup sentinel')
    return [pscustomobject]@{
        Installation = $installation
        Msbuild = $msbuild
        Vswhere = $vswhere
        Compiler = $compiler
        EmptyPath = $emptyPath
    }
}

function Test-ToolDiscovery {
    param([pscustomobject]$Fixture)

    . (Join-Path (Split-Path $Fixture.Script) 'windows_build_tools.ps1')
    $tools = New-DiscoveryTools -Fixture $Fixture
    $originalPath = $env:PATH
    try {
        $env:MOCK_VS_PATH = $tools.Installation
        $env:MOCK_VS_EXIT = '0'
        Assert-Equal (Find-MSBuild) $Fixture.MockMsbuild 'PATH must win over Visual Studio discovery'
        Assert-Equal (Find-RoslynCompiler) $tools.Compiler 'Roslyn must come from the selected MSBuild'
        Remove-Item -LiteralPath $tools.Compiler
        $missingCompiler = $false
        try { Find-RoslynCompiler | Out-Null } catch {
            $missingCompiler = $_.Exception.Message -like '*no Roslyn csc.exe*'
        }
        Assert-Equal $missingCompiler $true 'Missing Roslyn must fail before compilation'
        $env:PATH = $tools.EmptyPath
        Assert-Equal (Find-MSBuild) $tools.Msbuild 'Native vswhere must resolve installation paths with spaces'
        foreach ($invalid in @('', (Join-Path $Fixture.Root 'Missing Studio'), ($tools.Installation + "`nother"))) {
            $env:MOCK_VS_PATH = $invalid
            Assert-Equal (Find-MSBuild) $null 'Empty, missing or ambiguous Visual Studio output must fail'
        }
        $env:MOCK_VS_PATH = $tools.Installation
        $env:MOCK_VS_EXIT = '17'
        Assert-Equal (Find-MSBuild) $null 'A failed native discovery command must not supply MSBuild'
        $env:MOCK_VS_EXIT = '0'
        Remove-Item -LiteralPath $tools.Vswhere
        Assert-Equal (Find-MSBuild) $null 'Missing PATH and vswhere must report no MSBuild'
    } finally {
        $env:PATH = $originalPath
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
    $env:MissionPlannerDir = Join-Path $Fixture.Root 'Unreviewed Host Override'
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
        [switch]$SkipArtifact,
        [string]$SourceMutation = '',
        [string]$BundleMutation = ''
    )

    [IO.File]::WriteAllText($Fixture.MockLog, '')
    $env:MOCK_CLEAN_EXIT = $CleanExitCode
    $env:MOCK_BUILD_EXIT = $BuildExitCode
    $env:MOCK_SKIP_ARTIFACT = if ($SkipArtifact) { '1' } else { '0' }
    $env:MOCK_SOURCE_MUTATION = $SourceMutation
    $env:MOCK_BUNDLE_MUTATION = $BundleMutation

    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $output = & $Fixture.PowerShell -NoProfile -ExecutionPolicy Bypass -File $Fixture.Script 2>&1
        $exitCode = $LASTEXITCODE
    } finally { $ErrorActionPreference = $previousPreference }
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
    Assert-Equal @($result.Log.Split([Environment]::NewLine, [StringSplitOptions]::RemoveEmptyEntries) |
        Where-Object { $_.Contains('/p:MissionPlannerDir=' + $Fixture.Install) }).Count 2 `
        'Clean and build must explicitly use the verified host despite an environment override'
    Assert-Contains $result.Log "media-host=$($Fixture.Install)" 'Worker preparation must receive the verified host'
    $clean = $result.Log.IndexOf('/t:Clean', [StringComparison]::Ordinal)
    $media = $result.Log.IndexOf('media-worker=', [StringComparison]::Ordinal)
    $build = $result.Log.IndexOf('/t:Build', [StringComparison]::Ordinal)
    Assert-Equal ($clean -lt $media -and $media -lt $build) $true 'Worker preparation must run after clean and before plugin build'

    if (-not (Test-Path -LiteralPath $Fixture.Artifact -PathType Leaf)) {
        throw "Build-only did not produce the expected artifact at $($Fixture.Artifact)"
    }
    $targetPath = Join-Path $Fixture.Repo 'scripts/release/mission-planner-target.json'
    $targetVersion = (Get-Content $targetPath -Raw | ConvertFrom-Json).version
    if (-not (Test-Path -LiteralPath (Join-Path $Fixture.Repo 'build\release\ReleaseIdentity.cs'))) {
        throw 'Plugin build did not generate its C# embedded identity input.'
    }
    $csharp = Get-Content -LiteralPath (Join-Path $Fixture.Repo 'build\release\ReleaseIdentity.cs') -Raw
    $sourceSha = & git -C $Fixture.Repo rev-parse HEAD
    Assert-Contains $csharp "SourceSha = `"$sourceSha`"" 'Plugin identity omitted exact source SHA'
    Assert-Contains $csharp 'AssemblyVersion("0.5.0.0")' 'Assembly version must match the plugin release'
    Assert-Contains $csharp 'AssemblyFileVersion("0.5.0.0")' 'File version must match the plugin release'
    Assert-Contains $csharp "AssemblyInformationalVersion(`"0.5.0+$sourceSha`")" `
        'Informational version must identify the release and exact source'
    Assert-Contains $csharp 'Version = "0.5.0"' 'Mission Planner must display the plugin release version'
    if (Test-Path (Join-Path $Fixture.Repo 'build/release/package-identity.json')) {
        throw 'Plugin-only build unexpectedly produced a runtime package identity.'
    }

    Assert-Contains $csharp "MissionPlannerTarget = `"$targetVersion`"" `
        'C# build metadata did not carry the reviewed Mission Planner target'
    $manifest = Get-Content (Join-Path $Fixture.Repo 'build/release/plugin-build.json') -Raw | ConvertFrom-Json
    Assert-Equal $manifest.plugin_sha256 (Get-Sha256 $Fixture.Artifact) 'Build manifest omitted the actual DLL hash'
    Assert-Equal $manifest.source_files.'mission_planner\src\NOMADPlugin.csproj' `
        (Get-Sha256 (Join-Path $Fixture.Project 'NOMADPlugin.csproj')) 'Build manifest omitted its source inputs'
    Assert-Equal @($manifest.media_bundle.PSObject.Properties).Count 7 'Build proof omitted its private media bundle'
    Assert-Equal $manifest.media_bundle.'NOMADMediaWorker.exe' `
        ((Get-Sha256 (Join-Path $Fixture.Repo 'build/media_worker/NOMADMediaWorker.exe')).ToLowerInvariant()) `
        'Build proof omitted its worker payload hash'
}

function Test-SourceChangedDuringBuild {
    param([pscustomobject]$Fixture)
    $project = Join-Path $Fixture.Project 'NOMADPlugin.csproj'
    $original = [IO.File]::ReadAllText($project)
    foreach ($mutation in @('change','add')) {
        try {
            $result = Invoke-BuildScript -Fixture $Fixture -SourceMutation $mutation
            Assert-ExitCode $result 1 "Source $mutation during build"
            Assert-Contains $result.Output 'Product source changed during compilation' 'Changed source was qualified'
        } finally {
            [IO.File]::WriteAllText($project, $original)
            $added = Join-Path $Fixture.Project 'Unexpected.cs'
            if (Test-Path -LiteralPath $added) { Remove-Item -LiteralPath $added }
        }
    }
}

function Test-BuildProof {
    param([pscustomobject]$Fixture)
    . (Join-Path (Split-Path $Fixture.Script) 'windows_build_tools.ps1')
    $null = Get-CurrentPluginBuild -RepoRoot $Fixture.Repo -PluginPath $Fixture.Artifact
    $project = Join-Path $Fixture.Project 'NOMADPlugin.csproj'
    $added = Join-Path $Fixture.Project 'Unexpected.cs'
    $addedXaml = Join-Path $Fixture.Project 'Unexpected.xaml'
    $proof = Join-Path $Fixture.Repo 'build/release/plugin-build.json'
    $target = Join-Path $Fixture.Repo 'scripts/release/mission-planner-target.json'
    $preparation = Join-Path (Split-Path $Fixture.Script) 'prepare_vision.ps1'
    $originals = @{}
    foreach ($path in @($project, $proof, $target, $preparation, $Fixture.Artifact)) {
        $originals[$path] = [IO.File]::ReadAllBytes($path)
    }
    foreach ($mutation in @('change', 'add', 'xaml', 'payload', 'target', 'preparation', 'proof', 'media')) {
        try {
            switch ($mutation) {
                'change' { [IO.File]::AppendAllText($project, '<!-- changed after build -->') }
                'add' { [IO.File]::WriteAllText($added, '// added after build') }
                'xaml' { [IO.File]::WriteAllText($addedXaml, '<Grid />') }
                'payload' { [IO.File]::AppendAllText($Fixture.Artifact, 'changed payload') }
                'target' { [IO.File]::WriteAllText($target, '{"version":"9.9.9"}') }
                'preparation' { [IO.File]::AppendAllText($preparation, '# changed asset preparation') }
                'proof' { [IO.File]::WriteAllText($proof, '{}') }
                'media' {
                    $changed = Get-Content -LiteralPath $proof -Raw | ConvertFrom-Json
                    $changed.media_bundle.'NOMADMediaWorker.exe' = '0' * 64
                    $changed | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $proof -Encoding UTF8
                }
            }
            $refused = $false
            try { $null = Get-CurrentPluginBuild -RepoRoot $Fixture.Repo -PluginPath $Fixture.Artifact }
            catch {
                if ($mutation -eq 'media') {
                    Assert-Contains $_.Exception.Message 'Media bundle differs from its build proof' `
                        'Media proof was refused for the wrong reason'
                }
                $refused = $true
            }
            if (!$refused) { throw "Stale build proof accepted after $mutation." }
        } finally {
            foreach ($path in $originals.Keys) { [IO.File]::WriteAllBytes($path, $originals[$path]) }
            if (Test-Path -LiteralPath $added) { Remove-Item -LiteralPath $added }
            if (Test-Path -LiteralPath $addedXaml) { Remove-Item -LiteralPath $addedXaml }
        }
    }
    $null = Get-CurrentPluginBuild -RepoRoot $Fixture.Repo -PluginPath $Fixture.Artifact
    Write-Host 'Build proof rejects changed/added sources, asset preparation, payload, host target and invalid plugin/media proof.'
}

function Test-EmbeddedMediaBundle {
    param([pscustomobject]$Fixture)
    . (Join-Path (Split-Path $Fixture.Script) 'windows_build_tools.ps1')
    $proof = Join-Path $Fixture.Repo 'build/release/plugin-build.json'
    $originalProof = [IO.File]::ReadAllBytes($proof)
    $originalArtifact = [IO.File]::ReadAllBytes($Fixture.Artifact)
    $messages = @{ payload = 'checksum mismatch'; unlisted = 'Unexpected embedded media resource';
        'worker-name' = 'Invalid embedded media checksum' }
    foreach ($mutation in @('payload', 'unlisted', 'worker-name')) {
        try {
            $result = Invoke-BuildScript -Fixture $Fixture -BundleMutation $mutation
            Assert-ExitCode $result 0 "Synthetic bundle $mutation compilation"
            $copy = Join-Path $Fixture.Tools ("Bundle-$mutation.dll")
            Copy-Item -LiteralPath $Fixture.Artifact -Destination $copy
            $refused = $false
            try { $null = Get-CurrentPluginBuild -RepoRoot $Fixture.Repo -PluginPath $copy }
            catch {
                Assert-Contains $_.Exception.Message $messages[$mutation] 'Embedded bundle was refused for the wrong reason'
                $refused = $true
            }
            if (!$refused) { throw "Embedded bundle accepted $mutation." }
        } finally {
            [IO.File]::WriteAllBytes($proof, $originalProof)
            [IO.File]::WriteAllBytes($Fixture.Artifact, $originalArtifact)
        }
    }
    Write-Host 'Embedded media checks reject changed payload bytes, unlisted resources and a renamed worker entry.'
}

function Test-WorkerBuildFailure {
    param([pscustomobject]$Fixture)
    $step = Join-Path (Split-Path $Fixture.Script) 'build_media_worker.ps1'
    $original = [IO.File]::ReadAllBytes($step)
    try {
        [IO.File]::WriteAllText($step, "throw 'Synthetic media worker failure'")
        $result = Invoke-BuildScript -Fixture $Fixture
        Assert-ExitCode $result 1 'Media worker failure'
        Assert-Contains $result.Output 'Synthetic media worker failure' 'Worker preparation failure was hidden'
        Assert-Equal $result.Log.Contains('/t:Build') $false 'Plugin compilation ran after worker preparation failed'
    } finally { [IO.File]::WriteAllBytes($step, $original) }
}

function Test-ChangedReviewedTarget {
    param([pscustomobject]$Fixture)

    $targetPath = Join-Path $Fixture.Repo 'scripts/release/mission-planner-target.json'
    $original = Get-Content -LiteralPath $targetPath -Raw
    try {
        [IO.File]::WriteAllText($targetPath, '{"version":"2.3.4"}')
        $result = Invoke-BuildScript -Fixture $Fixture
        Assert-ExitCode $result 1 'Unreviewed reference host'
        Assert-Contains $result.Output 'Mission Planner reference version mismatch' 'Unreviewed host lacked a diagnostic'
        Assert-Equal ([string]::IsNullOrEmpty($result.Log)) $true 'MSBuild ran against an unreviewed host'
        Assert-Equal $result.Output.Contains('Synthetic pinned vision preparation') $false `
            'Asset preparation ran against an unreviewed host'
    } finally {
        [IO.File]::WriteAllText($targetPath, $original)
    }
}

function Test-MissingReferenceHost {
    param([pscustomobject]$Fixture)
    $hostPath = Join-Path $Fixture.Install 'MissionPlanner.exe'
    $preserved = Join-Path $Fixture.Tools 'PreservedMissionPlanner.exe'
    Move-Item -LiteralPath $hostPath -Destination $preserved
    try {
        $result = Invoke-BuildScript -Fixture $Fixture
        Assert-ExitCode $result 1 'Missing reference host'
        Assert-Contains $result.Output 'Mission Planner reference host is missing' 'Missing host lacked a diagnostic'
        Assert-Equal ([string]::IsNullOrEmpty($result.Log)) $true 'MSBuild ran without a reference host'
        Assert-Equal $result.Output.Contains('Synthetic pinned vision preparation') $false `
            'Asset preparation ran without a reference host'
    } finally { Move-Item -LiteralPath $preserved -Destination $hostPath }
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
    $unrelatedRoot = Split-Path (Split-Path $Fixture.UnrelatedPlugin)
    Assert-Equal -Actual (Get-TreeSnapshot -Path $unrelatedRoot) -Expected $Fixture.UnrelatedBefore `
        -Message 'Build-only changed the unrelated AppData plugin fixture'
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
        MissionPlannerDir = [Environment]::GetEnvironmentVariable('MissionPlannerDir', 'Process')
        MockLog = [Environment]::GetEnvironmentVariable('MOCK_MSBUILD_LOG', 'Process')
        CleanExit = [Environment]::GetEnvironmentVariable('MOCK_CLEAN_EXIT', 'Process')
        BuildExit = [Environment]::GetEnvironmentVariable('MOCK_BUILD_EXIT', 'Process')
        SkipArtifact = [Environment]::GetEnvironmentVariable('MOCK_SKIP_ARTIFACT', 'Process')
        SourceMutation = [Environment]::GetEnvironmentVariable('MOCK_SOURCE_MUTATION', 'Process')
        BundleMutation = [Environment]::GetEnvironmentVariable('MOCK_BUNDLE_MUTATION', 'Process')
        VsPath = [Environment]::GetEnvironmentVariable('MOCK_VS_PATH', 'Process')
        VsExit = [Environment]::GetEnvironmentVariable('MOCK_VS_EXIT', 'Process')
    }
}

function Restore-ProcessEnvironment {
    param([pscustomobject]$Original)

    [Environment]::SetEnvironmentVariable('PATH', $Original.Path, 'Process')
    [Environment]::SetEnvironmentVariable('ProgramFiles(x86)', $Original.ProgramFilesX86, 'Process')
    [Environment]::SetEnvironmentVariable('LOCALAPPDATA', $Original.LocalAppData, 'Process')
    [Environment]::SetEnvironmentVariable('USERPROFILE', $Original.UserProfile, 'Process')
    [Environment]::SetEnvironmentVariable('MissionPlannerDir', $Original.MissionPlannerDir, 'Process')
    [Environment]::SetEnvironmentVariable('MOCK_MSBUILD_LOG', $Original.MockLog, 'Process')
    [Environment]::SetEnvironmentVariable('MOCK_CLEAN_EXIT', $Original.CleanExit, 'Process')
    [Environment]::SetEnvironmentVariable('MOCK_BUILD_EXIT', $Original.BuildExit, 'Process')
    [Environment]::SetEnvironmentVariable('MOCK_SKIP_ARTIFACT', $Original.SkipArtifact, 'Process')
    [Environment]::SetEnvironmentVariable('MOCK_SOURCE_MUTATION', $Original.SourceMutation, 'Process')
    [Environment]::SetEnvironmentVariable('MOCK_BUNDLE_MUTATION', $Original.BundleMutation, 'Process')
    [Environment]::SetEnvironmentVariable('MOCK_VS_PATH', $Original.VsPath, 'Process')
    [Environment]::SetEnvironmentVariable('MOCK_VS_EXIT', $Original.VsExit, 'Process')
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

    Test-ToolDiscovery -Fixture $fixture
    Test-BuildOnlySuccess -Fixture $fixture
    Test-BuildProof -Fixture $fixture
    Test-EmbeddedMediaBundle -Fixture $fixture
    Test-ChangedReviewedTarget -Fixture $fixture
    Test-MissingReferenceHost -Fixture $fixture
    Test-SourceChangedDuringBuild -Fixture $fixture
    Test-WorkerBuildFailure -Fixture $fixture
    Test-BuildFailure -Fixture $fixture
    Test-CleanFailure -Fixture $fixture
    Test-MissingArtifact -Fixture $fixture
    Test-InstallationUnchanged -Fixture $fixture

    $summary = 'Tool discovery and build-only dispatch tests passed: artifact, paths with spaces, '
    $summary += 'private media bundle/proof, exact failures, and unchanged installation.'
    Write-Host $summary -ForegroundColor Green
} finally {
    Restore-ProcessEnvironment -Original $originalEnvironment
    if ($fixture) {
        Remove-TestFixture -Fixture $fixture
    }
}
