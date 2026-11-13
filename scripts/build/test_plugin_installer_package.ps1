# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
param([string]$InstallerDirectory)
$ErrorActionPreference = 'Stop'
Import-Module (Join-Path $PSHOME 'Modules/Microsoft.PowerShell.Utility/Microsoft.PowerShell.Utility.psd1') -ErrorAction Stop
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
if (!$InstallerDirectory) { $InstallerDirectory = Join-Path $root 'build/installer' }
$manifest = Get-Content -LiteralPath (Join-Path $InstallerDirectory 'installer-manifest.json') -Raw | ConvertFrom-Json
$exe = (Resolve-Path -LiteralPath (Join-Path $InstallerDirectory ('NOMAD-' + $manifest.version + '-Setup.exe'))).Path
if ((Get-FileHash -LiteralPath $exe).Hash -ne $manifest.installer_sha256) { throw 'Setup checksum differs.' }
$checksumLines = @(Get-Content -LiteralPath (Join-Path $InstallerDirectory 'SHA256SUMS'))
if ($checksumLines.Count -ne 2) { throw 'Setup requires two separate checksum entries.' }
$checksumFiles = @([IO.Path]::GetFileName($exe), 'installer-manifest.json')
$seenFiles = @()
foreach ($line in $checksumLines) {
    $parts = $line.Split(@('  '), 2, [StringSplitOptions]::None)
    if ($parts.Count -ne 2 -or $parts[0] -notmatch '^[0-9a-f]{64}$' -or
        $checksumFiles -cnotcontains $parts[1] -or $seenFiles -ccontains $parts[1]) {
        throw 'Invalid or duplicate setup checksum entry.'
    }
    if ((Get-FileHash -LiteralPath (Join-Path $InstallerDirectory $parts[1])).Hash -ne $parts[0]) {
        throw ('Setup checksum entry differs: ' + $parts[1])
    }
    $seenFiles += $parts[1]
}
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
$assembly = [Reflection.Assembly]::LoadFile($exe)
if ($assembly.GetName().Version.ToString() -ne ($manifest.version + '.0')) { throw 'Setup version differs.' }
$names = @($assembly.GetManifestResourceNames() | Sort-Object)
if (($names -join '|') -ne 'NOMAD.LICENSE|NOMAD.NOTICE|NOMAD.Payload.dll') { throw 'Unexpected embedded files.' }
foreach ($resource in @(@('NOMAD.Payload.dll', $manifest.plugin_sha256),
    @('NOMAD.LICENSE', (Get-FileHash (Join-Path $root 'LICENSE')).Hash),
    @('NOMAD.NOTICE', (Get-FileHash (Join-Path $root 'NOTICE')).Hash))) {
    $stream = $assembly.GetManifestResourceStream($resource[0])
    $algorithm = [Security.Cryptography.SHA256]::Create()
    try { $hash = [BitConverter]::ToString($algorithm.ComputeHash($stream)).Replace('-','') }
    finally { $stream.Dispose(); $algorithm.Dispose() }
    if ($hash -ne $resource[1]) { throw ('Embedded resource differs: ' + $resource[0]) }
}
. (Join-Path $PSScriptRoot 'windows_build_tools.ps1')
$payloadPath = [IO.Path]::GetTempFileName()
try {
    $payload = $assembly.GetManifestResourceStream('NOMAD.Payload.dll')
    $output = [IO.File]::Create($payloadPath)
    try { $payload.CopyTo($output) } finally { $output.Dispose(); $payload.Dispose() }
    $null = Get-EmbeddedMediaBundle -PluginPath $payloadPath
} finally { Remove-Item -LiteralPath $payloadPath -Force }
$allowed = @('mscorlib','System','System.Core','System.Windows.Forms','System.Drawing')
foreach ($reference in $assembly.GetReferencedAssemblies()) {
    if ($allowed -notcontains $reference.Name) { throw ('Non-framework setup dependency: ' + $reference.Name) }
}
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class SetupManifestReader {
    [DllImport("kernel32.dll", CharSet=CharSet.Unicode)]
    private static extern IntPtr LoadLibraryExW(string path, IntPtr file, uint flags);
    [DllImport("kernel32.dll")]
    private static extern IntPtr FindResourceW(IntPtr module, IntPtr name, IntPtr type);
    [DllImport("kernel32.dll")]
    private static extern IntPtr LoadResource(IntPtr module, IntPtr resource);
    [DllImport("kernel32.dll")]
    private static extern IntPtr LockResource(IntPtr resource);
    [DllImport("kernel32.dll")]
    private static extern uint SizeofResource(IntPtr module, IntPtr resource);
    [DllImport("kernel32.dll")]
    private static extern bool FreeLibrary(IntPtr module);
    public static string Read(string path) {
        IntPtr module = LoadLibraryExW(path, IntPtr.Zero, 2);
        if (module == IntPtr.Zero) throw new Exception("Setup native resources unavailable");
        try {
            IntPtr resource = FindResourceW(module, new IntPtr(1), new IntPtr(24));
            if (resource == IntPtr.Zero) throw new Exception("Setup execution manifest missing");
            byte[] bytes = new byte[SizeofResource(module, resource)];
            Marshal.Copy(LockResource(LoadResource(module, resource)), bytes, 0, bytes.Length);
            return System.Text.Encoding.UTF8.GetString(bytes).Trim('\0', '\uFEFF');
        } finally { FreeLibrary(module); }
    }
}
'@
[xml]$native = [SetupManifestReader]::Read($exe)
$level = @($native.SelectNodes("//*[local-name()='requestedExecutionLevel']"))
if ($level.Count -ne 1 -or $level[0].level -ne 'requireAdministrator' -or $level[0].uiAccess -ne 'false') {
    throw 'Setup does not request the intended Windows administrator consent.'
}
$form = [Activator]::CreateInstance($assembly.GetType('NOMAD.Installer.InstallerForm', $true), $true)
try {
    $form.CreateControl()
    $form.PerformLayout()
    if ($form.Text -ne 'NOMAD Setup') { throw 'Setup window failed to construct.' }
} finally { $form.Dispose() }

# Exercise the real packager with a compiler that changes an input after consuming it.
$temporaryRoot = [IO.Path]::GetFullPath([IO.Path]::GetTempPath()).TrimEnd('\') + '\'
$fixture = Join-Path $temporaryRoot ('NOMAD-installer-build-' + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $fixture | Out-Null
try {
    foreach ($relative in @('scripts/build/build_plugin_installer.ps1',
        'mission_planner/packaging/installer/Program.cs', 'mission_planner/packaging/installer/InstallEngine.cs',
        'mission_planner/packaging/installer/app.manifest', 'mission_planner/src/Plugin/MissionPlannerVersion.cs',
        'scripts/release/mission-planner-target.json', 'LICENSE', 'NOTICE')) {
        $destination = Join-Path $fixture $relative
        New-Item -ItemType Directory -Path (Split-Path $destination) -Force | Out-Null
        Copy-Item -LiteralPath (Join-Path $root $relative) -Destination $destination
    }
    $fixturePayload = Join-Path $fixture 'payload.dll'
    $payload = $assembly.GetManifestResourceStream('NOMAD.Payload.dll')
    $output = [IO.File]::Create($fixturePayload)
    try { $payload.CopyTo($output) } finally { $output.Dispose(); $payload.Dispose() }
    [IO.File]::WriteAllText((Join-Path $fixture 'payload.sha256'), $manifest.plugin_sha256)
    [IO.File]::WriteAllText((Join-Path $fixture 'source.sha'), $manifest.plugin_source_sha)
    @'
function Get-CurrentPluginBuild {
    param($RepoRoot, $PluginPath)
    if ((Get-FileHash -LiteralPath $PluginPath).Hash -ne (Get-Content (Join-Path $RepoRoot 'payload.sha256'))) {
        throw 'Fixture payload changed during compilation.'
    }
}
function Find-RoslynCompiler { return (Join-Path $PSScriptRoot 'compiler.ps1') }
'@ | Set-Content -LiteralPath (Join-Path $fixture 'scripts/build/windows_build_tools.ps1')
    @'
$fixtureRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$outputArgument = @($args | Where-Object { $_.StartsWith('/out:') })
if ($outputArgument.Count -ne 1) { throw 'Fixture compiler did not receive one output path.' }
[IO.File]::WriteAllText($outputArgument[0].Substring(5), 'Synthetic setup; never execute.')
switch (Get-Content (Join-Path $fixtureRoot 'mutation.txt')) {
    'source' {
        [IO.File]::AppendAllText((Join-Path $fixtureRoot 'mission_planner/packaging/installer/InstallEngine.cs'), '// changed')
    }
    'proof' { [IO.File]::WriteAllText((Join-Path $fixtureRoot 'payload.sha256'), '0' * 64) }
    'payload' {
        $stream = [IO.File]::OpenWrite((Join-Path $fixtureRoot 'payload.dll'))
        try { $stream.SetLength($stream.Length + 1) } finally { $stream.Dispose() }
    }
}
$global:LASTEXITCODE = 0
'@ | Set-Content -LiteralPath (Join-Path $fixture 'scripts/build/compiler.ps1')
    @'
param($OutputDirectory)
function git { $global:LASTEXITCODE = 0; return (Get-Content (Join-Path $PSScriptRoot 'source.sha')) }
& (Join-Path $PSScriptRoot 'scripts/build/build_plugin_installer.ps1') `
    -PluginPath (Join-Path $PSScriptRoot 'payload.dll') -OutputDirectory $OutputDirectory
'@ | Set-Content -LiteralPath (Join-Path $fixture 'run.ps1')
    foreach ($mutation in @('none', 'source', 'proof', 'payload')) {
        Copy-Item -LiteralPath (Join-Path $root 'mission_planner/packaging/installer/InstallEngine.cs') `
            -Destination (Join-Path $fixture 'mission_planner/packaging/installer/InstallEngine.cs') -Force
        [IO.File]::WriteAllText((Join-Path $fixture 'payload.sha256'), $manifest.plugin_sha256)
        [IO.File]::WriteAllText((Join-Path $fixture 'mutation.txt'), $mutation)
        $caseOutput = Join-Path $fixture $mutation
        $start = New-Object Diagnostics.ProcessStartInfo
        $start.FileName = Join-Path $env:WINDIR 'System32/WindowsPowerShell/v1.0/powershell.exe'
        $start.Arguments = '-NoProfile -ExecutionPolicy Bypass -File "' + (Join-Path $fixture 'run.ps1') + '" "' + $caseOutput + '"'
        $start.UseShellExecute = $false
        $start.CreateNoWindow = $true
        $start.RedirectStandardOutput = $start.RedirectStandardError = $true
        $process = [Diagnostics.Process]::Start($start)
        try {
            $stdout = $process.StandardOutput.ReadToEndAsync()
            $stderr = $process.StandardError.ReadToEndAsync()
            if (!$process.WaitForExit(30000)) {
                $process.Kill()
                [void]$process.WaitForExit(3000)
                throw "Installer build fixture exceeded 30 seconds: $mutation"
            }
            $diagnostic = $stdout.Result + $stderr.Result
            $proof = Join-Path $caseOutput 'installer-manifest.json'
            if ($mutation -eq 'none') {
                if ($process.ExitCode -ne 0 -or !(Test-Path -LiteralPath $proof)) {
                    throw "Unchanged installer inputs failed: $diagnostic"
                }
                $saved = Get-Content -LiteralPath $proof -Raw | ConvertFrom-Json
                $expected = (Get-FileHash -LiteralPath (Join-Path $root 'mission_planner/packaging/installer/InstallEngine.cs')).Hash
                if ($saved.inputs_sha256.'mission_planner/packaging/installer/InstallEngine.cs' -ne $expected) {
                    throw 'Installer proof omitted the source used by the compiler.'
                }
            } else {
                $expected = switch ($mutation) {
                    'source' { 'Installer inputs changed' }
                    'proof' { 'Fixture payload changed' }
                    'payload' { 'Plugin payload changed' }
                }
                if ($process.ExitCode -eq 0 -or (Test-Path -LiteralPath $proof) -or !$diagnostic.Contains($expected)) {
                    throw "Installer accepted changed $mutation inputs or failed for the wrong reason: $diagnostic"
                }
            }
        } finally { $process.Dispose() }
    }
} finally {
    $resolved = [IO.Path]::GetFullPath($fixture)
    if (!$resolved.StartsWith($temporaryRoot, [StringComparison]::OrdinalIgnoreCase) -or
        (Split-Path $resolved -Leaf) -notlike 'NOMAD-installer-build-*') { throw 'Unexpected installer fixture cleanup path.' }
    Remove-Item -LiteralPath $resolved -Recurse -Force
}
Write-Output ('Installer package passed: embedded DLL/media bundle/licenses, version/hash, framework-only references, ' +
    'administrator manifest, UI construction and rejection of inputs changed during packaging. No installation invoked.')
