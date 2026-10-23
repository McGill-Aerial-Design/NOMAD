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
Write-Output 'Installer package passed: embedded DLL/licenses, version/hash, framework-only references, administrator manifest and UI construction. No installation invoked.'
