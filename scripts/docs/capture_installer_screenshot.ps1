# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
param([Parameter(Mandatory=$true)][string]$InstallerPath, [string]$OutputPath = 'docs/images/windows-installer.png')
$ErrorActionPreference = 'Stop'
if ([Threading.Thread]::CurrentThread.ApartmentState -ne 'STA') { throw 'Run with powershell -STA.' }
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[Windows.Forms.Application]::EnableVisualStyles()
$assembly = [Reflection.Assembly]::LoadFile((Resolve-Path -LiteralPath $InstallerPath).Path)
$form = [Activator]::CreateInstance($assembly.GetType('NOMAD.Installer.InstallerForm', $true), $true)
try {
    $form.StartPosition = 'Manual'
    $form.Location = [Drawing.Point]::new(-20000, -20000)
    $form.ShowInTaskbar = $false
    $form.Show()
    $form.PerformLayout()
    [Windows.Forms.Application]::DoEvents()
    $bitmap = [Drawing.Bitmap]::new($form.Width, $form.Height)
    try {
        $form.DrawToBitmap($bitmap, [Drawing.Rectangle]::new(0, 0, $form.Width, $form.Height))
        $bitmap.Save([IO.Path]::GetFullPath($OutputPath), [Drawing.Imaging.ImageFormat]::Png)
    } finally { $bitmap.Dispose() }
} finally { $form.Dispose() }
Write-Output 'Captured actual setup controls; installation was not invoked.'
