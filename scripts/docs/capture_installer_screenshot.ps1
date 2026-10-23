# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
param([Parameter(Mandatory=$true)][string]$InstallerPath, [string]$OutputPath = 'docs/images/windows-installer.png')
$ErrorActionPreference = 'Stop'
if ([Threading.Thread]::CurrentThread.ApartmentState -ne 'STA') { throw 'Run with powershell -STA.' }
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
. (Join-Path $PSScriptRoot 'render_controls.ps1')
[Windows.Forms.Application]::EnableVisualStyles()
$assembly = [Reflection.Assembly]::LoadFile((Resolve-Path -LiteralPath $InstallerPath).Path)
$form = [Activator]::CreateInstance($assembly.GetType('NOMAD.Installer.InstallerForm', $true), $true)
Save-ControlImage $form $OutputPath
Write-Output 'Captured actual setup controls; installation was not invoked.'
