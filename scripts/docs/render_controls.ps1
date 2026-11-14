# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName PresentationFramework,PresentationCore,WindowsBase,WindowsFormsIntegration,System.Xaml
$renderReferences = @('System.Windows.Forms', 'System.Drawing',
    [Windows.Controls.UserControl].Assembly.Location, [Windows.Media.Visual].Assembly.Location,
    [Windows.DependencyObject].Assembly.Location, [Windows.Forms.Integration.ElementHost].Assembly.Location,
    [Windows.Markup.IUriContext].Assembly.Location)
Add-Type -Path (Join-Path $PSScriptRoot '../../mission_planner/tests/ui/ControlRender.cs') -ReferencedAssemblies $renderReferences
Add-Type -ReferencedAssemblies System.Windows.Forms,System.Drawing -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
using System.Windows.Forms;
namespace NOMAD.Documentation {
    public sealed class RenderHost : Form {
        protected override bool ShowWithoutActivation { get { return true; } }
        protected override CreateParams CreateParams {
            get { var value = base.CreateParams; value.ExStyle |= 0x08000000; return value; }
        }
        [DllImport("user32.dll")]
        public static extern IntPtr GetForegroundWindow();
        [DllImport("user32.dll")]
        public static extern IntPtr SetThreadDpiAwarenessContext(IntPtr context);
    }
}
'@

function Save-ControlImage([Windows.Forms.Control]$Control, [string]$ImagePath, [scriptblock]$BeforeDraw = $null) {
    # Match the reviewed Mission Planner host rather than PowerShell's DPI mode.
    $renderDpiContext = [NOMAD.Documentation.RenderHost]::SetThreadDpiAwarenessContext([IntPtr]::new(-1))
    $renderSize = if ($Control -is [Windows.Forms.Form]) { $Control.ClientSize } else {
        [Drawing.Size]::new(1180, 760)
    }
    $renderForm = [NOMAD.Documentation.RenderHost]::new()
    $renderForeground = [NOMAD.Documentation.RenderHost]::GetForegroundWindow()
    try {
        $renderForm.FormBorderStyle = 'None'
        $renderForm.ClientSize = $renderSize
        $renderForm.StartPosition = 'Manual'
        $renderForm.Location = [Drawing.Point]::new(-20000, -20000)
        $renderForm.ShowInTaskbar = $false
        if ($Control -is [Windows.Forms.Form]) {
            $Control.TopLevel = $false
            $Control.FormBorderStyle = 'None'
        }
        $Control.Dock = 'Fill'
        $renderForm.Controls.Add($Control)
        $Control.Show()
        $renderForm.Show()
        $renderForm.PerformLayout()
        [Windows.Forms.Application]::DoEvents()
        if ($BeforeDraw) { & $BeforeDraw }
        $renderBitmap = [NOMAD.Testing.ControlRender]::Capture($renderForm)
        try {
            $renderBitmap.Save([IO.Path]::GetFullPath($ImagePath), [Drawing.Imaging.ImageFormat]::Png)
        } finally { $renderBitmap.Dispose() }
        if ([NOMAD.Documentation.RenderHost]::GetForegroundWindow() -ne $renderForeground) {
            throw 'Foreground changed during the owned render; no other window was captured or activated.'
        }
    } finally {
        $renderForm.Dispose()
        if ($renderDpiContext -ne [IntPtr]::Zero) {
            [NOMAD.Documentation.RenderHost]::SetThreadDpiAwarenessContext($renderDpiContext) | Out-Null
        }
    }
}
