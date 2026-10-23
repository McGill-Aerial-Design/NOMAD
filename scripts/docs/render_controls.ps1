# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
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
    }
}
'@

function Save-ControlImage([Windows.Forms.Control]$Control, [string]$ImagePath) {
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
        $renderBitmap = [Drawing.Bitmap]::new($renderSize.Width, $renderSize.Height)
        try {
            $renderForm.DrawToBitmap($renderBitmap, [Drawing.Rectangle]::new(0, 0, $renderSize.Width, $renderSize.Height))
            $renderBitmap.Save([IO.Path]::GetFullPath($ImagePath), [Drawing.Imaging.ImageFormat]::Png)
        } finally { $renderBitmap.Dispose() }
        if ([NOMAD.Documentation.RenderHost]::GetForegroundWindow() -ne $renderForeground) {
            throw 'Foreground changed during the owned render; no other window was captured or activated.'
        }
    } finally { $renderForm.Dispose() }
}
