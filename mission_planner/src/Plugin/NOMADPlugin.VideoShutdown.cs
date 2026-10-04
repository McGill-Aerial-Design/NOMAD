// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
namespace NOMAD.MissionPlanner
{
    public partial class NOMADPlugin
    {
        private System.Windows.Forms.ToolStripMenuItem _hudVideoMenuItem;

        private void OnHudVideoMenuClicked(object sender, System.EventArgs e)
        {
            ToggleHudVideo();
            if (_hudVideoMenuItem != null)
            {
                _hudVideoMenuItem.Text = _hudVideoStarted ? "Stop HUD Video" : "Start HUD Video";
            }
        }

        private void ShutdownVideo()
        {
            if (Host?.MainForm != null && !Host.MainForm.IsDisposed && Host.MainForm.InvokeRequired)
            {
                Host.MainForm.Invoke((System.Action)ShutdownVideo);
                return;
            }
            var shutdown = _videoShutdown;
            _videoShutdown = null;
            try
            {
                shutdown?.Cancel();
            }
            finally
            {
                shutdown?.Dispose();
                _hudVideo?.Dispose();
                _hudVideo = null;
                if (_hudVideoMenuItem != null)
                {
                    _hudVideoMenuItem.Click -= OnHudVideoMenuClicked;
                    _hudVideoMenuItem.Dispose();
                    _hudVideoMenuItem = null;
                }
            }
        }
    }
}
