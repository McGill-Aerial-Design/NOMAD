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

        private void ShutdownVideo(bool final = false)
        {
            var shutdown = System.Threading.Interlocked.Exchange(ref _videoShutdown, null);
            var hud = System.Threading.Interlocked.Exchange(ref _hudVideo, null);
            var menu = System.Threading.Interlocked.Exchange(ref _hudVideoMenuItem, null);
            NOMADMainScreen.SetVideoShutdown(new System.Threading.CancellationToken(true));
            NOMADMainScreen.SetVideoInputSelection(null);
            NOMADMainScreen.SetVideoModelSelection(null);
            try { shutdown?.Cancel(); }
            catch (System.Exception error) { Log.Error("Video shutdown cancellation failed: " + error); }
            finally
            {
                shutdown?.Dispose();
                hud?.Retire();
                if (final) { MediaWorkerHost.Shutdown(); }
            }
            System.Action cleanup = () =>
            {
                try { hud?.Dispose(); }
                catch (System.Exception error) { Log.Error("HUD video cleanup failed: " + error); }
                if (menu != null)
                {
                    menu.Click -= OnHudVideoMenuClicked;
                    menu.Dispose();
                }
            };
            var form = Host?.MainForm;
            if (form != null && !form.IsDisposed && form.InvokeRequired)
            {
                try { form.BeginInvoke(cleanup); }
                catch (System.InvalidOperationException) { }
            }
            else { cleanup(); }
        }
    }
}
