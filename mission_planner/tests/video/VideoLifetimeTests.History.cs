// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Linq;
using System.Reflection;
using System.Threading;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void TrackingHistoryMenuLifetime()
    {
        using (var form = new Form { ShowInTaskbar = false, Location = new Point(-2000, -2000),
            StartPosition = FormStartPosition.Manual, Size = new Size(800, 600) })
        using (var player = new EmbeddedVideoPlayer(VideoConfig().SelectedVideoInput, true, CancellationToken.None,
            () => throw new InvalidOperationException("History must not activate video")))
        {
            form.Controls.Add(player); form.Show();
            ClickVideoButton(FindVideoButton(player, "Video options"));
            var button = player.VideoControls.HistoryButton;
            var show = typeof(EmbeddedVideoPlayer).GetMethod("ShowTrackingHistory", BindingFlags.Instance | BindingFlags.NonPublic);
            var menu = GetField<System.Windows.Controls.ContextMenu>(player, "_trackingHistoryMenu");
            for (int index = 0; index < 4; index++)
            {
                show.Invoke(player, new object[] { button, EventArgs.Empty });
                Application.DoEvents();
                Check(menu.IsOpen && menu.Items.Count > 0, "History menu failed to open");
                var peer = new System.Windows.Automation.Peers.MenuItemAutomationPeer((System.Windows.Controls.MenuItem)menu.Items[0]);
                ((System.Windows.Automation.Provider.IInvokeProvider)peer.GetPattern(
                    System.Windows.Automation.Peers.PatternInterface.Invoke)).Invoke();
                WaitUi(() => !menu.IsOpen, "history item click closes its popup");
                Check(ReferenceEquals(menu, GetField<System.Windows.Controls.ContextMenu>(player, "_trackingHistoryMenu")),
                    "History click/close lost its player-owned menu");
            }
            show.Invoke(player, new object[] { button, EventArgs.Empty });
            Check(menu.IsOpen, "Final History menu did not open before owner close");
            player.Dispose(); Application.DoEvents();
            Check(!menu.IsOpen && menu.PlacementTarget == null && menu.Items.Count == 0,
                "Player disposal leaked its owned History menu or placement target");
            using (var other = new ContextMenuStrip())
            { other.Items.Add("Unrelated popup"); other.Show(form, Point.Empty); other.Close(); Application.DoEvents(); }
        }
    }
}
