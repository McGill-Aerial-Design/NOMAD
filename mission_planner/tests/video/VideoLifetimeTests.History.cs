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
            FindVideoButton(player, "Video options").PerformClick();
            var button = player.Toolbar.Controls.OfType<FlowLayoutPanel>().SelectMany(row => row.Controls.OfType<Button>())
                .Single(item => item.Text == "History");
            var show = typeof(EmbeddedVideoPlayer).GetMethod("ShowTrackingHistory", BindingFlags.Instance | BindingFlags.NonPublic);
            var menu = GetField<ContextMenuStrip>(player, "_trackingHistoryMenu");
            for (int index = 0; index < 4; index++)
            {
                show.Invoke(player, new object[] { button, EventArgs.Empty });
                Application.DoEvents();
                Check(menu.Visible && menu.Items.Count > 0, "History menu failed to open");
                typeof(ToolStripDropDown).GetMethod("OnItemClicked", BindingFlags.Instance | BindingFlags.NonPublic)
                    .Invoke(menu, new object[] { new ToolStripItemClickedEventArgs(menu.Items[0]) });
                Application.DoEvents();
                Check(!menu.Visible && !menu.IsDisposed &&
                    ReferenceEquals(menu, GetField<ContextMenuStrip>(player, "_trackingHistoryMenu")),
                    "History click/close disposed the menu while WinForms still owned its click processing");
            }
            show.Invoke(player, new object[] { button, EventArgs.Empty });
            Check(menu.Visible, "Final History menu did not open before owner close");
            player.Dispose(); Application.DoEvents();
            Check(menu.IsDisposed, "Player disposal leaked its owned History menu");
            using (var other = new ContextMenuStrip())
            { other.Items.Add("Unrelated popup"); other.Show(form, Point.Empty); other.Close(); Application.DoEvents(); }
        }
    }
}
