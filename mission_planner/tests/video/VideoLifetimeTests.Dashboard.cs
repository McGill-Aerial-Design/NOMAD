// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System.Threading;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void DashboardShutdown()
    {
        var previousService = NotificationService.Shared;
        using (var sharedService = new NotificationService())
        using (var shutdown = new CancellationTokenSource())
        {
            NotificationService.Shared = sharedService;
            try
            {
                CheckDashboardReopen(shutdown);
            }
            finally
            {
                NotificationService.Shared = previousService;
            }
        }
    }

    private static void CheckDashboardReopen(CancellationTokenSource shutdown)
    {
        var config = VideoConfig();
        var first = new FakeVideoPipeline();
        var dashboard = new NOMADDashboardView(config, null, shutdown.Token, () => first);
        var oldPlayer = GetField<VideoInputPlayer>(dashboard, "_videoHost").Player;
        var handle = oldPlayer.Handle;
        Wait(first.ReadEntered, "dashboard automatic preview");
        dashboard.Dispose();
        CheckClean(first);
        Check(oldPlayer.IsDisposed, "Dashboard disposal retained preview controls");
        var second = new FakeVideoPipeline();
        using (var reopened = new NOMADDashboardView(config, null, shutdown.Token, () => second))
        {
            var player = GetField<VideoInputPlayer>(reopened, "_videoHost").Player;
            handle = player.Handle;
            Wait(second.ReadEntered, "reopened dashboard preview");
            var plugin = new NOMADPlugin();
            SetField(plugin, "_videoShutdown", shutdown);
            plugin.Exit();
            CheckClean(second);
            Check(player.IsDisposed, "Plugin exit retained dashboard preview");
            Check(handle != System.IntPtr.Zero, "Dashboard preview had no UI handle");
        }
    }
}
