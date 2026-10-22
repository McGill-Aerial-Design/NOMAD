// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Diagnostics;
using System.IO;
using System.Reflection;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static class RouterStatusUiTests
{
    private const BindingFlags PrivateInstance = BindingFlags.Instance | BindingFlags.NonPublic;
    private static int _checks;
    private static int _failures;

    [STAThread]
    private static int Main(string[] args)
    {
        if (args.Length != 1)
        {
            Console.Error.WriteLine("Pass the Mission Planner reference directory.");
            return 2;
        }
        AppDomain.CurrentDomain.AssemblyResolve += (sender, request) =>
        {
            string name = new AssemblyName(request.Name).Name;
            string path = Path.Combine(args[0], name == "MissionPlanner" ? name + ".exe" : name + ".dll");
            return File.Exists(path) ? Assembly.LoadFrom(path) : null;
        };
        Run("dashboard router summary", DashboardSummary);
        Run("Links view availability", LinksViewAvailability);
        Run("settings save preserves router and unload session", SettingsRouterLifetime);
        Run("aircraft profile retires cached views", AircraftProfileRetiresCachedViews);
        Run("deferred theme and session-owned Settings", SidebarThemeAndSettings);
        Run("native Fence input survives screen navigation", FenceInputSurvivesNavigation);
        Console.WriteLine($"Router status UI checks: {_checks}; failures: {_failures}");
        return _failures == 0 ? 0 : 1;
    }

    private static void Run(string name, Action test)
    {
        int before = _failures;
        try { test(); }
        catch (Exception ex)
        {
            _failures++;
            Console.WriteLine($"FAIL: {name} raised {ex.GetBaseException().GetType().Name}");
        }
        Console.WriteLine((_failures == before ? "PASS: " : "FAIL: ") + name);
    }

    private static void Check(bool condition, string message)
    {
        _checks++;
        if (!condition)
        {
            _failures++;
            Console.WriteLine("FAIL: " + message);
        }
    }

    private static void SettingsRouterLifetime()
    {
        var plugin = (NOMADPlugin)System.Runtime.Serialization.FormatterServices.GetUninitializedObject(typeof(NOMADPlugin));
        var type = typeof(NOMADPlugin);
        using (var session = new System.Threading.CancellationTokenSource())
        using (var replacement = new System.Threading.CancellationTokenSource())
        using (var manager = new MAVLinkConnectionManager(new MAVLinkConnectionManager.ConnectionConfig
        {
            RouterLocalPort = 33200, Links = new List<LinkConfig>
            { new LinkConfig { Id = "primary", Port = 33201, BindAddress = "127.0.0.1" } }
        }))
        {
            type.GetField("_videoShutdown", PrivateInstance).SetValue(plugin, session);
            var active = type.GetMethod("IsActiveSession", PrivateInstance);
            Check((bool)active.Invoke(plugin, new object[] { session }), "current plugin session permits settings");
            manager.StartMonitoring();
            type.GetField("_connectionManager", PrivateInstance).SetValue(plugin, manager);
            type.GetField("_config", PrivateInstance).SetValue(plugin, new NOMADConfig());
            var routerField = typeof(MAVLinkConnectionManager).GetField("_router", PrivateInstance);
            object original = routerField.GetValue(manager);
            type.GetMethod("ApplyRouterClientSettings", PrivateInstance).Invoke(plugin, new object[] { false });
            Check(manager.IsMonitoring && ReferenceEquals(original, routerField.GetValue(manager)),
                "unchanged router settings Save preserves the active router and endpoints");
            type.GetField("_videoShutdown", PrivateInstance).SetValue(plugin, replacement);
            Check(!(bool)active.Invoke(plugin, new object[] { session }), "dialog from an earlier session cannot apply");
            replacement.Cancel();
            Check(!(bool)active.Invoke(plugin, new object[] { replacement }), "cancelled session cannot apply");
            foreach (string action in new[] { "ShowSettings", "ShowMainScreen", "ShowPopOutWindow", "ShowLinkHealthPanel" })
            {
                type.GetMethod(action, PrivateInstance).Invoke(plugin, null);
            }
            Check(ReferenceEquals(manager, type.GetField("_connectionManager", PrivateInstance).GetValue(plugin)),
                "stale menu callbacks cannot recreate plugin owners");
        }
        NOMADMainScreen.SetStaticConfig(null);
    }

    private static void DashboardSummary()
    {
        using (var manager = new MAVLinkConnectionManager())
        using (var view = new NOMADDashboardView(new NOMADConfig(), manager))
        {
            typeof(NOMADDashboardView).GetMethod("UpdateLinksCard", PrivateInstance).Invoke(view, null);
            var label = (Label)typeof(NOMADDashboardView).GetField("_lblLinks", PrivateInstance).GetValue(view);
            Check(label.Text.Contains("stopped"), "dashboard reports an unconfigured router as stopped");
        }
    }

    private static void LinksViewAvailability()
    {
        CheckLinksView(null, new NOMADConfig { RouterClientEnabled = false }, true, "disabled client");
        CheckLinksView(null, new NOMADConfig { RouterClientEnabled = true }, false, "missing enabled client");
        using (var manager = new MAVLinkConnectionManager())
        {
            CheckLinksView(manager, null, false, "failed panel initialization");
            CheckLinksView(manager, new NOMADConfig { RouterClientEnabled = false }, true, "disabled retained client");
        }
    }

    private static void CheckLinksView(MAVLinkConnectionManager manager, NOMADConfig config, bool disabled, string name)
    {
        using (var view = new NOMADLinksView(manager, config))
        {
            string text = ReadControlText(view).ToLowerInvariant();
            Check(text.Contains(disabled ? "disabled" : "unavailable"), name + " reports its actual state");
            Check(!text.Contains(disabled ? "unavailable" : "disabled"), name + " avoids a false alternative state");
            Check(view.Controls.Count == 1 && !(view.Controls[0] is LinkHealthPanel), name + " shows guidance");
        }
    }

    private static string ReadControlText(Control parent)
    {
        var text = new List<string> { parent.Text };
        foreach (Control child in parent.Controls) { text.Add(ReadControlText(child)); }
        return string.Join("\n", text);
    }

    private static void AircraftProfileRetiresCachedViews()
    {
        var previousService = NotificationService.Shared;
        using var service = new NotificationService();
        NotificationService.Shared = service;
        var alpha = new NOMADConfig { AircraftName = "Alpha UI", RouterClientEnabled = false };
        var beta = new NOMADConfig
        {
            AircraftName = "Beta UI", RouterClientEnabled = false,
            VideoInputs = new System.Collections.Generic.List<VideoInputProfile>
                { new VideoInputProfile { Name = "Beta video", Url = "rtsp://profile-beta.invalid/stream" } },
            SelectedVideoInputName = "Beta video",
        };
        NOMADMainScreen.SetStaticConfig(alpha, null);
        try
        {
            using var screen = new NOMADMainScreen(alpha);
            var type = typeof(NOMADMainScreen);
            var show = type.GetMethod("ShowView", PrivateInstance);
            show.Invoke(screen, new object[] { "Video" });
            var oldVideo = (Control)type.GetField("_videoView", PrivateInstance).GetValue(screen);
            show.Invoke(screen, new object[] { "Links" });
            var oldLinks = (Control)type.GetField("_linksView", PrivateInstance).GetValue(screen);
            NOMADMainScreen.SetStaticConfig(beta, null);
            type.GetMethod("RefreshConfiguration", PrivateInstance).Invoke(screen, null);
            Check(oldVideo.IsDisposed && oldLinks.IsDisposed,
                "changing aircraft disposes cached video and router controls");
            var label = screen.Controls.Find("ActiveAircraft", true);
            var dashboard = type.GetField("_dashboardView", PrivateInstance).GetValue(screen);
            Check(label.Length == 1 && label[0].Text == "Aircraft: Beta UI" &&
                ReferenceEquals(typeof(NOMADDashboardView).GetField("_config", PrivateInstance)
                    .GetValue(dashboard), beta),
                "the active aircraft label and recreated dashboard use the newly saved configuration");
            show.Invoke(screen, new object[] { "Links" });
            var links = type.GetField("_linksView", PrivateInstance).GetValue(screen);
            Check(ReferenceEquals(typeof(NOMADLinksView).GetField("_config", PrivateInstance).GetValue(links), beta),
                "reopening a cached section uses the newly selected aircraft endpoints");
        }
        finally
        {
            NOMADMainScreen.SetStaticConfig(null);
            NotificationService.Shared = previousService;
        }
    }

    private static void SidebarThemeAndSettings()
    {
        using var session = new System.Threading.CancellationTokenSource();
        var type = typeof(NOMADMainScreen);
        var setSession = type.GetMethod("SetVideoShutdown", BindingFlags.Static | BindingFlags.NonPublic);
        int opened = 0;
        setSession.Invoke(null, new object[] { session.Token, (Action)(() => opened++) });
        var config = new NOMADConfig { AircraftName = "Synthetic UI" };
        NOMADMainScreen.SetStaticConfig(config, null);
        try
        {
            using var screen = new NOMADMainScreen(config);
            type.GetMethod("ShowView", PrivateInstance).Invoke(screen, new object[] { "Links" });
            var handle = screen.Handle;
            type.GetMethod("QueueThemeReapply", PrivateInstance).Invoke(screen, null);
            Application.DoEvents();
            var selected = (Button)type.GetField("_btnLinks", PrivateInstance).GetValue(screen);
            var dashboard = (Button)type.GetField("_btnDashboard", PrivateInstance).GetValue(screen);
            Check(selected.BackColor == NOMADTheme.ACCENT && selected.ForeColor == NOMADTheme.TEXT_PRIMARY &&
                dashboard.BackColor != NOMADTheme.ACCENT,
                "deferred host theme restoration preserves the selected sidebar button");
            var settings = (Button)screen.Controls.Find("OpenSettings", true)[0];
            settings.PerformClick();
            Check(opened == 1, "sidebar Settings uses the current plugin callback");
            session.Cancel();
            settings.PerformClick();
            Check(opened == 1, "a retired plugin session cannot reopen Settings");
            var plugin = new NOMADPlugin();
            typeof(NOMADPlugin).GetMethod("ShutdownVideo", PrivateInstance).Invoke(plugin, null);
            Check(type.GetField("_showSettings", BindingFlags.Static | BindingFlags.NonPublic).GetValue(null) == null,
                "plugin shutdown releases the Settings callback");
            settings.PerformClick();
            Check(opened == 1, "a cached sidebar cannot reopen Settings after plugin shutdown");
        }
        finally
        {
            setSession.Invoke(null, new object[] { System.Threading.CancellationToken.None, null });
            NOMADMainScreen.SetStaticConfig(null);
        }
    }

    private static void FenceInputSurvivesNavigation()
    {
        var previousService = NotificationService.Shared;
        using var service = new NotificationService();
        NotificationService.Shared = service;
        var screenType = typeof(NOMADMainScreen);
        var inputType = screenType.Assembly.GetType("NOMAD.MissionPlanner.FenceSetupInput", true);
        var input = Activator.CreateInstance(inputType, true);
        var setInput = screenType.GetMethod("SetFenceSetupInput", BindingFlags.Static | BindingFlags.NonPublic);
        setInput.Invoke(null, new[] { input });
        var config = new NOMADConfig { AircraftName = "Synthetic fence navigation" };
        NOMADMainScreen.SetStaticConfig(config);
        var show = screenType.GetMethod("ShowView", PrivateInstance);
        const string coordinates = "1,1\r\n1,2\r\n2,1";
        try
        {
            using (var first = new NOMADMainScreen(config))
            {
                show.Invoke(first, new object[] { "Boundaries" });
                var view = (Control)screenType.GetField("_boundaryView", PrivateInstance).GetValue(first);
                view.Controls.Find("FenceCoordinates", true)[0].Text = coordinates;
                ((ComboBox)view.GetType().GetField("_order", PrivateInstance).GetValue(view)).SelectedIndex = 1;
            }
            using (var second = new NOMADMainScreen(config))
            {
                show.Invoke(second, new object[] { "Boundaries" });
                var view = (Control)screenType.GetField("_boundaryView", PrivateInstance).GetValue(second);
                Check(view.Controls.Find("FenceCoordinates", true)[0].Text == coordinates &&
                    ((ComboBox)view.GetType().GetField("_order", PrivateInstance).GetValue(view)).SelectedIndex == 1,
                    "disposing and recreating the NOMAD screen preserves session-owned coordinates/order");
            }
        }
        finally
        {
            setInput.Invoke(null, new object[] { null });
            NOMADMainScreen.SetStaticConfig(null);
            NotificationService.Shared = previousService;
        }
    }
}
