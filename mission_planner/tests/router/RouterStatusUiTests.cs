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
using NOMAD.MissionPlanner.Connectivity;

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
        Run("dashboard runtime status entry", DashboardRuntimeStatusEntry);
        Run("Links view availability", LinksViewAvailability);
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

    private static StandaloneRouterClient CreateStatusClient(Func<double> seconds)
    {
        var config = new MAVLinkConnectionManager.ConnectionConfig();
        Type clockType = typeof(StandaloneRouterClient).Assembly.GetType("NOMAD.MissionPlanner.RouterClock", true);
        object clock = Activator.CreateInstance(clockType, PrivateInstance, null, new object[] { seconds, null }, null);
        return (StandaloneRouterClient)Activator.CreateInstance(typeof(StandaloneRouterClient), PrivateInstance,
            null, new object[] { config, clock }, null);
    }

    private static void ApplyStatus(StandaloneRouterClient client)
    {
        var status = new RouterStatusSnapshot { Running = true, ActiveLinkId = "cell" };
        status.Links.Add(new RouterLinkStatusSnapshot
            { StableId = "cell", Enabled = true, IsOpen = true, IsConnected = true, Health = "Good" });
        status.Links.Add(new RouterLinkStatusSnapshot { StableId = "wifi", Enabled = true });
        status.Links.Add(new RouterLinkStatusSnapshot { StableId = "spare", Enabled = false });
        typeof(StandaloneRouterClient).GetMethod("ApplyStatus", PrivateInstance).Invoke(client, new object[] { status });
    }

    private static void DashboardSummary()
    {
        double now = 0;
        var previousService = NotificationService.Shared;
        using (var service = new NotificationService())
        using (var manager = new MAVLinkConnectionManager())
        {
            NotificationService.Shared = service;
            try
            {
                var client = CreateStatusClient(() => now);
                typeof(MAVLinkConnectionManager).GetField("_standalone", PrivateInstance).SetValue(manager, client);
                using (var view = new NOMADDashboardView(new NOMADConfig { VideoUrl = "" }, manager))
                {
                    var update = typeof(NOMADDashboardView).GetMethod("UpdateLinksCard", PrivateInstance);
                    var label = (Label)typeof(NOMADDashboardView).GetField("_lblLinks", PrivateInstance).GetValue(view);
                    ApplyStatus(client);
                    update.Invoke(view, null);
                    Check(manager.IsRouterAvailable, "zero-time status is fresh");
                    Check(label.Text.Contains("1/3"), "dashboard counts arbitrary configured links");
                    now = 4;
                    Check(manager.IsStatusStale && manager.ActiveLink == "cell", "aged status retains the active ID");
                    update.Invoke(view, null);
                    Check(label.Text.Contains("stale"), "dashboard reports aged management data");
                    Check(label.ForeColor != NOMADTheme.SUCCESS, "stale dashboard summary must not be green");
                    client.Stop();
                    update.Invoke(view, null);
                    Check(label.Text.Contains("unavailable"), "dashboard reports disconnected management");
                    Check(label.ForeColor != NOMADTheme.SUCCESS, "unavailable dashboard summary must not be green");
                }
            }
            finally { NotificationService.Shared = previousService; }
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

    private static void DashboardRuntimeStatusEntry()
    {
        var previousService = NotificationService.Shared;
        var controller = typeof(NOMADDashboardView).Assembly.GetType("NOMAD.MissionPlanner.OutputController", true);
        var initialize = controller.GetMethod("Initialize", BindingFlags.Static | BindingFlags.NonPublic);
        using var service = new NotificationService();
        NotificationService.Shared = service;
        try
        {
            var cached = new NOMADConfig { CoreClientCredential = "fixture-key", VideoUrl = "" };
            using var view = new NOMADDashboardView(cached, null);
            using var host = new Form { ShowInTaskbar = false, Opacity = 0, Width = 900, Height = 650 };
            view.Dock = DockStyle.Fill;
            host.Controls.Add(view);
            host.Show();
            Application.DoEvents();
            var update = typeof(NOMADDashboardView).GetMethod("UpdateCoreCard", PrivateInstance);
            var label = (Label)typeof(NOMADDashboardView).GetField("_lblCore", PrivateInstance).GetValue(view);
            initialize.Invoke(null, new object[] { cached });
            update.Invoke(view, null);
            Check(label.Text.Contains("configured") && label.ForeColor != NOMADTheme.SUCCESS,
                "dashboard configuration remains distinct from runtime readiness");
            initialize.Invoke(null, new object[] { null });
            update.Invoke(view, null);
            Check(label.Text == "Not configured", "cached dashboard rechecks the current core client");
            var invalid = new NomadCoreClient("fixture-key", 0);
            Check(!invalid.IsConfigured && invalid.RuntimePort == 0, "invalid modal fixture port has no default fallback");
            if (invalid.IsConfigured || invalid.RuntimePort != 0) { return; }
            initialize.Invoke(null, new object[] { new NOMADConfig { CoreRuntimePort = 0 } });
            var buttons = view.Controls.Find("ReadRuntimeStatus", true);
            Check(buttons.Length == 1 && buttons[0] is Button, "dashboard provides its explicit runtime read button");
            if (buttons.Length != 1) { return; }
            Check(!buttons[0].Bounds.IntersectsWith(label.Bounds), "runtime read button preserves the core value's space");
            InspectRuntimeDialog((Button)buttons[0]);
        }
        finally
        {
            initialize.Invoke(null, new object[] { null });
            NotificationService.Shared = previousService;
        }
    }

    private static void InspectRuntimeDialog(Button button)
    {
        bool inspected = false;
        var watch = Stopwatch.StartNew();
        using var timer = new System.Windows.Forms.Timer { Interval = 25 };
        timer.Tick += (sender, args) =>
        {
            foreach (Form dialog in Application.OpenForms)
            {
                if (!dialog.Modal) { continue; }
                if (dialog.Text == "NOMAD runtime snapshot")
                {
                    var summaries = dialog.Controls.Find("RuntimeStatusSummary", true);
                    Check(summaries.Length == 1 && summaries[0].Text.Contains("port is invalid"),
                        "real dashboard click reads the current client and reports its local configuration failure");
                    inspected = true;
                    dialog.Close();
                    return;
                }
                if (watch.ElapsedMilliseconds > 5000)
                {
                    dialog.Close();
                    return;
                }
            }
        };
        timer.Start();
        button.PerformClick();
        timer.Stop();
        Check(inspected, "dashboard runtime dialog completes under its UI test deadline");
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
}
