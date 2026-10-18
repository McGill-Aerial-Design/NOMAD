// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Reflection;
using System.Threading;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class NomadCoreClientTests
{
    private static void StatusPanel_Tests()
    {
        using var host = new Form { ShowInTaskbar = false, Opacity = 0 };
        Exception failure = null;
        host.Shown += (sender, args) => host.BeginInvoke(new Action(() =>
        {
            try
            {
                StatusPanel_ReportsSeparateFactsAndDatums();
                StatusPanel_UnknownFieldsStayUnknown();
                StatusPanel_ReplacedClientCannotPublishQueuedResponse();
                StatusPanel_ViewLifetimeFencesQueuedResponse();
                StatusPanel_DisposalClosesPendingRead();
                StatusPanel_FailedRefreshClearsPreviousSnapshot();
                StatusPanel_QueuedResponseExpires();
            }
            catch (Exception error)
            {
                failure = error;
            }
            finally
            {
                host.Close();
            }
        }));
        Application.Run(host);
        if (failure != null)
        {
            throw failure;
        }
    }

    private static Form ShowStatusPanel(RuntimeStatusPanel panel)
    {
        var form = new Form { ShowInTaskbar = false, Opacity = 0, Width = 780, Height = 680 };
        panel.Dock = DockStyle.Fill;
        form.Controls.Add(panel);
        form.Show();
        Application.DoEvents();
        return form;
    }

    private static Button StatusButton(RuntimeStatusPanel panel) =>
        (Button)panel.Controls.Find("RefreshRuntimeStatus", true)[0];

    private static string StatusValue(RuntimeStatusPanel panel, string name) =>
        ((ListView)panel.Controls.Find("RuntimeStatusValues", true)[0]).Items[name].SubItems[1].Text;

    private static void StatusPanel_ReportsSeparateFactsAndDatums()
    {
        using var runtime = new StatusRuntime();
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        using var panel = new RuntimeStatusPanel();
        using var form = ShowStatusPanel(panel);
        Expect(runtime.Requests().Length == 0 && StatusValue(panel, "IPC listener ready") == "Unknown",
            "STATUS view starts unknown and does not poll before explicit refresh");
        StatusButton(panel).PerformClick();
        WaitForPanel(panel.ActiveRequest);
        Expect(StatusValue(panel, "IPC listener ready") == "Yes" && StatusValue(panel, "Audit healthy") == "No"
            && StatusValue(panel, "Actuation enabled") == "No" && StatusValue(panel,
                "Authority owner (reported)") == "None",
            "real status controls keep listener, audit, actuation and authority separate");
        Expect(StatusValue(panel, "Protocol advertised verbs").Contains("land")
            && StatusValue(panel, "Aircraft-supported operations") == "None supported",
            "real controls separate protocol vocabulary from current aircraft support");
        Expect(StatusValue(panel, "Position (degrees)").StartsWith("0, 0;")
            && StatusValue(panel, "Altitude MSL (metres)") == "123.5"
            && StatusValue(panel, "Altitude above home (metres)") == "23.5",
                "real controls preserve zero position and altitude datums");
        Expect(panel.OperatorStatus.Contains("UTC") && panel.OperatorStatus.Contains("Read-only"),
            "STATUS view reports its capture date and observation boundary");
        Expect(StatusButton(panel).Enabled && runtime.Requests().Length == 2,
            "refresh finishes without automatic polling");
    }

    private static void StatusPanel_UnknownFieldsStayUnknown()
    {
        using var runtime = new StatusRuntime(changeResponse: response =>
        {
            var status = (Dictionary<string, object>)response["status"];
            status["aircraft_class"] = "unrecognized";
            status["supported_flight_operations"] = new[] { "land" };
            var telemetry = (Dictionary<string, object>)status["telemetry"];
            telemetry["vtol_state_valid"] = false;
            telemetry["landed_state_age_ms"] = 60000;
            telemetry["position_age_ms"] = null;
        });
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        using var panel = new RuntimeStatusPanel();
        using var form = ShowStatusPanel(panel);
        StatusButton(panel).PerformClick();
        WaitForPanel(panel.ActiveRequest);
        Expect(StatusValue(panel, "Aircraft class") == "Unknown"
            && StatusValue(panel, "Aircraft-supported operations") == "Unknown",
                "unknown aircraft identity does not establish operation support");
        Expect(StatusValue(panel, "VTOL state").StartsWith("Unknown; validity No")
            && StatusValue(panel, "Landed state (autopilot)").StartsWith("Unknown;"),
            "invalid and stale telemetry values stay unknown while preserving validity and age");
        Expect(StatusValue(panel, "Position (degrees)").StartsWith("Unknown;")
            && StatusValue(panel, "Altitude MSL (metres)") == "Unknown"
            && StatusValue(panel, "Altitude above home (metres)") == "Unknown",
                "a missing position age cannot become a live coordinate or altitude");
    }

    private static void StatusPanel_ReplacedClientCannotPublishQueuedResponse()
    {
        using var oldRuntime = new StatusRuntime();
        using var newRuntime = new StatusRuntime(incarnation: "new-runtime", changeResponse: response =>
        {
            var status = (Dictionary<string, object>)response["status"];
            status["vehicle_session"] = 22;
            status["authority_generation"] = 33;
        });
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = oldRuntime.Port });
        using var panel = new RuntimeStatusPanel();
        using var form = ShowStatusPanel(panel);
        StatusButton(panel).PerformClick();
        var oldRequest = panel.ActiveRequest;
        Expect(SpinWait.SpinUntil(() => panel.PendingRead.IsCompleted, 3000),
            "old STATUS read completes before its UI callback is pumped");
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = newRuntime.Port });
        var newRequest = panel.RefreshStatusAsync();
        WaitForPanel(newRequest);
        WaitForPanel(oldRequest);
        Expect(StatusValue(panel, "Runtime incarnation") == "new-runtime"
            && StatusValue(panel, "Vehicle session (reported)") == "22"
            && StatusValue(panel, "Authority generation") == "33",
                "late old client/incarnation/session response cannot overwrite the new observation");
        OutputController.Initialize(null);
        WaitForPanel(panel.RefreshStatusAsync());
        Expect(StatusValue(panel, "Runtime incarnation") == "Unknown" && StatusButton(panel).Enabled,
            "cleared configuration discards the old snapshot and permits an explicit later refresh");
    }

    private static void StatusPanel_ViewLifetimeFencesQueuedResponse()
    {
        foreach (var action in new[] { "hide", "handle", "dispose" })
        {
            using var runtime = new StatusRuntime();
            OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
            using var panel = new RuntimeStatusPanel();
            using var form = ShowStatusPanel(panel);
            StatusButton(panel).PerformClick();
            var pending = panel.ActiveRequest;
            Expect(SpinWait.SpinUntil(() => panel.PendingRead.IsCompleted, 3000),
                "STATUS is queued before the " + action + " action");
            if (action == "dispose")
            {
                panel.Dispose();
            }
            else if (action == "hide")
            {
                panel.Hide();
                panel.Show();
            }
            else
            {
                typeof(Control).GetMethod("RecreateHandle", BindingFlags.Instance | BindingFlags.NonPublic)
                    .Invoke(panel, null);
            }
            WaitForPanel(pending);
            Expect(runtime.Requests().Length == 2 && !runtime.HasExtraConnection,
                action + " never retries a retired STATUS request");
            if (action != "dispose")
            {
                Expect(StatusValue(panel, "Runtime incarnation") == "Unknown" && StatusButton(panel).Enabled,
                    action + " prevents queued STATUS publication into the new view context");
            }
        }
    }

    private static void StatusPanel_QueuedResponseExpires()
    {
        using var runtime = new StatusRuntime();
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        using var panel = new RuntimeStatusPanel();
        using var form = ShowStatusPanel(panel);
        StatusButton(panel).PerformClick();
        Expect(SpinWait.SpinUntil(() => panel.PendingRead.IsCompleted, 3000),
            "STATUS response is received before the delayed UI publication");
        Thread.Sleep(3100);
        WaitForPanel(panel.ActiveRequest);
        Expect(panel.OperatorStatus.Contains("expired") && panel.OperatorStatus.Contains("UTC")
            && StatusValue(panel, "IPC listener ready") == "Unknown"
            && StatusValue(panel, "Authority owner (reported)") == "Unknown",
                "old queued snapshots expire before publishing readiness or authority");
    }

    private static void StatusPanel_DisposalClosesPendingRead()
    {
        using var runtime = new StatusRuntime(hold: true);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        using var panel = new RuntimeStatusPanel();
        using var form = ShowStatusPanel(panel);
        StatusButton(panel).PerformClick();
        var pending = panel.ActiveRequest;
        Expect(runtime.WaitForStatuses(1), "view disposal begins with an observed pending STATUS read");
        panel.Dispose();
        Expect(runtime.ClientClosed(), "disposing the view closes its actual in-flight STATUS socket");
        WaitForPanel(pending);
        Expect(runtime.Requests().Length == 2 && !runtime.HasExtraConnection,
            "view disposal drains its read without replaying HELLO or STATUS");
    }

    private static void StatusPanel_FailedRefreshClearsPreviousSnapshot()
    {
        var responses = 0;
        using var runtime = new StatusRuntime(count: 2, changeResponse: response =>
        {
            if (++responses == 2)
            {
                ((Dictionary<string, object>)response["status"])["runtime_incarnation"] = "wrong-runtime";
            }
        });
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        using var panel = new RuntimeStatusPanel();
        using var form = ShowStatusPanel(panel);
        StatusButton(panel).PerformClick();
        WaitForPanel(panel.ActiveRequest);
        Expect(StatusValue(panel, "Runtime incarnation") == "status-runtime",
            "first refresh establishes an observation");
        StatusButton(panel).PerformClick();
        WaitForPanel(panel.ActiveRequest);
        Expect(StatusValue(panel, "Runtime incarnation") == "Unknown"
            && StatusValue(panel, "IPC listener ready") == "Unknown" && StatusButton(panel).Enabled,
            "failed refresh cannot retain a previous successful snapshot");
    }
}
