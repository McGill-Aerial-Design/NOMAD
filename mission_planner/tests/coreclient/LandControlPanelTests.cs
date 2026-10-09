// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class NomadCoreClientTests
{
    private static void LandPanel_Tests()
    {
        using var host = new Form { ShowInTaskbar = false, Opacity = 0 };
        Exception failure = null;
        host.Shown += (sender, args) => host.BeginInvoke(new Action(() =>
        {
            try
            {
                LandPanel_ConfirmationAndConfigurationDoNotSend();
                LandPanel_ReportsObservedEngagementAndFailures();
                LandPanel_RemainsResponsiveWithoutOverlappingRequests();
                LandPanel_DisposalDoesNotReplayPendingLand();
                LandPanel_RefusesUnavailableCapability();
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

    private static Form ShowLandPanel(LandControlPanel panel)
    {
        var form = new Form { ShowInTaskbar = false };
        form.Controls.Add(panel);
        form.Show();
        Application.DoEvents();
        return form;
    }

    private static Button LandButton(LandControlPanel panel) =>
        (Button)panel.Controls.Find("RequestLand", true)[0];

    private static void LandPanel_ConfirmationAndConfigurationDoNotSend()
    {
        OutputController.Initialize(null);
        using (var panel = new LandControlPanel(() => false))
        using (var form = ShowLandPanel(panel))
        {
            var before = panel.OperatorStatus;
            LandButton(panel).PerformClick();
            WaitForPanel(panel.ActiveRequest);
            Expect(panel.OperatorStatus == before && LandButton(panel).Enabled,
                "cancelling LAND confirmation preserves the idle panel and sends no mutation");
        }
        using (var panel = new LandControlPanel(() => true))
        using (var form = ShowLandPanel(panel))
        {
            LandButton(panel).PerformClick();
            WaitForPanel(panel.ActiveRequest);
            Expect(panel.OperatorStatus.Contains("No mutation request was sent"),
                "unconfigured LAND panel explicitly reports no mutation send");
        }
        Expect(LandControlPanel.ConfirmationMessage.Contains("touchdown is not verified") &&
            LandControlPanel.ConfirmationMessage.Contains("LAND is not termination") &&
            LandControlPanel.ConfirmationMessage.Contains("explicitly admitted"),
            "LAND confirmation separates engagement, touchdown, termination and authority");
    }

    private static void LandPanel_ReportsObservedEngagementAndFailures()
    {
        var outcomes = new[] { "success", "rejected", "failed", "interrupted", "unknown" };
        foreach (var outcome in outcomes)
        {
            using var runtime = new MockRuntime(1, outcome: outcome, acknowledged: outcome != "rejected",
                capabilities: LandCapabilities);
            OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
            using var panel = new LandControlPanel(() => true);
            using var form = ShowLandPanel(panel);
            LandButton(panel).PerformClick();
            WaitForPanel(panel.ActiveRequest);
            runtime.Wait();
            Expect(runtime.CommandCount == 1 && runtime.LastCommandType == "land",
                "real LAND button sends exactly one typed LAND request for " + outcome);
            Expect(LandButton(panel).Enabled, "LAND button is restored after " + outcome);
            if (outcome == "success")
            {
                Expect(panel.OperatorStatus == "LAND mode observed; touchdown not verified.",
                    "LAND success reports observed engagement without claiming touchdown");
                continue;
            }
            Expect(!panel.OperatorStatus.Contains("LAND mode observed"),
                "non-success LAND result cannot display engagement success: " + outcome);
            if (outcome == "interrupted" || outcome == "unknown")
            {
                Expect(panel.OperatorStatus.Contains("Do not retry blindly"),
                    "uncertain LAND panel outcome retains no-retry guidance: " + outcome);
            }
        }
    }

    private static void LandPanel_RemainsResponsiveWithoutOverlappingRequests()
    {
        using var runtime = new AsyncRuntime(1, capabilities: LandCapabilities, semanticResponse: LandResponse);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        var confirmations = 0;
        using var panel = new LandControlPanel(() => { confirmations++; return true; });
        using var form = ShowLandPanel(panel);
        var button = LandButton(panel);
        button.PerformClick();
        runtime.WaitForCommands(1);
        Expect(!panel.ActiveRequest.IsCompleted && !button.Enabled,
            "LAND response wait returns control while its request button is disabled");
        button.PerformClick();
        WaitForPanel(panel.RequestLandAsync());
        Expect(confirmations == 1 && runtime.Commands.Count == 1,
            "overlapping LAND input cannot confirm, admit authority or send another request");
        runtime.ReleaseResponses();
        WaitForPanel(panel.ActiveRequest);
        runtime.Wait();
        Expect(panel.OperatorStatus == "LAND mode observed; touchdown not verified." && button.Enabled,
            "the original asynchronous LAND request completes on the UI context");
    }

    private static void LandPanel_DisposalDoesNotReplayPendingLand()
    {
        using var runtime = new AsyncRuntime(1, capabilities: LandCapabilities, semanticResponse: LandResponse);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        using var panel = new LandControlPanel(() => true);
        using var form = ShowLandPanel(panel);
        LandButton(panel).PerformClick();
        runtime.WaitForCommands(1);
        var pending = panel.ActiveRequest;
        panel.Dispose();
        WaitForPanel(pending);
        runtime.ReleaseResponses();
        runtime.Wait();
        Expect(runtime.Commands.Count == 1 && runtime.Commands[0]["type"].ToString() == "land",
            "disposing a pending LAND panel cancels local waiting without replay or vehicle undo");
    }

    private static void LandPanel_RefusesUnavailableCapability()
    {
        using var runtime = new MockRuntime(1);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        using var panel = new LandControlPanel(() => true);
        using var form = ShowLandPanel(panel);
        LandButton(panel).PerformClick();
        WaitForPanel(panel.ActiveRequest);
        runtime.Wait();
        Expect(runtime.CommandCount == 0 && panel.OperatorStatus.Contains("does not advertise LAND"),
            "real LAND panel refuses an old runtime without sending mutation or auto-admission");
    }
}
