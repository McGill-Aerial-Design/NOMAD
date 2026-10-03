// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System.Windows.Forms;
using NOMAD.MissionPlanner;
using NOMAD.MissionPlanner.Connectivity;

internal static partial class NomadCoreClientTests
{
    private static void Output_ReportsTruthfulOutcomeWording()
    {
        var outcomes = new[] { "rejected", "failed", "interrupted", "unknown" };
        var wording = new[] { "not sent", "definite failure", "Authority or session changed",
            "may have been transmitted" };
        for (var index = 0; index < outcomes.Length; index++)
        {
            using var runtime = new MockRuntime(1, outcome: outcomes[index],
                acknowledged: outcomes[index] != "rejected");
            OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
            var result = OutputController.SendServoPwmAsync(8, 1500).GetAwaiter().GetResult();
            Expect(!result.Succeeded, "non-success output preserves exact request result");
            runtime.Wait();
            var message = OutputController.DescribeFailure("Release", result);
            Expect(message.Contains(wording[index]), $"operator sees truthful {outcomes[index]} disposition");
            Expect(Log.Messages[Log.Messages.Count - 1].Contains("result=" + outcomes[index]),
                "companion output audit preserves normalized classification");
            if (index >= 2)
            {
                Expect(message.Contains("Do not retry blindly"), "uncertain mutation has no retry advice");
            }
        }
    }

    private static void PayloadPanel_UpdatesOnlySuccessfulCommandedState()
    {
        foreach (var outcome in new[] { "rejected", "failed", "interrupted", "unknown", "success" })
        {
            using var runtime = new MockRuntime(2, outcome: outcome, acknowledged: outcome != "rejected");
            var config = new NOMADConfig { CoreRuntimePort = runtime.Port };
            OutputController.Initialize(config);
            using var panel = new PayloadControlPanel(config);
            PayloadControlPanel.RaisePayloadReleaseCommandedState(0, false);
            WaitForPanel(panel.TestDrop());
            Expect(PayloadControlPanel.IsPayloadReleaseCommanded(0) == (outcome == "success"),
                $"{outcome} release changes commanded state only on software success");
            if (outcome == "success")
            {
                Expect(panel.TestStatus.Contains("physical release unverified"),
                    "release status makes evidence limit explicit");
            }
            PayloadControlPanel.RaisePayloadReleaseCommandedState(0, true);
            WaitForPanel(panel.TestRetract());
            Expect(PayloadControlPanel.IsPayloadReleaseCommanded(0) == (outcome != "success"),
                $"{outcome} retract cannot clear previous commanded release before success");
            if (outcome == "success")
            {
                Expect(panel.TestStatus.Contains("physical retraction unverified"),
                    "retract does not claim physical completion");
            }
            runtime.Wait();
            Expect(runtime.CommandCount == 2, "panel sends each explicit operator command exactly once");
            PayloadControlPanel.RaisePayloadReleaseCommandedState(0, false);
        }
    }

    private static void PayloadActions_PreserveStateOnUnknownRetract()
    {
        using var runtime = new MockRuntime(1, dropCommandResponse: true);
        var config = new NOMADConfig { CoreRuntimePort = runtime.Port };
        OutputController.Initialize(config);
        PayloadControlPanel.RaisePayloadReleaseCommandedState(0, true);
        PayloadActions.Retract(config, 1).GetAwaiter().GetResult();
        runtime.Wait();
        Expect(PayloadControlPanel.IsPayloadReleaseCommanded(0),
            "headless unknown retract preserves previous commanded state");

        Expect(runtime.CommandCount == 1, "headless unknown retract is not replayed");
        PayloadControlPanel.RaisePayloadReleaseCommandedState(0, false);
    }

    private static void RelayPanel_PreservesCommandedStateOnFailure()
    {
        using var runtime = new MockRuntime(1, outcome: "failed");
        var config = new NOMADConfig { CoreRuntimePort = runtime.Port };
        OutputController.Initialize(config);
        using var panel = new PayloadControlPanel(config);
        using var button = new Button { Text = "No confirmed command" };
        button.CreateControl();
        WaitForPanel(panel.TestToggleRelay(new PayloadControl { Kind = PayloadKind.Relay, Channel = 3 }, button));
        runtime.Wait();
        Expect(button.Text == "No confirmed command", "failed relay cannot assert a commanded ON state");
        Expect(panel.TestStatus.Contains("definite failure"), "relay panel preserves definite failed category");
    }
    private static void ReelPanel_DoesNotClaimPhysicalMovementOrStop()
    {
        foreach (var outcome in new[] { "failed", "interrupted", "unknown", "success" })
        {
            using var runtime = new MockRuntime(4, outcome: outcome);
            var config = new NOMADConfig { CoreRuntimePort = runtime.Port };
            OutputController.Initialize(config);
            using var panel = new PayloadControlPanel(config);
            WaitForPanel(panel.TestStartReel());
            CheckReelEvidence(panel.TestStatus, outcome, "start");
            WaitForPanel(panel.TestStopReel());
            CheckReelEvidence(panel.TestStatus, outcome, "stop");
            WaitForPanel(panel.TestStartFullReel());
            CheckReelEvidence(panel.TestStatus, outcome, "timed start");
            WaitForPanel(panel.TestStopFullReel());
            CheckReelEvidence(panel.TestStatus, outcome, "timed stop");
            runtime.Wait();
            Expect(runtime.CommandCount == 4, "reel requests are sent exactly once without uncertain retry");
        }
    }

    private static void CheckReelEvidence(string message, string outcome, string operation)
    {
        Expect(!message.Contains(" stopped") && !message.Contains(" complete"),
            $"{operation} cannot claim physical stop or completion");
        var evidence = outcome == "success" ? "unverified" : outcome == "failed" ? "definite failure" : "unknown";
        Expect(message.Contains(evidence), $"reel {operation} reports {outcome} software evidence");
    }
    private static void WaitForPanel(System.Threading.Tasks.Task request)
    {
        var deadline = System.DateTime.UtcNow.AddSeconds(10);
        while (!request.IsCompleted && System.DateTime.UtcNow < deadline)
        {
            Application.DoEvents();
            System.Threading.Thread.Sleep(1);
        }
        Expect(request.IsCompleted, "panel request completes while its UI context is pumped");
        request.GetAwaiter().GetResult();
    }
}
