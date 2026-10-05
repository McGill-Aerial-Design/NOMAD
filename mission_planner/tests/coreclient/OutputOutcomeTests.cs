// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using NOMAD.MissionPlanner;
using NOMAD.MissionPlanner.Connectivity;

internal static partial class NomadCoreClientTests
{
    private static void Output_ReportsTruthfulOutcomeWording()
    {
        foreach (var outcome in new[]
        {
            "rejected", "failed", "interrupted", "unknown"
        }
        )
        {
            using var runtime = new MockRuntime(1, outcome: outcome, acknowledged: outcome != "rejected");
            var result = new NomadCoreClient("test-key", runtime.Port).ServoAsync(8, 1500).GetAwaiter().GetResult();
            runtime.Wait();
            Expect(!result.Succeeded, "raw primitive preserves exact request outcome");
            if (outcome == "unknown" || outcome == "interrupted")
            { Expect(OutputController.DescribeFailure("Output", result).Contains("Do not retry blindly"), "uncertainty has no retry advice"); }
        }
    }

    private static Dictionary<string, object> DisplayState(string id = "release", ulong revision = 1,
        int remaining = 2, bool recovery = false, bool success = false) => new Dictionary<string, object>
    {
        ["id"] = id, ["state_revision"] = revision, ["confirmation_remaining"] = remaining,
        ["recovery_required"] = recovery, ["pending"] = false, ["commanded_position"] = null,
        ["software_command_success"] = success, ["pulse_on_succeeded"] = false,
        ["activation_commanded"] = false, ["activation_outcome"] = "none", ["safe_outcome"] = "none"
    };

    private static Dictionary<string, object> DiscoveredActuator() => new Dictionary<string, object>
    {
        ["id"] = "release", ["name"] = "Egg Gripper", ["config"] = new Dictionary<string, object> { ["id"] = "release" },
        ["state"] = DisplayState(), ["actions"] = new object[]
        {
            new Dictionary<string, object> { ["operation"] = "activate", ["label"] = "Open", ["control"] = "button" },
            new Dictionary<string, object> { ["operation"] = "safe", ["label"] = "Close", ["control"] = "button" },
            new Dictionary<string, object> { ["operation"] = "positive", ["label"] = "In", ["control"] = "button", ["release_operation"] = "release_input" },
            new Dictionary<string, object> { ["operation"] = "negative", ["label"] = "Out", ["control"] = "button", ["release_operation"] = "release_input" },
            new Dictionary<string, object> { ["operation"] = "position", ["label"] = "Set angle", ["control"] = "position" }
        }
    };

    private static Dictionary<string, object> SemanticResponse(Dictionary<string, object> request,
        bool attempted = false, bool commandSuccess = false, string outcome = "success")
    {
        string type = Convert.ToString(request["type"]);
        var result = new Dictionary<string, object>
        {
            ["protocol"] = "nomad-core", ["version"] = 1, ["id"] = request["id"], ["ok"] = true,
            ["runtime_incarnation"] = "mock-runtime-incarnation", ["outcome"] = outcome
        };
        if (type == "get_actuators")
        {
            result["type"] = "actuators_response";
            result["configuration_recovery_required"] = false;
            result["actuators"] = new object[] { DiscoveredActuator() };
        }
        else
        {
            result["type"] = "actuator_response";
            result["request_result"] = new Dictionary<string, object> { ["success"] = outcome == "success", ["message"] = "Backend confirmation accepted." };
            result["command_result"] = new Dictionary<string, object> { ["success"] = commandSuccess, ["acknowledged"] = commandSuccess };
            result["execution_attempted"] = attempted;
            result["actuator_state"] = DisplayState(request.ContainsKey("actuator_id") ? Convert.ToString(request["actuator_id"]) : "release", success: commandSuccess);
            if (type == "configure_actuators") { result["actuators"] = new object[] { DiscoveredActuator() }; }
        }
        return result;
    }

    private static void ActuatorClient_ReportsBackendEvidence()
    {
        using (var runtime = new MockRuntime(3, semanticResponse: request => SemanticResponse(request)))
        {
            var client = new NomadCoreClient("test-key", runtime.Port);
            var discovery = client.GetActuatorsAsync().GetAwaiter().GetResult();
            var confirm = client.ActuatorActionAsync("release", "activate").GetAwaiter().GetResult();
            var configure = client.ConfigureActuatorsAsync("[{\"id\":\"arbitrary-data\"}]").GetAwaiter().GetResult();
            runtime.Wait();
            Expect(discovery.Succeeded && discovery.Actuators[0].Name == "Egg Gripper", "backend names and action labels arrive as data");
            Expect(confirm.Succeeded && confirm.ExecutionAttempted == false && confirm.SoftwareCommandSuccess == false,
                "accepted confirmation is distinguished from attempted execution and software output success");
            Expect(configure.Succeeded && configure.SoftwareCommandSuccess == false, "saved configuration does not claim vehicle command success");
            Expect(runtime.Commands[1]["actuator_id"].ToString() == "release" && !runtime.Commands[1].ContainsKey("channel"),
                "semantic UI request carries ID and operation without physical output mapping");
        }
        using (var runtime = new MockRuntime(1, semanticResponse: request => SemanticResponse(request, true, true, "interrupted")))
        {
            var result = new NomadCoreClient("test-key", runtime.Port).ActuatorActionAsync("release", "safe").GetAwaiter().GetResult();
            runtime.Wait();
            Expect(result.Outcome == NomadCoreRequestOutcome.Interrupted && result.SoftwareCommandSuccess == true,
                "request disposition and observed command success remain independent");
        }
        Projection_EmptyRuntimeRetiresPriorDisplay();
        Configuration_ErrorPreservesReturnedFacts();
        Panel_RendersBackendLabelsAndState();
        Panel_HeldKeyboardDoesNotMultiplyInput();
    }

    private static void Projection_EmptyRuntimeRetiresPriorDisplay()
    {
        OutputController.Initialize(new NOMADConfig());
        OutputController.PublishState("old-runtime", new NomadActuatorState { Id = "release", Revision = 20 }, 1);
        Expect(OutputController.ObserveIncarnation("empty-new-runtime", 2), "empty discovery still advances runtime display incarnation");
        Expect(OutputController.GetDisplayState("release") == null &&
            !OutputController.PublishState("old-runtime", new NomadActuatorState { Id = "release", Revision = 21 }, 3),
            "higher-sequence late old discovery cannot revive controls retired by an empty new runtime");
    }

    private static void Configuration_ErrorPreservesReturnedFacts()
    {
        using var runtime = new MockRuntime(1, semanticResponse: request =>
        {
            var response = SemanticResponse(request);
            response["ok"] = false;
            response["outcome"] = "unknown";
            response["configuration_changed"] = true;
            response["configuration_recovery_required"] = true;
            response["error"] = new Dictionary<string, object> { ["code"] = "configuration_recovery_required", ["message"] = "Persistence result needs review." };
            return response;
        });
        var result = new NomadCoreClient("test-key", runtime.Port).ConfigureActuatorsAsync("[]").GetAwaiter().GetResult();
        runtime.Wait();
        var description = OutputController.DescribeConfigurationResult(result);
        Expect(result.ConfigurationChanged && result.ConfigurationRecoveryRequired &&
            description.Contains("Restart and review") && !description.Contains("vehicle state"),
            "configuration error preserves backend changed/recovery facts without claiming vehicle uncertainty");
        var saved = new NomadCoreRequestResult(NomadCoreRequestOutcome.Succeeded, "",
            "Backend configuration saved; explicit safe commands required.") { ConfigurationChanged = true };
        Expect(OutputController.DescribeConfigurationResult(saved) == saved.Message,
            "known successful live configuration reports backend success without requiring restart");
    }

    private static void Panel_RendersBackendLabelsAndState()
    {
        using var runtime = new MockRuntime(2, semanticResponse: request => SemanticResponse(request));
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        using var panel = new ActuatorControlPanel(new NOMADConfig());
        WaitForPanel(panel.RefreshActuatorsAsync());
        Expect(panel.Controls.Find("release:activate", true)[0].Text == "Open", "dynamic configured action label rendered");
        Expect(panel.Controls.Find("release:safe", true)[0].Text == "Close", "dynamic configured safe label rendered");
        WaitForPanel(panel.SendActionAsync("release", "activate"));
        runtime.Wait();
        Expect(panel.Controls.Find("release:State", true)[0].Text.Contains("confirmations remaining=2"), "UI renders backend confirmation state");
        Expect(runtime.CommandCount == 2 && runtime.Commands[1]["type"].ToString() == "actuator_action", "one click sends one semantic request");
        var newer = new NomadActuatorState { Id = "release", Revision = 9, RecoveryRequired = true };
        OutputController.PublishState("mock-runtime-incarnation", newer, 900);
        OutputController.PublishState("mock-runtime-incarnation", new NomadActuatorState { Id = "release", Revision = 8 });
        Expect(panel.Controls.Find("release:State", true)[0].Text.Contains("recovery required=True"), "stale display revision cannot overwrite newer backend evidence");
        OutputController.PublishState("new-process", new NomadActuatorState { Id = "release", Revision = 1, RecoveryRequired = true }, 902);
        OutputController.PublishState("mock-runtime-incarnation", new NomadActuatorState { Id = "release", Revision = 100 }, 901);
        OutputController.PublishState("mock-runtime-incarnation", new NomadActuatorState { Id = "release", Revision = 101 }, 903);
        Expect(OutputController.GetDisplayState("release").Revision == 1 && OutputController.GetDisplayState("release").RecoveryRequired,
            "older request sequence cannot replace newer process display snapshot");
    }

    [DllImport("user32.dll")] private static extern IntPtr SendMessage(IntPtr window, uint message, IntPtr wParam, IntPtr lParam);
    private static void Panel_HeldKeyboardDoesNotMultiplyInput()
    {
        using var form = new Form();
        var button = new ActuatorButton { Text = "Action", Dock = DockStyle.Fill };
        form.Controls.Add(button);
        int clicks = 0;
        button.Click += (s, e) => clicks++;
        form.Show();
        button.Focus();
        Application.DoEvents();
        foreach (int key in new[]
        {
            32, 13
        }
        )
        {
            int before = clicks;
            DispatchButtonKey(button, 0x100, key, 1);
            for (int repeat = 0; repeat < 5; repeat++)
            {
                DispatchButtonKey(button, 0x100, key, 1L | (1L << 30));
            }
            DispatchButtonKey(button, 0x101, key, 1L | (1L << 30) | (1L << 31));
            Expect(clicks == before + 1, "held keyboard key=" + key + " produces one discrete actuator intent; observed=" + (clicks - before));
        }
        form.Close();
    }

    private static void DispatchButtonKey(Button button, uint type, int key, long flags)
    {
        var message = Message.Create(button.Handle, (int)type, new IntPtr(key), new IntPtr(flags));
        if (!button.PreProcessMessage(ref message))
        {
            SendMessage(button.Handle, type, new IntPtr(key), new IntPtr(flags));
        }
    }

    private static void WaitForPanel(Task request)
    {
        var deadline = DateTime.UtcNow.AddSeconds(10);
        while (!request.IsCompleted && DateTime.UtcNow < deadline)
        {
            Application.DoEvents(); Thread.Sleep(1);
        }
        Expect(request.IsCompleted, "UI request completes while its context is pumped");
        request.GetAwaiter().GetResult();
    }
}
