// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using NOMAD.MissionPlanner;
using NOMAD.MissionPlanner.Connectivity;

internal static partial class NomadCoreClientTests
{
    private static Dictionary<string, object> GimbalSuccessResponse(Dictionary<string, object> command)
    {
        return new Dictionary<string, object>
        {
            ["protocol"] = "nomad-core", ["version"] = 1, ["id"] = command["id"],
            ["ok"] = true, ["type"] = "command_response", ["outcome"] = "success",
            ["command_result"] = new Dictionary<string, object>
            {
                ["success"] = true, ["acknowledged"] = true, ["message"] = "mode and target accepted"
            }
        };
    }

    private static void Gimbal_ModeAndImmediateTargetAreSequenced()
    {
        using var runtime = new AsyncRuntime(2, semanticResponse: GimbalSuccessResponse);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        GimbalController.SetMode(MountMode.MavlinkTargeting);
        var target = GimbalController.RequestPitchRollTargetAsync(15.0f, -5.0f);
        runtime.WaitForCommands(1);
        Expect(runtime.Commands.Count == 1 && runtime.Commands[0]["type"].ToString() == "configure_gimbal",
            "mode preset sends its own immediate configuration request");
        Expect(!target.IsCompleted, "target stays pending while its mode configuration is unresolved");

        runtime.ReleaseResponses();
        runtime.WaitForCommands(2);
        Expect(runtime.Commands[1]["type"].ToString() == "set_gimbal_target",
            "target is sent after the mode request completes");
        Expect(Convert.ToUInt64(runtime.Commands[1]["sequence"]) >
            Convert.ToUInt64(runtime.Commands[0]["sequence"]), "target receives a fresh later runtime sequence");
        Expect(WaitResult(target).Succeeded, "sequenced mode and target both succeed");
        runtime.Wait();
        Expect(runtime.Commands.Count == 2 && runtime.AcceptedConnections == 2,
            "one selected mode and one requested target produce exactly two ordered mutations");
    }

    private static void Gimbal_ModeAndTargetUseOneRuntimeRequest()
    {
        using var runtime = new AsyncRuntime(1, semanticResponse: GimbalSuccessResponse);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        var request = GimbalController.SetModeAndTargetAsync(MountMode.MavlinkTargeting, 15.0f, -5.0f);
        runtime.WaitForCommands(1);
        Expect(runtime.Commands.Count == 1, "paired mode-and-target action uses one bounded runtime request");
        Expect(runtime.Commands[0]["type"].ToString() == "configure_gimbal_target",
            "paired action carries its selected mount mode to the runtime");
        Expect(Convert.ToInt32(runtime.Commands[0]["mount_mode"]) == (int)MountMode.MavlinkTargeting,
            "combined request preserves the selected mode");
        Expect(Math.Abs(Convert.ToDouble(runtime.Commands[0]["pitch_deg"]) - 15.0) < 1e-9 &&
            Math.Abs(Convert.ToDouble(runtime.Commands[0]["roll_deg"]) + 5.0) < 1e-9,
            "combined request preserves the requested angles");
        runtime.ReleaseResponses();
        Expect(WaitResult(request).Succeeded, "paired mode and target result is returned once");
        runtime.Wait();
        Expect(runtime.Commands.Count == 1 && runtime.AcceptedConnections == 1,
            "mode selection and target produce no follow-up mutation or retry");
    }

    private static void Gimbal_UnknownModeOutcomeDoesNotReplay()
    {
        using var runtime = new AsyncRuntime(1) { ResponseSize = 65537 };
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        GimbalController.SetMode(MountMode.MavlinkTargeting);
        var request = GimbalController.RequestPitchRollTargetAsync(-10.0f, 4.0f);
        runtime.WaitForCommands(1);
        Expect(runtime.Commands[0]["type"].ToString() == "configure_gimbal",
            "the unresolved standalone mode request is not replaced by a combined retry");
        runtime.ReleaseResponses();
        var result1 = WaitResult(request);
        runtime.Wait();
        Expect(result1.Outcome == NomadCoreRequestOutcome.UnknownOutcome,
            "oversized response leaves the paired command outcome unknown");

        var declined = WaitResult(GimbalController.RequestPitchRollTargetAsync(-10.0f, 4.0f));
        Expect(declined.Outcome == NomadCoreRequestOutcome.NotAttempted &&
            declined.ErrorCode == "gimbal_mode_not_confirmed",
            "uncertain mode configuration blocks later automatic target requests");
        Expect(runtime.Commands.Count == 1 && runtime.AcceptedConnections == 1,
            "unknown mode outcome causes no target write or configuration replay");
    }

    private static void Gimbal_UnknownCompositeDoesNotReplay()
    {
        using var runtime = new AsyncRuntime(1) { ResponseSize = 65537 };
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        var request = GimbalController.SetModeAndTargetAsync(MountMode.MavlinkTargeting, -10.0f, 4.0f);
        runtime.WaitForCommands(1);
        Expect(runtime.Commands[0]["type"].ToString() == "configure_gimbal_target",
            "a fresh target action uses one combined request");
        runtime.ReleaseResponses();
        var result1 = WaitResult(request);
        runtime.Wait();
        Expect(result1.Outcome == NomadCoreRequestOutcome.UnknownOutcome,
            "a lost composite response leaves both command outcomes uncertain");

        var declined = WaitResult(GimbalController.RequestPitchRollTargetAsync(-10.0f, 4.0f));
        Expect(declined.Outcome == NomadCoreRequestOutcome.NotAttempted &&
            declined.ErrorCode == "gimbal_mode_not_confirmed",
            "an uncertain composite is not automatically reissued on a later target tick");
        Expect(runtime.Commands.Count == 1 && runtime.AcceptedConnections == 1,
            "uncertain composite produced no second mutation");
    }

    private static void Gimbal_StaleStandaloneModeDoesNotWriteAfterHello()
    {
        using var runtime = new AsyncRuntime(1, delayHello: true);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        var staleMode = GimbalController.SetModeAsync(MountMode.MavlinkTargeting);
        runtime.WaitForHello();
        var declinedMode = WaitResult(GimbalController.SetModeAsync(MountMode.RcTargeting));
        runtime.ReleaseHello();
        var staleResult = WaitResult(staleMode);
        runtime.Wait();

        Expect(declinedMode.Outcome == NomadCoreRequestOutcome.NotAttempted,
            "new mode selection is bounded while the old mode request holds the gimbal gate");
        Expect(staleResult.Outcome == NomadCoreRequestOutcome.NotAttempted && staleResult.ErrorCode == "stale_input",
            "old standalone mode is rejected after HELLO when selection changed");
        Expect(runtime.Commands.Count == 0, "stale standalone mode never becomes a mutation");
    }

    private static void Gimbal_StaleTargetDoesNotWriteAfterHello()
    {
        using (var setup = new AsyncRuntime(1, semanticResponse: GimbalSuccessResponse))
        {
            OutputController.Initialize(new NOMADConfig { CoreRuntimePort = setup.Port });
            var mode = GimbalController.SetModeAsync(MountMode.MavlinkTargeting);
            setup.WaitForCommands(1);
            setup.ReleaseResponses();
            Expect(WaitResult(mode).Succeeded, "setup confirms MAVLink-targeting mode");
            setup.Wait();
        }

        using var runtime = new AsyncRuntime(1, delayHello: true);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        var staleTarget = GimbalController.RequestPitchRollTargetAsync(12.0f, -3.0f);
        runtime.WaitForHello();
        var declinedMode = WaitResult(GimbalController.SetModeAsync(MountMode.RcTargeting));
        runtime.ReleaseHello();
        var targetResult = WaitResult(staleTarget);
        runtime.Wait();

        Expect(declinedMode.Outcome == NomadCoreRequestOutcome.NotAttempted,
            "new mode selection is not queued behind a target waiting for HELLO");
        Expect(targetResult.Outcome == NomadCoreRequestOutcome.NotAttempted && targetResult.ErrorCode == "stale_input",
            "old angle target is rejected after HELLO when selection changed");
        Expect(runtime.Commands.Count == 0, "stale target never becomes a mutation");
    }

    private static void Gimbal_StaleCompositeDoesNotWriteAfterHello()
    {
        using var runtime = new AsyncRuntime(1, delayHello: true);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        var stalePair = GimbalController.SetModeAndTargetAsync(MountMode.MavlinkTargeting, 12.0f, -3.0f);
        runtime.WaitForHello();
        var declinedMode = WaitResult(GimbalController.SetModeAsync(MountMode.RcTargeting));
        runtime.ReleaseHello();
        var pairResult = WaitResult(stalePair);
        runtime.Wait();

        Expect(declinedMode.Outcome == NomadCoreRequestOutcome.NotAttempted,
            "new mode selection is bounded while the old pair is awaiting HELLO");
        Expect(pairResult.Outcome == NomadCoreRequestOutcome.NotAttempted && pairResult.ErrorCode == "stale_input",
            "old combined mode and target are rejected after HELLO when selection changed");
        Expect(runtime.Commands.Count == 0, "stale combined request never becomes a mutation");
    }
}
