// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;
using NOMAD.MissionPlanner;
using NOMAD.MissionPlanner.Connectivity;

internal static partial class NomadCoreClientTests
{
    private static void Profile_SamePortRetiredConfigCannotCreateControls()
    {
        using var runtime = new MockRuntime(1, semanticResponse: request => SemanticResponse(request));
        var retired = new NOMADConfig { CoreRuntimePort = runtime.Port };
        OutputController.Initialize(retired);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        using var panel = new ActuatorControlPanel(retired);
        WaitForPanel(panel.SendActionAsync("release", "activate"));
        Expect(runtime.CommandCount == 0,
            "a newly opened stale view cannot control another profile sharing the same runtime port");
    }

    private static void Profile_PendingHidReleaseCannotRetarget()
    {
        using var entered = new ManualResetEventSlim();
        using var released = new ManualResetEventSlim();
        using var oldRuntime = new MockRuntime(1, semanticResponse: request =>
        {
            entered.Set();
            if (!released.Wait(5000))
            {
                throw new TimeoutException("Old HID release was not released.");
            }
            return SemanticResponse(request);
        });
        using var nextRuntime = new MockRuntime(1, semanticResponse: request => SemanticResponse(request));
        var config = HidConfig(oldRuntime.Port);
        config.JoystickSw1UpAction = "first:positive";
        config.JoystickSw1DownAction = "second:negative";
        OutputController.Initialize(config);
        var service = new NomadJoystickService(config);
        var contradictory = HidButtons(4);
        contradictory[5] = true;
        var pending = service.TestButtons(contradictory);
        Expect(entered.Wait(5000), "first old-aircraft safe request is pending before profile change");
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = nextRuntime.Port });
        released.Set();
        Expect(pending.Wait(5000), "old HID batch finishes without waiting on the new aircraft");
        oldRuntime.Wait();
        Expect(oldRuntime.CommandCount == 1 && nextRuntime.CommandCount == 0,
            "a queued HID release retains its original aircraft context instead of retargeting");
    }

    private static void Profile_DefinitionsRequireRuntimeReadback()
    {
        using var runtime = new MockRuntime(4, semanticResponse: request => SemanticResponse(request));
        var config = new NOMADConfig
        {
            CoreRuntimePort = runtime.Port,
            ActuatorDefinitions = JArray.Parse("[{\"id\":\"another_aircraft\"}]"),
        };
        OutputController.Initialize(config);
        var blocked = OutputController.ActuatorActionAsync("release", "activate").GetAwaiter().GetResult();
        Expect(blocked.Outcome == NomadCoreRequestOutcome.NotAttempted &&
            blocked.ErrorCode == "aircraft_profile_unverified" && runtime.CommandCount == 0,
            "saving desired mappings does not authorize an actuator before runtime readback");
        Expect(OutputController.ActuatorActionAsync("release", "safe").GetAwaiter().GetResult().Succeeded,
            "safe actions remain available while desired mappings need review");
        OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
        Expect(!OutputController.ProfileActuatorsCompatible &&
            OutputController.GetContinuousAxisMetadata("release") == null,
            "a different runtime catalog cannot enable the selected profile's continuous input");

        config.ActuatorDefinitions = JArray.Parse("[{\"id\":\"release\"}]");
        OutputController.Initialize(config);
        OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
        Expect(OutputController.ProfileActuatorsCompatible,
            "matching authoritative readback verifies the saved profile's desired actuator mappings");
        var accepted = OutputController.ActuatorActionAsync("release", "activate").GetAwaiter().GetResult();
        runtime.Wait();
        Expect(accepted.Succeeded && runtime.Commands.Count == 4 &&
            runtime.Commands[0]["operation"].ToString() == "safe" &&
            runtime.Commands[3]["operation"].ToString() == "activate",
            "only explicit requests reach the backend; selecting a profile never applies mappings");
    }

    private static void Profile_LateCatalogCannotReplaceNewAircraft()
    {
        using var oldRuntime = new AsyncRuntime(1, semanticResponse: request => SemanticResponse(request));
        var oldConfig = new NOMADConfig { CoreRuntimePort = oldRuntime.Port };
        OutputController.Initialize(oldConfig);
        using var retiredPanel = new ActuatorControlPanel(oldConfig);
        var oldRead = OutputController.GetActuatorsAsync();
        oldRuntime.WaitForCommands(1);

        using var newRuntime = new MockRuntime(1, semanticResponse: request =>
        {
            var response = SemanticResponse(request);
            response["actuators"] = new object[0];
            return response;
        });
        OutputController.Initialize(new NOMADConfig
        {
            CoreRuntimePort = newRuntime.Port, ActuatorDefinitions = new JArray(),
        });
        WaitForPanel(retiredPanel.SendActionAsync("release", "activate"));
        var newRead = OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
        newRuntime.Wait();
        oldRuntime.ReleaseResponses();
        var late = oldRead.GetAwaiter().GetResult();
        oldRuntime.Wait();
        Expect(newRead.Succeeded && newRuntime.CommandCount == 1 &&
            newRuntime.LastCommandType == "get_actuators",
            "a retired panel cannot send its action to the newly selected aircraft");
        Expect(!late.PresentationCurrent && OutputController.ProfileActuatorsCompatible &&
            OutputController.GetDisplayState("release") == null &&
            OutputController.GetContinuousAxisMetadata("release") == null,
            "a late catalog from the old client cannot change new aircraft display, mappings or input eligibility");
    }

    private static void Profile_GimbalRequestsKeepTheirOriginalClient()
    {
        Func<Task<NomadCoreRequestResult>>[] requests =
        {
            () => OutputController.ConfigureGimbalAsync(2),
            () => OutputController.SendGimbalTargetAsync(12, -3),
            () => OutputController.ConfigureGimbalAndSendTargetAsync(2, 12, -3),
        };
        foreach (var send in requests)
        {
            using var oldRuntime = new AsyncRuntime(1, delayHello: true);
            using var newRuntime = new MockRuntime(0);
            OutputController.Initialize(new NOMADConfig { CoreRuntimePort = oldRuntime.Port });
            var pending = send();
            oldRuntime.WaitForHello();
            OutputController.Initialize(new NOMADConfig { CoreRuntimePort = newRuntime.Port });
            oldRuntime.ReleaseHello();
            var result = pending.GetAwaiter().GetResult();
            oldRuntime.Wait();
            Expect(result.Outcome == NomadCoreRequestOutcome.NotAttempted && result.ErrorCode == "stale_input" &&
                oldRuntime.Commands.Count == 0 && newRuntime.CommandCount == 0,
                "a gimbal request prepared by the previous aircraft cannot mutate either aircraft after switching");
        }
    }

    private static void Profile_GimbalPresetRetiresPendingMode()
    {
        using var runtime = new AsyncRuntime(1, delayHello: true);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        var pending = GimbalController.SetModeAsync(MountMode.MavlinkTargeting);
        runtime.WaitForHello();
        GimbalController.SetTargetAngles(12, -3);
        GimbalController.ApplyProfile(new NOMADConfig { GimbalMountMode = 3, JoystickGimbalMaxRateDegSec = 90 });
        runtime.ReleaseHello();
        var result = pending.GetAwaiter().GetResult();
        runtime.Wait();
        Expect(result.Outcome == NomadCoreRequestOutcome.NotAttempted && runtime.Commands.Count == 0 &&
            GimbalController.SelectedMode == MountMode.RcTargeting && GimbalController.MaxRateDegSec == 90 &&
            GimbalController.TargetPitchDeg == 0 && GimbalController.TargetRollDeg == 0,
            "profile loading restores gimbal presets and retires targets without sending commands");
    }

    private static void Profile_SettingsMutationsRejectChangedInput()
    {
        Func<NomadCoreClient, Func<bool>, Task<NomadCoreRequestResult>>[] requests =
        {
            (client, current) => client.AdmitAuthorityAsync(inputStillCurrent: current),
            (client, current) => client.RevokeAuthorityAsync(inputStillCurrent: current),
            (client, current) => client.HandbackAuthorityAsync(inputStillCurrent: current),
            (client, current) => client.LandAsync(inputStillCurrent: current),
            (client, current) => OutputController.ConfigureActuatorsAsync("[]", current),
        };
        foreach (var send in requests)
        {
            using var runtime = new AsyncRuntime(1, delayHello: true, capabilities: LandCapabilities);
            OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
            bool current = true;
            var pending = send(OutputController.CreateCoreClient(), () => current);
            runtime.WaitForHello();
            current = false;
            runtime.ReleaseHello();
            var result = pending.GetAwaiter().GetResult();
            runtime.Wait();
            Expect(result.Outcome == NomadCoreRequestOutcome.NotAttempted && result.ErrorCode == "stale_input" &&
                runtime.Commands.Count == 0, "profile edits during HELLO fence settings mutations before send");
        }
    }

    private static void Profile_FirstGimbalInputConfiguresItsTarget()
    {
        using var runtime = new AsyncRuntime(1, semanticResponse: GimbalSuccessResponse);
        var config = new NOMADConfig { CoreRuntimePort = runtime.Port, GimbalMountMode = 2 };
        GimbalController.ApplyProfile(config);
        OutputController.Initialize(config);
        var target = GimbalController.RequestPitchRollTargetAsync(12, -3);
        runtime.WaitForCommands(1);
        runtime.ReleaseResponses();
        var result = target.GetAwaiter().GetResult();
        runtime.Wait();
        Expect(result.Succeeded && runtime.Commands.Count == 1 &&
            runtime.Commands[0]["type"].ToString() == "configure_gimbal_target",
            "first explicit gimbal input configures and targets once without a startup mode command");
    }

    private static void Profile_LandConfirmationCannotRetarget()
    {
        using var oldRuntime = new MockRuntime(0);
        using var newRuntime = new MockRuntime(0);
        var next = new NOMADConfig { CoreRuntimePort = newRuntime.Port };
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = oldRuntime.Port });
        using var panel = new LandControlPanel(() =>
        {
            OutputController.Initialize(next);
            return true;
        });
        WaitForPanel(panel.RequestLandAsync());
        WaitForPanel(panel.RequestLandAsync());
        Expect(oldRuntime.CommandCount == 0 && newRuntime.CommandCount == 0 &&
            panel.OperatorStatus.Contains("Aircraft profile changed"),
            "LAND confirmation and later clicks on a retired panel cannot retarget the selected aircraft");
    }
}
