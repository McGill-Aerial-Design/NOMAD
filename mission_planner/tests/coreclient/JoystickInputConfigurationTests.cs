// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Threading;
using NOMAD.MissionPlanner;

internal static partial class NomadCoreClientTests
{
    private static void Joystick_InputConfigurationTests()
    {
        Mapping_ActiveAliasesAndTerminationCollisionFailClosed();
        Mapping_InvalidLiveConfigUsesOnlyLastValidatedTargets();
        Mapping_InactiveAliasesDoNotCreateContradictions();
        Mapping_ConfigurableTerminationKeepsUnavailableSemantics();
        AxisEligibility_MissingAndBlockedMetadataFailClosed();
        AxisEligibility_ActionResponseDoesNotWipeCatalog();
        AxisEligibility_FullCatalogRevokesCachedPermission(true);
        AxisEligibility_FullCatalogRevokesCachedPermission(false);
        Catalog_ServerRevisionOrdersDelayedConfiguration(true);
        Catalog_ServerRevisionOrdersDelayedConfiguration(false);
        Catalog_MissingRevisionCannotPublishCapabilities();
        Mapping_EnteringContradictionStopsAllTargets(false);
        Mapping_EnteringContradictionStopsAllTargets(true);
    }

    private static void Mapping_ActiveAliasesAndTerminationCollisionFailClosed()
    {
        using var runtime = new MockRuntime(0);
        var config = HidConfig(runtime.Port);
        config.JoystickSw2UpAction = "other:activate";
        config.JoystickButtonIndices[2] = config.JoystickButtonIndices[0];
        OutputController.Initialize(config);
        var service = new NomadJoystickService(config);
        service.TestButtons(HidButtons(4)).GetAwaiter().GetResult();
        Expect(config.GetInputMappingError().Contains("unique") && runtime.CommandCount == 0,
            "same physical button cannot activate distinct semantic bindings");
        config.JoystickSw2UpAction = "None";
        config.JoystickKillSwitchEnabled = true;
        config.JoystickTerminationButtonIndex = 4;
        Log.LastError = "";
        service.TestButtons(HidButtons(4)).GetAwaiter().GetResult();
        Expect(config.GetInputMappingError().Contains("disjoint") && runtime.CommandCount == 0 && Log.LastError == "",
            "colliding termination button emits neither an actuator intent nor termination request");
        runtime.Wait();
    }

    private static void Mapping_InvalidLiveConfigUsesOnlyLastValidatedTargets()
    {
        using var runtime = new MockRuntime(2, semanticResponse: request => SemanticResponse(request));
        var config = HidConfig(runtime.Port);
        OutputController.Initialize(config);
        var service = new NomadJoystickService(config);
        service.TestButtons(HidButtons()).GetAwaiter().GetResult();
        config.JoystickSw1UpAction = "changed:activate";
        config.JoystickSw2UpAction = "other:activate";
        config.JoystickButtonIndices[2] = config.JoystickButtonIndices[0];
        service.TestButtons(HidButtons(4)).GetAwaiter().GetResult();
        WaitForHid(runtime, 2);
        service.TestButtons(HidButtons(4)).GetAwaiter().GetResult();
        runtime.Wait();
        Expect(runtime.Commands[1]["actuator_id"].ToString() == "release" && runtime.Commands[1]["operation"].ToString() == "safe",
            "live invalid mapping resets only last validated target, without using changed ambiguous bindings");
        Expect(runtime.Commands.Count == 2, "repeated invalid samples do not repeat safe/reset or send ambiguous activation");
    }

    private static void Mapping_InactiveAliasesDoNotCreateContradictions()
    {
        using var runtime = new MockRuntime(2, semanticResponse: request => SemanticResponse(request));
        var config = HidConfig(runtime.Port);
        config.JoystickButtonIndices = new[] { 4, 4, 4, 4, 4, 4 };
        OutputController.Initialize(config);
        var service = new NomadJoystickService(config);
        Expect(config.GetInputMappingError() == null, "inactive duplicate physical indices are allowed");
        service.TestButtons(HidButtons()).GetAwaiter().GetResult();
        service.TestButtons(HidButtons(4)).GetAwaiter().GetResult();
        runtime.Wait();
        Expect(runtime.Commands[1]["operation"].ToString() == "activate",
            "inactive aliases do not fabricate a second position or block the sole active binding");
    }

    private static void Mapping_ConfigurableTerminationKeepsUnavailableSemantics()
    {
        using var runtime = new MockRuntime(0);
        var config = new NOMADConfig { CoreRuntimePort = runtime.Port, JoystickKillSwitchEnabled = true,
            JoystickTerminationButtonIndex = 9 };
        OutputController.Initialize(config);
        var service = new NomadJoystickService(config);
        service.TestButtons(HidButtons()).GetAwaiter().GetResult();
        Log.LastError = "";
        service.TestButtons(HidButtons(9)).GetAwaiter().GetResult();
        runtime.Wait();
        Expect(Log.LastError.Contains("Termination unavailable") && runtime.CommandCount == 0,
            "remapped monitor preserves unavailable-termination reporting without a vehicle command");
    }

    private static Dictionary<string, object> EligibilityResponse(Dictionary<string, object> request, bool? allowed)
    {
        var response = SemanticResponse(request);
        if (request["type"].ToString() != "get_actuators") { return response; }
        var actor = DiscoveredActuator();
        var actions = (object[])actor["actions"];
        foreach (Dictionary<string, object> action in actions)
        {
            if (action["control"].ToString() != "position") { continue; }
            if (allowed.HasValue)
            {
                action["continuous_axis_allowed"] = allowed.Value;
                action["continuous_axis_blocked_reason"] = allowed.Value ? "" : "Use discrete UI confirmations.";
            }
            else { action.Remove("continuous_axis_allowed"); action.Remove("continuous_axis_blocked_reason"); }
        }
        response["actuators"] = new object[] { actor };
        return response;
    }

    private static void AxisEligibility_MissingAndBlockedMetadataFailClosed()
    {
        foreach (bool? allowed in new bool?[] { null, false })
        {
            using var runtime = new MockRuntime(1, semanticResponse: request => EligibilityResponse(request, allowed));
            var config = HidConfig(runtime.Port);
            config.JoystickPositionActuatorId = "release";
            OutputController.Initialize(config);
            OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
            var service = new NomadJoystickService(config);
            service.TranslatePositionInput(true, 0.75f);
            runtime.Wait();
            Expect(runtime.CommandCount == 1 && runtime.LastCommandType == "get_actuators",
                "missing or blocked backend eligibility sends no continuous position request");
        }
    }

    private static void AxisEligibility_ActionResponseDoesNotWipeCatalog()
    {
        using var runtime = new MockRuntime(2, semanticResponse: request => EligibilityResponse(request, true));
        OutputController.Initialize(HidConfig(runtime.Port));
        OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
        OutputController.ActuatorActionAsync("release", "activate").GetAwaiter().GetResult();
        runtime.Wait();
        Expect(OutputController.GetContinuousAxisMetadata("release")?.ContinuousAxisAllowed == true,
            "action-only state response does not wipe the full backend input capability catalog");
    }

    private static void AxisEligibility_FullCatalogRevokesCachedPermission(bool removed)
    {
        int catalogs = 0;
        using var runtime = new MockRuntime(2, semanticResponse: request =>
        {
            var response = EligibilityResponse(request, true);
            if (++catalogs == 2)
            {
                response["actuator_configuration_revision"] = 2UL;
                if (removed) { response["actuators"] = new object[0]; }
                else
                {
                    var actor = DiscoveredActuator();
                    actor["actions"] = new object[] { new Dictionary<string, object> { ["operation"] = "activate", ["label"] = "On", ["control"] = "button" } };
                    response["actuators"] = new object[] { actor };
                }
            }
            return response;
        });
        OutputController.Initialize(HidConfig(runtime.Port));
        OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
        Expect(OutputController.GetContinuousAxisMetadata("release")?.ContinuousAxisAllowed == true,
            "backend allowed metadata is received as data");
        OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
        runtime.Wait();
        Expect(OutputController.GetContinuousAxisMetadata("release") == null,
            "same-incarnation full catalog removes prior continuous-axis eligibility after removal or behavior change");
    }
    private static void Catalog_ServerRevisionOrdersDelayedConfiguration(bool empty)
    {
        using var configureEntered = new ManualResetEventSlim();
        using var allowConfigure = new ManualResetEventSlim();
        using var runtime = new AsyncRuntime(3, sequenceFloor: 7, semanticResponse: request =>
        {
            if (request["type"].ToString() == "configure_actuators")
            {
                configureEntered.Set();
                if (!allowConfigure.Wait(5000)) { throw new TimeoutException("Delayed configuration was not released."); }
                var configured = SemanticResponse(request);
                configured["actuator_configuration_revision"] = 2UL;
                if (empty) { configured["actuators"] = new object[0]; }
                else
                {
                    var actor = DiscoveredActuator();
                    foreach (Dictionary<string, object> action in (object[])actor["actions"])
                    { if (action["control"].ToString() == "position") { action["continuous_axis_allowed"] = false; } }
                    configured["actuators"] = new object[] { actor };
                }
                return configured;
            }
            return EligibilityResponse(request, true);
        });
        runtime.ReleaseResponses();
        OutputController.Initialize(HidConfig(runtime.Port));
        var configure = OutputController.ConfigureActuatorsAsync("[]");
        Expect(configureEntered.Wait(5000), "configuration request is deliberately delayed before new catalog return");
        var oldRead = OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
        Expect(oldRead.PresentationCurrent && oldRead.ActuatorConfigurationRevision == 1 &&
            OutputController.GetContinuousAxisMetadata("release")?.ContinuousAxisAllowed == true,
            "higher-client-sequence read publishes the still-old server revision");
        allowConfigure.Set();
        var newConfig = configure.GetAwaiter().GetResult();
        Expect(newConfig.RequestSequence < oldRead.RequestSequence && newConfig.PresentationCurrent &&
            newConfig.ActuatorConfigurationRevision == 2 &&
            OutputController.GetContinuousAxisMetadata("release")?.ContinuousAxisAllowed != true,
            "lower-client-sequence configuration rev2 replaces old read rev1, including empty/blocked catalogs");
        var lateOld = OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
        runtime.Wait();
        Expect(!lateOld.PresentationCurrent && OutputController.GetContinuousAxisMetadata("release")?.ContinuousAxisAllowed != true,
            "late old server revision cannot restore catalog controls or streaming eligibility");
    }

    private static void Catalog_MissingRevisionCannotPublishCapabilities()
    {
        using var runtime = new MockRuntime(1, semanticResponse: request =>
        {
            var response = EligibilityResponse(request, true);
            response.Remove("actuator_configuration_revision");
            return response;
        });
        OutputController.Initialize(HidConfig(runtime.Port));
        var responseResult = OutputController.GetActuatorsAsync().GetAwaiter().GetResult();
        runtime.Wait();
        Expect(responseResult.Succeeded && !responseResult.PresentationCurrent &&
            OutputController.GetContinuousAxisMetadata("release") == null,
            "unversioned catalog preserves request facts but cannot publish controls or continuous-axis permission");
    }

    private static void Mapping_EnteringContradictionStopsAllTargets(bool oppositeMapped)
    {
        using var runtime = new MockRuntime(oppositeMapped ? 5 : 3,
            semanticResponse: request => SemanticResponse(request));
        var config = HidConfig(runtime.Port);
        if (oppositeMapped) { config.JoystickSw1DownAction = "other:activate"; }
        OutputController.Initialize(config);
        var service = new NomadJoystickService(config);
        service.TestButtons(HidButtons()).GetAwaiter().GetResult();
        service.TestButtons(HidButtons(4)).GetAwaiter().GetResult();
        var both = HidButtons(4);
        both[5] = true;
        service.TestButtons(both).GetAwaiter().GetResult();
        service.TestButtons(both).GetAwaiter().GetResult();
        runtime.Wait();
        Expect(runtime.Commands.FindAll(command => command.ContainsKey("operation") &&
            command["operation"].ToString() == "safe" && command["actuator_id"].ToString() == "release").Count == 1,
            "entering both-pressed contradiction sends one safe intent to the already-held UP target");
        if (oppositeMapped)
        {
            Expect(runtime.Commands.FindAll(command => command.ContainsKey("operation") &&
                command["operation"].ToString() == "safe" && command["actuator_id"].ToString() == "other").Count == 1,
                "contradiction entry also sends one safe intent to distinct DOWN target");
        }
        Expect(runtime.Commands.FindAll(command => command.ContainsKey("operation") &&
            command["operation"].ToString() == "activate").Count == 1,
            "persistent contradiction emits no activation or repeated safe retry");
    }

}
