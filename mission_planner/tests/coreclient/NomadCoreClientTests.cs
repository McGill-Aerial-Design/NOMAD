// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NomadCoreClient runtime IPC tests
// ============================================================
// Compiled with the Mission Planner-free client by
// scripts/build/test_plugin_core_client.ps1 (plain csc, no test framework).
// Run via `pixi run test-plugin-core-client`.
// ============================================================

using System;
using NOMAD.MissionPlanner;
using NOMAD.MissionPlanner.Connectivity;

internal static partial class NomadCoreClientTests
{
    private static int _failures;

    [STAThread]
    private static int Main(string[] args)
    {
        if (args.Length == 2 && args[0] == "--async-restart-child")
        {
            return RunAsyncRestartChild(int.Parse(args[1]));
        }
        System.Windows.Forms.Control.CheckForIllegalCrossThreadCalls = true;
        if (args.Length == 1 && args[0] == "--actuator-only")
        {
            ActuatorClient_ReportsBackendEvidence();
            Joystick_AuthorizationTests();
        Joystick_InputConfigurationTests();
            return _failures == 0 ? 0 : 1;
        }
        Runtime_AsyncTests();
        Termination_ReportsUnavailableWithoutVehicleDispatch();
        RuntimeUnavailable_FailsClosedBeforeSend();
        Servo_FailsClosedOnInvalidInput();
        SetRelay_FailsClosedOnInvalidInput();
        MotorTest_FailsClosedOnInvalidInput();
        GimbalConfigure_FailsClosedOnInvalidInput();
        GimbalTarget_FailsClosedOnInvalidInput();
        GimbalConfigureAndTarget_FailsClosedOnInvalidInput();
        Runtime_SendsTypedRequestOnce();
        Runtime_PreservesCommandOutcomeMatrix();
        Runtime_PreservesErrorOutcomeMatrix();
        Runtime_ContradictoryRejectionIsUnknown();
        Runtime_AllMutationsPreserveSuccessAndAcknowledgement();
        LocalValidation_ClearsPreviousSuccess();
        Output_ReportsTruthfulOutcomeWording();
        ActuatorClient_ReportsBackendEvidence();
        Joystick_AuthorizationTests();
        Joystick_InputConfigurationTests();
        Runtime_RejectsRogueRuntimeWithoutCredentialDisclosure();
        Runtime_AuditFailureAfterSendIsUnknown();
        Runtime_GimbalTarget_UsesTypedRequestAndRequiresAuthority();
        Runtime_GimbalTargetDoesNotReplayUnknownOutcome();
        Gimbal_ModeAndImmediateTargetAreSequenced();
        Gimbal_ModeAndTargetUseOneRuntimeRequest();
        Gimbal_UnknownModeOutcomeDoesNotReplay();
        Gimbal_UnknownCompositeDoesNotReplay();
        Gimbal_StaleStandaloneModeDoesNotWriteAfterHello();
        Gimbal_StaleTargetDoesNotWriteAfterHello();
        Gimbal_StaleCompositeDoesNotWriteAfterHello();
        Runtime_ReportsUnknownOutcomeWithoutReplay();
        Runtime_RejectsIncompatibleHelloBeforeCommand();
        Runtime_RejectsIncompatibleCommandResponseAsUnknown();
        Runtime_ReconnectsForNextRequest();
        Runtime_RequiresExplicitOwnershipAcrossClients();
        Runtime_RejectsWrongAuthorityResponseType();

        Console.WriteLine(_failures == 0
            ? "All core-client tests passed."
            : $"{_failures} core-client test(s) FAILED.");
        return _failures == 0 ? 0 : 1;
    }

    private static void RuntimeUnavailable_FailsClosedBeforeSend()
    {
        var client = new NomadCoreClient("test-key", ReservePort());

        var result1 = client.ServoAsync(8, 1500).GetAwaiter().GetResult();

        Expect(!result1.Succeeded, "unavailable runtime makes the action unavailable");
        var result2 = client.GimbalTargetAsync(0.0, 0.0).GetAwaiter().GetResult();
        Expect(!result2.Succeeded, "unavailable runtime fails closed for a gimbal target");
        Expect(result2.Outcome == NomadCoreRequestOutcome.FailedBeforeSend,
            "runtime connection failure is reported before command send");
        Expect(result2.ErrorCode == "runtime_unavailable",
            "unavailable runtime has a stable error code");
    }

    private static void Servo_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient("test-key", ReservePort());
        var result1 = client.ServoAsync(0, 1500).GetAwaiter().GetResult();
        Expect(!result1.Succeeded, "channel 0 rejected");
        var result2 = client.ServoAsync(-1, 1500).GetAwaiter().GetResult();
        Expect(!result2.Succeeded, "negative channel rejected");
        var result3 = client.ServoAsync(1, 499).GetAwaiter().GetResult();
        Expect(!result3.Succeeded, "pwm below 500 rejected");
        var result4 = client.ServoAsync(1, 2501).GetAwaiter().GetResult();
        Expect(!result4.Succeeded, "pwm above 2500 rejected");
    }

    private static void SetRelay_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient("test-key", ReservePort());
        var result1 = client.SetRelayAsync(-1, true).GetAwaiter().GetResult();
        Expect(!result1.Succeeded, "negative relay rejected");
        var result2 = client.SetRelayAsync(16, true).GetAwaiter().GetResult();
        Expect(!result2.Succeeded, "relay above 15 rejected");
    }

    private static void MotorTest_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient("test-key", ReservePort());
        var result1 = client.MotorTestAsync(0, 1000, 1.0).GetAwaiter().GetResult();
        Expect(!result1.Succeeded, "motor instance 0 rejected");
        var result2 = client.MotorTestAsync(1, 400, 1.0).GetAwaiter().GetResult();
        Expect(!result2.Succeeded, "pwm below 500 rejected");
        var result3 = client.MotorTestAsync(1, 2600, 1.0).GetAwaiter().GetResult();
        Expect(!result3.Succeeded, "pwm above 2500 rejected");
        var result4 = client.MotorTestAsync(1, 1000, double.NaN).GetAwaiter().GetResult();
        Expect(!result4.Succeeded, "NaN timeout rejected");
        var result5 = client.MotorTestAsync(1, 1000, double.PositiveInfinity).GetAwaiter().GetResult();
        Expect(!result5.Succeeded, "infinite timeout rejected");
    }

    private static void GimbalConfigure_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient("test-key", ReservePort());
        var result1 = client.GimbalConfigureAsync(-1).GetAwaiter().GetResult();
        Expect(!result1.Succeeded, "negative mount mode rejected");
        var result2 = client.GimbalConfigureAsync(5).GetAwaiter().GetResult();
        Expect(!result2.Succeeded, "mount mode above 4 rejected");
    }

    private static void GimbalTarget_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient("test-key", ReservePort());
        var result1 = client.GimbalTargetAsync(double.NaN, 0).GetAwaiter().GetResult();
        Expect(!result1.Succeeded, "NaN pitch rejected");
        var result2 = client.GimbalTargetAsync(double.PositiveInfinity, 0).GetAwaiter().GetResult();
        Expect(!result2.Succeeded, "infinite pitch rejected");
        var result3 = client.GimbalTargetAsync(-90.01, 0).GetAwaiter().GetResult();
        Expect(!result3.Succeeded, "pitch below limit rejected");
        var result4 = client.GimbalTargetAsync(0, 30.01).GetAwaiter().GetResult();
        Expect(!result4.Succeeded, "roll above limit rejected");
        Expect(result4.Outcome == NomadCoreRequestOutcome.NotAttempted, "invalid target is never attempted");
        Expect(result4.ErrorCode == "invalid_argument", "invalid target has a stable error code");
    }

    private static void GimbalConfigureAndTarget_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient("test-key", ReservePort());
        Expect(!client.GimbalConfigureAndTargetAsync(5, 0, 0).GetAwaiter().GetResult().Succeeded,
            "invalid paired mount mode rejected");
        Expect(!client.GimbalConfigureAndTargetAsync(2, 90.01, 0).GetAwaiter().GetResult().Succeeded,
            "invalid paired pitch rejected before mode write");
        Expect(!client.GimbalConfigureAndTargetAsync(2, 0, 30.01).GetAwaiter().GetResult().Succeeded,
            "invalid paired roll rejected before mode write");

    }

    private static void Expect(bool condition, string message)
    {
        if (condition)
        {
            return;
        }
        _failures++;
        Console.WriteLine($"FAIL: {message}");
    }
}
