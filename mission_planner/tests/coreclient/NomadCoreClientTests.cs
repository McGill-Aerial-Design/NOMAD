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
using NOMAD.MissionPlanner.Connectivity;

internal static partial class NomadCoreClientTests
{
    private static int _failures;

    private static int Main()
    {
        Termination_ReportsUnavailableWithoutVehicleDispatch();
        GuidedGoto_ReportsUnavailableWithoutDispatch();
        RuntimeUnavailable_FailsClosedBeforeSend();
        Servo_FailsClosedOnInvalidInput();
        SetRelay_FailsClosedOnInvalidInput();
        MotorTest_FailsClosedOnInvalidInput();
        GimbalConfigure_FailsClosedOnInvalidInput();
        GimbalTarget_FailsClosedOnInvalidInput();
        Runtime_SendsTypedRequestOnce();
        Runtime_GimbalTarget_UsesTypedRequestAndRequiresAuthority();
        Runtime_GimbalTargetDoesNotReplayUnknownOutcome();
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

        Expect(!client.Servo(8, 1500), "unavailable runtime makes the action unavailable");
        Expect(!client.GimbalTarget(0.0, 0.0), "unavailable runtime fails closed for a gimbal target");
        Expect(client.LastOutcome == NomadCoreRequestOutcome.FailedBeforeSend,
            "runtime connection failure is reported before command send");
        Expect(client.LastErrorCode == "runtime_unavailable",
            "unavailable runtime has a stable error code");
    }

    private static void Servo_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient("test-key", ReservePort());
        Expect(!client.Servo(0, 1500), "channel 0 rejected");
        Expect(!client.Servo(-1, 1500), "negative channel rejected");
        Expect(!client.Servo(1, 499), "pwm below 500 rejected");
        Expect(!client.Servo(1, 2501), "pwm above 2500 rejected");
    }

    private static void SetRelay_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient("test-key", ReservePort());
        Expect(!client.SetRelay(-1, true), "negative relay rejected");
        Expect(!client.SetRelay(16, true), "relay above 15 rejected");
    }

    private static void MotorTest_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient("test-key", ReservePort());
        Expect(!client.MotorTest(0, 1000, 1.0), "motor instance 0 rejected");
        Expect(!client.MotorTest(1, 400, 1.0), "pwm below 500 rejected");
        Expect(!client.MotorTest(1, 2600, 1.0), "pwm above 2500 rejected");
        Expect(!client.MotorTest(1, 1000, double.NaN), "NaN timeout rejected");
        Expect(!client.MotorTest(1, 1000, double.PositiveInfinity), "infinite timeout rejected");
    }

    private static void GimbalConfigure_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient("test-key", ReservePort());
        Expect(!client.GimbalConfigure(-1), "negative mount mode rejected");
        Expect(!client.GimbalConfigure(5), "mount mode above 4 rejected");
    }

    private static void GimbalTarget_FailsClosedOnInvalidInput()
    {
        var client = new NomadCoreClient("test-key", ReservePort());
        Expect(!client.GimbalTarget(double.NaN, 0), "NaN pitch rejected");
        Expect(!client.GimbalTarget(double.PositiveInfinity, 0), "infinite pitch rejected");
        Expect(!client.GimbalTarget(-90.01, 0), "pitch below limit rejected");
        Expect(!client.GimbalTarget(0, 30.01), "roll above limit rejected");
        Expect(client.LastOutcome == NomadCoreRequestOutcome.Rejected, "invalid target is reported as rejected");
        Expect(client.LastErrorCode == "invalid_argument", "invalid target has a stable error code");
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
