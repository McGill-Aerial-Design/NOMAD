// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using NOMAD.MissionPlanner.Connectivity;

internal static partial class NomadCoreClientTests
{
    private static void Runtime_PreservesCommandOutcomeMatrix()
    {
        var outcomes = new[] { "rejected", "failed", "interrupted", "unknown", null, "future-value" };
        var expected = new[] { NomadCoreRequestOutcome.Rejected, NomadCoreRequestOutcome.Failed,
            NomadCoreRequestOutcome.Interrupted, NomadCoreRequestOutcome.UnknownOutcome,
            NomadCoreRequestOutcome.UnknownOutcome, NomadCoreRequestOutcome.UnknownOutcome };
        for (var index = 0; index < outcomes.Length; index++)
        {
            using var runtime = new MockRuntime(1, outcome: outcomes[index], acknowledged: index == 1);
            var client = new NomadCoreClient("test-key", runtime.Port);
            Expect(!client.Servo(8, 1500), $"{outcomes[index]} is not software success");
            runtime.Wait();
            Expect(client.LastOutcome == expected[index], $"command preserves {outcomes[index] ?? "missing"} outcome");
            Expect(client.LastAcknowledged == (index == 1), "negative ACK remains independent of failure outcome");
            Expect(runtime.CommandCount == 1, "non-success mutation is never replayed");
        }
    }

    private static void Runtime_ContradictoryRejectionIsUnknown()
    {
        foreach (var error in new[] { false, true })
        {
            foreach (var acknowledged in new[] { false, true })
            {
                using var runtime = new MockRuntime(1, outcome: "rejected", acknowledged: acknowledged,
                    errorCode: error ? "internal_error" : null, includeErrorResult: error,
                    resultSuccess: !acknowledged);
                var client = new NomadCoreClient("test-key", runtime.Port);
                Expect(!client.Servo(8, 1500), "contradictory no-send response cannot report success");
                runtime.Wait();
                Expect(client.LastOutcome == NomadCoreRequestOutcome.UnknownOutcome,
                    "ACK or software success contradicts rejected-before-send and must remain unknown");
                Expect(client.LastAcknowledged == acknowledged, "contradictory response preserves actual ACK evidence");
                Expect(runtime.CommandCount == 1, "contradictory response is never replayed");
            }
        }
    }

    private static void Runtime_PreservesErrorOutcomeMatrix()
    {
        CheckRuntimeError("stale_authority", "rejected", NomadCoreRequestOutcome.Rejected);
        CheckRuntimeError("expired_request", "rejected", NomadCoreRequestOutcome.Rejected);
        CheckRuntimeError("stale_request", "rejected", NomadCoreRequestOutcome.Rejected);
        CheckRuntimeError("invalid_argument", "rejected", NomadCoreRequestOutcome.Rejected);
        CheckRuntimeError("admission_cancelled", "rejected", NomadCoreRequestOutcome.Rejected);
        CheckRuntimeError("authority_interrupted", "interrupted", NomadCoreRequestOutcome.Interrupted);
        CheckRuntimeError("internal_error", "unknown", NomadCoreRequestOutcome.UnknownOutcome);
        CheckRuntimeError("request_in_progress", "unknown", NomadCoreRequestOutcome.UnknownOutcome);
        CheckRuntimeError("audit_failure", "unknown", NomadCoreRequestOutcome.UnknownOutcome);
        CheckRuntimeError("audit_failure", "rejected", NomadCoreRequestOutcome.Rejected);
        CheckRuntimeError("authority_interrupted", null, NomadCoreRequestOutcome.UnknownOutcome);
        CheckRuntimeError("internal_error", "failed", NomadCoreRequestOutcome.Failed);
        CheckRuntimeError("invalid_argument", "success", NomadCoreRequestOutcome.UnknownOutcome);
        using var interrupted = new MockRuntime(1, outcome: "interrupted", errorCode: "authority_interrupted",
            includeErrorResult: true);
        var client = new NomadCoreClient("test-key", interrupted.Port);
        Expect(!client.Servo(8, 1500), "interrupted error is not success even with ACK evidence");
        interrupted.Wait();
        Expect(client.LastOutcome == NomadCoreRequestOutcome.Interrupted,
            "ACK does not erase interrupted classification");
        Expect(client.LastAcknowledged == true, "interrupted error preserves received acknowledgement");
        Expect(interrupted.CommandCount == 1, "acknowledged interruption is never replayed");
    }

    private static void CheckRuntimeError(string code, string outcome, NomadCoreRequestOutcome expected)
    {
        using var runtime = new MockRuntime(1, outcome: outcome, errorCode: code);
        var client = new NomadCoreClient("test-key", runtime.Port);
        Expect(!client.Servo(8, 1500), $"{code} response is not success");
        runtime.Wait();
        Expect(client.LastOutcome == expected, $"{code} preserves {outcome ?? "missing"} classification");
        Expect(client.LastErrorCode == code, "error reason remains independent of outcome");
        Expect(client.LastAcknowledged == null, "error without command result does not invent an ACK");
        Expect(runtime.CommandCount == 1, $"{code} is not automatically replayed");
    }

    private static void Runtime_AllMutationsPreserveSuccessAndAcknowledgement()
    {
        using var runtime = new MockRuntime(5);
        var client = new NomadCoreClient("test-key", runtime.Port);
        CheckSoftwareSuccess(client, () => client.Servo(8, 1500));
        CheckSoftwareSuccess(client, () => client.SetRelay(3, true));
        CheckSoftwareSuccess(client, () => client.MotorTest(1, 1000, 0.5));
        CheckSoftwareSuccess(client, () => client.GimbalConfigure(2));
        CheckSoftwareSuccess(client, () => client.GimbalTarget(10, 5));
        runtime.Wait();
        Expect(runtime.CommandCount == 5, "all five supported mutations are sent exactly once");
    }

    private static void CheckSoftwareSuccess(NomadCoreClient client, Func<bool> operation)
    {
        Expect(operation(), "software success retains Boolean convenience result");
        Expect(client.LastOutcome == NomadCoreRequestOutcome.Succeeded, "explicit runtime success is preserved");
        Expect(client.LastAcknowledged == true, "protocol acknowledgement is separately preserved");
    }

    private static void LocalValidation_ClearsPreviousSuccess()
    {
        using var runtime = new MockRuntime(1);
        var client = new NomadCoreClient("test-key", runtime.Port);
        Expect(client.Servo(8, 1500), "establish previous success and acknowledgement");
        runtime.Wait();
        var invalid = new Func<bool>[] { () => client.Servo(0, 1500), () => client.SetRelay(-1, true),
            () => client.MotorTest(0, 1500, 1), () => client.GimbalConfigure(5),
            () => client.GimbalTarget(double.NaN, 0) };
        foreach (var operation in invalid)
        {
            Expect(!operation(), "local invalid request fails without dispatch");
            Expect(client.LastOutcome == NomadCoreRequestOutcome.NotAttempted, "local validation clears stale success");
            Expect(client.LastErrorCode == "invalid_argument", "local validation gives its own reason");
            Expect(client.LastAcknowledged == null, "local validation clears stale ACK evidence");
        }
        Expect(runtime.CommandCount == 1, "local validation transmits no additional mutation");
    }
}
