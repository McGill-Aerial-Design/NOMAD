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
            var result1 = client.ServoAsync(8, 1500).GetAwaiter().GetResult();
            Expect(!result1.Succeeded, $"{outcomes[index]} is not software success");
            runtime.Wait();
            Expect(result1.Outcome == expected[index], $"command preserves {outcomes[index] ?? "missing"} outcome");
            Expect(result1.Acknowledged == (index == 1), "negative ACK remains independent of failure outcome");
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
                var result1 = client.ServoAsync(8, 1500).GetAwaiter().GetResult();
                Expect(!result1.Succeeded, "contradictory no-send response cannot report success");
                runtime.Wait();
                Expect(result1.Outcome == NomadCoreRequestOutcome.UnknownOutcome,
                    "ACK or software success contradicts rejected-before-send and must remain unknown");
                Expect(result1.Acknowledged == acknowledged, "contradictory response preserves actual ACK evidence");
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
        var result1 = client.ServoAsync(8, 1500).GetAwaiter().GetResult();
        Expect(!result1.Succeeded, "interrupted error is not success even with ACK evidence");
        interrupted.Wait();
        Expect(result1.Outcome == NomadCoreRequestOutcome.Interrupted,
            "ACK does not erase interrupted classification");
        Expect(result1.Acknowledged == true, "interrupted error preserves received acknowledgement");
        Expect(interrupted.CommandCount == 1, "acknowledged interruption is never replayed");
    }

    private static void CheckRuntimeError(string code, string outcome, NomadCoreRequestOutcome expected)
    {
        using var runtime = new MockRuntime(1, outcome: outcome, errorCode: code);
        var client = new NomadCoreClient("test-key", runtime.Port);
        var result1 = client.ServoAsync(8, 1500).GetAwaiter().GetResult();
        Expect(!result1.Succeeded, $"{code} response is not success");
        runtime.Wait();
        Expect(result1.Outcome == expected, $"{code} preserves {outcome ?? "missing"} classification");
        Expect(result1.ErrorCode == code, "error reason remains independent of outcome");
        Expect(result1.Acknowledged == null, "error without command result1 does not invent an ACK");
        Expect(runtime.CommandCount == 1, $"{code} is not automatically replayed");
    }

    private static void Runtime_AllMutationsPreserveSuccessAndAcknowledgement()
    {
        using var runtime = new MockRuntime(5);
        var client = new NomadCoreClient("test-key", runtime.Port);
        CheckSoftwareSuccess(() => client.ServoAsync(8, 1500).GetAwaiter().GetResult());
        CheckSoftwareSuccess(() => client.SetRelayAsync(3, true).GetAwaiter().GetResult());
        CheckSoftwareSuccess(() => client.MotorTestAsync(1, 1000, 0.5).GetAwaiter().GetResult());
        CheckSoftwareSuccess(() => client.GimbalConfigureAsync(2).GetAwaiter().GetResult());
        CheckSoftwareSuccess(() => client.GimbalTargetAsync(10, 5).GetAwaiter().GetResult());
        runtime.Wait();
        Expect(runtime.CommandCount == 5, "all five supported mutations are sent exactly once");
    }

    private static void CheckSoftwareSuccess(Func<NomadCoreRequestResult> operation)
    {
        var result1 = operation();
        Expect(result1.Succeeded, "software success is preserved");
        Expect(result1.Outcome == NomadCoreRequestOutcome.Succeeded, "explicit runtime success is preserved");
        Expect(result1.Acknowledged == true, "protocol acknowledgement is separately preserved");
    }

    private static void LocalValidation_ClearsPreviousSuccess()
    {
        using var runtime = new MockRuntime(1);
        var client = new NomadCoreClient("test-key", runtime.Port);
        var result1 = client.ServoAsync(8, 1500).GetAwaiter().GetResult();
        Expect(result1.Succeeded, "establish previous success and acknowledgement");
        runtime.Wait();
        var invalid = new Func<NomadCoreRequestResult>[]
        {
            () => client.ServoAsync(0, 1500).GetAwaiter().GetResult(),
            () => client.SetRelayAsync(-1, true).GetAwaiter().GetResult(),
            () => client.MotorTestAsync(0, 1500, 1).GetAwaiter().GetResult(),
            () => client.GimbalConfigureAsync(5).GetAwaiter().GetResult(),
            () => client.GimbalTargetAsync(double.NaN, 0).GetAwaiter().GetResult(),
        };
        foreach (var operation in invalid)
        {
            var invalidResult = operation();
            Expect(!invalidResult.Succeeded, "local invalid request fails without dispatch");
            Expect(invalidResult.Outcome == NomadCoreRequestOutcome.NotAttempted, "validation has its own outcome");
            Expect(invalidResult.ErrorCode == "invalid_argument", "local validation gives its own reason");
            Expect(invalidResult.Acknowledged == null, "local validation does not inherit ACK evidence");
        }
        Expect(runtime.CommandCount == 1, "local validation transmits no additional mutation");
    }
}
