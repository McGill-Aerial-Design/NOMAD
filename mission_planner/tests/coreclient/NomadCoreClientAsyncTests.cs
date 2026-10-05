// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Diagnostics;
using System.Globalization;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using System.Text;
using System.Web.Script.Serialization;
using NOMAD.MissionPlanner;
using NOMAD.MissionPlanner.Connectivity;

internal static partial class NomadCoreClientTests
{
    private static void Runtime_AsyncTests()
    {
        Runtime_DelayedResponseReturnsControl();
        Runtime_ConcurrentResultsStayIsolated();
        Output_ConcurrentRequestsUseExactResultsAndRejectOverlap();
        Output_ExplicitStopHasOneBoundedWaiter();
        Output_QueuedStopRetainsOriginalRuntimeAcrossConfigurationChange();
        Runtime_ConcurrentSequencesRespectRuntimeFloor();
        Runtime_RevokeInterruptsActiveMutation();
        Runtime_MockRejectsReversedSequences();
        Runtime_BufferedResponsesPreserveSizeLimit();
        Runtime_InvalidSequenceFloorsFailBeforeMutation();
        Runtime_StaleMutationIsNotRetried();
        Runtime_FreshProcessUsesConsumedSequenceFloor();
        Runtime_RestartRefreshesAuthorityBinding();
        Runtime_CancellationBeforeWriteIsNotSent();
        Runtime_CancellationAfterWriteIsUnknown();
        Runtime_ResponseTimeoutAfterWriteIsUnknown();
        Runtime_AsyncConnectionAndAuthenticationFailures();
    }

    private static int RunAsyncRestartChild(int port)
    {
        var result = new NomadCoreClient("test-key", port).ServoAsync(8, 1500).GetAwaiter().GetResult();
        return result.Succeeded ? 0 : 1;
    }

    private static void Runtime_DelayedResponseReturnsControl()
    {
        using var runtime = new AsyncRuntime(1);
        var request = new NomadCoreClient("test-key", runtime.Port).ServoAsync(8, 1500);
        runtime.WaitForCommands(1);
        Expect(!request.IsCompleted, "async API returns to its caller while runtime response is delayed");
        runtime.ReleaseResponses();
        Expect(WaitResult(request).Succeeded, "delayed response completes the original async request");
        runtime.Wait();
    }

    private static void Runtime_ConcurrentResultsStayIsolated()
    {
        using var runtime = new AsyncRuntime(1);
        using var rejectionRuntime = new AsyncRuntime(1);
        using var interruptionRuntime = new AsyncRuntime(1);
        var first = new NomadCoreClient("test-key", runtime.Port);
        var success = first.ServoAsync(1, 1500);
        runtime.WaitForCommands(1);
        var overlap = first.ServoAsync(2, 1500);
        var overlapResult = WaitResult(overlap);
        Expect(overlapResult.Outcome == NomadCoreRequestOutcome.NotAttempted &&
            overlapResult.ErrorCode == "request_in_progress", "same client overlap returns its own local result");
        var rejected = new NomadCoreClient("test-key", rejectionRuntime.Port).ServoAsync(2, 1500);
        rejectionRuntime.WaitForCommands(1);
        var interrupted = new NomadCoreClient("test-key", interruptionRuntime.Port).ServoAsync(3, 1500);
        interruptionRuntime.WaitForCommands(1);

        interruptionRuntime.ReleaseResponse(3);
        var interruptedResult = WaitResult(interrupted);
        Expect(!success.IsCompleted && !rejected.IsCompleted, "third request completes before earlier requests");
        rejectionRuntime.ReleaseResponse(2);
        var rejectedResult = WaitResult(rejected);
        Expect(!success.IsCompleted, "second request completes before the first request");
        runtime.ReleaseResponse(1);
        var successResult = WaitResult(success);
        Expect(successResult.Succeeded && successResult.Acknowledged == true,
            "first request retains its own success and acknowledgement");
        Expect(successResult.Message == "request-1" && successResult.ErrorCode == "",
            "first request retains its own message and empty error");
        Expect(rejectedResult.Outcome == NomadCoreRequestOutcome.Rejected &&
            rejectedResult.Acknowledged == false && rejectedResult.ErrorCode == "vehicle_rejected" &&
            rejectedResult.Message == "request-2", "second request retains its own complete rejection result");
        Expect(interruptedResult.Outcome == NomadCoreRequestOutcome.Interrupted &&
            interruptedResult.Acknowledged == true && interruptedResult.ErrorCode == "vehicle_interrupted" &&
            interruptedResult.Message == "request-3", "third request retains its own complete interruption result");
        Expect(!ReferenceEquals(successResult, rejectedResult), "requests return separate immutable result objects");
        runtime.Wait();
    }

    private static void Runtime_ConcurrentSequencesRespectRuntimeFloor()
    {
        const int count = 16;
        using var runtime = new AsyncRuntime(2, sequenceFloor: 900000);
        var first = new NomadCoreClient("test-key", runtime.Port);
        var second = new NomadCoreClient("test-key", runtime.Port);
        var active = first.ServoAsync(8, 1500);
        runtime.WaitForCommands(1);
        var overlaps = Enumerable.Range(0, count).Select(index => Task.Run(() =>
            (index % 2 == 0 ? first : second).ServoAsync(8, 1500))).ToArray();
        foreach (var request in overlaps)
        {
            var result = WaitResult(request);
            Expect(result.Outcome == NomadCoreRequestOutcome.NotAttempted &&
                result.ErrorCode == "request_in_progress", "identity overlap is declined locally without queueing");
        }
        Expect(runtime.AcceptedConnections == 1 && runtime.Commands.Count == 1,
            "overlaps cannot connect, allocate or overtake the delayed active request");
        runtime.ReleaseResponses();
        Expect(WaitResult(active).Succeeded, "active request succeeds against real high-water enforcement");
        Expect(WaitResult(second.ServoAsync(8, 1500)).Succeeded,
            "gate is released after result so another client can issue the next request");
        runtime.Wait();
        var sequences = runtime.Commands.Select(command => Convert.ToUInt64(command["sequence"])).ToArray();
        Expect(sequences[0] >= 900000 && sequences[1] > sequences[0],
            "accepted requests use unique increasing allocations above authenticated runtime floor");
        Expect(runtime.StaleRequests == 0 && runtime.AcceptedConnections == 2,
            "same-process overlap produces no accidental stale_request or deferred mutation retry");
    }

    private static void Runtime_RevokeInterruptsActiveMutation()
    {
        using var runtime = new AsyncRuntime(2, sequenceFloor: 900000, generation: 7);
        var client = new NomadCoreClient("test-key", runtime.Port);
        var mutation = client.ServoAsync(8, 1500);
        runtime.WaitForCommands(1);
        Expect(!mutation.IsCompleted, "vehicle mutation remains active before operator revoke");

        var revoke = WaitResult(client.RevokeAuthorityAsync());
        Expect(revoke.Succeeded, "revoke bypasses occupied vehicle gate and reaches runtime");
        runtime.WaitForCommands(2);
        Expect(runtime.AuthorityGeneration == 8, "runtime revoke advances authority generation from seven to eight");
        Expect(!mutation.IsCompleted, "revoke completes while original vehicle response is still delayed");
        var overlap = WaitResult(new NomadCoreClient("test-key", runtime.Port).ServoAsync(8, 1600));
        Expect(overlap.Outcome == NomadCoreRequestOutcome.NotAttempted && overlap.ErrorCode == "request_in_progress",
            "authority bypass does not release the active vehicle mutation gate");

        runtime.ReleaseResponse(8);
        var interrupted = WaitResult(mutation);
        Expect(interrupted.Outcome == NomadCoreRequestOutcome.Interrupted &&
            interrupted.ErrorCode == "authority_interrupted" && interrupted.Acknowledged == true,
            "original request observes runtime authority interruption despite vehicle acknowledgement");
        Expect(revoke.Succeeded, "original interruption cannot overwrite the revoke result");
        runtime.Wait();
        var commands = runtime.Commands;
        Expect(commands.Count == 2 && runtime.AcceptedConnections == 2 && runtime.StaleRequests == 0,
            "one mutation and one revoke are sent with no retry or queued overlap");
        Expect(commands[1]["type"].ToString() == "revoke_authority" &&
            Convert.ToInt32(commands[1]["authority_generation"]) == 7 &&
            Convert.ToUInt64(commands[1]["sequence"]) > Convert.ToUInt64(commands[0]["sequence"]),
            "revoke uses fresh hello context and the same increasing identity sequence allocator");
    }

    private static void Output_ConcurrentRequestsUseExactResultsAndRejectOverlap()
    {
        using var runtime = new AsyncRuntime(1);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        var payloadSuccess = OutputController.SendServoPwmAsync(1, 1500);
        runtime.WaitForCommands(1);
        var payloadOverlap = WaitResult(OutputController.SendServoPwmAsync(2, 1500));
        var gimbalOverlap = WaitResult(OutputController.SendGimbalTargetAsync(0, 0));
        Expect(payloadOverlap.Outcome == NomadCoreRequestOutcome.NotAttempted &&
            payloadOverlap.ErrorCode == "request_in_progress", "different payload channel respects identity gate");
        Expect(gimbalOverlap.Outcome == NomadCoreRequestOutcome.NotAttempted &&
            gimbalOverlap.ErrorCode == "request_in_progress", "gimbal respects shared payload identity gate");
        runtime.ReleaseResponses();
        var result = WaitResult(payloadSuccess);
        Expect(result.Succeeded && result.Message == "request-1",
            "active payload caller retains its exact result despite overlapping local failures");
        runtime.Wait();
        Expect(runtime.Commands.Count == 1 && runtime.AcceptedConnections == 1 && runtime.StaleRequests == 0,
            "overlapping production callers generate no stale IPC mutation or retry");
    }

    private static void Runtime_MockRejectsReversedSequences()
    {
        using var runtime = new AsyncRuntime(2, sequenceFloor: 1000);
        runtime.ReleaseResponses();
        var higher = SendRawSequence(runtime.Port, 1001);
        var lower = SendRawSequence(runtime.Port, 1000);
        runtime.Wait();
        Expect((bool)higher["ok"], "mock accepts the higher sequence arriving first");
        var error = (Dictionary<string, object>)lower["error"];
        Expect(!(bool)lower["ok"] && error["code"].ToString() == "stale_request" && runtime.StaleRequests == 1,
            "mock applies real high-water rule and rejects the later lower sequence");
    }

    private static Dictionary<string, object> SendRawSequence(int port, ulong sequence)
    {
        using var client = new TcpClient();
        client.Connect(IPAddress.Loopback, port);
        using var stream = client.GetStream();
        stream.ReadTimeout = 5000;
        using var reader = new StreamReader(stream, Encoding.UTF8, false, 4096, true);
        using var writer = new StreamWriter(stream, new UTF8Encoding(false), 4096, true) { AutoFlush = true };
        var serializer = new JavaScriptSerializer();
        writer.WriteLine(serializer.Serialize(new Dictionary<string, object>
            { ["id"] = "hello", ["auth_nonce"] = "test" }));
        reader.ReadLine();
        writer.WriteLine(serializer.Serialize(new Dictionary<string, object>
        {
            ["id"] = Guid.NewGuid().ToString("N"), ["type"] = "set_servo", ["channel"] = 8, ["sequence"] = sequence
        }));
        return serializer.Deserialize<Dictionary<string, object>>(reader.ReadLine());
    }

    private static void Runtime_BufferedResponsesPreserveSizeLimit()
    {
        foreach (var size in new[] { 4096, 9000, 65536, 65537 })
        {
            using var runtime = new AsyncRuntime(1) { ResponseSize = size };
            runtime.ReleaseResponses();
            var result = WaitResult(new NomadCoreClient("test-key", runtime.Port).ServoAsync(8, 1500));
            runtime.Wait();
            Expect(size <= 65536 ? result.Succeeded : result.Outcome == NomadCoreRequestOutcome.UnknownOutcome,
                "buffered response of " + size + " bytes preserves framing and 65536-byte limit");
            Expect(runtime.Commands.Count == 1, "response size failure cannot replay a mutation");
        }
    }

    private static void Runtime_FreshProcessUsesConsumedSequenceFloor()
    {
        using var runtime = new AsyncRuntime(2, sequenceFloor: 700000);
        runtime.ReleaseResponses();
        Expect(WaitResult(new NomadCoreClient("test-key", runtime.Port).ServoAsync(8, 1500)).Succeeded,
            "existing Mission Planner consumes a high runtime sequence");
        var executable = Assembly.GetExecutingAssembly().Location;
        using var child = Process.Start(new ProcessStartInfo(executable,
            "--async-restart-child " + runtime.Port.ToString(CultureInfo.InvariantCulture))
        {
            UseShellExecute = false, CreateNoWindow = true
        });
        if (!child.WaitForExit(5000))
        {
            throw new TimeoutException("Restarted Mission Planner client did not finish.");
        }
        Expect(child.ExitCode == 0, "fresh Mission Planner process immediately succeeds against live runtime");
        runtime.Wait();
        Expect(Convert.ToUInt64(runtime.Commands[1]["sequence"]) > Convert.ToUInt64(runtime.Commands[0]["sequence"]),
            "fresh process follows consumed next_sequence without process-local counter history");
        Expect(runtime.Commands.Count == 2, "restart sends each mutation exactly once");
    }

    private static void Runtime_InvalidSequenceFloorsFailBeforeMutation()
    {
        foreach (var floor in new object[] { null, 0, -1, 1.5, "invalid", "18446744073709551616" })
        {
            using var runtime = new AsyncRuntime(1)
            {
                SequenceFloorOverride = floor, OmitSequenceFloor = floor == null
            };
            var result = WaitResult(new NomadCoreClient("test-key", runtime.Port).ServoAsync(8, 1500));
            Expect(result.Outcome == NomadCoreRequestOutcome.FailedBeforeSend && result.ErrorCode == "invalid_response",
                "missing, zero, negative, fractional, malformed or overflowing next_sequence fails before mutation");
            runtime.Wait();
            Expect(runtime.Commands.Count == 0, "invalid sequence floor sends no mutation and does not retry");
        }
    }

    private static void Runtime_StaleMutationIsNotRetried()
    {
        using var runtime = new MockRuntime(1, outcome: "rejected", errorCode: "stale_request");
        var result = WaitResult(new NomadCoreClient("test-key", runtime.Port).ServoAsync(8, 1500));
        runtime.Wait();
        Expect(result.Outcome == NomadCoreRequestOutcome.Rejected && result.ErrorCode == "stale_request",
            "stale mutation preserves its exact runtime rejection");
        Expect(runtime.CommandCount == 1, "stale mutation is never automatically reallocated and retried");
    }

    private static void Output_ExplicitStopHasOneBoundedWaiter()
    {
        using var runtime = new AsyncRuntime(2);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        var start = OutputController.SendServoPwmAsync(8, 2000);
        runtime.WaitForCommands(1);
        var stop = OutputController.SendServoStopAsync(8, 1500);
        Expect(!stop.IsCompleted, "explicit stop waits asynchronously behind the in-flight channel command");
        var duplicate = WaitResult(OutputController.SendServoStopAsync(8, 1500));
        var newStart = WaitResult(OutputController.SendServoPwmAsync(8, 2000));
        Expect(duplicate.Outcome == NomadCoreRequestOutcome.NotAttempted,
            "a second stop is rejected without adding a waiter");
        Expect(newStart.Outcome == NomadCoreRequestOutcome.NotAttempted,
            "new starts are rejected while an explicit stop is pending");
        Expect(runtime.Commands.Count == 1, "pending stop and rejected overlaps send nothing before current result");
        runtime.ReleaseResponses();
        Expect(WaitResult(start).Succeeded && WaitResult(stop).Succeeded,
            "original command finishes and one explicit stop receives its exact result");
        runtime.Wait();
        Expect(runtime.Commands.Count == 2 && Convert.ToInt32(runtime.Commands[1]["pwm_microseconds"]) == 1500,
            "exactly one neutral stop follows the initial command with no stale start replay");
    }

    private static void Runtime_RestartRefreshesAuthorityBinding()
    {
        var port = ReservePort();
        var client = new NomadCoreClient("test-key", port);
        ulong previous;
        using (var runtime = new AsyncRuntime(1, port, 600000, "before-restart", 7, 9))
        {
            runtime.ReleaseResponses();
            Expect(WaitResult(client.ServoAsync(8, 1500)).Succeeded, "request before runtime restart succeeds");
            runtime.Wait();
            previous = Convert.ToUInt64(runtime.Commands[0]["sequence"]);
        }
        using (var runtime = new AsyncRuntime(1, port, 1, "after-restart", 1, 1))
        {
            runtime.ReleaseResponses();
            Expect(WaitResult(client.ServoAsync(8, 1500)).Succeeded, "live client succeeds after runtime restart");
            runtime.Wait();
            var command = runtime.Commands[0];
            Expect(command["runtime_incarnation"].ToString() == "after-restart" &&
                Convert.ToInt32(command["vehicle_session"]) == 1 &&
                Convert.ToInt32(command["authority_generation"]) == 1,
                "live client binds fresh runtime incarnation, vehicle session and authority generation");
            Expect(Convert.ToUInt64(command["sequence"]) > previous,
                "runtime restart preserves safe monotonic client allocation even when runtime floor resets");
        }
    }

    private static void Output_QueuedStopRetainsOriginalRuntimeAcrossConfigurationChange()
    {
        using var original = new AsyncRuntime(2);
        using var replacement = new AsyncRuntime(1);
        replacement.ReleaseResponses();
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = original.Port });
        var movement = OutputController.SendServoPwmAsync(8, 2000);
        original.WaitForCommands(1);
        var stop = OutputController.SendServoStopAsync(8, 1500);
        Expect(!stop.IsCompleted, "old output's explicit stop is queued behind its controlled movement request");
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = replacement.Port });
        original.ReleaseResponses();
        Expect(WaitResult(movement).Succeeded, "original movement completes with its original runtime result");
        Expect(WaitResult(stop).Succeeded, "queued stop completes using the transport captured at invocation");
        original.Wait();
        Expect(original.Commands.Count == 2 &&
            Convert.ToInt32(original.Commands[1]["pwm_microseconds"]) == 1500,
            "old output stop reaches the original runtime exactly once after configuration changes");
        Expect(replacement.AcceptedConnections == 0 && replacement.Commands.Count == 0,
            "replacement runtime receives no connection or mutation from an old queued stop");
    }

    private static void Runtime_CancellationBeforeWriteIsNotSent()
    {
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        var result = WaitResult(new NomadCoreClient("test-key", ReservePort()).ServoAsync(8, 1500, cancelled.Token));
        Expect(result.Outcome == NomadCoreRequestOutcome.FailedBeforeSend && result.Acknowledged == null,
            "pre-cancelled request fails before connection and mutation write");
        using var runtime = new AsyncRuntime(1, delayHello: true);
        using var duringHello = new CancellationTokenSource();
        var request = new NomadCoreClient("test-key", runtime.Port).ServoAsync(8, 1500, duringHello.Token);
        runtime.WaitForHello();
        var overlap = WaitResult(new NomadCoreClient("test-key", runtime.Port).GimbalTargetAsync(0, 0));
        Expect(overlap.Outcome == NomadCoreRequestOutcome.NotAttempted && overlap.ErrorCode == "request_in_progress",
            "identity gate already excludes other mutations while the first request awaits hello");
        duringHello.Cancel();
        Expect(WaitResult(request).Outcome == NomadCoreRequestOutcome.FailedBeforeSend,
            "cancellation waiting for authenticated hello fails before mutation write");
        runtime.ReleaseHello();
        runtime.Wait();
        Expect(runtime.Commands.Count == 0, "hello cancellation sends no mutation and does not retry");
    }

    private static void Runtime_CancellationAfterWriteIsUnknown()
    {
        var port = ReservePort();
        var client = new NomadCoreClient("test-key", port);
        using (var runtime = new AsyncRuntime(1, port))
        {
            using var cancellation = new CancellationTokenSource();
            var request = client.ServoAsync(8, 1500, cancellation.Token);
            runtime.WaitForCommands(1);
            cancellation.Cancel();
            Expect(WaitResult(request).Outcome == NomadCoreRequestOutcome.UnknownOutcome,
                "cancellation after observed mutation write preserves unknown outcome");
            runtime.ReleaseResponses();
            runtime.Wait();
            Expect(runtime.Commands.Count == 1 && runtime.AcceptedConnections == 1,
                "uncertain cancelled mutation has no automatic retry");
        }
        using (var runtime = new AsyncRuntime(1, port))
        {
            runtime.ReleaseResponses();
            Expect(WaitResult(client.ServoAsync(8, 1500)).Succeeded,
                "unknown cancellation releases the gate for a later explicit request");
            runtime.Wait();
        }
    }

    private static void Runtime_ResponseTimeoutAfterWriteIsUnknown()
    {
        using var runtime = new AsyncRuntime(1);
        var client = new NomadRuntimeClient(runtime.Port, "test-key", "mission-planner", 100);
        var request = client.RunAsync("servo", new[] { "8", "1500" }, CancellationToken.None);
        runtime.WaitForCommands(1);
        Expect(WaitResult(request).Outcome == NomadCoreRequestOutcome.UnknownOutcome,
            "response timeout after observed mutation write preserves unknown outcome");
        runtime.ReleaseResponses();
        runtime.Wait();
        Expect(runtime.Commands.Count == 1 && runtime.AcceptedConnections == 1,
            "timed out mutation has no automatic retry");
    }

    private static void Runtime_AsyncConnectionAndAuthenticationFailures()
    {
        var unavailable = WaitResult(new NomadCoreClient("test-key", ReservePort()).ServoAsync(8, 1500));
        Expect(unavailable.Outcome == NomadCoreRequestOutcome.FailedBeforeSend,
            "async connect failure fails before mutation write");
        using var rogue = new MockRuntime(1, rogueRuntime: true);
        var rejected = WaitResult(new NomadCoreClient("test-key", rogue.Port).ServoAsync(8, 1500));
        rogue.Wait();
        Expect(rejected.Outcome == NomadCoreRequestOutcome.FailedBeforeSend && rogue.CommandCount == 0,
            "async authentication failure sends no mutation");
        using var dropped = new MockRuntime(1, dropCommandResponse: true);
        var unknown = WaitResult(new NomadCoreClient("test-key", dropped.Port).ServoAsync(8, 1500));
        dropped.Wait();
        Expect(unknown.Outcome == NomadCoreRequestOutcome.UnknownOutcome && dropped.CommandCount == 1,
            "async socket close after mutation is unknown with no retry");
    }

    private static NomadCoreRequestResult WaitResult(Task<NomadCoreRequestResult> request)
    {
        if (!request.Wait(5000))
        {
            throw new TimeoutException("Async client request did not finish within test deadline.");
        }
        return request.GetAwaiter().GetResult();
    }

}
