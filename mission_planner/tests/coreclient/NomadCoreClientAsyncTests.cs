// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Diagnostics;
using System.Globalization;
using System.Linq;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
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
        Runtime_ConcurrentSequencesRespectRuntimeFloor();
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
        using var runtime = new AsyncRuntime(3);
        var first = new NomadCoreClient("test-key", runtime.Port);
        var second = new NomadCoreClient("test-key", runtime.Port);
        var success = first.ServoAsync(1, 1500);
        runtime.WaitForCommands(1);
        var rejected = first.ServoAsync(2, 1500);
        runtime.WaitForCommands(2);
        var interrupted = second.ServoAsync(3, 1500);
        runtime.WaitForCommands(3);

        runtime.ReleaseResponse(3);
        var interruptedResult = WaitResult(interrupted);
        Expect(!success.IsCompleted && !rejected.IsCompleted, "third request completes before earlier requests");
        runtime.ReleaseResponse(2);
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
        using var runtime = new AsyncRuntime(count, sequenceFloor: 900000);
        var first = new NomadCoreClient("test-key", runtime.Port);
        var second = new NomadCoreClient("test-key", runtime.Port);
        var requests = Enumerable.Range(0, count)
            .Select(index => (index % 2 == 0 ? first : second).ServoAsync(8, 1500)).ToArray();
        runtime.WaitForCommands(count);
        runtime.ReleaseResponses();
        foreach (var request in requests)
        {
            Expect(WaitResult(request).Succeeded, "concurrent client request receives software success");
        }
        runtime.Wait();
        var sequences = runtime.Commands.Select(command => Convert.ToUInt64(command["sequence"])).ToArray();
        Expect(sequences.Distinct().Count() == count, "same identity has no duplicate concurrent sequence allocations");
        Expect(sequences.All(sequence => sequence >= 900000), "no allocation regresses below hello next_sequence");
        Expect(sequences.Max() - sequences.Min() == count - 1, "concurrent allocations form one shared sequence range");
        Expect(runtime.Commands.Select(command => command["id"].ToString()).Distinct().Count() == count,
            "concurrent requests retain independent request IDs");
    }

    private static void Output_ConcurrentRequestsUseExactResultsAndRejectOverlap()
    {
        using var runtime = new AsyncRuntime(3);
        OutputController.Initialize(new NOMADConfig { CoreRuntimePort = runtime.Port });
        var payloadSuccess = OutputController.SendServoPwmAsync(1, 1500);
        runtime.WaitForCommands(1);
        var payloadRejected = OutputController.SendServoPwmAsync(2, 1500);
        runtime.WaitForCommands(2);
        var gimbalInterrupted = OutputController.SendGimbalTargetAsync(0, 0);
        runtime.WaitForCommands(3);
        var payloadOverlap = WaitResult(OutputController.SendServoPwmAsync(1, 1600));
        var gimbalOverlap = WaitResult(OutputController.SendGimbalTargetAsync(10, 0));
        Expect(payloadOverlap.Outcome == NomadCoreRequestOutcome.NotAttempted &&
            payloadOverlap.ErrorCode == "request_in_progress", "overlapping payload input is rejected without queueing");
        Expect(gimbalOverlap.Outcome == NomadCoreRequestOutcome.NotAttempted &&
            gimbalOverlap.ErrorCode == "request_in_progress", "overlapping gimbal input is rejected without queueing");
        runtime.ReleaseResponse(3);
        var gimbalResult = WaitResult(gimbalInterrupted);
        runtime.ReleaseResponse(2);
        var rejectedResult = WaitResult(payloadRejected);
        runtime.ReleaseResponse(1);
        var successResult = WaitResult(payloadSuccess);
        Expect(successResult.Succeeded && successResult.Message == "request-1",
            "payload caller uses success from its exact request despite later competing failure");
        Expect(rejectedResult.Outcome == NomadCoreRequestOutcome.Rejected && rejectedResult.Message == "request-2",
            "concurrent payload caller receives its exact rejection");
        Expect(gimbalResult.Outcome == NomadCoreRequestOutcome.Interrupted && gimbalResult.Message == "request-3",
            "concurrent gimbal caller receives its exact interruption");
        runtime.Wait();
        Expect(runtime.Commands.Count == 3 && runtime.AcceptedConnections == 3,
            "rejected overlaps produce no extra IPC mutation or delayed retry");
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
        duringHello.Cancel();
        Expect(WaitResult(request).Outcome == NomadCoreRequestOutcome.FailedBeforeSend,
            "cancellation waiting for authenticated hello fails before mutation write");
        runtime.ReleaseHello();
        runtime.Wait();
        Expect(runtime.Commands.Count == 0, "hello cancellation sends no mutation and does not retry");
    }

    private static void Runtime_CancellationAfterWriteIsUnknown()
    {
        using var runtime = new AsyncRuntime(1);
        using var cancellation = new CancellationTokenSource();
        var request = new NomadCoreClient("test-key", runtime.Port).ServoAsync(8, 1500, cancellation.Token);
        runtime.WaitForCommands(1);
        cancellation.Cancel();
        Expect(WaitResult(request).Outcome == NomadCoreRequestOutcome.UnknownOutcome,
            "cancellation after observed mutation write preserves unknown outcome");
        runtime.ReleaseResponses();
        runtime.Wait();
        Expect(runtime.Commands.Count == 1 && runtime.AcceptedConnections == 1,
            "uncertain cancelled mutation has no automatic retry");
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
