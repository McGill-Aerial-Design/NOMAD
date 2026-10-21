// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using NOMAD.MissionPlanner.Connectivity;

internal static partial class NomadCoreClientTests
{
    private static int ReadActualRuntimeStatus(int port, string expectedClass)
    {
        var credential = Console.In.ReadLine();
        var result = StatusResult(new NomadCoreClient(credential, port).GetStatusAsync());
        var status = result.Snapshot;
        var connected = expectedClass == "Copter";
        var checks = new Dictionary<string, bool>
        {
            ["authenticated_snapshot"] = result.Succeeded,
            ["listener_ready"] = status?.RuntimeReady == true,
            ["protocol_support"] = status?.ProtocolCapabilities?.Contains("status") == true
                && status.ProtocolCapabilities.Contains("land"),
            ["authority_unowned"] = status?.AuthorityOwnerKnown == true && status.AuthorityOwner == null,
            ["aircraft_class"] = status?.AircraftClass == expectedClass,
            ["profile_support"] = connected ? status?.SupportedFlightOperations?.SequenceEqual(new[] { "land" }) == true
                : status != null && status.SupportedFlightOperations == null,
            ["telemetry"] = connected ? status?.IdentityResolved == true && status.HeartbeatFresh == true
                && status.LatitudeDeg.HasValue && status.LongitudeDeg.HasValue
                && Math.Abs(status.LatitudeDeg.Value - 45.5017) < 1e-7
                && Math.Abs(status.LongitudeDeg.Value + 73.5673) < 1e-7
                : status?.IdentityResolved == false && status.LatitudeDeg == null && status.LongitudeDeg == null,
            ["altitude_datums"] = connected ? status?.AbsoluteAltitudeM.HasValue == true
                && Math.Abs(status.AbsoluteAltitudeM.Value - 25) < 0.001 && status.RelativeAltitudeM == 0
                : status != null && status.AbsoluteAltitudeM == null && status.RelativeAltitudeM == null,
        };
        Console.WriteLine(new System.Web.Script.Serialization.JavaScriptSerializer().Serialize(checks));
        return checks.Values.All(value => value) ? 0 : 1;
    }

    private static NomadStatusResult StatusResult(Task<NomadStatusResult> task)
    {
        if (!task.Wait(7500))
        {
            throw new TimeoutException("STATUS request did not finish within its test deadline.");
        }
        return task.GetAwaiter().GetResult();
    }

    private static void Runtime_StatusTests()
    {
        Status_ReadsWithoutAuthorityAndPreservesDistinctFacts();
        Status_ReadsFlightRequestPhases();
        Status_InvalidFlightRequestStaysUnknown();
        Status_RejectsMismatchedEnvelopeAndIncarnation();
        Status_RejectsUnauthenticatedServer();
        Status_RejectsMalformedHelloAndDroppedResponse();
        Status_PreservesMissingAndInvalidObservations();
        Status_AcceptsFragmentedFramesAndRejectsOversize();
        Status_ReadsIndependentlyAndCancels();
        Status_OverallDeadlineCannotBeRenewed();
        Status_InvalidEndpointDoesNotSelectDefault();
        StatusPanel_Tests();
        Expect(!StatusResult(new NomadCoreClient("", ReservePort()).GetStatusAsync()).Succeeded,
            "STATUS with empty credential fails before connecting");
        Expect(StatusResult(new NomadCoreClient("test-key", -1).GetStatusAsync()).ErrorCode == "invalid_configuration",
            "STATUS with an invalid port does not query the default runtime");
    }

    private static Dictionary<string, object> FlightRequestSnapshot(string phase = "observing",
        bool acknowledged = true, string outcome = null) => new Dictionary<string, object>
    {
        ["request_id"] = "fixture-land",
        ["operation"] = "land",
        ["vehicle_session"] = 7,
        ["authority_generation"] = 9,
        ["age_ms"] = 0,
        ["phase"] = phase,
        ["acknowledged"] = acknowledged,
        ["outcome"] = outcome,
    };

    private static void Status_ReadsFlightRequestPhases()
    {
        var cases = new (string Phase, bool Ack, string Outcome)[]
        {
            ("sending", false, null), ("sending", true, null), ("observing", true, null),
            ("completed", true, "success"), ("completed", true, "failed"),
            ("completed", false, "rejected"), ("completed", false, "unknown"),
            ("completed", true, "unknown"), ("completed", false, "interrupted"),
            ("completed", true, "interrupted"),
        };
        foreach (var sample in cases)
        {
            using var runtime = new StatusRuntime(changeResponse: response =>
                ((Dictionary<string, object>)response["status"])["last_flight_request"] =
                    FlightRequestSnapshot(sample.Phase, sample.Ack, sample.Outcome));
            var status = StatusResult(new NomadCoreClient("test-key", runtime.Port).GetStatusAsync()).Snapshot;
            var request = status?.LastFlightRequest;
            Expect(request?.RequestId == "fixture-land" && request.Operation == "land"
                && request.VehicleSession == 7 && request.AuthorityGeneration == 9 && request.AgeMilliseconds == 0,
                "flight request projection retains its original context and observed zero age");
            Expect(request?.Phase == sample.Phase && request.Acknowledged == sample.Ack
                && request.Outcome == sample.Outcome, "flight request phase, ACK and outcome remain distinct");
            Expect(status?.ActuationEnabled == false && status.AuthorityOwner == null
                && runtime.Requests().Length == 2 && runtime.Requests()[1]["type"].Equals("status"),
                "flight request history grants no authority and dispatches no mutation");
        }
    }

    private static void Status_InvalidFlightRequestStaysUnknown()
    {
        var changes = new Action<Dictionary<string, object>>[]
        {
            value => value["request_id"] = "", value => value["request_id"] = "bad\nrequest",
            value => value["operation"] = "takeoff", value => value["vehicle_session"] = 0,
            value => value["authority_generation"] = -1, value => value["age_ms"] = -1,
            value => value["age_ms"] = 0.5, value => value["acknowledged"] = "yes",
            value => value["phase"] = "running", value => value["outcome"] = "success",
            value => value.Remove("outcome"), value => value["acknowledged"] = false,
            value => value["phase"] = "completed",
            value =>
            {
                value["phase"] = "completed";
                value["outcome"] = "success";
                value["acknowledged"] = false;
            },
            value =>
            {
                value["phase"] = "completed";
                value["outcome"] = "failed";
                value["acknowledged"] = false;
            },
            value =>
            {
                value["phase"] = "completed";
                value["outcome"] = "rejected";
            },
        };
        foreach (var change in changes)
        {
            using var runtime = new StatusRuntime(changeResponse: response =>
            {
                var request = FlightRequestSnapshot();
                change(request);
                ((Dictionary<string, object>)response["status"])["last_flight_request"] = request;
            });
            var result = StatusResult(new NomadCoreClient("test-key", runtime.Port).GetStatusAsync());
            Expect(result.Succeeded && result.Snapshot.LastFlightRequest == null,
                "malformed optional flight request data stays unknown without hiding other status facts");
        }
    }

    private static void Status_InvalidEndpointDoesNotSelectDefault()
    {
        foreach (var port in new[] { 1, 65535 })
        {
            var client = new NomadCoreClient("test-key", port);
            Expect(client.RuntimePort == port && client.IsConfigured,
                "both valid endpoint boundaries retain their configuration without connecting");
        }
        foreach (var port in new[] { -1, 0, 65536 })
        {
            var client = new NomadCoreClient("test-key", port);
            Expect(client.RuntimePort == port, "invalid endpoint is not silently rewritten to the default runtime");
            // Avoid contacting an operator's default runtime on the known-bad baseline.
            if (client.RuntimePort != port)
            {
                continue;
            }
            using var cancelled = new CancellationTokenSource();
            cancelled.Cancel();
            var mutation = WaitResult(client.ServoAsync(8, 1500, cancelled.Token));
            var authority = WaitResult(client.AdmitAuthorityAsync(cancelled.Token));
            Expect(mutation.Outcome == NomadCoreRequestOutcome.NotAttempted
                && mutation.ErrorCode == "invalid_configuration", "invalid endpoint rejects a mutation locally");
            Expect(authority.Outcome == NomadCoreRequestOutcome.NotAttempted
                && authority.ErrorCode == "invalid_configuration", "invalid endpoint cannot acquire authority");
        }
    }

    private static void Status_ReadsWithoutAuthorityAndPreservesDistinctFacts()
    {
        using var runtime = new StatusRuntime();
        var result = StatusResult(new NomadCoreClient("test-key", runtime.Port).GetStatusAsync());
        var status = result.Snapshot;
        Expect(result.Succeeded && status != null, "authenticated STATUS works without HELLO authority/sequence");
        if (status == null)
        {
            return;
        }
        Expect(status.RuntimeReady == true && status.AuditHealthy == false && status.ActuationEnabled == false,
            "listener readiness does not imply audit or actuation readiness");
        Expect(status.LastFlightRequest == null, "older runtime without flight request data remains readable");
        Expect(status.ProtocolCapabilities.Count == 2 && status.SupportedFlightOperations.Count == 0,
            "protocol LAND advertisement does not create QuadPlane operation support");
        Expect(status.VehicleSession == 7 && status.AuthorityGeneration == 9 && status.AuthorityOwnerKnown
            && status.AuthorityOwner == null, "STATUS retains session, generation and explicit unowned authority");
        Expect(status.AircraftClass == "QuadPlane" && status.Armed == true && status.CustomMode == 10,
            "STATUS preserves reported aircraft identity and numeric mode");
        Expect(status.LatitudeDeg == 0 && status.LongitudeDeg == 0
            && status.AbsoluteAltitudeM == 123.5 && status.RelativeAltitudeM == 23.5,
            "legal zero coordinates and distinct MSL/home-relative altitudes survive parsing");
        Expect(status.VtolState == "fixed_wing" && status.VtolStateAgeMs == 60
            && status.LandedState == "in_air" && status.LandedStateAgeMs == 70, "VTOL/landed values retain ages");
        var requests = runtime.Requests();
        Expect(requests.Length == 2 && (string)requests[0]["type"] == "hello"
            && (string)requests[1]["type"] == "status",
            "STATUS sends exactly HELLO and STATUS, with no authority acquisition or mutation");
        Expect(!requests[1].ContainsKey("sequence") && !requests[1].ContainsKey("credential")
            && !requests[1].ContainsKey("authority_generation"),
                "STATUS consumes no mutation sequence or reusable credential");
        Expect(!runtime.HasExtraConnection, "STATUS does not open a retry connection");
    }

    private static void Status_RejectsMismatchedEnvelopeAndIncarnation()
    {
        var corruptions = new Action<Dictionary<string, object>>[]
        {
            response => response["id"] = "wrong-request",
            response => response["protocol"] = "wrong-protocol",
            response => response["version"] = 2,
            response => response["type"] = "command_response",
            response => ((Dictionary<string, object>)response["status"])["runtime_incarnation"] = "other-runtime",
            response => response["status"] = new object[0],
        };
        foreach (var corrupt in corruptions)
        {
            using var runtime = new StatusRuntime(changeResponse: corrupt);
            var result = StatusResult(new NomadCoreClient("test-key", runtime.Port).GetStatusAsync());
            Expect(!result.Succeeded && result.Snapshot == null && result.ErrorCode == "invalid_response",
                "mismatched STATUS envelope/incarnation cannot publish a snapshot");
            Expect(runtime.Requests().Length == 2 && !runtime.HasExtraConnection, "invalid STATUS is never retried");
        }
    }

    private static void Status_RejectsUnauthenticatedServer()
    {
        using var runtime = new StatusRuntime(changeHello: hello => hello["server_proof"] = new string('0', 64));
        var result = StatusResult(new NomadCoreClient("test-key", runtime.Port).GetStatusAsync());
        Expect(!result.Succeeded && result.ErrorCode == "authentication_required",
            "STATUS rejects a wrong server proof");
        Expect(runtime.Requests().Length == 1, "untrusted HELLO never receives a STATUS request or credential");
    }

    private static void Status_RejectsMalformedHelloAndDroppedResponse()
    {
        foreach (var version in new object[] { "1", true, new Dictionary<string, object>() })
        {
            using var runtime = new StatusRuntime(changeHello: hello => hello["version"] = version);
            var result = StatusResult(new NomadCoreClient("test-key", runtime.Port).GetStatusAsync());
            Expect(!result.Succeeded && result.ErrorCode == "invalid_response" && runtime.Requests().Length == 1,
                "malformed authenticated HELLO never receives STATUS or authority requests");
        }
        using (var runtime = new StatusRuntime(writeStatus: (stream, bytes) => stream.Close()))
        {
            var result = StatusResult(new NomadCoreClient("test-key", runtime.Port).GetStatusAsync());
            Expect(!result.Succeeded && result.ErrorCode == "runtime_unavailable" && !runtime.HasExtraConnection,
                "dropped STATUS response remains unavailable without a retry");
        }
        using (var runtime = new StatusRuntime(writeStatus: (stream, bytes) =>
        {
            var invalid = Encoding.UTF8.GetBytes("[]\n");
            stream.Write(invalid, 0, invalid.Length);
        }))
        {
            var result = StatusResult(new NomadCoreClient("test-key", runtime.Port).GetStatusAsync());
            Expect(result.ErrorCode == "invalid_response",
                "a non-object STATUS frame cannot become an observation");
        }
    }

    private static void Status_PreservesMissingAndInvalidObservations()
    {
        using (var runtime = new StatusRuntime(changeResponse: response => response["status"] =
            new Dictionary<string, object> { ["runtime_incarnation"] = "status-runtime" }))
        {
            var status = StatusResult(new NomadCoreClient("test-key", runtime.Port).GetStatusAsync()).Snapshot;
            Expect(status != null && status.RuntimeReady == null && status.HeartbeatFresh == null
                && status.VehicleSession == null && status.SupportedFlightOperations == null
                && !status.AuthorityOwnerKnown,
                "missing STATUS facts remain unknown rather than false, zero or unsupported");
        }
        using (var runtime = new StatusRuntime(changeResponse: response =>
        {
            var telemetry = (Dictionary<string, object>)((Dictionary<string, object>)response["status"])["telemetry"];
            telemetry["vtol_state"] = "undefined";
            telemetry["vtol_state_valid"] = false;
        }))
        {
            var status = StatusResult(new NomadCoreClient("test-key", runtime.Port).GetStatusAsync()).Snapshot;
            Expect(status != null && status.VtolState == "undefined" && status.VtolStateValid == false,
                "undefined VTOL remains a reported enum with a separate invalid flag");
        }
        using (var runtime = new StatusRuntime(changeResponse: response =>
        {
            var status = (Dictionary<string, object>)response["status"];
            status["runtime_ready"] = "true";
            status["vehicle_session"] = -1;
            status["custom_mode"] = 4294967296m;
            status["supported_flight_operations"] = new[] { "land", "land" };
            status["authority_owner"] = "bad\nowner";
            var telemetry = (Dictionary<string, object>)status["telemetry"];
            telemetry["heartbeat_age_ms"] = -1;
            telemetry["vtol_state"] = "invented-state";
            ((Dictionary<string, object>)telemetry["position"])["latitude_deg"] = 91;
        }))
        {
            var status = StatusResult(new NomadCoreClient("test-key", runtime.Port).GetStatusAsync()).Snapshot;
            Expect(status != null && status.RuntimeReady == null && status.VehicleSession == null
                && status.CustomMode == null && status.HeartbeatAgeMs == null && status.VtolState == null,
                "wrong types, negative ages, overflow and unknown enums stay unknown");
            Expect(status.SupportedFlightOperations == null && !status.AuthorityOwnerKnown
                && status.LatitudeDeg == null && status.LongitudeDeg == null,
                    "invalid collections/coordinates are not partial facts");
        }
    }

    private static void Status_AcceptsFragmentedFramesAndRejectsOversize()
    {
        using (var runtime = new StatusRuntime(writeStatus: (stream, bytes) =>
        {
            foreach (var value in bytes)
            {
                stream.WriteByte(value);
            }
        }))
        {
            Expect(StatusResult(new NomadCoreClient("test-key", runtime.Port).GetStatusAsync()).Succeeded,
                "fragmented STATUS preserves buffered framing");
        }
        using (var runtime = new StatusRuntime(
            changeResponse: response => response["padding"] = new string('x', 70000)))
        {
            var result = StatusResult(new NomadCoreClient("test-key", runtime.Port).GetStatusAsync());
            Expect(!result.Succeeded && result.ErrorCode == "invalid_response",
                "STATUS retains the 65536-byte frame bound");
        }
    }

    private static void Status_ReadsIndependentlyAndCancels()
    {
        using (var runtime = new StatusRuntime(count: 2, hold: true))
        {
            var client = new NomadCoreClient("test-key", runtime.Port);
            var first = client.GetStatusAsync();
            var second = client.GetStatusAsync();
            Expect(runtime.WaitForStatuses(2),
                "overlapping STATUS reads reach the server without mutation serialization");
            runtime.Release();
            Expect(StatusResult(first).Succeeded && StatusResult(second).Succeeded, "each STATUS owns its response");
        }
        using (var runtime = new StatusRuntime(hold: true))
        using (var cancel = new CancellationTokenSource())
        {
            var task = new NomadCoreClient("test-key", runtime.Port).GetStatusAsync(cancel.Token);
            Expect(runtime.WaitForStatuses(1), "cancellation starts from an observed STATUS request");
            cancel.Cancel();
            Expect(StatusResult(task).ErrorCode == "cancelled" && runtime.ClientClosed(),
                "STATUS cancellation closes the actual owned socket");
        }
    }

    private static void Status_OverallDeadlineCannotBeRenewed()
    {
        foreach (var duringHello in new[] { true, false })
        {
            StatusRuntime runtime = null;
            using (runtime = new StatusRuntime(dripHello: duringHello, writeStatus: (stream, bytes) =>
            {
                foreach (var value in bytes)
                {
                    stream.WriteByte(value);
                    Thread.Sleep(100);
                }
            }))
            {
                var watch = Stopwatch.StartNew();
                var result = StatusResult(new NomadCoreClient("test-key", runtime.Port).GetStatusAsync());
                Expect(result.ErrorCode == "status_timeout" && watch.ElapsedMilliseconds >= 4500
                    && watch.ElapsedMilliseconds < 7500,
                        "overall STATUS deadline covers dripping HELLO and STATUS frames");
                Expect(runtime.Requests().Length == (duringHello ? 1 : 2),
                    "deadline never retries or dispatches a mutation");
            }
        }
    }
}
