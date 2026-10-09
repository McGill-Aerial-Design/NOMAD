// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Threading;
using NOMAD.MissionPlanner;
using NOMAD.MissionPlanner.Connectivity;

internal static partial class NomadCoreClientTests
{
    private static readonly object[] LandCapabilities = { "status", "land" };

    private static void Runtime_LandTests()
    {
        Runtime_LandRejectsArgumentsAndMissingCredential();
        Runtime_LandRequiresExplicitAuthority();
        Runtime_LandRejectsMissingOrMalformedCapabilities();
        Runtime_LandRefreshesCapabilitiesAfterRestart();
        Runtime_LandRejectsUnauthenticatedCapability();
        Runtime_LandPreservesDefiniteAndUncertainOutcomes();
        Runtime_LandPreservesErrorEvidence();
        Runtime_LandRejectsMalformedResults();
        Runtime_LandNeverReplaysUnknownOutcome();
        LandPanel_Tests();
    }

    private static void Runtime_LandRejectsArgumentsAndMissingCredential()
    {
        var runtimeClient = new NomadRuntimeClient(ReservePort(), "test-key", "mission-planner");
        var invalid = WaitResult(runtimeClient.RunAsync("land", new[] { "10" }, CancellationToken.None));
        Expect(invalid.Outcome == NomadCoreRequestOutcome.NotAttempted &&
            invalid.ErrorCode == "unsupported_request", "LAND accepts zero operation arguments only");
        var missing = WaitResult(new NomadCoreClient("", ReservePort()).LandAsync());
        Expect(missing.Outcome == NomadCoreRequestOutcome.NotAttempted && missing.ErrorCode == "missing_credential",
            "LAND without credential is rejected locally before any connection");
    }

    private static void Runtime_LandRequiresExplicitAuthority()
    {
        using var runtime = new MockRuntime(3, enforceAuthority: true, capabilities: LandCapabilities);
        var client = new NomadCoreClient("test-key", runtime.Port);
        var unowned = WaitResult(client.LandAsync());
        Expect(unowned.Outcome == NomadCoreRequestOutcome.Rejected && unowned.ErrorCode == "not_authoritative",
            "LAND never automatically admits an unowned client");
        Expect(WaitResult(client.AdmitAuthorityAsync()).Succeeded, "operator explicitly admits LAND client");
        var engaged = WaitResult(client.LandAsync());
        runtime.Wait();
        Expect(engaged.Succeeded && engaged.Acknowledged == true, "admitted LAND requires positive ACK and success");
        Expect(runtime.CommandCount == 3 && runtime.Commands[0]["type"].ToString() == "land" &&
            runtime.Commands[1]["type"].ToString() == "admit_authority" &&
            runtime.Commands[2]["type"].ToString() == "land",
            "LAND is one typed request with explicit admission between");
        Expect(runtime.LastCommand.Count == 13 && !runtime.LastCommand.ContainsKey("mode"),
            "LAND request contains only protocol, authentication and authority metadata");
        var payload = runtime.LastCommand["auth_payload"].ToString();
        var proof = MockProof("test-key", "nomad-core:request:v1:" + payload);
        Expect(runtime.LastCommand["auth_proof"].ToString() == proof,
            "LAND mutation carries the existing authenticated request proof");
    }

    private static void Runtime_LandRejectsMissingOrMalformedCapabilities()
    {
        var cases = new object[]
        {
            null, new object[0], new object[] { "status" }, "land", new object[] { "LAND" },
            new object[] { 1, "land" }, new object[] { null, "land" },
            new Dictionary<string, object> { ["land"] = true }
        };
        foreach (var capabilities in cases)
        {
            using var runtime = new MockRuntime(1, capabilities: capabilities);
            var result = WaitResult(new NomadCoreClient("test-key", runtime.Port).LandAsync());
            runtime.Wait();
            Expect(result.Outcome == NomadCoreRequestOutcome.FailedBeforeSend &&
                result.ErrorCode == "unsupported_request", "absent or malformed LAND capability fails before mutation");
            Expect(runtime.CommandCount == 0, "capability refusal sends no fallback, admission or LAND request");
        }
    }

    private static void Runtime_LandRefreshesCapabilitiesAfterRestart()
    {
        var port = ReservePort();
        var client = new NomadCoreClient("test-key", port);
        using (var runtime = new MockRuntime(1, port: port, capabilities: LandCapabilities))
        {
            Expect(WaitResult(client.LandAsync()).Succeeded, "first runtime advertises LAND");
            runtime.Wait();
        }
        using (var runtime = new MockRuntime(1, port: port))
        {
            var result = WaitResult(client.LandAsync());
            runtime.Wait();
            Expect(result.Outcome == NomadCoreRequestOutcome.FailedBeforeSend && runtime.CommandCount == 0,
                "reconnected client uses fresh HELLO capabilities rather than cached LAND support");
        }
    }

    private static void Runtime_LandRejectsUnauthenticatedCapability()
    {
        using var runtime = new MockRuntime(1, rogueRuntime: true, capabilities: LandCapabilities);
        var result = WaitResult(new NomadCoreClient("test-key", runtime.Port).LandAsync());
        runtime.Wait();
        Expect(result.Outcome == NomadCoreRequestOutcome.FailedBeforeSend &&
            result.ErrorCode == "authentication_required" && runtime.CommandCount == 0,
            "an advertised LAND capability cannot bypass fresh server proof verification");
    }

    private static void Runtime_LandPreservesDefiniteAndUncertainOutcomes()
    {
        var outcomes = new[] { "success", "rejected", "failed", "interrupted", "unknown" };
        var expected = new[] { NomadCoreRequestOutcome.Succeeded, NomadCoreRequestOutcome.Rejected,
            NomadCoreRequestOutcome.Failed, NomadCoreRequestOutcome.Interrupted,
            NomadCoreRequestOutcome.UnknownOutcome };
        for (var index = 0; index < outcomes.Length; index++)
        {
            using var runtime = new MockRuntime(1, outcome: outcomes[index],
                acknowledged: outcomes[index] != "rejected",
                capabilities: LandCapabilities);
            var result = WaitResult(new NomadCoreClient("test-key", runtime.Port).LandAsync());
            runtime.Wait();
            Expect(result.Outcome == expected[index], "LAND preserves explicit " + outcomes[index] + " outcome");
            Expect(runtime.CommandCount == 1, "LAND " + outcomes[index] + " outcome produces no automatic retry");
        }
    }

    private static Dictionary<string, object> LandResponse(Dictionary<string, object> request)
    {
        return new Dictionary<string, object>
        {
            ["protocol"] = "nomad-core", ["version"] = 1, ["id"] = request["id"], ["ok"] = true,
            ["type"] = "command_response", ["outcome"] = "success",
            ["command_result"] = new Dictionary<string, object>
            {
                ["success"] = true, ["acknowledged"] = true, ["message"] = "LAND mode observed; touchdown not verified."
            }
        };
    }

    private static void Runtime_LandPreservesErrorEvidence()
    {
        var outcomes = new[] { "rejected", "interrupted", "unknown" };
        var expected = new[] { NomadCoreRequestOutcome.Rejected, NomadCoreRequestOutcome.Interrupted,
            NomadCoreRequestOutcome.UnknownOutcome };
        for (var index = 0; index < outcomes.Length; index++)
        {
            using var runtime = new MockRuntime(1, capabilities: LandCapabilities, outcome: outcomes[index],
                errorCode: "runtime_fault", includeErrorResult: index != 0, resultSuccess: index == 2);
            var result = WaitResult(new NomadCoreClient("test-key", runtime.Port).LandAsync());
            runtime.Wait();
            Expect(result.Outcome == expected[index] && result.ErrorCode == "runtime_fault",
                "LAND preserves runtime error disposition " + outcomes[index]);
            Expect(result.Acknowledged == (index == 0 ? (bool?)null : true),
                "LAND preserves error ACK evidence separately from disposition " + outcomes[index]);
            Expect(runtime.CommandCount == 1, "LAND runtime error never causes admission or replay");
        }
        foreach (var outcome in new[] { "unknown", "interrupted" })
        {
            using var runtime = new MockRuntime(1, capabilities: LandCapabilities, outcome: outcome,
                acknowledged: false);
            var result = WaitResult(new NomadCoreClient("test-key", runtime.Port).LandAsync());
            runtime.Wait();
            Expect(!result.Succeeded && result.Acknowledged == false && runtime.CommandCount == 1,
                "LAND preserves absent ACK for " + outcome + " without claiming engagement or retrying");
        }
    }

    private static void Runtime_LandRejectsMalformedResults()
    {
        var changes = new Action<Dictionary<string, object>>[]
        {
            response => response.Remove("outcome"), response => response["outcome"] = 1,
            response => response["outcome"] = "unknown", response => response["version"] = "1",
            response => response["ok"] = "true", response => response["id"] = "another-request",
            response => response["type"] = "status_response", response => response.Remove("command_result"),
            response => ((Dictionary<string, object>)response["command_result"])["success"] = false,
            response => ((Dictionary<string, object>)response["command_result"])["acknowledged"] = false,
            response => ((Dictionary<string, object>)response["command_result"]).Remove("acknowledged"),
            response => ((Dictionary<string, object>)response["command_result"])["message"] = 1,
            response => response["ok"] = false,
            response =>
            {
                response["ok"] = false;
                response["outcome"] = "rejected";
                response["error"] = new Dictionary<string, object>
                {
                    ["code"] = "rejected", ["message"] = "Contradictory ACK"
                };
            }
        };
        foreach (var change in changes)
        {
            using var runtime = new MockRuntime(1, capabilities: LandCapabilities, semanticResponse: request =>
            {
                var response = LandResponse(request);
                change(response);
                return response;
            });
            var result = WaitResult(new NomadCoreClient("test-key", runtime.Port).LandAsync());
            runtime.Wait();
            Expect(result.Outcome == NomadCoreRequestOutcome.UnknownOutcome && result.ErrorCode == "unknown_outcome",
                "malformed or contradictory LAND response cannot claim engagement");
            Expect(runtime.CommandCount == 1, "malformed LAND response never triggers replay");
        }
    }

    private static void Runtime_LandNeverReplaysUnknownOutcome()
    {
        using var runtime = new MockRuntime(1, dropCommandResponse: true, capabilities: LandCapabilities);
        var result = WaitResult(new NomadCoreClient("test-key", runtime.Port).LandAsync());
        runtime.Wait();
        Expect(result.Outcome == NomadCoreRequestOutcome.UnknownOutcome && runtime.CommandCount == 1,
            "connection loss after LAND send remains unknown without replay");
        Expect(OutputController.DescribeFailure("LAND engagement", result).Contains("Do not retry blindly"),
            "unknown LAND result uses the established no-retry operator wording");
    }
}
