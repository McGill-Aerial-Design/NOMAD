// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Security.Cryptography;
using System.Threading;
using System.Web.Script.Serialization;
using NOMAD.MissionPlanner.Connectivity;

internal static partial class NomadCoreClientTests
{
    private static void Runtime_SendsTypedRequestOnce()
    {
        using var runtime = new MockRuntime(2);
        var client = new NomadCoreClient("test-key", runtime.Port);

        Expect(client.Servo(8, 1500), "runtime sends a typed servo request");
        Expect(client.SetRelay(3, true), "runtime sends a typed relay request");
        runtime.Wait();
        Expect(runtime.CommandCount == 2, "each action was sent once to the runtime");
        Expect(runtime.LastCommandType == "set_relay", "relay maps to its semantic protocol type");
        Expect(Convert.ToString(runtime.LastCommand["runtime_incarnation"], CultureInfo.InvariantCulture) ==
            "mock-runtime-incarnation", "mutation binds the runtime incarnation");
        Expect(Convert.ToInt32(runtime.LastCommand["vehicle_session"], CultureInfo.InvariantCulture) == 1,
            "mutation binds the aircraft session");
        Expect(Convert.ToInt32(runtime.LastCommand["authority_generation"], CultureInfo.InvariantCulture) == 1,
            "mutation binds the authority generation");
        Expect(runtime.LastCommand.ContainsKey("expires_at_ms"), "mutation has a bounded validity deadline");
        Expect(!runtime.LastCommand.ContainsKey("credential"), "client never sends the reusable credential");
        Expect(Convert.ToString(runtime.LastCommand["auth_proof"], CultureInfo.InvariantCulture) ==
            MockProof("test-key", "nomad-core:request:v1:" + runtime.LastCommand["auth_payload"]),
            "client proof binds exact request bytes");
        Expect(Convert.ToString(runtime.LastCommand["client_id"], CultureInfo.InvariantCulture) == "mission-planner",
            "credential uses a stable configured identity");
        Expect(client.LastOutcome == NomadCoreRequestOutcome.Succeeded, "structured success is reported");
    }

    private static void Runtime_GimbalTarget_UsesTypedRequestAndRequiresAuthority()
    {
        using var runtime = new MockRuntime(3, enforceAuthority: true);
        var client = new NomadCoreClient("test-key", runtime.Port);

        Expect(!client.GimbalTarget(-20.5, 12.25), "gimbal target fails closed before authority admission");
        Expect(client.LastErrorCode == "not_authoritative", "gimbal target reports missing authority");
        Expect(client.AdmitAuthority(), "operator explicitly admits gimbal client");
        Expect(client.GimbalTarget(-20.5, 12.25), "admitted gimbal target succeeds through runtime");
        runtime.Wait();

        Expect(runtime.CommandCount == 3, "denied target, admission and accepted target are each sent once");
        Expect(runtime.LastCommandType == "set_gimbal_target", "angles use their typed request name");
        Expect(Math.Abs(Convert.ToDouble(runtime.LastCommand["pitch_deg"], CultureInfo.InvariantCulture) + 20.5) < 1e-9,
            "request preserves pitch in degrees");
        Expect(Math.Abs(Convert.ToDouble(runtime.LastCommand["roll_deg"], CultureInfo.InvariantCulture) - 12.25) < 1e-9,
            "request preserves roll in degrees");
        Expect(runtime.LastCommand.ContainsKey("sequence"), "gimbal target binds a request sequence");
        Expect(runtime.LastCommand.ContainsKey("expires_at_ms"), "gimbal target has a bounded validity deadline");
    }

    private static void Runtime_GimbalTargetDoesNotReplayUnknownOutcome()
    {
        using var runtime = new MockRuntime(1, dropCommandResponse: true);
        var client = new NomadCoreClient("test-key", runtime.Port);

        Expect(!client.GimbalTarget(0.0, 0.0), "missing gimbal response is not reported as success");
        runtime.Wait();
        Expect(client.LastOutcome == NomadCoreRequestOutcome.UnknownOutcome,
            "gimbal request reports an unknown vehicle outcome");
        Expect(runtime.CommandCount == 1, "unknown gimbal outcome is not replayed");
        Expect(runtime.LastCommandType == "set_gimbal_target", "unknown result belongs to the typed gimbal request");
    }

    private static void Runtime_ReportsUnknownOutcomeWithoutReplay()
    {
        using var runtime = new MockRuntime(1, dropCommandResponse: true);
        var client = new NomadCoreClient("test-key", runtime.Port);

        Expect(!client.Servo(8, 1500), "missing command response reports failure to the Boolean caller");
        runtime.Wait();
        Expect(client.LastOutcome == NomadCoreRequestOutcome.UnknownOutcome,
            "disconnect after send is reported as an unknown vehicle outcome");
        Expect(runtime.CommandCount == 1, "the client did not replay the mutating request");
    }

    private static void Runtime_RejectsIncompatibleHelloBeforeCommand()
    {
        using var runtime = new MockRuntime(1, helloVersion: 2);
        var client = new NomadCoreClient("test-key", runtime.Port);

        Expect(!client.Servo(8, 1500), "incompatible runtime is rejected");
        runtime.Wait();
        Expect(client.LastOutcome == NomadCoreRequestOutcome.FailedBeforeSend,
            "incompatible protocol fails before command send");
        Expect(client.LastErrorCode == "incompatible_version", "version mismatch is explicit");
        Expect(runtime.CommandCount == 0, "no command is sent before successful negotiation");
    }

    private static void Runtime_RejectsIncompatibleCommandResponseAsUnknown()
    {
        using var runtime = new MockRuntime(1, commandResponseVersion: 2);
        var client = new NomadCoreClient("test-key", runtime.Port);

        Expect(!client.Servo(8, 1500), "incompatible command response is not reported as success");
        runtime.Wait();
        Expect(client.LastOutcome == NomadCoreRequestOutcome.UnknownOutcome,
            "incompatible command response leaves the vehicle outcome unknown");
        Expect(runtime.CommandCount == 1, "the client did not replay after an incompatible response");
    }

    private static void Runtime_ReconnectsForNextRequest()
    {
        var port = ReservePort();
        var client = new NomadCoreClient("test-key", port);
        using (var firstRuntime = new MockRuntime(1, port: port))
        {
            Expect(client.Servo(8, 1500), "first runtime request succeeds");
            firstRuntime.Wait();
        }
        using (var restartedRuntime = new MockRuntime(1, port: port))
        {
            Expect(client.SetRelay(3, true), "same client reconnects after runtime restart");
            restartedRuntime.Wait();
            Expect(restartedRuntime.LastCommandType == "set_relay", "reconnected client sends typed relay request");
        }
    }

    private static void Runtime_RequiresExplicitOwnershipAcrossClients()
    {
        using var runtime = new MockRuntime(8, enforceAuthority: true);
        var first = new NomadCoreClient("test-key", runtime.Port);
        var second = new NomadCoreClient("test-key", runtime.Port);

        Expect(!first.Servo(8, 1500), "Mission Planner starts without command authority");
        Expect(first.LastErrorCode == "not_authoritative", "startup rejection names the missing owner");
        Expect(first.AdmitAuthority(), "operator explicitly admits Mission Planner");
        Expect(first.Servo(8, 1500), "admitted client can issue a typed command");
        Expect(second.SetRelay(3, true), "another client instance retains the same logical source");
        Expect(first.RevokeAuthority(), "operator explicitly revokes the source");
        Expect(!second.Servo(8, 1500), "reconnect after revoke does not reclaim authority");
        Expect(second.HandbackAuthority(), "operator explicitly creates a handback generation");
        Expect(first.Servo(8, 1500), "the shared source can command after handback");
        runtime.Wait();

        Expect(runtime.CommandCount == 8, "every authority and mutation request was observed once");
        Expect(runtime.CommandSources.TrueForAll(source => source == runtime.CommandSources[0]),
            "all Mission Planner client instances use one process-lifetime source");
        for (var index = 1; index < runtime.Sequences.Count; index++)
        {
            Expect(runtime.Sequences[index - 1] < runtime.Sequences[index],
                "request sequence increases across separate client instances");
        }
    }

    private static void Runtime_RejectsWrongAuthorityResponseType()
    {
        using var runtime = new MockRuntime(1, enforceAuthority: true, wrongAuthorityResponseType: true);
        var client = new NomadCoreClient("test-key", runtime.Port);
        Expect(!client.AdmitAuthority(), "command acknowledgement cannot impersonate authority admission");
        runtime.Wait();
        Expect(client.LastOutcome == NomadCoreRequestOutcome.UnknownOutcome,
            "malformed authority response is not reported as admitted");
    }

    private static void Runtime_RejectsRogueRuntimeWithoutCredentialDisclosure()
    {
        using var runtime = new MockRuntime(1, rogueRuntime: true);
        var client = new NomadCoreClient("test-key", runtime.Port);
        Expect(!client.AdmitAuthority(), "rogue runtime fails authentication");
        runtime.Wait();
        Expect(client.LastOutcome == NomadCoreRequestOutcome.FailedBeforeSend, "rogue proof fails before send");
        Expect(runtime.CommandCount == 0, "no command or reusable credential is disclosed to rogue runtime");
    }

    private static void Runtime_AuditFailureAfterSendIsUnknown()
    {
        using var runtime = new MockRuntime(1, auditFailure: true);
        var client = new NomadCoreClient("test-key", runtime.Port);
        Expect(!client.Servo(8, 1500), "audit failure is not success");
        runtime.Wait();
        Expect(client.LastOutcome == NomadCoreRequestOutcome.UnknownOutcome, "possible send audit failure is unknown");
        Expect(runtime.CommandCount == 1, "audit failure is never retried");
    }

    private static int ReservePort()
    {
        var listener = new TcpListener(IPAddress.Loopback, 0);
        listener.Start();
        var port = ((IPEndPoint)listener.LocalEndpoint).Port;
        listener.Stop();
        return port;
    }

    private static string MockProof(string secret, string payload)
    {
        using var hmac = new HMACSHA256(Encoding.UTF8.GetBytes(secret));
        return BitConverter.ToString(hmac.ComputeHash(Encoding.UTF8.GetBytes(payload))).Replace("-", "").ToLowerInvariant();
    }

    private sealed class MockRuntime : IDisposable
    {
        private readonly TcpListener _listener;
        private readonly Thread _worker;
        private readonly int _expectedConnections;
        private readonly bool _dropCommandResponse;
        private readonly int _helloVersion;
        private readonly int _commandResponseVersion;
        private readonly bool _enforceAuthority;
        private readonly bool _wrongAuthorityResponseType;
        private readonly bool _rogueRuntime;
        private readonly bool _auditFailure;
        private readonly string _outcome;
        private readonly string _errorCode;
        private readonly bool _acknowledged;
        private readonly bool _includeErrorResult;
        private readonly bool? _resultSuccess;
        private string _owner = "";
        private int _generation;
        private bool _everAdmitted;
        private long _lastSequence;
        private Exception _failure;

        public int Port { get; }
        public int CommandCount { get; private set; }
        public string LastCommandType { get; private set; } = "";
        public Dictionary<string, object> LastCommand { get; private set; } = new Dictionary<string, object>();
        public List<string> CommandSources { get; } = new List<string>();
        public List<long> Sequences { get; } = new List<long>();

        public MockRuntime(int expectedConnections, bool dropCommandResponse = false, int helloVersion = 1,
                           int commandResponseVersion = 1, int port = 0, bool enforceAuthority = false,
                           bool wrongAuthorityResponseType = false, bool rogueRuntime = false, bool auditFailure = false,
                           string outcome = "success", string errorCode = null, bool acknowledged = true,
                           bool includeErrorResult = false, bool? resultSuccess = null)
        {
            _expectedConnections = expectedConnections;
            _dropCommandResponse = dropCommandResponse;
            _helloVersion = helloVersion;
            _commandResponseVersion = commandResponseVersion;
            _enforceAuthority = enforceAuthority;
            _wrongAuthorityResponseType = wrongAuthorityResponseType;
            _rogueRuntime = rogueRuntime;
            _auditFailure = auditFailure;
            _outcome = outcome;
            _errorCode = errorCode;
            _acknowledged = acknowledged;
            _includeErrorResult = includeErrorResult;
            _resultSuccess = resultSuccess;
            _listener = new TcpListener(IPAddress.Loopback, port);
            _listener.Start();
            Port = ((IPEndPoint)_listener.LocalEndpoint).Port;
            _worker = new Thread(ServeConnections) { IsBackground = true };
            _worker.Start();
        }

        private void ServeConnections()
        {
            try
            {
                for (var index = 0; index < _expectedConnections; index++)
                {
                    using var client = _listener.AcceptTcpClient();
                    using var stream = client.GetStream();
                    using var reader = new StreamReader(stream, Encoding.UTF8, false, 4096, true);
                    using var writer = new StreamWriter(stream, new UTF8Encoding(false), 4096, true)
                    {
                        AutoFlush = true
                    };
                    var hello = Parse(reader.ReadLine());
                    writer.WriteLine(Serialize(new Dictionary<string, object>
                    {
                        ["protocol"] = "nomad-core", ["version"] = _helloVersion,
                        ["id"] = hello["id"], ["ok"] = true, ["type"] = "hello_response",
                        ["runtime_incarnation"] = "mock-runtime-incarnation",
                        ["client_authentication"] = "hmac-sha256-v1",
                        ["server_proof"] = _rogueRuntime ? new string('0', 64) : MockProof("test-key", "nomad-core:server:v1:mission-planner:" +
                            hello["auth_nonce"] + ":mock-runtime-incarnation"),
                        ["authority"] = new Dictionary<string, object>
                        {
                            ["vehicle_session"] = 1, ["generation"] = _enforceAuthority ? _generation : 1,
                            ["next_sequence"] = _lastSequence + 1
                        }
                    }));
                    if (_helloVersion != 1 || _rogueRuntime)
                    {
                        continue;
                    }
                    LastCommand = Parse(reader.ReadLine());
                    LastCommandType = Convert.ToString(LastCommand["type"], CultureInfo.InvariantCulture);
                    CommandCount++;
                    CommandSources.Add(Convert.ToString(LastCommand["command_source"], CultureInfo.InvariantCulture));
                    if (LastCommand.ContainsKey("sequence"))
                    {
                        Sequences.Add(Convert.ToInt64(LastCommand["sequence"], CultureInfo.InvariantCulture));
                    }
                    if (_dropCommandResponse)
                    {
                        continue;
                    }
                    writer.WriteLine(Serialize(CreateResponse(LastCommand)));
                }
            }
            catch (Exception error)
            {
                _failure = error;
            }
        }

        private Dictionary<string, object> CreateResponse(Dictionary<string, object> command)
        {
            var response = new Dictionary<string, object>
            {
                ["protocol"] = "nomad-core", ["version"] = _commandResponseVersion,
                ["id"] = command["id"], ["ok"] = true
            };
            if (_errorCode != null)
            {
                response["ok"] = false;
                if (_outcome != null)
                {
                    response["outcome"] = _outcome;
                }
                if (_includeErrorResult)
                {
                    response["command_result"] = new Dictionary<string, object>
                    {
                        ["success"] = _resultSuccess ?? false, ["acknowledged"] = _acknowledged
                    };
                }
                response["error"] = new Dictionary<string, object>
                {
                    ["code"] = _errorCode, ["message"] = "Controlled runtime error"
                };
                return response;
            }
            if (_auditFailure)
            {
                response["ok"] = false;
                response["outcome"] = "unknown";
                response["error"] = new Dictionary<string, object>
                {
                    ["code"] = "audit_failure", ["message"] = "Audit append failed after possible send"
                };
                return response;
            }
            if (_enforceAuthority && !ApplyAuthority(command, response))
            {
                return response;
            }
            if (LastCommandType.EndsWith("_authority", StringComparison.Ordinal) && !_wrongAuthorityResponseType)
            {
                response["type"] = "authority_response";
                response["authority_generation"] = _generation;
                response["authority_owner"] = string.IsNullOrEmpty(_owner) ? null : _owner;
                return response;
            }
            response["type"] = "command_response";
            if (_outcome != null)
            {
                response["outcome"] = _outcome;
            }
            response["command_result"] = new Dictionary<string, object>
            {
                ["success"] = _resultSuccess ?? (_outcome == "success"), ["acknowledged"] = _acknowledged,
                ["message"] = "Controlled software result"
            };
            return response;
        }

        private bool ApplyAuthority(Dictionary<string, object> command, Dictionary<string, object> response)
        {
            var source = Convert.ToString(command["command_source"], CultureInfo.InvariantCulture);
            var clientId = Convert.ToString(command["client_id"], CultureInfo.InvariantCulture);
            var generation = Convert.ToInt32(command["authority_generation"], CultureInfo.InvariantCulture);
            if (generation != _generation || source != clientId)
            {
                return Reject(response, "stale_authority");
            }
            if (LastCommandType == "revoke_authority")
            {
                _generation++;
                _owner = "";
                _lastSequence = 0;
                return true;
            }
            if (LastCommandType == "admit_authority" || LastCommandType == "handback_authority")
            {
                if (_owner != "" || (LastCommandType == "handback_authority") != _everAdmitted)
                {
                    return Reject(response, "authority_unavailable");
                }
                _generation++;
                _owner = source;
                _everAdmitted = true;
                _lastSequence = 0;
                return true;
            }
            if (_owner != source)
            {
                return Reject(response, "not_authoritative");
            }
            var sequence = Convert.ToInt64(command["sequence"], CultureInfo.InvariantCulture);
            if (sequence <= _lastSequence)
            {
                return Reject(response, "stale_request");
            }
            _lastSequence = sequence;
            return true;
        }

        private static bool Reject(Dictionary<string, object> response, string code)
        {
            response["ok"] = false;
            response["outcome"] = "rejected";
            response["error"] = new Dictionary<string, object> { ["code"] = code, ["message"] = code };
            return false;
        }

        public void Wait()
        {
            if (!_worker.Join(TimeSpan.FromSeconds(5)))
            {
                throw new TimeoutException("Mock runtime did not receive its expected request.");
            }
            if (_failure != null)
            {
                throw new InvalidOperationException("Mock runtime failed.", _failure);
            }
        }

        public void Dispose()
        {
            _listener.Stop();
            if (_worker.IsAlive)
            {
                _worker.Join(TimeSpan.FromSeconds(1));
            }
        }

        private static Dictionary<string, object> Parse(string line)
        {
            return new JavaScriptSerializer().Deserialize<Dictionary<string, object>>(line);
        }

        private static string Serialize(Dictionary<string, object> value)
        {
            return new JavaScriptSerializer().Serialize(value);
        }
    }

}
