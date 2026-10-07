// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Web.Script.Serialization;

internal static partial class NomadCoreClientTests
{
    private sealed class AsyncRuntime : IDisposable
    {
        private readonly TcpListener _listener;
        private readonly Thread _acceptor;
        private readonly int _connectionCount;
        private readonly string _incarnation;
        private readonly int _session;
        private int _generation;
        private readonly bool _delayHello;
        private readonly Func<Dictionary<string, object>, Dictionary<string, object>> _semanticResponse;
        private readonly object _state = new object();
        private readonly List<Thread> _workers = new List<Thread>();
        private readonly ManualResetEventSlim _helloObserved = new ManualResetEventSlim();
        private readonly ManualResetEventSlim _helloReleased = new ManualResetEventSlim();
        private readonly ManualResetEventSlim[] _responses = Enumerable.Range(0, 33)
            .Select(_ => new ManualResetEventSlim()).ToArray();
        private ulong _nextSequence;
        private Exception _failure;

        internal int Port { get; }
        internal int AcceptedConnections { get; private set; }
        internal object SequenceFloorOverride { get; set; }
        internal bool OmitSequenceFloor { get; set; }
        internal int StaleRequests { get; private set; }
        internal int ResponseSize { get; set; }
        internal int AuthorityGeneration
        {
            get
            {
                lock (_state)
                {
                    return _generation;
                }
            }
        }
        internal List<Dictionary<string, object>> Commands { get; } = new List<Dictionary<string, object>>();

        internal AsyncRuntime(int count, int port = 0, ulong sequenceFloor = 1,
            string incarnation = "async-runtime", int session = 1, int generation = 1, bool delayHello = false,
            Func<Dictionary<string, object>, Dictionary<string, object>> semanticResponse = null)
        {
            _connectionCount = count;
            _incarnation = incarnation;
            _session = session;
            _generation = generation;
            _delayHello = delayHello;
            _semanticResponse = semanticResponse;
            _nextSequence = sequenceFloor;
            _listener = new TcpListener(IPAddress.Loopback, port);
            _listener.Start();
            Port = ((IPEndPoint)_listener.LocalEndpoint).Port;
            _acceptor = new Thread(AcceptConnections) { IsBackground = true };
            _acceptor.Start();
        }

        private void AcceptConnections()
        {
            try
            {
                for (var index = 0; index < _connectionCount; index++)
                {
                    var connection = _listener.AcceptTcpClient();
                    AcceptedConnections++;
                    var worker = new Thread(() => ServeConnection(connection)) { IsBackground = true };
                    _workers.Add(worker);
                    worker.Start();
                }
            }
            catch (Exception exception)
            {
                _failure = exception;
            }
        }

        private void ServeConnection(TcpClient connection)
        {
            try
            {
                using (connection)
                using (var stream = connection.GetStream())
                using (var reader = new StreamReader(stream, Encoding.UTF8, false, 4096, true))
                using (var writer = new StreamWriter(stream, new UTF8Encoding(false), 4096, true)
                    { AutoFlush = true, NewLine = "\n" })
                {
                    var serializer = new JavaScriptSerializer();
                    var hello = serializer.Deserialize<Dictionary<string, object>>(reader.ReadLine());
                    _helloObserved.Set();
                    if (_delayHello && !_helloReleased.Wait(5000))
                    {
                        throw new TimeoutException("Test did not release hello.");
                    }
                    writer.WriteLine(serializer.Serialize(CreateHelloResponse(hello)));
                    var line = reader.ReadLine();
                    if (line == null)
                    {
                        return;
                    }
                    var command = serializer.Deserialize<Dictionary<string, object>>(line);
                    var accepted = RecordCommand(command);
                    if (command["type"].ToString() == "revoke_authority")
                    {
                        writer.WriteLine(serializer.Serialize(CreateAuthorityResponse(command)));
                        return;
                    }
                    var channel = command.ContainsKey("channel") ? Convert.ToInt32(command["channel"]) : 3;
                    if (!_responses[channel].Wait(5000))
                    {
                        throw new TimeoutException("Test did not release command response.");
                    }
                    var response = CreateCommandResponse(command, channel);
                    if (ResponseSize > 0)
                    {
                        var result = (Dictionary<string, object>)response["command_result"];
                        result["message"] = "";
                        result["message"] = new string('x', ResponseSize - serializer.Serialize(response).Length);
                    }
                    if (!accepted)
                    {
                        response["ok"] = false;
                        response["outcome"] = "rejected";
                        response.Remove("command_result");
                        response["error"] = new Dictionary<string, object>
                        {
                            ["code"] = "stale_request", ["message"] = "sequence was already consumed"
                        };
                    }
                    writer.WriteLine(serializer.Serialize(response));
                }
            }
            catch (IOException)
            {
                // Cancellation tests deliberately close the client before a response write.
            }
            catch (Exception exception)
            {
                _failure = exception;
            }
        }

        private Dictionary<string, object> CreateHelloResponse(Dictionary<string, object> hello)
        {
            ulong floor;
            int generation;
            lock (_state)
            {
                floor = _nextSequence;
                generation = _generation;
            }
            var authority = new Dictionary<string, object>
            {
                ["vehicle_session"] = _session, ["generation"] = generation,
                ["next_sequence"] = SequenceFloorOverride ?? floor
            };
            if (OmitSequenceFloor)
            {
                authority.Remove("next_sequence");
            }
            return new Dictionary<string, object>
            {
                ["protocol"] = "nomad-core", ["version"] = 1, ["id"] = hello["id"],
                ["ok"] = true, ["type"] = "hello_response", ["runtime_incarnation"] = _incarnation,
                ["client_authentication"] = "hmac-sha256-v1",
                ["server_proof"] = MockProof("test-key", "nomad-core:server:v1:mission-planner:" +
                    hello["auth_nonce"] + ":" + _incarnation),
                ["authority"] = authority
            };
        }

        private bool RecordCommand(Dictionary<string, object> command)
        {
            lock (_state)
            {
                Commands.Add(command);
                if (command["type"].ToString() == "revoke_authority")
                {
                    _generation++;
                    _nextSequence = 1;
                    Monitor.PulseAll(_state);
                    return true;
                }
                var sequence = Convert.ToUInt64(command["sequence"]);
                var accepted = sequence >= _nextSequence;
                if (accepted)
                {
                    _nextSequence = sequence + 1;
                }
                else
                {
                    StaleRequests++;
                }
                Monitor.PulseAll(_state);
                return accepted;
            }
        }

        private Dictionary<string, object> CreateAuthorityResponse(Dictionary<string, object> command)
        {
            return new Dictionary<string, object>
            {
                ["protocol"] = "nomad-core", ["version"] = 1, ["id"] = command["id"],
                ["ok"] = true, ["type"] = "authority_response",
                ["authority_generation"] = AuthorityGeneration, ["authority_owner"] = ""
            };
        }

        private Dictionary<string, object> CreateCommandResponse(Dictionary<string, object> command, int channel)
        {
            if (_semanticResponse != null)
            {
                var response = _semanticResponse(command);
                response["runtime_incarnation"] = _incarnation;
                return response;
            }
            if (command.TryGetValue("authority_generation", out var generation) &&
                Convert.ToInt32(generation) != AuthorityGeneration)
            {
                return new Dictionary<string, object>
                {
                    ["protocol"] = "nomad-core", ["version"] = 1, ["id"] = command["id"],
                    ["ok"] = false, ["type"] = "error", ["outcome"] = "interrupted",
                    ["command_result"] = new Dictionary<string, object>
                    {
                        ["success"] = true, ["acknowledged"] = true
                    },
                    ["error"] = new Dictionary<string, object>
                    {
                        ["code"] = "authority_interrupted", ["message"] = "authority changed during vehicle operation"
                    }
                };
            }
            var outcome = channel == 2 ? "rejected" : channel == 3 ? "interrupted" : "success";
            return new Dictionary<string, object>
            {
                ["protocol"] = "nomad-core", ["version"] = 1, ["id"] = command["id"],
                ["ok"] = true, ["type"] = "command_response", ["outcome"] = outcome,
                ["command_result"] = new Dictionary<string, object>
                {
                    ["success"] = outcome == "success", ["acknowledged"] = channel != 2,
                    ["message"] = "request-" + channel
                }
            };
        }

        internal void WaitForCommands(int count)
        {
            var deadline = DateTime.UtcNow.AddSeconds(5);
            lock (_state)
            {
                while (Commands.Count < count)
                {
                    var remaining = deadline - DateTime.UtcNow;
                    if (remaining <= TimeSpan.Zero || !Monitor.Wait(_state, remaining))
                    {
                        throw new TimeoutException(
                            "Expected " + count + " requests; observed " + Commands.Count + ".");
                    }
                }
            }
        }

        internal void WaitForHello()
        {
            if (!_helloObserved.Wait(5000))
            {
                throw new TimeoutException("Runtime did not observe hello.");
            }
        }

        internal void ReleaseHello() => _helloReleased.Set();
        internal void DelayHelloAgain() { _helloObserved.Reset(); _helloReleased.Reset(); }
        internal void ReleaseResponse(int channel) => _responses[channel].Set();

        internal void ReleaseResponses()
        {
            foreach (var response in _responses)
            {
                response.Set();
            }
        }

        internal void Wait()
        {
            if (!_acceptor.Join(5000))
            {
                throw new TimeoutException("Runtime did not accept expected client connections.");
            }
            foreach (var worker in _workers)
            {
                if (!worker.Join(5000))
                {
                    throw new TimeoutException("Runtime connection did not finish.");
                }
            }
            if (_failure != null)
            {
                throw new InvalidOperationException("Async mock runtime failed.", _failure);
            }
        }

        public void Dispose()
        {
            ReleaseHello();
            ReleaseResponses();
            _listener.Stop();
            _acceptor.Join(1000);
            foreach (var worker in _workers)
            {
                worker.Join(1000);
            }
        }
    }
}
