// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Web.Script.Serialization;

internal static partial class NomadCoreClientTests
{
    private sealed class StatusRuntime : IDisposable
    {
        private readonly TcpListener _listener;
        private readonly Thread _acceptor;
        private readonly List<Thread> _workers = new List<Thread>();
        private readonly List<TcpClient> _clients = new List<TcpClient>();
        private readonly List<Dictionary<string, object>> _requests = new List<Dictionary<string, object>>();
        private readonly ManualResetEventSlim _stop = new ManualResetEventSlim();
        private readonly ManualResetEventSlim _release;
        private readonly Action<Dictionary<string, object>> _changeHello;
        private readonly Action<Dictionary<string, object>> _changeResponse;
        private readonly Action<NetworkStream, byte[]> _writeStatus;
        private readonly bool _dripHello;
        private readonly string _incarnation;
        private readonly object _gate = new object();
        private Exception _failure;
        private int _statuses;
        internal int Port { get; }

        internal StatusRuntime(int count = 1, bool hold = false, bool dripHello = false,
            Action<Dictionary<string, object>> changeHello = null,
            Action<Dictionary<string, object>> changeResponse = null,
            Action<NetworkStream, byte[]> writeStatus = null, string incarnation = "status-runtime")
        {
            _changeHello = changeHello;
            _changeResponse = changeResponse;
            _writeStatus = writeStatus;
            _dripHello = dripHello;
            _incarnation = incarnation;
            _release = new ManualResetEventSlim(!hold);
            _listener = new TcpListener(IPAddress.Loopback, 0);
            _listener.Start();
            Port = ((IPEndPoint)_listener.LocalEndpoint).Port;
            _acceptor = new Thread(() => Accept(count)) { IsBackground = true };
            _acceptor.Start();
        }

        private void Accept(int count)
        {
            try
            {
                for (var index = 0; index < count; index++)
                {
                    var client = _listener.AcceptTcpClient();
                    client.NoDelay = true;
                    client.ReceiveTimeout = 7000;
                    var worker = new Thread(() => Serve(client)) { IsBackground = true };
                    lock (_gate)
                    {
                        _clients.Add(client);
                        _workers.Add(worker);
                    }
                    worker.Start();
                }
            }
            catch (SocketException exception)
            {
                if (!_stop.IsSet)
                {
                    _failure = exception;
                }
            }
        }

        private void Serve(TcpClient client)
        {
            try
            {
                using (client)
                using (var stream = client.GetStream())
                using (var reader = new StreamReader(stream, Encoding.UTF8, false, 4096, true))
                {
                    var serializer = new JavaScriptSerializer { MaxJsonLength = 200000 };
                    var hello = serializer.Deserialize<Dictionary<string, object>>(reader.ReadLine());
                    Record(hello);
                    var response = Hello(hello);
                    _changeHello?.Invoke(response);
                    Write(stream, serializer.Serialize(response), _dripHello);
                    var line = reader.ReadLine();
                    if (line == null)
                    {
                        return;
                    }
                    var request = serializer.Deserialize<Dictionary<string, object>>(line);
                    Record(request);
                    Interlocked.Increment(ref _statuses);
                    if (!_release.Wait(7000))
                    {
                        throw new TimeoutException("STATUS fixture was not released.");
                    }
                    response = Status(request);
                    _changeResponse?.Invoke(response);
                    var bytes = Encoding.UTF8.GetBytes(serializer.Serialize(response) + "\n");
                    if (_writeStatus != null)
                    {
                        _writeStatus(stream, bytes);
                    }
                    else
                    {
                        stream.Write(bytes, 0, bytes.Length);
                    }
                }
            }
            catch (IOException)
            {
            } // Cancellation and rejected oversized frames close this owned socket.
            catch (ObjectDisposedException)
            {
            }
            catch (Exception exception)
            {
                _failure = exception;
            }
        }

        private Dictionary<string, object> Hello(Dictionary<string, object> request) =>
            new Dictionary<string, object>
            {
                ["protocol"] = "nomad-core", ["version"] = 1, ["id"] = request["id"], ["ok"] = true,
                ["type"] = "hello_response", ["runtime_incarnation"] = _incarnation, ["runtime_version"] = "fixture",
                ["client_authentication"] = "hmac-sha256-v1", ["capabilities"] = new[] { "status", "land" },
                ["server_proof"] = MockProof("test-key", "nomad-core:server:v1:" + request["client_id"]
                    + ":" + request["auth_nonce"] + ":" + _incarnation),
                // No authority or sequence context: STATUS must remain an independent read.
            };

        private Dictionary<string, object> Status(Dictionary<string, object> request) =>
            new Dictionary<string, object>
            {
                ["protocol"] = "nomad-core", ["version"] = 1, ["id"] = request["id"], ["ok"] = true,
                ["type"] = "status_response",
                ["status"] = new Dictionary<string, object>
                {
                    ["runtime_incarnation"] = _incarnation, ["runtime_ready"] = true, ["lifecycle"] = "ready",
                    ["audit_healthy"] = false, ["actuation_enabled"] = false, ["authority_owner"] = null,
                    ["vehicle_session"] = 7, ["authority_generation"] = 9,
                    ["mavsdk_connection_open"] = true, ["vehicle_transport_connected"] = true,
                    ["vehicle_session_established"] = true, ["vehicle_connected"] = true,
                    ["identity_resolved"] = true, ["aircraft_class"] = "QuadPlane",
                    ["supported_flight_operations"] = new string[0],
                    ["armed"] = true, ["custom_mode"] = 10,
                    ["telemetry"] = new Dictionary<string, object>
                    {
                        ["heartbeat_fresh"] = true, ["heartbeat_age_ms"] = 50,
                        ["vtol_state"] = "fixed_wing", ["vtol_state_valid"] = true, ["vtol_state_age_ms"] = 60,
                        ["landed_state"] = "in_air", ["landed_state_valid"] = true, ["landed_state_age_ms"] = 70,
                        ["position_valid"] = true, ["position_age_ms"] = 80,
                        ["position"] = new Dictionary<string, object>
                        {
                            ["latitude_deg"] = 0, ["longitude_deg"] = 0,
                            ["absolute_altitude_m"] = 123.5, ["relative_altitude_m"] = 23.5,
                        },
                    },
                },
            };

        private void Record(Dictionary<string, object> request)
        {
            lock (_gate)
            {
                _requests.Add(request);
            }
        }

        internal Dictionary<string, object>[] Requests()
        {
            lock (_gate)
            {
                return _requests.ToArray();
            }
        }

        internal bool WaitForStatuses(int count) =>
            SpinWait.SpinUntil(() => Volatile.Read(ref _statuses) >= count, 3000);
        internal void Release() => _release.Set();
        internal bool HasExtraConnection => _listener.Pending();

        internal void Write(NetworkStream stream, string text, bool drip)
        {
            var bytes = Encoding.UTF8.GetBytes(text + "\n");
            if (!drip)
            {
                stream.Write(bytes, 0, bytes.Length);
                return;
            }
            foreach (var value in bytes)
            {
                if (_stop.IsSet)
                {
                    return;
                }
                stream.WriteByte(value);
                if (_stop.Wait(100))
                {
                    return;
                }
            }
        }

        internal bool ClientClosed()
        {
            TcpClient client;
            lock (_gate)
            {
                client = _clients[0];
            }
            try
            {
                client.Client.ReceiveTimeout = 2000;
                return client.Client.Receive(new byte[1], 0, 1, SocketFlags.Peek) == 0;
            }
            catch (SocketException exception)
            {
                return exception.SocketErrorCode == SocketError.ConnectionReset;
            }
        }

        public void Dispose()
        {
            _stop.Set();
            _release.Set();
            _listener.Stop();
            lock (_gate)
            {
                foreach (var client in _clients)
                {
                    client.Close();
                }
            }
            if (!_acceptor.Join(3000))
            {
                throw new TimeoutException("STATUS acceptor did not drain.");
            }
            foreach (var worker in _workers)
            {
                if (!worker.Join(3000))
                {
                    throw new TimeoutException("STATUS worker did not drain.");
                }
            }
            _stop.Dispose();
            _release.Dispose();
            if (_failure != null)
            {
                throw new InvalidOperationException("STATUS fixture failed.", _failure);
            }
        }
    }
}
