// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Mission Planner's non-owning client for nomad-link-router.exe.
    /// It observes status on a background thread and only sends router-selection requests.
    /// </summary>
    public sealed partial class StandaloneRouterClient : IRouterStatusProvider
    {
        private const int ConnectTimeoutMs = 1500;
        private const int ReadTimeoutMs = 250;
        private const int StatusIntervalMs = 1000;
        private const int ReconnectIntervalMs = 1000;
        private const int StaleAfterMs = 3500;

        private readonly MAVLinkConnectionManager.ConnectionConfig _config;
        private readonly object _gate = new object();
        private readonly List<LinkStatistics> _links = new List<LinkStatistics>();
        private readonly List<FailoverEventArgs> _failovers = new List<FailoverEventArgs>();
        private Thread _worker;
        private CancellationTokenSource _stop;
        private TcpClient _connection;
        private bool _connected;
        private bool _stale = true;
        private readonly RouterClock _clock;
        private double _lastStatus = RouterClock.Unset;
        private string _activeLink = "";
        private string _manualOverride = "";
        private string _lastUnavailableMessage;
        private int _nextRequestId;

        public StandaloneRouterClient(MAVLinkConnectionManager.ConnectionConfig config)
            : this(config, new RouterClock()) { }

        internal StandaloneRouterClient(MAVLinkConnectionManager.ConnectionConfig config, RouterClock clock)
        {
            _clock = clock;
            _config = config ?? throw new ArgumentNullException(nameof(config));
        }

        public string RouterMode => "Standalone";
        public bool IsMonitoring { get { lock (_gate) { return _connected; } } }
        public bool IsRouterAvailable
        {
            get { lock (_gate) { return _connected && !_stale && IsFreshStatusLocked(); } }
        }
        public bool IsStatusStale
        {
            get
            {
                lock (_gate)
                {
                    return !_connected || _stale || !IsFreshStatusLocked();
                }
            }
        }
        public string ActiveLink { get { lock (_gate) { return _activeLink; } } }
        public string ManualOverride { get { lock (_gate) { return _manualOverride; } } }
        public string LocalMergedEndpoint => $"udp://127.0.0.1:{_config.RouterLocalPort}";

        public IReadOnlyList<LinkStatistics> LinkStatistics
        {
            get { lock (_gate) { return _links.Select(Clone).ToArray(); } }
        }

        public IReadOnlyCollection<FailoverEventArgs> FailoverLog
        {
            get
            {
                lock (_gate)
                {
                    return _failovers.Select(CopyFailover).ToArray();
                }
            }
        }

        public event EventHandler<LinkStatusChangedEventArgs> LinkStatusChanged;
        public event EventHandler<FailoverEventArgs> FailoverOccurred;
        public event EventHandler<string> ActiveLinkChanged;
        public event EventHandler<string> LogMessage;

        public void Start()
        {
            lock (_gate)
            {
                if (_worker != null)
                {
                    return;
                }
                var cancellation = new CancellationTokenSource();
                _stop = cancellation;
                _worker = new Thread(() => Run(cancellation.Token))
                {
                    IsBackground = true,
                    Name = "NOMAD standalone router client",
                };
                _worker.Start();
            }
        }

        public void Stop()
        {
            Thread worker;
            CancellationTokenSource stop;
            lock (_gate)
            {
                worker = _worker;
                stop = _stop;
                _worker = null;
                _stop = null;
                stop?.Cancel();
                CloseConnectionLocked();
            }
            if (worker != null && worker != Thread.CurrentThread)
            {
                worker.Join(1500);
            }
            stop?.Dispose();
            MarkUnavailable("Standalone router management stopped.", false);
        }

        public bool SwitchToLink(string target)
        {
            if (target == null)
            {
                return false;
            }

            if (target.Length == 0)
            {
                return SendSelection("set_auto", target, null);
            }

            return SendSelection("select_link", target, target);
        }

        public void Dispose()
        {
            Stop();
        }

        private void Run(CancellationToken stop)
        {
            while (!stop.IsCancellationRequested)
            {
                try
                {
                    RunConnectedSession(stop);
                }
                catch (Exception ex)
                {
                    MarkUnavailable("Standalone router unavailable: " + ex.Message, true);
                }
                if (!stop.IsCancellationRequested)
                {
                    stop.WaitHandle.WaitOne(ReconnectIntervalMs);
                }
            }
        }

        private void RunConnectedSession(CancellationToken stop)
        {
            var client = Connect();
            var stream = client.GetStream();
            stream.ReadTimeout = ConnectTimeoutMs;
            stream.WriteTimeout = ConnectTimeoutMs;
            lock (_gate)
            {
                _connection = client;
            }

            try
            {
                SendHello(stream);
                SendRequest(stream, "subscribe", null);
                stream.ReadTimeout = ReadTimeoutMs;
                var nextStatus = RouterClock.Unset;
                var statusPending = false;
                var statusRequest = 0;
                while (!stop.IsCancellationRequested)
                {
                    var now = _clock.Seconds();
                    if (!statusPending && now >= nextStatus)
                    {
                        statusRequest = SendRequest(stream, "get_status", null);
                        statusPending = true;
                        nextStatus = now + StatusIntervalMs / 1000.0;
                    }

                    string line;
                    try
                    {
                        line = RouterManagementProtocol.ReadLine(stream);
                    }
                    catch (IOException)
                    {
                        statusPending = false;
                        continue;
                    }
                    if (line == null)
                    {
                        throw new IOException("Management server closed the connection.");
                    }

                    var message = RouterManagementProtocol.Parse(line);
                    var id = RouterManagementProtocol.GetInt(message, "id");
                    if (RouterManagementProtocol.GetString(message, "type") == "event")
                    {
                        HandleEvent(message);
                        continue;
                    }
                    if (!IsOk(message))
                    {
                        throw new InvalidOperationException(RouterManagementProtocol.GetString(message, "error")
                            ?? "Management request failed.");
                    }
                    if (id.HasValue && id.Value == statusRequest)
                    {
                        if (!RouterManagementProtocol.TryReadStatus(message, out var status))
                        {
                            throw new FormatException("Management status response is invalid.");
                        }
                        ApplyStatus(status);
                        statusPending = false;
                    }
                }
            }
            finally
            {
                lock (_gate)
                {
                    if (ReferenceEquals(_connection, client))
                    {
                        _connection = null;
                    }
                }
                try { client.Close(); } catch { }
            }
        }

        private TcpClient Connect()
        {
            if (_config.ManagementPort <= 0 || _config.ManagementPort > 65535)
            {
                throw new InvalidOperationException("Standalone management port is invalid.");
            }

            var client = new TcpClient(AddressFamily.InterNetwork) { NoDelay = true };
            var connect = client.ConnectAsync(IPAddress.Loopback, _config.ManagementPort);
            if (!connect.Wait(ConnectTimeoutMs))
            {
                client.Close();
                throw new TimeoutException("Management connection timed out.");
            }
            connect.GetAwaiter().GetResult();
            return client;
        }

        private void SendHello(NetworkStream stream)
        {
            var id = SendRequest(stream, "hello", null);
            var response = ReadResponse(stream, id);
            if (!IsOk(response) ||
                !string.Equals(RouterManagementProtocol.GetString(response, "protocol"),
                    RouterManagementProtocol.ProtocolName, StringComparison.Ordinal) ||
                RouterManagementProtocol.GetInt(response, "version") != RouterManagementProtocol.Version)
            {
                throw new InvalidOperationException("Incompatible standalone router management protocol.");
            }
            MarkConnected();
        }

        private int SendRequest(NetworkStream stream, string type, string link)
        {
            var id = Interlocked.Increment(ref _nextRequestId);
            var request = new Dictionary<string, object>
            {
                ["id"] = id,
                ["type"] = type,
                ["protocol"] = RouterManagementProtocol.ProtocolName,
                ["version"] = RouterManagementProtocol.Version,
            };
            if (link != null)
            {
                request["link"] = link;
            }
            var bytes = RouterManagementProtocol.EncodeLine(request);
            stream.Write(bytes, 0, bytes.Length);
            return id;
        }

        private Dictionary<string, object> ReadResponse(NetworkStream stream, int expectedId)
        {
            var response = RouterManagementProtocol.Parse(RouterManagementProtocol.ReadLine(stream));
            if (RouterManagementProtocol.GetInt(response, "id") != expectedId)
            {
                throw new FormatException("Management response ID did not match the request.");
            }
            return response;
        }

        private bool SendSelection(string type, string target, string link)
        {
            try
            {
                using (var client = Connect())
                {
                    var stream = client.GetStream();
                    stream.ReadTimeout = ConnectTimeoutMs;
                    stream.WriteTimeout = ConnectTimeoutMs;
                    SendHello(stream);
                    var id = SendRequest(stream, type, link);
                    var response = ReadResponse(stream, id);
                    if (!IsOk(response))
                    {
                        LogMessage?.Invoke(this, RouterManagementProtocol.GetString(response, "error")
                            ?? "Standalone router rejected the request.");
                        return false;
                    }
                    if (!RouterManagementProtocol.TryReadStatus(response, out var status))
                    {
                        return false;
                    }
                    ApplyStatus(status);
                    return string.Equals(status.ManualOverrideId, target, StringComparison.Ordinal);
                }
            }
            catch (Exception ex)
            {
                MarkUnavailable("Standalone router command failed: " + ex.Message, true);
                return false;
            }
        }

        private void HandleEvent(IDictionary<string, object> message)
        {
            if (RouterManagementProtocol.TryReadStatus(message, out var status))
            {
                ApplyStatus(status);
            }

            var eventName = RouterManagementProtocol.GetString(message, "event");
            if (eventName == "failover")
            {
                var failover = new FailoverEventArgs
                {
                    FromLink = RouterManagementProtocol.GetString(message, "fromLinkId") ?? "",
                    ToLink = RouterManagementProtocol.GetString(message, "toLinkId") ?? "",
                    Reason = RouterManagementProtocol.GetString(message, "reason") ?? "",
                    Timestamp = ParseTimestamp(RouterManagementProtocol.GetString(message, "timestampUtc")),
                };
                lock (_gate)
                {
                    _failovers.Add(failover);
                    while (_failovers.Count > 50) _failovers.RemoveAt(0);
                }
                FailoverOccurred?.Invoke(this, failover);
            }
        }

        private void MarkConnected()
        {
            lock (_gate)
            {
                _connected = true;
                _stale = true;
            }
        }

        private void MarkUnavailable(string message, bool notify)
        {
            var shouldNotify = false;
            lock (_gate)
            {
                _connected = false;
                _stale = true;
                _activeLink = "";
                _manualOverride = "";
                foreach (var link in _links)
                {
                    link.IsStale = true;
                }
                if (!string.Equals(_lastUnavailableMessage, message, StringComparison.Ordinal))
                {
                    _lastUnavailableMessage = message;
                    shouldNotify = notify;
                }
            }
            if (shouldNotify)
            {
                LogMessage?.Invoke(this, message);
                foreach (var stats in LinkStatistics)
                {
                    LinkStatusChanged?.Invoke(this, new LinkStatusChangedEventArgs
                    {
                        Link = stats.Type,
                        Statistics = stats,
                        IsActive = stats.Type == ActiveLink,
                    });
                }
            }
        }

        private void CloseConnectionLocked()
        {
            try { _connection?.Close(); } catch { }
            _connection = null;
        }

        private bool IsFreshStatusLocked()
        {
            return _lastStatus != RouterClock.Unset &&
                (_clock.Seconds() - _lastStatus) * 1000 <= StaleAfterMs;
        }

    }
}
