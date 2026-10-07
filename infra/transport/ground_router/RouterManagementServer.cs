// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Loopback-only management endpoint for a running ground router.
    /// Socket I/O is isolated from the MAVLink worker and every client queue is bounded.
    /// </summary>
    public sealed partial class RouterManagementServer : IDisposable
    {
        private const int MaxClients = 16;
        private const int MaxQueuedMessages = 64;

        private readonly GroundLinkRouter _router;
        private readonly string _bindAddress;
        private readonly int _port;
        private readonly string _implementationVersion;
        private readonly object _gate = new object();
        private readonly List<ManagementSession> _sessions = new List<ManagementSession>();
        private long _eventSequence;
        private TcpListener _listener;
        private Thread _acceptThread;
        private CancellationTokenSource _stop;
        private bool _running;
        private bool _disposed;

        public RouterManagementServer(
            GroundLinkRouter router,
            string bindAddress,
            int port,
            string implementationVersion = "standalone")
        {
            _router = router ?? throw new ArgumentNullException(nameof(router));
            _bindAddress = bindAddress ?? "";
            _port = port;
            _implementationVersion = implementationVersion ?? "unknown";
        }

        public bool IsRunning
        {
            get { lock (_gate) { return _running; } }
        }

        public IPEndPoint LocalEndpoint { get; private set; }

        public void Start()
        {
            lock (_gate)
            {
                if (_disposed)
                {
                    throw new ObjectDisposedException(nameof(RouterManagementServer));
                }
                if (_running)
                {
                    return;
                }
                ValidateEndpoint();
                _listener = new TcpListener(IPAddress.Loopback, _port);
                _listener.Start(MaxClients);
                LocalEndpoint = (IPEndPoint)_listener.LocalEndpoint;
                _stop = new CancellationTokenSource();
                _running = true;
                _router.FailoverOccurred += OnFailover;
                _router.ActiveLinkChanged += OnActiveLinkChanged;
                _router.StatsUpdated += OnStatsUpdated;
                _acceptThread = new Thread(() => AcceptClients(_stop.Token))
                {
                    IsBackground = true,
                    Name = "NOMAD router management acceptor",
                };
                _acceptThread.Start();
            }
        }

        public void Stop()
        {
            ManagementSession[] sessions;
            Thread acceptThread;
            CancellationTokenSource stop;
            lock (_gate)
            {
                if (!_running)
                {
                    return;
                }
                PublishEventLocked("router_stopping", null);
                _running = false;
                _router.FailoverOccurred -= OnFailover;
                _router.ActiveLinkChanged -= OnActiveLinkChanged;
                _router.StatsUpdated -= OnStatsUpdated;
                acceptThread = _acceptThread;
                stop = _stop;
                _acceptThread = null;
                _stop = null;
                sessions = _sessions.ToArray();
                _sessions.Clear();
                stop.Cancel();
                try { _listener?.Stop(); } catch { }
                _listener = null;
            }

            foreach (var session in sessions)
            {
                session.Stop();
            }
            if (acceptThread != null && acceptThread != Thread.CurrentThread)
            {
                acceptThread.Join(1000);
            }
            stop.Dispose();
        }

        public void Dispose()
        {
            lock (_gate)
            {
                _disposed = true;
            }
            Stop();
        }

        private void ValidateEndpoint()
        {
            if (!string.Equals(_bindAddress, "127.0.0.1", StringComparison.Ordinal) ||
                _port < 0 || _port > 65535)
            {
                throw new ArgumentException("Management endpoint must bind IPv4 loopback and use a valid port.");
            }
        }

        private void AcceptClients(CancellationToken stop)
        {
            while (!stop.IsCancellationRequested)
            {
                try
                {
                    if (!_listener.Pending())
                    {
                        stop.WaitHandle.WaitOne(25);
                        continue;
                    }

                    var client = _listener.AcceptTcpClient();
                    client.NoDelay = true;
                    ManagementSession session;
                    lock (_gate)
                    {
                        if (!_running || _sessions.Count >= MaxClients)
                        {
                            client.Close();
                            continue;
                        }
                        session = new ManagementSession(this, client, MaxQueuedMessages);
                        _sessions.Add(session);
                    }
                    session.Start();
                }
                catch (SocketException)
                {
                    if (!stop.IsCancellationRequested)
                    {
                        Thread.Sleep(25);
                    }
                }
                catch (ObjectDisposedException)
                {
                    return;
                }
            }
        }

        private void RemoveSession(ManagementSession session)
        {
            lock (_gate)
            {
                _sessions.Remove(session);
            }
        }

        private byte[] HandleRequest(ManagementSession session, string line)
        {
            Dictionary<string, object> request;
            try
            {
                request = RouterManagementProtocol.Parse(line);
            }
            catch (Exception ex)
            {
                return RouterManagementProtocol.EncodeLine(
                    RouterManagementProtocol.Error(null, "malformed_request", ex.Message));
            }

            var id = RouterManagementProtocol.GetValue(request, "id");
            var type = RouterManagementProtocol.GetString(request, "type");
            if (string.IsNullOrWhiteSpace(type))
            {
                return RouterManagementProtocol.EncodeLine(
                    RouterManagementProtocol.Error(id, "missing_type", "A management operation is required."));
            }

            if (!string.Equals(type, "hello", StringComparison.Ordinal))
            {
                if (!RouterManagementProtocol.TryGetCompatibleVersion(request, out var code, out var error))
                {
                    return RouterManagementProtocol.EncodeLine(
                        RouterManagementProtocol.Error(id, code, error));
                }
            }

            try
            {
                switch (type)
                {
                    case "hello":
                        if (!RouterManagementProtocol.TryGetCompatibleVersion(request, out var helloCode, out var helloError))
                        {
                            return RouterManagementProtocol.EncodeLine(
                                RouterManagementProtocol.Error(id, helloCode, helloError));
                        }
                        return RouterManagementProtocol.EncodeLine(
                            RouterManagementProtocol.HelloResponse(id, _implementationVersion));

                    case "get_status":
                    case "get_links":
                        return RouterManagementProtocol.EncodeLine(
                            RouterManagementProtocol.StatusResponse(id, _router.GetStatusSnapshot()));

                    case "select_link":
                        return HandleSelectLink(request, id);

                    case "set_auto":
                        return HandleAuto(id);

                    case "subscribe":
                        session.SetSubscribed(true);
                        return RouterManagementProtocol.EncodeLine(
                            RouterManagementProtocol.CommandResponse(id, "subscribed", _router.GetStatusSnapshot()));

                    case "ping":
                        return RouterManagementProtocol.EncodeLine(
                            RouterManagementProtocol.Response(id, "pong", true));

                    default:
                        return RouterManagementProtocol.EncodeLine(
                            RouterManagementProtocol.Error(id, "unsupported_operation", "The operation is not supported."));
                }
            }
            catch (Exception ex)
            {
                return RouterManagementProtocol.EncodeLine(
                    RouterManagementProtocol.Error(id, "request_failed", ex.Message));
            }
        }

        private byte[] HandleSelectLink(IDictionary<string, object> request, object id)
        {
            var target = RouterManagementProtocol.GetString(request, "link");
            if (target == null)
            {
                return RouterManagementProtocol.EncodeLine(
                    RouterManagementProtocol.Error(id, "missing_link", "A stable link ID is required."));
            }

            if (!_router.TrySetManualOverride(target, out var errorCode, out var error))
            {
                return RouterManagementProtocol.EncodeLine(
                    RouterManagementProtocol.Error(id, errorCode, error));
            }

            return RouterManagementProtocol.EncodeLine(
                RouterManagementProtocol.CommandResponse(id, "select_link", _router.GetStatusSnapshot()));
        }

        private byte[] HandleAuto(object id)
        {
            if (!_router.TrySetManualOverride(LinkType.None, out var errorCode, out var error))
            {
                return RouterManagementProtocol.EncodeLine(
                    RouterManagementProtocol.Error(id, errorCode, error));
            }

            return RouterManagementProtocol.EncodeLine(
                RouterManagementProtocol.CommandResponse(id, "set_auto", _router.GetStatusSnapshot()));
        }

        private void OnFailover(object sender, FailoverEventArgs args)
        {
            PublishEvent("failover", args);
        }

        private void OnActiveLinkChanged(object sender, string link)
        {
            PublishEvent("active_link_changed", null);
        }

        private void OnStatsUpdated(object sender, EventArgs args)
        {
            PublishEvent("link_health_changed", null);
        }

        private void PublishEvent(string eventName, FailoverEventArgs failover)
        {
            lock (_gate)
            {
                if (!_running)
                {
                    return;
                }
                PublishEventLocked(eventName, failover);
            }
        }

        private void PublishEventLocked(string eventName, FailoverEventArgs failover)
        {
            var message = RouterManagementProtocol.EncodeLine(
                RouterManagementProtocol.Event(eventName, ++_eventSequence,
                    _router.GetStatusSnapshot(), failover));
            foreach (var session in _sessions.Where(item => item.IsSubscribed).ToArray())
            {
                session.EnqueueEvent(eventName, message);
            }
        }

    }
}
