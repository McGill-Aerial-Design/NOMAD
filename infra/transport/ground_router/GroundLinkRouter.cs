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
    public partial class GroundLinkRouter : IDisposable
    {
        private readonly RouterConfig _cfg;
        private readonly List<PhysicalLink> _links;
        private readonly List<LocalConsumer> _consumers;
        private readonly object _gate = new object();
        private readonly Queue<Action> _notifications = new Queue<Action>();
        private readonly Queue<FailoverEventArgs> _failovers = new Queue<FailoverEventArgs>();
        private Thread _worker;
        private CancellationTokenSource _stop;
        private volatile bool _running;
        private DateTime _lastTick, _lastSwitch;
        public string ActiveLink { get; private set; } = LinkType.None;
        public string ManualOverride { get; private set; } = LinkType.None;
        public bool IsRunning => _running;
        public RouterConfig Config => _cfg;
        public IPEndPoint LocalEndpoint => new IPEndPoint(IPAddress.Loopback, _consumers[0].Config.RouterPort);
        public IReadOnlyList<LinkSourceStats> Links
        { get { lock (_gate) { return _links.Select(l => CopyStats(l.Stats)).ToList(); } } }
        // Compatibility projections; all routing operates on _links.
        public LinkSourceStats Lte => GetStats(LinkType.LTE);
        public LinkSourceStats Radio => GetStats(LinkType.RadioMaster);
        public IReadOnlyCollection<FailoverEventArgs> FailoverLog
        { get { lock (_gate) { return _failovers.ToArray(); } } }
        public event EventHandler<string> ActiveLinkChanged;
        public event EventHandler<FailoverEventArgs> FailoverOccurred;
        public event EventHandler StatsUpdated;
        public event EventHandler<string> LogMessage;

        public GroundLinkRouter(RouterConfig config)
        {
            _cfg = config ?? throw new ArgumentNullException(nameof(config));
            var links = TranslateLinks(config);
            var consumers = config.Consumers ?? new List<ConsumerConfig>
            { new ConsumerConfig { Id = "mission_planner", RouterPort = config.LocalPort } };
            Validate(config, links, consumers);
            var localPorts = new HashSet<int>(links.Where(l => l.Enabled && l.Transport == "UDP")
                .Select(l => l.Port).Concat(consumers.Select(c => c.RouterPort))
                .Concat(consumers.Where(c => c.ClientPort != 0).Select(c => c.ClientPort)));
            _links = links.Select(l => new PhysicalLink(l.Snapshot(), localPorts)).ToList();
            _consumers = consumers.Select(c => new LocalConsumer(c.Snapshot())).ToList();
        }

        private LinkSourceStats GetStats(string id)
        { lock (_gate) { return CopyStats(_links.FirstOrDefault(l => l.Config.Id == id)?.Stats ??
            new LinkSourceStats { Type = id }); } }
        private static LinkSourceStats CopyStats(LinkSourceStats stats) => stats.Snapshot();

        public void Start()
        {
            lock (_gate)
            {
                if (_running)
                {
                    return;
                }
                try
                {
                    _notifications.Clear();
                    ResetCounters();
                    foreach (var consumer in _consumers)
                    {
                        consumer.Open();
                    }
                    foreach (var link in _links.Where(l => l.Config.Enabled))
                    {
                        TryOpen(link, DateTime.UtcNow);
                    }
                    ActiveLink = string.IsNullOrEmpty(_cfg.PreferredLink)
                        ? _links.Where(l => l.Config.Enabled).OrderByDescending(l => l.Config.Priority)
                            .ThenBy(l => l.Config.Id, StringComparer.Ordinal).First().Config.Id
                        : _cfg.PreferredLink;
                    ManualOverride = LinkType.None;
                    _lastTick = DateTime.UtcNow;
                    _lastSwitch = DateTime.MinValue;
                    _running = true;
                    _stop = new CancellationTokenSource();
                    var token = _stop.Token;
                    _worker = new Thread(() => Run(token)) { IsBackground = true, Name = "NOMAD MAVLink router" };
                    _worker.Start();
                }
                catch
                {
                    CloseSockets(); throw;
                }
            }
        }

        public void Stop()
        {
            Thread worker;
            CancellationTokenSource stop;
            lock (_gate)
            {
                _running = false;
                worker = _worker;
                stop = _stop;
                _worker = null;
                _stop = null;
                stop?.Cancel();
                CloseSockets();
            }
            if (worker != null && worker != Thread.CurrentThread)
            {
                worker.Join();
            }
            stop?.Dispose();
        }
        public void Dispose() => Stop();
        private void CloseSockets()
        {
            foreach (var link in _links)
            {
                link.Dispose();
            }
            foreach (var consumer in _consumers)
            {
                consumer.Dispose();
            }
        }

        private void Run(CancellationToken stop)
        {
            while (!stop.IsCancellationRequested)
            {
                try
                {
                    lock (_gate)
                    {
                        if (!stop.IsCancellationRequested)
                        {
                            Poll();
                        }
                    }
                    PublishNotifications(stop);
                }
                catch (Exception ex)
                {
                    EmitLog("Router worker: " + ex.Message);
                }
                Thread.Sleep(2);
            }
        }

        private void EnqueueNotification(Action notification)
        {
            if (_notifications.Count >= 128)
            {
                _notifications.Dequeue();
            }
            _notifications.Enqueue(notification);
        }

        private void PublishNotifications(CancellationToken stop)
        {
            Action[] notifications;
            lock (_gate)
            {
                notifications = _notifications.ToArray();
                _notifications.Clear();
            }
            foreach (var notification in notifications)
            {
                if (stop.IsCancellationRequested)
                {
                    return;
                }
                try { notification(); }
                catch (Exception ex) { EmitLog("Status subscriber: " + ex.Message); }
            }
        }

        private void Poll()
        {
            var now = DateTime.UtcNow;
            foreach (var link in _links.Where(l => l.Config.Enabled))
            {
                try
                {
                    if (!link.Stats.IsOpen && !link.Opening &&
                        (now - link.LastAttempt).TotalSeconds >= link.Config.ReconnectSeconds) { TryOpen(link, now); }
                    link.Poll((bytes, count) => ProcessIncoming(link, bytes, count), now);
                }
                catch (Exception ex)
                {
                    link.Dispose(); EmitLog(link.Config.Id + ": " + ex.Message);
                }
            }
            if ((now - _lastTick).TotalMilliseconds >= _cfg.StatsTickMs)
            {
                Tick(now);
            }
            foreach (var consumer in _consumers)
            {
                try { consumer.Poll(frame => ForwardOutbound(consumer.Config, frame)); }
                catch (Exception ex)
                {
                    EmitLog(consumer.Config.Id + ": " + ex.Message);
                }
            }
        }

        private void TryOpen(PhysicalLink link, DateTime now)
        {
            try { link.Open(now); }
            catch (Exception ex)
            {
                link.Dispose(); EmitLog(link.Config.Id + " open: " + ex.Message);
            }
        }

        public bool SetManualOverride(string target)
        {
            return TrySetManualOverride(target, out _, out _);
        }

        public bool TrySetManualOverride(string target, out string errorCode, out string error)
        {
            lock (_gate)
            {
                errorCode = null;
                error = null;
                if (!_running)
                {
                    errorCode = "router_stopped";
                    error = "The router is not running.";
                    return false;
                }
                if (target == null)
                {
                    errorCode = "invalid_link";
                    error = "A link ID is required.";
                    EmitLog("Override rejected: missing link ID");
                    return false;
                }
                var link = _links.FirstOrDefault(l => l.Config.Id == target);
                if (target != LinkType.None && link == null)
                {
                    errorCode = "unknown_link";
                    error = "The requested link ID is not configured.";
                    EmitLog("Override rejected: unknown link");
                    return false;
                }
                if (target != LinkType.None && !link.Config.Enabled)
                {
                    errorCode = "link_disabled";
                    error = "The requested link is disabled.";
                    EmitLog("Override rejected: disabled link");
                    return false;
                }
                ManualOverride = target;
                if (target != LinkType.None)
                {
                    SetActiveLink(target, "manual override");
                }
                else
                {
                    SelectLink(DateTime.UtcNow);
                }
                return true;
            }
        }

        private void SetActiveLink(string id, string reason)
        {
            if (id == ActiveLink)
            {
                return;
            }
            var change = new FailoverEventArgs { FromLink = ActiveLink, ToLink = id,
                Reason = reason, Timestamp = DateTime.UtcNow };
            ActiveLink = id;
            _lastSwitch = change.Timestamp;
            _failovers.Enqueue(change);
            while (_failovers.Count > 50)
            {
                _failovers.Dequeue();
            }
            EnqueueNotification(() => FailoverOccurred?.Invoke(this, change));
            EnqueueNotification(() => ActiveLinkChanged?.Invoke(this, id));
        }

        public void ResetCounters()
        {
            lock (_gate)
            {
                foreach (var link in _links)
                {
                    var s = link.Stats;
                    s.FramesReceived = s.FramesForwarded = s.FramesDuplicate = s.FrameErrors = 0;
                    s.BytesReceived = s.BytesSentOutbound = link.PreviousBytes = link.Seen = link.Lost = 0;
                    s.PacketLossPercent = s.DataRateBps = 0;
                    link.Sequences.Clear();
                }
                _forwarded.Clear(); _paramLink = null; _paramActivity = DateTime.MinValue;
            }
        }
        public string GetStatusSummary() => "Active: " + ActiveLink + " | " +
            string.Join(" | ", Links.Select(s => s.Name + ": " + s.Health));
        private void EmitLog(string message) { try { LogMessage?.Invoke(this, message); } catch { } }
    }
}
