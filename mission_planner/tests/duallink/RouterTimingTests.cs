// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using NOMAD.MissionPlanner;

internal static partial class DualLinkStressTests
{
    private sealed class TimingClock
    {
        private double _seconds;
        internal double Seconds
        {
            get { return Volatile.Read(ref _seconds); }
            set { Volatile.Write(ref _seconds, value); }
        }
        internal DateTime Utc = new DateTime(2026, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        internal RouterClock Source => new RouterClock(() => Seconds, () => Utc);
        internal void Jump(double hours) { Utc = Utc.AddHours(hours); }
    }

    // Drive the actual worker operations without a background thread or real-time sleeps.
    private sealed class TimingRouter : IDisposable
    {
        internal readonly TimingClock Clock = new TimingClock();
        internal readonly GroundLinkRouter Router;
        internal readonly List<PhysicalLink> Links;

        internal TimingRouter()
        {
            var config = MultiConfig(32400);
            config.Links = config.Links.Take(2).ToList();
            config.Consumers = config.Consumers.Take(1).ToList();
            config.StatsTickMs = 250;
            config.HeartbeatTimeoutSec = 3;
            config.FailoverCooldownSec = 2;
            config.PreferredLinkReconnectDelaySec = 1;
            Router = new GroundLinkRouter(config, Clock.Source);
            Links = TimingField<List<PhysicalLink>>(Router, "_links");
            foreach (var consumer in TimingField<List<LocalConsumer>>(Router, "_consumers"))
            {
                consumer.Open();
            }
            foreach (var link in Links)
            {
                link.Open(Clock.Seconds);
            }
            TimingCall(Router, "SetActiveLink", "cell", "timing fixture");
        }

        internal void Receive(int index, byte sequence = 0)
        {
            var bytes = Frames.Heartbeat(1, 1, sequence);
            TimingCall(Router, "ProcessIncoming", Links[index], bytes, bytes.Length);
        }

        internal void Poll() { TimingCall(Router, "Poll"); }
        internal void Select() { TimingCall(Router, "SelectLink", Clock.Seconds); }
        public void Dispose() { Router.Dispose(); }
    }

    private static void CheckTimingEqual(double actual, double expected, string message)
    {
        Check(Math.Abs(actual - expected) < .000001, message + ": observed " + actual + ", expected " + expected);
    }

    private static object TimingCall(object target, string name, params object[] arguments)
    {
        return target.GetType().GetMethod(name, BindingFlags.NonPublic | BindingFlags.Instance)
            .Invoke(target, arguments);
    }

    private static T TimingField<T>(object target, string name)
    {
        return (T)target.GetType().GetField(name, BindingFlags.NonPublic | BindingFlags.Instance).GetValue(target);
    }

    private static void TimingSet(object target, string name, object value)
    {
        target.GetType().GetField(name, BindingFlags.NonPublic | BindingFlags.Instance).SetValue(target, value);
    }

    private static void ManagementClockJumps()
    {
        foreach (var jump in new[] { 48.0, -96.0 })
        {
            var clock = new TimingClock();
            using (var client = new StandaloneRouterClient(
                new MAVLinkConnectionManager.ConnectionConfig(), clock.Source))
            {
                Check(client.IsStatusStale, "no status is stale even at monotonic zero");
                var status = new RouterStatusSnapshot { Running = true, TimestampUtc = clock.Utc.ToString("O") };
                TimingCall(client, "ApplyStatus", status);
                clock.Jump(jump);
                Check(client.IsRouterAvailable, "UTC jump " + jump + "h does not expire management status");
                clock.Seconds = 3.5;
                Check(!client.IsStatusStale, "status remains fresh exactly at 3500ms");
                clock.Seconds = 3.501;
                Check(client.IsStatusStale, "status expires above 3500ms despite UTC jump");
                TimingCall(client, "ApplyStatus", status);
                clock.Seconds += 1;
                Check(client.IsRouterAvailable, "new status resets monotonic freshness");
            }
        }
    }

    private static void PhysicalFreshnessClockJumps()
    {
        foreach (var jump in new[] { 48.0, -96.0 })
        using (var fixture = new TimingRouter())
        {
            fixture.Receive(0);
            var observedUtc = fixture.Clock.Utc;
            fixture.Clock.Jump(jump);
            fixture.Clock.Seconds = 1;
            fixture.Receive(0, 1);
            CheckTimingEqual(fixture.Links[0].Stats.LatencyMs, 0.0, "UTC jump does not create heartbeat jitter");
            Check(fixture.Links[0].Stats.LastPacketTime == fixture.Clock.Utc, "public packet timestamp remains UTC");
            Check(fixture.Links[0].Stats.LastHeartbeatTime != observedUtc,
                "public heartbeat timestamp follows UTC jump");
            fixture.Clock.Seconds = 2.5;
            fixture.Poll();
            var status = fixture.Router.GetStatusSnapshot();
            Check(status.TimestampUtc == fixture.Clock.Utc.ToString("O"), "management timestamp remains UTC");
            CheckTimingEqual(status.Links[0].LastPacketAgeMs.Value, 1500.0, "management age uses monotonic time");
            Check(fixture.Links[0].Stats.Health == LinkHealth.Excellent, "heartbeat age exactly 1.5s is Excellent");
            fixture.Clock.Seconds = 2.751;
            fixture.Poll();
            Check(fixture.Links[0].Stats.Health == LinkHealth.Good, "heartbeat age above 1.5s is Good");
            fixture.Clock.Seconds = 3.999;
            fixture.Select();
            Check(fixture.Router.ActiveLink == "cell", "packet age below 3s remains usable");
            fixture.Clock.Seconds = 4;
            fixture.Select();
            Check(fixture.Router.ActiveLink == LinkType.None, "packet age exactly 3s is unavailable");
            fixture.Poll();
            Check(!fixture.Links[0].Stats.IsConnected, "stats report disconnected at packet timeout");
            Check(fixture.Router.FailoverLog.Last().Timestamp == fixture.Clock.Utc, "failover timestamp remains UTC");
        }
    }

    private static void StatsClockJumps()
    {
        foreach (var jump in new[] { 48.0, -96.0 })
        using (var fixture = new TimingRouter())
        {
            fixture.Receive(0);
            var bytes = fixture.Links[0].Stats.BytesReceived;
            fixture.Clock.Jump(jump);
            fixture.Poll();
            CheckTimingEqual(TimingField<double>(fixture.Router,
                "_lastTick"), 0.0, "UTC jump alone does not tick stats");
            fixture.Clock.Seconds = .249;
            fixture.Poll();
            CheckTimingEqual(fixture.Links[0].Stats.DataRateBps, 0.0, "stats wait until 250ms");
            fixture.Clock.Seconds = .250;
            fixture.Poll();
            CheckTimingEqual(fixture.Links[0].Stats.DataRateBps, bytes / .25 * .4,
                "stats rate uses 250ms elapsed window");
            CheckEq(TimingField<Queue<Action>>(fixture.Router, "_notifications").Count, 3,
                "initial selection plus exactly one stats notification");
            fixture.Clock.Jump(-jump);
            fixture.Poll();
            CheckTimingEqual(TimingField<double>(fixture.Router,
                "_lastTick"), .25, "reverse UTC jump does not tick again");
            fixture.Clock.Seconds = .5;
            fixture.Poll();
            CheckTimingEqual(fixture.Links[0].Stats.DataRateBps, bytes / .25 * .4 * .6,
                "normal next tick applies rate smoothing");
        }
    }

    private static void ReconnectClockJumps()
    {
        foreach (var jump in new[] { 48.0, -96.0 })
        using (var fixture = new TimingRouter())
        {
            var link = fixture.Links[0];
            link.Config.ReconnectSeconds = 2;
            link.Dispose();
            fixture.Clock.Jump(jump);
            fixture.Poll();
            Check(!link.Stats.IsOpen, "UTC jump alone does not reconnect");
            fixture.Clock.Seconds = 1.999;
            fixture.Poll();
            Check(!link.Stats.IsOpen, "reconnect waits below 2s");
            fixture.Clock.Seconds = 2;
            fixture.Poll();
            Check(link.Stats.IsOpen, "reconnect is eligible exactly at 2s");
            CheckTimingEqual(link.LastAttempt, 2.0, "attempt time is monotonic");
            Check(fixture.Router.GetStatusSnapshot().Links[0].LastPacketAgeMs == null,
                "reopened link has no packet age before receiving");
        }
    }

    private static void OpeningClockJumps()
    {
        foreach (var jump in new[] { 48.0, -96.0 })
        foreach (var operation in new[] { "_connecting", "_resolving" })
        using (var fixture = new TimingRouter())
        {
            var link = fixture.Links[0];
            link.Dispose();
            object task = operation == "_connecting" ? (object)new TaskCompletionSource<bool>().Task
                : new TaskCompletionSource<IPAddress[]>().Task;
            TimingSet(link, operation, task);
            fixture.Clock.Jump(jump);
            link.Poll((bytes, count) => { }, 1.999);
            var timedOut = false;
            try { link.Poll((bytes, count) => { }, 2); }
            catch (TimeoutException) { timedOut = true; }
            Check(timedOut, operation + " deadline is exactly 2s despite UTC jump");
        }
    }

    private static void FailoverClockJumps()
    {
        foreach (var jump in new[] { 48.0, -96.0 })
        using (var fixture = new TimingRouter())
        {
            fixture.Receive(0);
            fixture.Receive(1);
            TimingCall(fixture.Router, "SetActiveLink", "radio", "recovery fixture");
            fixture.Links[0].HealthySince = 0;
            fixture.Clock.Jump(jump);
            fixture.Clock.Seconds = 1.999;
            fixture.Select();
            Check(fixture.Router.ActiveLink == "radio", "recovered preferred link waits for 2s cooldown");
            fixture.Clock.Seconds = 2;
            fixture.Select();
            Check(fixture.Router.ActiveLink == "cell", "preferred link recovers at cooldown boundary");
            TimingCall(fixture.Router, "SetActiveLink", "radio", "dwell fixture");
            fixture.Clock.Seconds = 4;
            fixture.Receive(1, 1);
            fixture.Links[0].LastPacket = 4;
            fixture.Links[0].HealthySince = 3.001;
            fixture.Select();
            Check(fixture.Router.ActiveLink == "radio", "preferred link waits below 1s healthy dwell");
            fixture.Clock.Jump(-jump);
            fixture.Select();
            Check(fixture.Router.ActiveLink == "radio", "reverse UTC jump does not complete dwell");
            fixture.Links[0].HealthySince = 3;
            fixture.Select();
            Check(fixture.Router.ActiveLink == "cell", "preferred link recovers exactly at 1s healthy dwell");
            fixture.Clock.Seconds = 7;
            fixture.Receive(1, 2);
            Check(fixture.Router.ActiveLink == "radio", "dead active link fails over immediately despite cooldown");
        }
    }

    private static void ExpiryClockJumps()
    {
        foreach (var jump in new[] { 48.0, -96.0 })
        using (var fixture = new TimingRouter())
        {
            fixture.Receive(0);
            fixture.Clock.Jump(jump);
            fixture.Receive(1);
            CheckEq(fixture.Links[1].Stats.FramesDuplicate, 1L, "UTC jump does not expire cross-link duplicate window");
            var echo = TimingField<Dictionary<string, Tuple<string, double>>>(fixture.Router, "_forwarded");
            fixture.Clock.Seconds = .749;
            TimingCall(fixture.Router, "Sweep", fixture.Clock.Seconds);
            CheckEq(echo.Count, 1, "duplicate window retains frame below 750ms");
            fixture.Clock.Seconds = .750;
            fixture.Receive(1);
            CheckEq(fixture.Links[1].Stats.FramesDuplicate, 1L, "duplicate expires exactly at 750ms");
            TimingCall(fixture.Router, "Sweep", fixture.Clock.Seconds);
            CheckEq(echo.Count, 1, "sweep waits until its next 250ms cadence despite UTC jump");
            fixture.Clock.Seconds = .999;
            TimingCall(fixture.Router, "Sweep", fixture.Clock.Seconds);
            CheckEq(echo.Count, 0, "next sweep removes expired duplicate record");
            TimingSet(fixture.Router, "_paramLink", "cell");
            TimingSet(fixture.Router, "_paramActivity", 0.0);
            fixture.Clock.Seconds = 4;
            fixture.Receive(1, 1);
            Check(TimingField<string>(fixture.Router, "_paramLink") == "cell", "parameter pin remains at exactly 4s");
            fixture.Clock.Jump(-jump);
            fixture.Clock.Seconds = 4.001;
            fixture.Receive(1, 2);
            Check(TimingField<string>(fixture.Router, "_paramLink") == null, "parameter pin expires above 4s");
        }
    }

    private static void EchoClockJumps()
    {
        foreach (var jump in new[] { 48.0, -96.0 })
        using (var fixture = new TimingRouter())
        using (var peer = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0)))
        {
            fixture.Links[0].Stats.LastRemote = (IPEndPoint)peer.Client.LocalEndPoint;
            fixture.Receive(0);
            var bytes = Frames.Heartbeat(1, 1, 0);
            MavlinkFrame frame = default;
            new MavlinkFrameParser().Push(bytes, bytes.Length, parsed => frame = parsed);
            var consumer = new ConsumerConfig { AllowOutbound = true };
            fixture.Clock.Jump(jump);
            TimingCall(fixture.Router, "ForwardOutbound", consumer, frame);
            CheckEq(fixture.Links[0].Stats.BytesSentOutbound, 0, "UTC jump does not expire echo protection");
            fixture.Clock.Seconds = .749;
            TimingCall(fixture.Router, "ForwardOutbound", consumer, frame);
            CheckEq(fixture.Links[0].Stats.BytesSentOutbound, 0, "echo is blocked below 750ms");
            fixture.Clock.Seconds = .750;
            TimingCall(fixture.Router, "ForwardOutbound", consumer, frame);
            CheckEq(fixture.Links[0].Stats.BytesSentOutbound, bytes.Length, "echo window expires at exactly 750ms");
            if (fixture.Links[0].Stats.BytesSentOutbound == bytes.Length)
            {
                peer.Client.ReceiveTimeout = 1000;
                IPEndPoint sender = null;
                Check(peer.Receive(ref sender).SequenceEqual(bytes), "only the expired echo reaches physical peer");
            }
        }
    }

    private static async Task ManagementPollingClockJumps()
    {
        var clock = new TimingClock();
        var listener = new TcpListener(IPAddress.Loopback, 0);
        listener.Start();
        int requests = 0;
        using (var serverStarted = new ManualResetEventSlim())
        {
            var server = StartTimingStatusServer(
                listener, () => Interlocked.Increment(ref requests), serverStarted);
            bool serverReady = serverStarted.Wait(2000);
            Check(serverReady, "blocking status server worker starts before client polling");
            if (!serverReady)
            {
                listener.Stop();
                try { await server; }
                catch (SocketException) { }
                return;
            }

            var config = new MAVLinkConnectionManager.ConnectionConfig
                { ManagementPort = ((IPEndPoint)listener.LocalEndpoint).Port };
            using (var client = new StandaloneRouterClient(config, clock.Source))
            {
                client.Start();
                Check(await WaitUntil(() => Volatile.Read(ref requests) == 1, 2000),
                    "initial status poll is immediate");
                clock.Jump(48);
                Check(!await WaitUntil(() => Volatile.Read(ref requests) > 1, 1100),
                    "forward UTC jump and real-time waits do not advance injected polling clock");
                clock.Seconds = .999;
                Check(!await WaitUntil(() => Volatile.Read(ref requests) > 1, 350),
                    "polling waits below 1000ms");
                clock.Seconds = 1;
                Check(await WaitUntil(() => Volatile.Read(ref requests) == 2, 1000),
                    "poll occurs at exactly 1000ms");
                clock.Jump(-96);
                Check(!await WaitUntil(() => Volatile.Read(ref requests) > 2, 350),
                    "backward UTC jump does not repoll");
                clock.Seconds = 2;
                Check(await WaitUntil(() => Volatile.Read(ref requests) == 3, 1000),
                    "polling continues after backward jump");
                client.Stop();
            }
            listener.Stop();
            Check(await WaitUntil(() => server.IsCompleted, 2000), "timing status server closes with client");
            await server;
        }
    }

    private static Task StartTimingStatusServer(
        TcpListener listener,
        Action observedStatus,
        ManualResetEventSlim started)
    {
        return Task.Factory.StartNew(
            () =>
            {
                started.Set();
                ServeTimingStatus(listener, observedStatus);
            },
            CancellationToken.None,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default);
    }

    private static void ServeTimingStatus(TcpListener listener, Action observedStatus)
    {
        using (var peer = listener.AcceptTcpClient())
        using (var stream = peer.GetStream())
        {
            try
            {
                string line;
                while ((line = RouterManagementProtocol.ReadLine(stream)) != null)
                {
                    var request = RouterManagementProtocol.Parse(line);
                    var type = RouterManagementProtocol.GetString(request, "type");
                    var id = RouterManagementProtocol.GetValue(request, "id");
                    var response = type == "hello" ? RouterManagementProtocol.HelloResponse(id, "timing-test")
                        : RouterManagementProtocol.StatusResponse(id, new RouterStatusSnapshot { Running = true });
                    if (type == "get_status")
                    {
                        observedStatus();
                    }
                    var bytes = RouterManagementProtocol.EncodeLine(response);
                    stream.Write(bytes, 0, bytes.Length);
                }
            }
            catch (System.IO.IOException) { }
        }
    }

    private static async Task ConsumerEndpointClockJumps()
    {
        var clock = new TimingClock();
        using (var consumer = new LocalConsumer(new ConsumerConfig { Id = "timing", RouterPort = 32410 }, clock.Source))
        using (var first = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0)))
        using (var second = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0)))
        {
            consumer.Open();
            int received = 0;
            Action<MavlinkFrame> observe = frame => received++;
            await SendTimingConsumer(first, consumer, observe);
            CheckEq(received, 1, "first consumer endpoint is learned at monotonic zero");
            clock.Jump(48);
            await SendTimingConsumer(second, consumer, observe);
            CheckEq(received, 1, "UTC forward jump does not replace live consumer endpoint");
            clock.Seconds = 2.999;
            await SendTimingConsumer(second, consumer, observe);
            CheckEq(received, 1, "consumer endpoint remains pinned below 3s");
            clock.Jump(-96);
            clock.Seconds = 3;
            await SendTimingConsumer(second, consumer, observe);
            CheckEq(received, 2, "consumer endpoint may change at exactly 3s after backward UTC jump");
        }
    }

    private static async Task SendTimingConsumer(UdpClient peer, LocalConsumer consumer, Action<MavlinkFrame> observe)
    {
        var bytes = Frames.Heartbeat(255, 190, 0);
        peer.Send(bytes, bytes.Length, new IPEndPoint(IPAddress.Loopback, consumer.Config.RouterPort));
        var socket = TimingField<UdpClient>(consumer, "_socket");
        Check(await WaitUntil(() => socket.Available > 0, 1000), "consumer datagram arrives");
        consumer.Poll(observe);
    }

}
