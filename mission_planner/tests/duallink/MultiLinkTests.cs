// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using NOMAD.MissionPlanner;

internal static partial class DualLinkStressTests
{
    private static async Task LinkCollectionSizes()
    {
        foreach (int count in new[] { 1, 2, 4 })
        {
            int port = 31300 + count * 10;
            var config = MultiConfig(port);
            config.Links = Enumerable.Range(0, count).Select(i => new LinkConfig
                { Id = "path-" + i, Port = port + i + 1, Priority = 100 - i }).ToList();
            config.PreferredLink = "path-0";
            config.Consumers = new List<ConsumerConfig> { new ConsumerConfig { Id = "mp", RouterPort = port } };
            var peers = new List<UdpSink>();
            var pumps = new List<Pump>();
            try
            {
                using (var router = new GroundLinkRouter(config))
                using (var mp = UdpSink.ConnectedTo(port))
                {
                    router.Start();
                    mp.Send(Frames.Heartbeat(255, 190, 0));
                    foreach (var link in config.Links)
                    {
                        var peer = UdpSink.ConnectedTo(link.Port);
                        peers.Add(peer);
                        pumps.Add(new Pump(peer.Send, 10));
                    }
                    Check(await WaitUntil(() => router.Links.All(l => l.IsConnected), 2000),
                        count + " configured links are live");
                    await SelectEveryLink(router, config, peers, mp);
                }
            }
            finally
            {
                foreach (var pump in pumps) { pump.Dispose(); }
                foreach (var peer in peers) { peer.Dispose(); }
            }
        }
    }

    private static async Task SelectEveryLink(GroundLinkRouter router, GroundLinkRouter.RouterConfig config,
        List<UdpSink> peers, UdpSink mp)
    {
        for (int i = 0; i < peers.Count; i++)
        {
            Check(router.SetManualOverride(config.Links[i].Id), "any configured ID can be selected");
            mp.Drain();
            uint value = (uint)(90000 + i);
            peers[i].Send(Frames.Marker(value, (byte)i, 1, 71));
            var received = new List<uint>();
            Check(await WaitUntil(() => { DrainMarkers(mp, received); return received.Contains(value); }, 1000),
                "telemetry delivered from " + config.Links[i].Id);
            foreach (var peer in peers) { peer.Drain(); }
            mp.Send(Frames.Marker(value + 100, (byte)i, 255, 190));
            received.Clear();
            Check(await WaitUntil(() => { DrainMarkers(peers[i], received);
                return received.Contains(value + 100); }, 1000), "selected outbound transport receives command");
            await Task.Delay(30);
            Check(peers.Where((peer, index) => index != i).All(peer => DrainMarkers(peer).Count == 0),
                "no command fan-out with " + peers.Count + " enabled links");
        }
    }

    private static GroundLinkRouter.RouterConfig MultiConfig(int port)
    {
        return new GroundLinkRouter.RouterConfig
        {
            PreferredLink = "cell", HeartbeatTimeoutSec = .6, StatsTickMs = 20,
            FailoverCooldownSec = .1, PreferredLinkReconnectDelaySec = 1,
            Links = new List<LinkConfig>
            {
                new LinkConfig { Id = "cell", Port = port + 1, Priority = 100 },
                new LinkConfig { Id = "radio", Port = port + 2, Priority = 80 },
                new LinkConfig { Id = "wifi", Port = port + 3, Priority = 60 },
                new LinkConfig { Id = "disabled", Port = port + 4, Enabled = false }
            },
            Consumers = new List<ConsumerConfig>
            {
                new ConsumerConfig { Id = "mp", RouterPort = port },
                new ConsumerConfig { Id = "core", RouterPort = port + 5 },
                new ConsumerConfig { Id = "absent", RouterPort = port + 6, ClientPort = port + 7 }
            }
        };
    }

    private static void MultiConfigValidation()
    {
        Action<Action<GroundLinkRouter.RouterConfig>> rejects = change =>
        {
            var config = MultiConfig(31000);
            change(config);
            bool rejected = false;
            try { using (var router = new GroundLinkRouter(config)) { } }
            catch (ArgumentException) { rejected = true; }
            Check(rejected, "invalid configuration rejected before opening sockets");
        };
        rejects(c => c.Links[1].Id = "cell");
        rejects(c => c.Links[1].Port = c.Links[0].Port);
        rejects(c => c.Links[0].Transport = "BOGUS");
        rejects(c => c.Links[0].Port = 65536);
        rejects(c => c.HeartbeatTimeoutSec = double.NaN);
        rejects(c => c.PreferredLink = "disabled");
        rejects(c => c.Consumers[0].RouterPort = c.Links[0].Port);
        rejects(c => c.Consumers[0].ClientPort = c.Consumers[1].RouterPort);
        rejects(c => c.Consumers[0].ClientPort = c.Links[0].Port);
        rejects(c => { c.Links[0].RemoteHost = "127.0.0.1"; c.Links[0].RemotePort = c.Consumers[0].RouterPort; });
        rejects(c => c.ManagementPort = 0);
    }

    private static async Task MultiLinkRouting()
    {
        const int port = 31000;
        using (var router = new GroundLinkRouter(MultiConfig(port)))
        using (var cell = UdpSink.ConnectedTo(port + 1))
        using (var radio = UdpSink.ConnectedTo(port + 2))
        using (var wifi = UdpSink.ConnectedTo(port + 3))
        using (var mp = UdpSink.ConnectedTo(port))
        using (var core = UdpSink.ConnectedTo(port + 5))
        {
            router.Start();
            mp.Send(Frames.Heartbeat(255, 190, 0));
            core.Send(Frames.Heartbeat(254, 191, 0));
            using (var pump = new Pump(cell.Send, 10))
            using (var backup = new Pump(radio.Send, 10))
            using (var third = new Pump(wifi.Send, 10))
            {
                Check(await WaitUntil(() => router.Links.Count(s => s.IsConnected) == 3, 3000), "three links live");
                Check(!router.SetManualOverride("unknown"), "unknown override rejected");
                Check(!router.SetManualOverride("disabled"), "disabled override rejected");
                Check(router.ManualOverride == "", "rejected override leaves auto unchanged");
                await CheckSequenceIsolation(router, cell, radio, wifi);
                await CheckHandoverDedup(router, cell, radio, wifi, mp, core);
                await CheckConsumerIsolation(router, cell, radio, wifi, mp, core);
                await CheckCommandNoFanout(router, cell, radio, wifi, core);
                await CheckParameterPin(router, cell, radio, wifi, mp);
            }
            router.Stop();
            foreach (var number in new[] { port, port + 1, port + 2, port + 3, port + 5, port + 6 })
            { using (var rebound = new UdpClient(new IPEndPoint(IPAddress.Loopback, number))) { } }
            router.Start();
            Check(router.Links.All(s => !s.IsConnected), "restart discards old liveness");
            router.Stop();
        }
    }

    private static async Task ReadOnlyConsumerOutboundAdmission()
    {
        const int port = 32100;
        var config = MultiConfig(port);
        config.Links = new List<LinkConfig>
        {
            new LinkConfig { Id = "cell", Port = port + 1, Priority = 100 }
        };
        config.PreferredLink = "cell";
        var configuredConsumers = new List<ConsumerConfig>
        {
            new ConsumerConfig { Id = "mission_planner", RouterPort = port, AllowOutbound = false },
            new ConsumerConfig { Id = "nomad_core", RouterPort = port + 5 },
            new ConsumerConfig { Id = "pilot_native", RouterPort = port + 6 }
        };
        config.Consumers = configuredConsumers;
        Check(!config.Consumers[0].AllowOutbound,
            "host configuration keeps the Mission Planner consumer receive-only");
        Check(config.Consumers[1].AllowOutbound && config.Consumers[2].AllowOutbound,
            "host configuration keeps core and external pilot consumers command-capable");

        using (var router = new GroundLinkRouter(config))
        using (var aircraft = UdpSink.ConnectedTo(port + 1))
        using (var missionPlanner = UdpSink.ConnectedTo(port))
        using (var core = UdpSink.ConnectedTo(port + 5))
        using (var pilotNative = UdpSink.ConnectedTo(port + 6))
        {
            router.Start();
            using (var heartbeat = new Pump(aircraft.Send, 10))
            {
                missionPlanner.Send(Frames.Heartbeat(255, 190, 0));
                core.Send(Frames.Heartbeat(254, 191, 0));
                Check(await WaitUntil(() => router.Links[0].IsConnected, 2000),
                    "physical peer is live before checking outbound admission");

                var downlink = Frames.Marker(91001, 4, 1, 1);
                aircraft.Send(downlink);
                var missionPlannerTelemetry = new List<uint>();
                Check(await WaitUntil(() =>
                {
                    DrainMarkers(missionPlanner, missionPlannerTelemetry);
                    return missionPlannerTelemetry.Contains(91001);
                }, 1000), "receive-only Mission Planner consumer still receives aircraft telemetry");

                var commandLong = MakeArmCommandLong(21);
                var commandInt = Frames.V2(255, 190, 75, 22, new byte[35]);
                missionPlanner.Send(commandLong);
                missionPlanner.Send(commandInt);
                await Task.Delay(150);
                var rejected = aircraft.Drain();
                CheckEq(rejected.Count(frame => Frames.MsgIdOf(frame) == 76 || Frames.MsgIdOf(frame) == 75), 0,
                    "receive-only Mission Planner consumer sends no COMMAND_LONG or COMMAND_INT to aircraft");

                var coreCommand = MakeArmCommandLong(23);
                core.Send(coreCommand);
                var receivedCoreCommands = new List<byte[]>();
                Check(await WaitUntil(() =>
                {
                    receivedCoreCommands.AddRange(aircraft.Drain());
                    return receivedCoreCommands.Any(frame => frame.SequenceEqual(coreCommand));
                }, 1000), "command-capable NOMAD core consumer still reaches the selected physical peer");
                CheckEq(receivedCoreCommands.Count(frame => frame.SequenceEqual(coreCommand)), 1,
                    "the admitted core command crosses the physical boundary once");

                var pilotCommand = MakeArmCommandLong(24);
                pilotNative.Send(pilotCommand);
                var receivedPilotCommands = new List<byte[]>();
                Check(await WaitUntil(() =>
                {
                    receivedPilotCommands.AddRange(aircraft.Drain());
                    return receivedPilotCommands.Any(frame => frame.SequenceEqual(pilotCommand));
                }, 1000), "external pilot consumer still reaches the selected physical peer");
                CheckEq(receivedPilotCommands.Count(frame => frame.SequenceEqual(pilotCommand)), 1,
                    "external pilot command crosses the router peer once");
            }
        }

        // Model an explicit software-source revoke by rebuilding the router
        // with NOMAD's consumer inhibited. This is router evidence only; it
        // does not model flight-controller RC/native input selection.
        config.Consumers = new List<ConsumerConfig>
        {
            new ConsumerConfig { Id = "mission_planner", RouterPort = port, AllowOutbound = false },
            new ConsumerConfig { Id = "nomad_core", RouterPort = port + 5, AllowOutbound = false },
            new ConsumerConfig { Id = "pilot_native", RouterPort = port + 6, AllowOutbound = true }
        };
        using (var router = new GroundLinkRouter(config))
        using (var aircraft = UdpSink.ConnectedTo(port + 1))
        using (var missionPlanner = UdpSink.ConnectedTo(port))
        using (var core = UdpSink.ConnectedTo(port + 5))
        using (var pilotNative = UdpSink.ConnectedTo(port + 6))
        {
            router.Start();
            using (var heartbeat = new Pump(aircraft.Send, 10))
            {
                missionPlanner.Send(Frames.Heartbeat(255, 190, 0));
                core.Send(Frames.Heartbeat(254, 191, 0));
                Check(await WaitUntil(() => router.Links[0].IsConnected, 2000),
                    "physical peer is live after simulated NOMAD source revoke");

                var staleCoreCommand = MakeArmCommandLong(25);
                core.Send(staleCoreCommand);
                await Task.Delay(150);
                var afterRevoke = aircraft.Drain();
                CheckEq(afterRevoke.Count(frame => frame.SequenceEqual(staleCoreCommand)), 0,
                    "reconfigured router blocks a command from the inhibited NOMAD consumer");

                var pilotCommand = MakeArmCommandLong(26);
                pilotNative.Send(pilotCommand);
                var receivedPilotCommands = new List<byte[]>();
                Check(await WaitUntil(() =>
                {
                    receivedPilotCommands.AddRange(aircraft.Drain());
                    return receivedPilotCommands.Any(frame => frame.SequenceEqual(pilotCommand));
                }, 1000), "external pilot software consumer passes while NOMAD consumer is inhibited");
                CheckEq(receivedPilotCommands.Count(frame => frame.SequenceEqual(pilotCommand)), 1,
                    "pilot software command reaches the peer exactly once after NOMAD inhibition");
            }
        }

        Check(new ConsumerConfig { Id = "legacy", RouterPort = port }.AllowOutbound,
            "omitting AllowOutbound preserves the existing bidirectional default");
    }

    private static byte[] MakeArmCommandLong(byte sequence)
    {
        var payload = new byte[33];
        payload[28] = 144;
        payload[29] = 1;
        return Frames.V2(255, 190, 76, sequence, payload);
    }

    private static async Task CheckSequenceIsolation(GroundLinkRouter router, UdpSink cell, UdpSink radio,
        UdpSink wifi)
    {
        router.ResetCounters();
        foreach (var link in new[] { cell, radio, wifi })
        {
            for (int i = 0; i < 20; i++)
            {
                link.Send(Frames.Marker((uint)i, (byte)(250 + i), 1, 70));
                link.Send(Frames.Marker((uint)i, (byte)(100 + i), 2, 70));
            }
        }
        Check(await WaitUntil(() => router.Links.Take(3).All(l => l.FramesReceived >= 40), 2000),
            "every physical link observes independent system/component streams");
        Check(router.Links.Take(3).All(l => l.PacketLossPercent < .5), "sequence wrap has no phantom loss");
        // A partial frame on one source must not consume another source's bytes.
        cell.Send(Frames.Marker(88111, 1).Take(5).ToArray());
        var before = router.Links[2].FramesReceived;
        wifi.Send(Frames.Marker(88112, 1));
        Check(await WaitUntil(() => router.Links[2].FramesReceived > before, 1000),
            "one partial parser cannot corrupt another link");
        cell.Send(Frames.Marker(88111, 1).Skip(5).ToArray());
    }

    private static async Task CheckHandoverDedup(GroundLinkRouter router, UdpSink cell, UdpSink radio,
        UdpSink wifi, UdpSink mp, UdpSink core)
    {
        router.SetManualOverride("cell");
        mp.Drain(); core.Drain();
        var frame = Frames.Marker(88001, 8);
        cell.Send(frame);
        var first = new List<uint>();
        Check(await WaitUntil(() => {
            DrainMarkers(mp, first);
            return first.Contains(88001);
        }, 2000), "first delivery");
        router.SetManualOverride("radio");
        radio.Send(frame);
        await Task.Delay(30);
        router.SetManualOverride("wifi");
        wifi.Send(frame);
        await Task.Delay(150);
        DrainMarkers(mp, first);
        CheckEq(first.Count(m => m == 88001), 1, "three-link handover delivers one copy to MP");
        CheckEq(DrainMarkers(core).Count(m => m == 88001), 1, "three-link handover delivers one copy to core");
        // Standby-first arrival must not hide the later active copy.
        var next = Frames.Marker(88002, 9);
        cell.Send(next); radio.Send(next); await Task.Delay(30); wifi.Send(next);
        var received = new List<uint>();
        Check(await WaitUntil(() => { DrainMarkers(core, received); return received.Contains(88002); }, 2000),
            "standby-first copy cannot suppress active delivery");
    }

    private static async Task CheckConsumerIsolation(GroundLinkRouter router, UdpSink cell, UdpSink radio,
        UdpSink wifi, UdpSink mp, UdpSink core)
    {
        router.SetManualOverride("wifi");
        cell.Drain(); radio.Drain(); wifi.Drain(); mp.Drain(); core.Drain();
        var command = Frames.Marker(89001, 10, 255, 190);
        mp.Send(command.Take(7).ToArray());
        core.Send(Frames.Marker(89002, 11, 254, 191));
        mp.Send(command.Skip(7).ToArray());
        var received = new List<uint>();
        Check(await WaitUntil(() => { DrainMarkers(wifi, received); return received.Count >= 2; }, 2000),
            "both local parsers complete independently");
        await Task.Delay(100);
        DrainMarkers(wifi, received);
        CheckEq(received.Count(m => m == 89001), 1, "MP command emitted once");
        CheckEq(received.Count(m => m == 89002), 1, "core command emitted once");
        CheckEq(DrainMarkers(cell).Count, 0, "command never reaches cell standby");
        CheckEq(DrainMarkers(radio).Count, 0, "command never reaches radio standby");
        CheckEq(DrainMarkers(core).Count, 0, "local command not reflected to another consumer");
        var telemetry = Frames.Marker(89003, 12);
        wifi.Send(telemetry);
        var downlink = new List<uint>();
        Check(await WaitUntil(() => { DrainMarkers(mp, downlink); return downlink.Contains(89003); }, 2000),
            "absent third consumer does not block telemetry");
        mp.Send(telemetry); core.Send(telemetry);
        await Task.Delay(100);
        CheckEq(DrainMarkers(wifi).Count, 0, "reflected downlink is not sent back to aircraft");
    }

    private static async Task CheckCommandNoFanout(GroundLinkRouter router, UdpSink cell, UdpSink radio,
        UdpSink wifi, UdpSink core)
    {
        router.SetManualOverride("radio");
        cell.Drain(); radio.Drain(); wifi.Drain();
        // COMMAND_LONG: command field 400 (arm/disarm), all other payload fields zero.
        var payload = new byte[33];
        payload[28] = 144; payload[29] = 1;
        var command = Frames.V2(255, 190, 76, 33, payload);
        core.Send(command);
        var received = new List<byte[]>();
        Check(await WaitUntil(() => { received.AddRange(radio.Drain());
            return received.Any(f => f.SequenceEqual(command)); }, 2000), "ARM command reaches selected radio");
        await Task.Delay(150);
        received.AddRange(radio.Drain());
        CheckEq(received.Count(f => f.SequenceEqual(command)), 1, "ARM appears once on selected radio");
        CheckEq(cell.Drain().Count(f => Frames.MsgIdOf(f) == 76), 0, "ARM never reaches cell standby");
        CheckEq(wifi.Drain().Count(f => Frames.MsgIdOf(f) == 76), 0, "ARM never reaches Wi-Fi standby");
    }

    private static async Task CheckParameterPin(GroundLinkRouter router, UdpSink cell, UdpSink radio,
        UdpSink wifi, UdpSink mp)
    {
        router.SetManualOverride("cell");
        cell.Drain(); radio.Drain(); wifi.Drain();
        mp.Send(Frames.ParamRequestList(20));
        var frames = new List<byte[]>();
        Check(await WaitUntil(() => { frames.AddRange(cell.Drain()); return frames.Any(f => Frames.MsgIdOf(f) == 21); },
            2000), "parameter transaction starts on cell");
        router.SetManualOverride("wifi");
        mp.Send(Frames.V2(255, 190, 23, 21, new byte[23]));
        frames.Clear();
        Check(await WaitUntil(() => { frames.AddRange(cell.Drain()); return frames.Any(f => Frames.MsgIdOf(f) == 23); },
            2000), "PARAM_SET stays pinned despite manual selection change");
        CheckEq(wifi.Drain().Count(f => Frames.MsgIdOf(f) == 23), 0, "parameter write not copied to new active link");
        CheckEq(radio.Drain().Count(f => Frames.MsgIdOf(f) == 23), 0, "parameter write not copied to standby");
    }

    private static async Task InitialAnnouncement()
    {
        const int port = 31200;
        using (var aircraft = new UdpSink())
        using (var standby = new UdpSink())
        using (var mp = UdpSink.ConnectedTo(port))
        {
            var config = MultiConfig(port);
            config.Links[0].RemoteHost = "localhost";
            config.Links[0].RemotePort = ((IPEndPoint)aircraft.Client.Client.LocalEndPoint).Port;
            config.Links[1].RemoteHost = "127.0.0.1";
            config.Links[1].RemotePort = ((IPEndPoint)standby.Client.Client.LocalEndPoint).Port;
            using (var router = new GroundLinkRouter(config))
            {
                router.Start();
                Check(await WaitUntil(() => router.Links[0].IsOpen, 2000), "UDP hostname resolves");
                mp.Send(Frames.Heartbeat(255, 190, 1));
                var received = new List<byte[]>();
                Check(await WaitUntil(() => { received.AddRange(aircraft.Drain()); return received.Count == 1; },
                    2000), "configured UDP peer receives initial GCS announcement before telemetry");
                CheckEq(standby.Drain().Count, 0, "initial announcement is not broadcast");
            }
        }
    }

    private static async Task TcpIsolation()
    {
        const int port = 31100;
        var config = MultiConfig(port);
        config.Links[0].Transport = "TCP";
        config.Links[0].RemoteHost = "127.0.0.1";
        config.Links[0].ReconnectSeconds = .1;
        var listener = new TcpListener(IPAddress.Loopback, port + 1);
        listener.Start();
        try
        {
            using (var router = new GroundLinkRouter(config))
            using (var wifi = UdpSink.ConnectedTo(port + 3))
            using (var mp = UdpSink.ConnectedTo(port))
            using (var pump = new Pump(wifi.Send, 10))
            {
                router.Start();
                using (var peer = await listener.AcceptTcpClientAsync())
                {
                    router.SetManualOverride("cell");
                    mp.Send(Frames.Heartbeat(255, 190, 0));
                    Check(await WaitUntil(() => peer.Available > 0, 2000),
                        "TCP server receives GCS announcement before sending telemetry");
                    var heartbeat = Frames.Heartbeat(1, 1, 1);
                    await peer.GetStream().WriteAsync(heartbeat, 0, 4);
                    await peer.GetStream().WriteAsync(heartbeat, 4, heartbeat.Length - 4);
                    Check(await WaitUntil(() => router.Links[0].FramesReceived > 0, 2000), "TCP split frame parsed");
                    router.SetManualOverride("");
                }
                Check(await WaitUntil(() => router.ActiveLink == "wifi", 3000), "TCP close leaves UDP routing live");
                using (var reconnected = await listener.AcceptTcpClientAsync())
                { Check(reconnected.Connected, "TCP reconnect opens a fresh connection"); }
            }
        }
        finally { listener.Stop(); }
    }
}
