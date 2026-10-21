// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using NOMAD.MissionPlanner;

internal static partial class GroundRouterTests
{
    private static async Task PluginOwnerLifecycle()
    {
        using (var manager = new MAVLinkConnectionManager())
        using (var primary = new UdpSink())
        using (var backup = new UdpSink())
        using (var mp = new UdpSink())
        {
            manager.StartMonitoring();
            Check(!manager.IsMonitoring, "empty profile opens no physical links");
            var config = new MAVLinkConnectionManager.ConnectionConfig
            {
                RouterLocalPort = 33100, PreferredLink = "elrs",
                Links = new List<LinkConfig>
                {
                    new LinkConfig { Id = "elrs", Name = "Primary", Port = 33101,
                        BindAddress = "127.0.0.1", RemoteHost = "127.0.0.1",
                        RemotePort = ((IPEndPoint)primary.Client.Client.LocalEndPoint).Port, Priority = 100 },
                    new LinkConfig { Id = "lte", Name = "Backup", Port = 33102,
                        BindAddress = "127.0.0.1", RemoteHost = "127.0.0.1",
                        RemotePort = ((IPEndPoint)backup.Client.Client.LocalEndPoint).Port, Priority = 50 }
                }
            };
            var topology = MAVLinkConnectionManager.BuildRouterConfig(config);
            Check(topology.Consumers.Count == 1 && topology.Consumers[0].Id == "mission_planner" &&
                topology.Consumers[0].AllowOutbound, "only Mission Planner owns outbound routing");
            manager.UpdateConfig(config);
            manager.StartMonitoring();
            using (var primaryPump = new Pump(bytes => primary.SendTo(33101, bytes), 10))
            using (var backupPump = new Pump(bytes => backup.SendTo(33102, bytes), 10))
            {
                Check(await WaitUntil(() => manager.LinkStatistics.Count == 2 &&
                    manager.LinkStatistics.All(link => link.IsConnected), 2000), "both physical links become observable");
                Check(manager.SwitchToLink("elrs"), "primary can be selected");
                primary.Drain(); backup.Drain();
                var command = Frames.Marker(451, 17, 255, 190);
                mp.SendTo(33100, command);
                Check(await WaitUntil(() => primary.Rx.Any(frame => Frames.MarkerIdOf(frame) == 451), 2000),
                    "native Mission Planner outbound reaches selected primary");
                await Task.Delay(100);
                CheckEq(primary.Drain().Count(frame => Frames.MarkerIdOf(frame) == 451), 1,
                    "command reaches primary once");
                CheckEq(backup.Drain().Count(frame => Frames.MarkerIdOf(frame) == 451), 0,
                    "command is not mirrored onto backup");
                Check(manager.SwitchToLink("lte"), "backup can be selected");
                mp.SendTo(33100, Frames.Marker(452, 18, 255, 190));
                Check(await WaitUntil(() => backup.Rx.Any(frame => Frames.MarkerIdOf(frame) == 452), 2000),
                    "manual selection directs native outbound to backup");
            }
            var oldRouter = typeof(MAVLinkConnectionManager).GetField("_router",
                System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic).GetValue(manager);
            manager.UpdateConfig(new MAVLinkConnectionManager.ConnectionConfig());
            int retiredEvents = 0;
            manager.FailoverOccurred += (sender, args) => retiredEvents++;
            manager.ActiveLinkChanged += (sender, link) => retiredEvents++;
            manager.LogMessage += (sender, message) => retiredEvents++;
            TimingCall(manager, "OnFailover", oldRouter, new FailoverEventArgs());
            TimingCall(manager, "OnActiveLinkChanged", oldRouter, "elrs");
            TimingCall(manager, "OnLogMessage", oldRouter, "retired");
            Check(retiredEvents == 0, "retired router cannot publish callbacks into the new profile");
            Check(!manager.IsMonitoring && manager.LinkStatistics.Count == 0,
                "profile replacement retires old router and observations");
            using (var endpoint = new UdpClient(new IPEndPoint(IPAddress.Loopback, 33100))) { }
            using (var endpoint = new UdpClient(new IPEndPoint(IPAddress.Loopback, 33101))) { }
            manager.UpdateConfig(config);
            manager.StartMonitoring();
            Check(manager.IsMonitoring, "new owner can rebind old profile endpoints");
            manager.Dispose();
            using (var endpoint = new UdpClient(new IPEndPoint(IPAddress.Loopback, 33100))) { }
            Check(!manager.IsMonitoring, "plugin exit releases local endpoint and worker");
        }
    }
}
