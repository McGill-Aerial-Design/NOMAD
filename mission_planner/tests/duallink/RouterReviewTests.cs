// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Threading.Tasks;
using System.Web.Script.Serialization;
using NOMAD.MissionPlanner;

internal static partial class DualLinkStressTests
{
    private static void MissionPlannerOutboundConfigRejection()
    {
        var serializer = new JavaScriptSerializer();
        const string omittedOutbound =
            "{\"Links\":[{\"Id\":\"cell\",\"Port\":31101}],"
            + "\"PreferredLink\":\"cell\","
            + "\"Consumers\":[{\"Id\":\"mission_planner\",\"RouterPort\":31100}]}";
        const string enabledOutbound =
            "{\"Links\":[{\"Id\":\"cell\",\"Port\":31101}],"
            + "\"PreferredLink\":\"cell\","
            + "\"Consumers\":[{\"Id\":\"mission_planner\",\"RouterPort\":31100,"
            + "\"AllowOutbound\":true}]}";
        var omittedConfig = serializer.Deserialize<GroundLinkRouter.RouterConfig>(omittedOutbound);
        var enabledConfig = serializer.Deserialize<GroundLinkRouter.RouterConfig>(enabledOutbound);

        Check(omittedConfig.Consumers[0].AllowOutbound,
            "omitted Mission Planner AllowOutbound resolves to the legacy true default");
        CheckMissionPlannerOutboundRejected(omittedConfig, "omitted AllowOutbound");
        CheckMissionPlannerOutboundRejected(enabledConfig, "AllowOutbound true");
    }

    private static void RouterRequiresExplicitTopology()
    {
        var missingLinks = MultiConfig(30900);
        missingLinks.Links = null;
        CheckConfigurationRejected(missingLinks, "Router Links are required");

        var missingConsumers = MultiConfig(30900);
        missingConsumers.Consumers = null;
        CheckConfigurationRejected(missingConsumers, "Router Consumers are required");
    }

    private static void CheckConfigurationRejected(GroundLinkRouter.RouterConfig config, string expected)
    {
        string error = null;
        try { using (var router = new GroundLinkRouter(config)) { } }
        catch (ArgumentException ex) { error = ex.Message; }

        Check(error != null && error.Contains(expected), "missing router topology fails clearly: " + expected);
    }

    private static void CheckMissionPlannerOutboundRejected(GroundLinkRouter.RouterConfig config, string scenario)
    {
        string error = null;
        try { using (var router = new GroundLinkRouter(config)) { } }
        catch (ArgumentException ex) { error = ex.Message; }

        Check(error == "The mission_planner consumer is receive-only; set AllowOutbound to false.",
            "unsafe Mission Planner " + scenario + " is rejected with a clear error");
    }

    private static void LocalAddressGuardChecks()
    {
        var local = new[] { IPAddress.Parse("192.0.2.10"), IPAddress.Parse("198.51.100.20") };
        var ports = new HashSet<int> { 31500, 31501, 31505, 31507 };
        foreach (string address in new[] { "127.0.0.1", "127.0.0.2", "::1", "192.0.2.10",
            "198.51.100.20", "::ffff:192.0.2.10" })
        {
            var parsed = IPAddress.Parse(address);
            Check(LocalAddressGuard.IsLocalAddress(parsed, local), address + " is local");
            foreach (int port in ports)
            {
                bool rejected = false;
                try { LocalAddressGuard.ValidateRemote(parsed, port, ports, local); }
                catch (ArgumentException) { rejected = true; }
                Check(rejected, address + ":" + port + " must reject a topology loop");
            }
            LocalAddressGuard.ValidateRemote(parsed, 31999, ports, local);
        }
        foreach (string address in new[] { "192.0.2.11", "203.0.113.10", "2001:db8::1" })
        {
            var parsed = IPAddress.Parse(address);
            Check(!LocalAddressGuard.IsLocalAddress(parsed, local), address + " is not a local IPv4 interface");
            LocalAddressGuard.ValidateRemote(parsed, 31501, ports, local);
        }
        Check(!LocalAddressGuard.IsLocalAddress(local[0], Array.Empty<IPAddress>()),
            "interface membership is derived from the supplied current addresses");
    }

    private static void LinkStatusDisplayChecks()
    {
        foreach (int count in new[] { 0, 1, 3, 4 })
        {
            var links = Enumerable.Range(0, count).Select(i => new LinkStatistics
                { Type = "path-" + i, IsConnected = i % 2 == 0 }).ToList();
            string expected = "Router: running — " + ((count + 1) / 2) + "/" + count + " links connected";
            Check(LinkStatusDisplay.FormatRouterStatus(true, links) == expected, expected);
            Check(LinkStatusDisplay.FormatRouterStatus(false, links) == "Router: stopped", "stopped router status");
            var reversed = links.Select(link => link.Type).Reverse().ToList();
            Check(!LinkStatusDisplay.HasMembershipChanged(links, reversed), "card order does not change membership");
            reversed.Add("extra");
            Check(LinkStatusDisplay.HasMembershipChanged(links, reversed), "extra card changes membership");
        }
        var wifi = new[] { new LinkStatistics { Type = "wifi", IsConnected = false } };
        Check(LinkStatusDisplay.FormatRouterStatus(true, wifi) == "Router: running — 0/1 links connected",
            "running does not imply connected");
        Check(LinkStatusDisplay.FormatRouterStatus(false, false, wifi) == "Router process: unavailable",
            "management failure is distinct from physical link health");
        wifi[0].IsConnected = true;
        wifi[0].IsStale = true;
        Check(
            LinkStatusDisplay.FormatRouterStatus(true, true, wifi) == "Router: connected — Physical links: 0/1 connected",
            "stale management data is not shown as current");
        Check(LinkStatusDisplay.HasMembershipChanged(wifi, new[] { "WiFi" }), "link IDs remain case sensitive");
        Check(LinkStatusDisplay.HasMembershipChanged(wifi, Array.Empty<string>()), "missing card changes membership");
    }

    private static async Task ResolvedLocalDestination()
    {
        var config = MultiConfig(31500);
        config.Links[0].RemoteHost = "localhost";
        config.Links[0].RemotePort = config.Consumers[0].RouterPort;
        var messages = new ConcurrentQueue<string>();
        using (var router = new GroundLinkRouter(config))
        using (var peer = UdpSink.ConnectedTo(config.Links[1].Port))
        {
            router.LogMessage += (sender, message) => messages.Enqueue(message);
            router.Start();
            using (var pump = new Pump(peer.Send, 10))
            {
                Check(await WaitUntil(() => messages.Any(message => message.Contains("local router topology")), 3000),
                    "hostname resolving to a local topology endpoint is rejected");
                Check(!router.Links[0].IsOpen, "rejected resolved destination leaves its link closed");
                Check(await WaitUntil(() => router.Links[1].IsConnected, 2000),
                    "another physical link remains connected after DNS loop rejection");
            }
        }
    }
}
