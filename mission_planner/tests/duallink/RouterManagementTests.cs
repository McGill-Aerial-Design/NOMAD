// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using NOMAD.MissionPlanner;

internal static partial class DualLinkStressTests
{
    private static void RouterManagementSingleLinkStatus()
    {
        const int routerPort = 32200;
        const int managementPort = 32210;
        var config = MultiConfig(routerPort);
        config.Links = config.Links.Take(1).ToList();
        using (var router = new GroundLinkRouter(config))
        using (var server = new RouterManagementServer(router, "127.0.0.1", managementPort, "test-build"))
        {
            router.Start();
            server.Start();
            var response = Exchange(managementPort, Message("get_status", 1));
            Check(RouterManagementProtocol.TryReadStatus(response, out var status),
                "one-link status snapshot parses");
            Check(status?.ConfiguredLinkCount == 1, "one-link configured count is reported");
            Check(status?.Links.Count == 1 && status.Links[0].StableId == "cell",
                "one-link stable ID is preserved");
            Check(status?.ConnectedLinkCount == 0, "one-link disconnected state is reported");
        }
    }

    private static async Task RouterManagementProtocolChecks()
    {
        const int routerPort = 32000;
        const int managementPort = 32010;
        var routerConfig = MultiConfig(routerPort);
        using (var router = new GroundLinkRouter(routerConfig))
        using (var server = new RouterManagementServer(router, "127.0.0.1", managementPort, "test-build"))
        {
            using (var nonLoopback = new RouterManagementServer(router, "0.0.0.0", 0))
            {
                var rejected = false;
                try { nonLoopback.Start(); }
                catch (ArgumentException) { rejected = true; }
                Check(rejected, "non-loopback management bind is rejected");
            }

            router.Start();
            server.Start();
            Check(server.LocalEndpoint.Address.Equals(IPAddress.Loopback), "management binds loopback only");
            Check(server.LocalEndpoint.Port == managementPort, "management endpoint uses configured port");

            var hello = Exchange(managementPort, Message("hello", 1));
            Check(IsOk(hello), "compatible hello accepted");
            Check(RouterManagementProtocol.GetString(hello, "protocol") == RouterManagementProtocol.ProtocolName,
                "hello identifies protocol");
            Check(RouterManagementProtocol.GetInt(hello, "version") == RouterManagementProtocol.Version,
                "hello identifies version");

            var incompatible = Exchange(managementPort, new Dictionary<string, object>
            {
                ["id"] = 2, ["type"] = "hello", ["protocol"] = RouterManagementProtocol.ProtocolName, ["version"] = 99,
            });
            Check(!IsOk(incompatible), "incompatible hello rejected");
            Check(RouterManagementProtocol.GetString(incompatible, "errorCode") == "incompatible_version",
                "incompatible hello has structured error");

            var status = Exchange(managementPort, Message("get_status", 3));
            Check(IsOk(status), "status request accepted");
            Check(RouterManagementProtocol.TryReadStatus(status, out var snapshot), "status snapshot parses");
            if (snapshot != null)
            {
                Check(snapshot.Links.Count == 4, "status includes four arbitrary configured links");
                Check(snapshot.ConfiguredLinkCount == 4, "configured link count is authoritative");
                Check(snapshot.ActiveLinkId == "", "status reports no active link without telemetry");
                Check(snapshot.ManualOverrideId == "", "status reports automatic selection");
                Check(snapshot.Links.Any(link => link.StableId == "disabled" && !link.Enabled),
                    "status reports disabled links without hiding them");
            }

            Check(IsOk(Exchange(managementPort, Select("cell", 4))), "valid link selection accepted");
            var selected = Exchange(managementPort, Message("get_status", 5));
            RouterManagementProtocol.TryReadStatus(selected, out var selectedSnapshot);
            Check(selectedSnapshot?.ManualOverrideId == "cell", "manual override is reported");
            Check(selectedSnapshot?.ActiveLinkId == "cell", "status reports selected active link");

            var unknown = Exchange(managementPort, Select("missing", 6));
            Check(!IsOk(unknown), "unknown link rejected");
            Check(RouterManagementProtocol.GetString(unknown, "errorCode") == "unknown_link",
                "unknown link has structured rejection");
            var disabled = Exchange(managementPort, Select("disabled", 7));
            Check(!IsOk(disabled), "disabled link rejected");
            Check(RouterManagementProtocol.GetString(disabled, "errorCode") == "link_disabled",
                "disabled link has structured rejection");

            Check(IsOk(Exchange(managementPort, Message("set_auto", 8))), "auto selection accepted");
            var automatic = Exchange(managementPort, Message("get_status", 9));
            RouterManagementProtocol.TryReadStatus(automatic, out var automaticSnapshot);
            Check(automaticSnapshot?.ManualOverrideId == "", "auto releases manual override");

            var unsupported = Exchange(managementPort, Message("send_mavlink", 10));
            Check(!IsOk(unsupported), "raw MAVLink operation is rejected");
            Check(RouterManagementProtocol.GetString(unsupported, "errorCode") == "unsupported_operation",
                "raw MAVLink rejection is structured");

            Check(EventSubscriptionReportsFailover(router, managementPort), "subscribed client receives failover event");
            Check(MalformedRequestIsRejected(managementPort), "malformed request is rejected");
            Check(OversizedRequestIsRejected(managementPort), "oversized request is rejected");

            using (var physical = UdpSink.ConnectedTo(routerConfig.Links[0].Port))
            using (var consumer = UdpSink.ConnectedTo(routerConfig.Consumers[0].RouterPort))
            using (var pump = new Pump(physical.Send, 10))
            using (var slow = ConnectManagement(managementPort))
            {
                consumer.Send(Frames.Heartbeat(254, 191, 0));
                physical.Send(Frames.Heartbeat(255, 190, 0));
                Check(await WaitUntil(() => router.Links.Any(link =>
                    link.Type == "cell" && link.IsConnected), 1000),
                    "slow-client forwarding fixture has a live physical link");
                Exchange(slow, Message("hello", 11));
                Exchange(slow, Message("subscribe", 12));
                for (var i = 0; i < 1000; i++)
                {
                    router.SetManualOverride(i % 2 == 0 ? "cell" : "radio");
                }
                router.SetManualOverride("cell");
                var marker = Frames.Marker(93001, 15);
                physical.Send(marker);
                Check(await WaitUntil(() => router.Links.Any(link =>
                    link.Type == "cell" && link.FramesReceived >= 2), 1000),
                    "slow management client does not stop physical-link reads");
                Check(await WaitUntil(() => router.Links.Any(link =>
                    link.Type == "cell" && link.FramesForwarded > 0), 1000),
                    "slow management client does not stop router forwarding");
                var received = new List<uint>();
                Check(await WaitUntil(() =>
                {
                    DrainMarkers(consumer, received);
                    return received.Contains(93001);
                }, 1000), "slow management client does not block MAVLink forwarding");
            }
            Check(await RepeatedManagementDisconnects(managementPort),
                "repeated management client disconnects are cleaned up");

            server.Stop();
            var rebound = new TcpListener(IPAddress.Loopback, managementPort);
            try
            {
                rebound.Start();
            }
            finally { rebound.Stop(); }
            server.Start();
            Check(IsOk(Exchange(managementPort, Message("ping", 13))),
                "management endpoint can restart on the same port");
            router.Stop();
            await Task.CompletedTask;
        }
    }

    private static async Task StandaloneClientReconnects()
    {
        const int routerPort = 32100;
        const int managementPort = 32110;
        using (var defaultClient = new StandaloneRouterClient(new MAVLinkConnectionManager.ConnectionConfig()))
        {
            Check(defaultClient.LinkStatistics.Count == 2,
                "legacy LTE and RadioMaster links remain visible before host status arrives");
        }

        var routerConfig = MultiConfig(routerPort);
        var clientConfig = new MAVLinkConnectionManager.ConnectionConfig
        {
            // A legacy value must not restore Mission Planner-owned routing.
            RouterMode = "Embedded",
            Links = routerConfig.Links,
            RouterBindAddress = "127.0.0.1",
            RouterLocalPort = routerConfig.Consumers[0].RouterPort,
            ManagementBindAddress = "127.0.0.1",
            ManagementPort = managementPort,
        };
        using (var router = new GroundLinkRouter(routerConfig))
        using (var server = new RouterManagementServer(router, "127.0.0.1", managementPort, "test-build"))
        using (var client = new StandaloneRouterClient(clientConfig))
        using (var manager = new MAVLinkConnectionManager(clientConfig))
        {
            router.Start();
            server.Start();
            client.Start();
            manager.StartMonitoring();
            Check(await WaitUntil(() => client.IsRouterAvailable, 3000),
                "standalone client connects and receives status");
            Check(await WaitUntil(() => manager.IsRouterAvailable, 3000),
                "connection manager uses the standalone host despite a legacy embedded setting");
            Check(manager.RouterMode == "Standalone", "manager reports standalone ownership");
            Check(!manager.SupportsLiveConfiguration, "standalone manager limits live controls to selection");
            Check(client.LinkStatistics.Count == 4, "standalone client exposes arbitrary link collection");
            Check(manager.LinkStatistics.Count == 4, "manager projects arbitrary standalone link collection");
            Check(client.SwitchToLink("wifi"), "standalone client selects a valid link");
            Check(await WaitUntil(() => client.ManualOverride == "wifi", 1000),
                "standalone client observes manual override");
            Check(client.SwitchToLink(""), "standalone client releases manual override");

            server.Stop();
            Check(await WaitUntil(() => !client.IsMonitoring && client.IsStatusStale, 3000),
                "client marks management connection unavailable and stale");
            Check(client.LinkStatistics.All(link => link.IsStale), "disconnected status is marked stale");

            server.Start();
            Check(await WaitUntil(() => client.IsRouterAvailable, 4000),
                "client reconnects after management server restart");
            Check(client.LinkStatistics.Count == 4, "reconnect restores link collection");
            client.Stop();
            manager.StopMonitoring();
            Check(router.IsRunning, "client exit leaves physical router running");
        }
    }

    private static async Task<bool> RepeatedManagementDisconnects(int port)
    {
        for (var i = 0; i < 128; i++)
        {
            using (var client = ConnectManagement(port))
            {
                if (!IsOk(Exchange(client, Message("hello", 100 + i))) ||
                    !IsOk(Exchange(client, Message("ping", 300 + i))))
                {
                    return false;
                }
            }
            await Task.Delay(2);
        }

        return true;
    }

    private static bool EventSubscriptionReportsFailover(GroundLinkRouter router, int port)
    {
        using (var client = ConnectManagement(port))
        {
            var hello = Exchange(client, Message("hello", 20));
            var subscribed = Exchange(client, Message("subscribe", 21));
            if (!IsOk(hello) || !IsOk(subscribed))
            {
                return false;
            }

            router.SetManualOverride("radio");
            var deadline = DateTime.UtcNow.AddSeconds(2);
            while (DateTime.UtcNow < deadline)
            {
                try
                {
                    var eventMessage = RouterManagementProtocol.Parse(
                        RouterManagementProtocol.ReadLine(client.GetStream()));
                    if (RouterManagementProtocol.GetString(eventMessage, "type") == "event" &&
                        RouterManagementProtocol.GetString(eventMessage, "event") == "failover")
                    {
                        return true;
                    }
                }
                catch (IOException)
                {
                    continue;
                }
            }
            return false;
        }
    }

    private static bool MalformedRequestIsRejected(int port)
    {
        using (var client = ConnectManagement(port))
        {
            var bytes = Encoding.UTF8.GetBytes("not-json\n");
            client.GetStream().Write(bytes, 0, bytes.Length);
            var response = RouterManagementProtocol.Parse(RouterManagementProtocol.ReadLine(client.GetStream()));
            return !IsOk(response) && RouterManagementProtocol.GetString(response, "errorCode") == "malformed_request";
        }
    }

    private static bool OversizedRequestIsRejected(int port)
    {
        using (var client = ConnectManagement(port))
        {
            var bytes = Encoding.UTF8.GetBytes(new string('x', RouterManagementProtocol.MaxMessageBytes + 10) + "\n");
            client.GetStream().Write(bytes, 0, bytes.Length);
            var response = RouterManagementProtocol.Parse(RouterManagementProtocol.ReadLine(client.GetStream()));
            return !IsOk(response) && RouterManagementProtocol.GetString(response, "errorCode") == "malformed_request";
        }
    }

    private static TcpClient ConnectManagement(int port)
    {
        var client = new TcpClient(AddressFamily.InterNetwork);
        client.Connect(IPAddress.Loopback, port);
        client.GetStream().ReadTimeout = 2000;
        client.GetStream().WriteTimeout = 2000;
        return client;
    }

    private static Dictionary<string, object> Exchange(int port, Dictionary<string, object> message)
    {
        using (var client = ConnectManagement(port))
        {
            return Exchange(client, message);
        }
    }

    private static Dictionary<string, object> Exchange(
        TcpClient client,
        Dictionary<string, object> message)
    {
        var bytes = RouterManagementProtocol.EncodeLine(message);
        client.GetStream().Write(bytes, 0, bytes.Length);
        return RouterManagementProtocol.Parse(RouterManagementProtocol.ReadLine(client.GetStream()));
    }

    private static Dictionary<string, object> Message(string type, int id)
    {
        return new Dictionary<string, object>
        {
            ["id"] = id,
            ["type"] = type,
            ["protocol"] = RouterManagementProtocol.ProtocolName,
            ["version"] = RouterManagementProtocol.Version,
        };
    }

    private static Dictionary<string, object> Select(string link, int id)
    {
        var message = Message("select_link", id);
        message["link"] = link;
        return message;
    }

    private static bool IsOk(IDictionary<string, object> message)
    {
        var value = RouterManagementProtocol.GetValue(message, "ok");
        return value is bool boolean && boolean;
    }
}
