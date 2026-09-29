// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;

namespace NOMAD.MissionPlanner
{
    public partial class GroundLinkRouter
    {
        private static List<LinkConfig> TranslateLinks(RouterConfig c)
        {
            return c.Links;
        }

        private static void Validate(RouterConfig config, List<LinkConfig> links, List<ConsumerConfig> consumers)
        {
            if (links == null || links.Count == 0 || links.Count > 64 || !links.Any(l => l != null && l.Enabled))
            {
                throw new ArgumentException(
                    "Router Links are required; move legacy LTE/RadioMaster settings into the Links array.");
            }
            if (config.StatsTickMs < 10 || !Positive(config.HeartbeatTimeoutSec) ||
                !Positive(config.FailoverCooldownSec) || config.PreferredLinkReconnectDelaySec < 0)
            {
                throw new ArgumentException("Invalid router timing");
            }
            if (config.ManagementPort <= 0 || config.ManagementPort > 65535)
            {
                throw new ArgumentException("Management endpoint requires a valid port");
            }
            var ids = new HashSet<string>(StringComparer.Ordinal);
            var ports = new HashSet<int>();
            foreach (var link in links)
            {
                if (link == null || string.IsNullOrWhiteSpace(link.Id) || !ids.Add(link.Id))
                {
                    throw new ArgumentException("Link IDs must be nonempty and unique");
                }
                ValidateLink(link);
                if (link.Enabled && link.Transport == "UDP" && !ports.Add(link.Port))
                {
                    throw new ArgumentException("Physical UDP ports must be unique");
                }
            }
            if (!string.IsNullOrEmpty(config.PreferredLink) &&
                !links.Any(l => l.Id == config.PreferredLink && l.Enabled))
            {
                throw new ArgumentException("Preferred link is unknown or disabled");
            }
            if (consumers == null)
            {
                throw new ArgumentException("Router Consumers are required; configure each local client in Consumers.");
            }
            ValidateConsumers(consumers, ports);
            foreach (var link in links.Where(l => l.Enabled && l.Transport == "UDP"))
            {
                if (IPAddress.TryParse(link.RemoteHost, out var remote))
                {
                    LocalAddressGuard.ValidateRemote(remote, link.RemotePort, ports);
                }
            }
        }

        private static bool Positive(double value) => value > 0 && !double.IsInfinity(value);
        private static bool Port(int port) => port > 0 && port <= 65535;

        private static void ValidateLink(LinkConfig link)
        {
            if (!Positive(link.ReconnectSeconds))
            {
                throw new ArgumentException("Invalid reconnect interval");
            }
            if (link.Transport == "COM")
            {
                if (string.IsNullOrWhiteSpace(link.Device) || link.BaudRate <= 0)
                {
                    throw new ArgumentException("Serial device and positive baud required");
                }
                return;
            }
            if (link.Transport != "UDP" && link.Transport != "TCP")
            {
                throw new ArgumentException("Transport must be UDP, TCP or COM");
            }
            if (!Port(link.Port))
            {
                throw new ArgumentException("Invalid physical port");
            }
            if (!IPAddress.TryParse(link.BindAddress, out var address) ||
                address.AddressFamily != System.Net.Sockets.AddressFamily.InterNetwork)
            {
                throw new ArgumentException("Physical bind address must be IPv4");
            }
            if (link.Transport == "TCP" && string.IsNullOrWhiteSpace(link.RemoteHost))
            {
                throw new ArgumentException("TCP host required");
            }
            if (link.Transport == "UDP" && !string.IsNullOrEmpty(link.RemoteHost) &&
                (Uri.CheckHostName(link.RemoteHost) == UriHostNameType.Unknown || !Port(link.RemotePort)))
            {
                throw new ArgumentException("UDP remote requires a valid host and port");
            }
        }

        private static void ValidateConsumers(List<ConsumerConfig> consumers, HashSet<int> ports)
        {
            if (consumers.Count == 0 || consumers.Count > 32)
            {
                throw new ArgumentException("Configure between one and 32 consumers");
            }
            var ids = new HashSet<string>();
            foreach (var consumer in consumers)
            {
                if (consumer == null || string.IsNullOrWhiteSpace(consumer.Id) || !ids.Add(consumer.Id) ||
                    !Port(consumer.RouterPort) || !ports.Add(consumer.RouterPort))
                {
                    throw new ArgumentException("Invalid or colliding consumer endpoint");
                }
                if (consumer.Id == "mission_planner" && consumer.AllowOutbound)
                {
                    throw new ArgumentException(
                        "The mission_planner consumer is receive-only; set AllowOutbound to false.");
                }
            }
            foreach (var consumer in consumers)
            {
                if (consumer.ClientPort != 0 && (!Port(consumer.ClientPort) || !ports.Add(consumer.ClientPort)))
                {
                    throw new ArgumentException("Client listener collides with a router or physical endpoint");
                }
            }
        }
    }
}
