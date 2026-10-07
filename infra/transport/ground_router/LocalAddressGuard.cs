// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;

namespace NOMAD.MissionPlanner
{
    internal static class LocalAddressGuard
    {
        internal static void ValidateRemote(IPAddress address, int port, ISet<int> localPorts)
        {
            if (!localPorts.Contains(port))
            {
                return;
            }
            ValidateRemote(address, port, localPorts, GetLocalIPv4Addresses());
        }

        internal static void ValidateRemote(IPAddress address, int port, ISet<int> localPorts,
            IEnumerable<IPAddress> localAddresses)
        {
            if (localPorts.Contains(port) && IsLocalAddress(address, localAddresses))
            {
                throw new ArgumentException("Physical remote points back into the local router topology");
            }
        }

        internal static bool IsLocalAddress(IPAddress address, IEnumerable<IPAddress> localAddresses)
        {
            if (address.IsIPv4MappedToIPv6)
            {
                address = address.MapToIPv4();
            }
            return IPAddress.IsLoopback(address) || localAddresses.Any(local =>
                local.AddressFamily == AddressFamily.InterNetwork && local.Equals(address));
        }

        private static IEnumerable<IPAddress> GetLocalIPv4Addresses()
        {
            // Query on each validation so reconnect/DNS checks include newly added VPN interfaces.
            return NetworkInterface.GetAllNetworkInterfaces()
                .SelectMany(adapter => adapter.GetIPProperties().UnicastAddresses)
                .Select(unicast => unicast.Address)
                .Where(address => address.AddressFamily == AddressFamily.InterNetwork);
        }
    }
}
