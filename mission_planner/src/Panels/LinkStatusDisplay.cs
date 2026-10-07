// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System.Collections.Generic;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    internal static class LinkStatusDisplay
    {
        internal static string FormatRouterStatus(bool running, IReadOnlyList<LinkStatistics> links)
        {
            return running
                ? $"Router: running — {links.Count(link => link.IsConnected)}/{links.Count} links connected"
                : "Router: stopped";
        }

        internal static string FormatRouterStatus(
            bool managementConnected,
            bool routerAvailable,
            IReadOnlyList<LinkStatistics> links)
        {
            if (!managementConnected || !routerAvailable)
            {
                return "Router process: unavailable";
            }

            int connected = links.Count(link => link.IsConnected && !link.IsStale);
            return $"Router: connected — Physical links: {connected}/{links.Count} connected";
        }

        internal static bool HasMembershipChanged(IReadOnlyList<LinkStatistics> links, ICollection<string> cardIds)
        {
            return links.Count != cardIds.Count || links.Any(link => !cardIds.Contains(link.Type));
        }
    }
}
