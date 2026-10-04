// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

namespace NOMAD.MissionPlanner
{
 public partial class GroundLinkRouter
 {
        private static void ClassifyHealth(LinkSourceStats s, double packetAge, double hbAge)
        {
            if (!s.IsOpen || !s.IsConnected) { s.Health = LinkHealth.Disconnected; return; }

            double loss = s.PacketLossPercent;
            double jitter = s.LatencyMs;
            double rate = s.DataRateBps;

            // Grade the MAVLink telemetry stream, not physical RF distance.
            // Heartbeat freshness and received byte rate are the failure signals.
            // Sequence-gap loss is useful for quality scoring, but routers and
            // redundant paths can make it misleading, so it should not alone
            // declare a live link critical.
            if (packetAge > 5.0 || hbAge > 8.0 ||
                (s.FramesReceived > 50 && rate < 5.0))
            {
                s.Health = LinkHealth.Critical;
            }
            else if (hbAge > 4.0 || loss >= 60.0 ||
                     (s.FramesReceived > 100 && rate < 20.0))
            {
                s.Health = LinkHealth.Poor;
            }
            else if (hbAge > 2.5 || loss >= 25.0 || jitter >= 1500.0 ||
                     (s.FramesReceived > 100 && rate < 80.0))
            {
                s.Health = LinkHealth.Fair;
            }
            else if (hbAge > 1.5 || loss >= 5.0 || jitter >= 500.0 ||
                     (s.FramesReceived > 100 && rate < 300.0))
            {
                s.Health = LinkHealth.Good;
            }
            else
            {
                s.Health = LinkHealth.Excellent;
            }
        }

 }
}
