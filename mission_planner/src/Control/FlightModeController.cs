// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// Explicitly reports vehicle actions that are not available through runtime IPC.

namespace NOMAD.MissionPlanner
{
    public static class FlightModeController
    {
        /// <summary>
        /// Report the blocked aircraft-side termination integration.
        /// No LAND, disarm or parameter recipe substitutes for termination.
        /// </summary>
        public static bool RequestTermination()
        {
            Log.Error("Termination unavailable: approved aircraft mechanism and authority integration are missing.");
            return false;
        }

        /// <summary>
        /// Report that navigation commands are unavailable in runtime protocol v1.
        /// No vehicle command is sent until the runtime exposes a typed goto request.
        /// </summary>
        public static bool GuidedGoto(double lat, double lng, double altRelM)
        {
            Log.Warn("GuidedGoto unavailable: runtime protocol v1 does not support navigation requests. " +
                     "No vehicle command was sent; take manual control.");
            return false;
        }
    }
}
