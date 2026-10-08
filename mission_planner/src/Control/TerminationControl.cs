// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// Explicitly reports vehicle actions that are not available through runtime IPC.

namespace NOMAD.MissionPlanner
{
    public static class TerminationControl
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

    }
}
