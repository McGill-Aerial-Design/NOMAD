// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// Core client requests and explicit reporting of unavailable termination.

using NOMAD.MissionPlanner.Connectivity;

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
        /// Switch to GUIDED and fly to the given position at the given relative
        /// altitude (meters AGL). Used by the soft-boundary "return to boundary"
        /// action. Routes through the C++ core client boundary: the core sends
        /// MAV_CMD_DO_REPOSITION with the change-mode flag and verifies the
        /// arrival position, so a true return means the vehicle is at the
        /// target. Returns false (fail closed) when the core is not configured,
        /// refuses, or cannot reach the vehicle.
        /// </summary>
        public static bool GuidedGoto(double lat, double lng, double altRelM)
        {
            var client = OutputController.CreateCoreClient();
            if (client == null)
            {
                Log.Warn("GuidedGoto: NOMAD core not configured.");
                return false;
            }
            var ok = client.Goto(lat, lng, altRelM);
            if (!ok)
            {
                var outcome = client.LastOutcome == NomadCoreRequestOutcome.UnknownOutcome
                    ? "vehicle outcome is unknown after the runtime connection ended"
                    : "core rejected or could not reach the vehicle";
                Log.Warn($"GuidedGoto: {outcome}.");
            }
            return ok;
        }
    }
}
