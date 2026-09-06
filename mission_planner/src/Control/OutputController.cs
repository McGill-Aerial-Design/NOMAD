// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Threading.Tasks;
using MissionPlanner;
using NOMAD.MissionPlanner.Connectivity;

namespace NOMAD.MissionPlanner
{
    // Sends standard ArduPilot output commands (DO_SET_SERVO / DO_SET_RELAY)
    // through the C++ core client boundary. These are generic ArduPilot
    // servo/relay channels that work on any ArduPilot flight controller, with
    // no board-specific assumptions. Payloads are config-declared client
    // profiles over these generic outputs (NOMADConfig.Payloads); the core
    // knows channels, never a specific payload.
    //
    // The direct-MAVLink fallback and the edge_core REST fallbacks were
    // removed in the C++ cutover (2026-09-05): commands that the core did not
    // acknowledge and verify must fail closed, and the core must not depend on
    // a GCS link being present.
    internal static class OutputController
    {
        private static NOMADConfig _config;

        /// <summary>
        /// Called at plugin load so output commands can build the core client
        /// (same wiring as FlightModeController).
        /// </summary>
        internal static void Initialize(NOMADConfig config)
        {
            _config = config;
        }

        internal static NomadCoreClient CreateCoreClient()
        {
            if (_config == null)
            {
                return null;
            }
            return new NomadCoreClient(_config.CoreExePath, _config.CoreMavlinkEndpoint, _config.CoreApiKey);
        }

        /// <summary>
        /// Drive an ArduPilot servo channel to a PWM value through the core
        /// (MAV_CMD_DO_SET_SERVO, acknowledged and verified by the core).
        /// Fails closed on invalid input or an unavailable/refusing core.
        /// </summary>
        public static Task<bool> SendServoPwmAsync(int channel, int pwmUs)
        {
            return Task.FromResult(SendServoPwm(channel, pwmUs));
        }

        public static bool SendServoPwm(int channel, int pwmUs)
        {
            if (channel <= 0 || pwmUs < 500 || pwmUs > 2500)
            {
                return false;
            }
            var client = CreateCoreClient();
            if (client == null)
            {
                Log.Warn("Servo command: NOMAD core not configured.");
                return false;
            }
            if (client.Servo(channel, pwmUs))
            {
                return true;
            }
            Log.Warn("Servo command: core refused or could not reach the vehicle.");
            return false;
        }

        /// <summary>
        /// Toggle an ArduPilot relay through the core (MAV_CMD_DO_SET_RELAY,
        /// acknowledged and verified by the core). Fails closed when the core
        /// is not configured, refuses, or cannot reach the vehicle.
        /// </summary>
        public static bool TrySetRelay(int relayNumber, bool on)
        {
            if (relayNumber < 0)
            {
                return false;
            }
            var client = CreateCoreClient();
            if (client == null)
            {
                Log.Warn("Relay command: NOMAD core not configured.");
                return false;
            }
            if (client.SetRelay(relayNumber, on))
            {
                return true;
            }
            Log.Warn("Relay command: core refused or could not reach the vehicle.");
            return false;
        }

        /// <summary>
        /// Fire a relay pulse through the core: on for the clamped duration,
        /// then off. SR-PAY-03: direct GCS-to-FC relay output bypasses the
        /// on-board interlock by design; the panel's armed click or the
        /// transmitter switch is the operator interlock documented in
        /// docs/safety.md.
        /// </summary>
        public static async Task<bool> FireRelayAsync(int relayNumber, int durationMs)
        {
            if (relayNumber < 0)
            {
                return false;
            }
            durationMs = Math.Max(50, Math.Min(durationMs, 5000));
            if (!TrySetRelay(relayNumber, true))
            {
                return false;
            }
            await Task.Delay(durationMs).ConfigureAwait(false);
            return TrySetRelay(relayNumber, false);
        }
    }
}
