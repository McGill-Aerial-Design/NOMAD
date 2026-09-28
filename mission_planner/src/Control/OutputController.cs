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
    // The direct-MAVLink and retired REST fallbacks were
    // removed in the C++ cutover (2026-09-05): commands that the core did not
    // acknowledge and verify must fail closed, and the core must not depend on
    // a GCS link being present.
    internal static class OutputController
    {
        private static NomadCoreClient _coreClient;
        internal static bool IntegratedFlightMode { get; private set; }

        /// <summary>
        /// GCS-side audit record for a core-routed actuation command. The core
        /// CLI already emits the authoritative machine-readable line
        /// (audit command=... result=... auth=...) on its own stderr; this
        /// companion Log line records the outcome where the operator and the
        /// plugin's log adapters can see it.
        /// </summary>
        private static void Audit(string command, bool accepted, string detail)
        {
            // The command names here are the core CLI verbs, matching the
            // core's own audit lines so a log can be correlated end to end.
            var outcome = accepted ? "accepted" : "failed";
            Log.Info($"audit command={command} result={outcome} auth=api-key {detail}");
        }

        /// <summary>
        /// Keep one client for output, gimbal and guided callers during the
        /// current plugin configuration session.
        /// </summary>
        internal static void Initialize(NOMADConfig config)
        {
            IntegratedFlightMode = config?.IntegratedFlightMode ?? false;
            _coreClient = config == null ? null :
                new NomadCoreClient(config.CoreExePath, config.CoreMavlinkEndpoint, config.CoreApiKey,
                                    config.CoreClientMode, config.CoreRuntimePort, config.IntegratedFlightMode);
        }

        internal static NomadCoreClient CreateCoreClient()
        {
            return _coreClient;
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
                Audit("servo", false, "reason=core_not_configured");
                return false;
            }
            if (client.Servo(channel, pwmUs))
            {
                Audit("servo", true, $"channel={channel} pwm_us={pwmUs}");
                return true;
            }
            Log.Warn(DescribeFailure("Servo command", client));
            Audit("servo", false, $"channel={channel} pwm_us={pwmUs} reason={FailureReason(client)}");
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
                Audit("relay", false, "reason=core_not_configured");
                return false;
            }
            if (client.SetRelay(relayNumber, on))
            {
                Audit("relay", true, $"relay={relayNumber} state={(on ? 1 : 0)}");
                return true;
            }
            Log.Warn(DescribeFailure("Relay command", client));
            Audit("relay", false, $"relay={relayNumber} state={(on ? 1 : 0)} reason={FailureReason(client)}");
            return false;
        }

        private static string DescribeFailure(string action, NomadCoreClient client)
        {
            if (client.LastOutcome == NomadCoreRequestOutcome.UnknownOutcome)
            {
                return $"{action}: runtime connection ended after send; vehicle outcome unknown. Do not retry blindly.";
            }
            return $"{action}: core refused or could not reach the vehicle.";
        }

        private static string FailureReason(NomadCoreClient client)
        {
            return client.LastOutcome == NomadCoreRequestOutcome.UnknownOutcome ? "unknown_outcome" : "core_refused";
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
