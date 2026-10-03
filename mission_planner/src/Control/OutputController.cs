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
        private static readonly object GimbalRequestLock = new object();
        private static readonly object GimbalFailureLock = new object();
        private static string _lastGimbalFailure = "";
        private static DateTime _lastGimbalFailureAt = DateTime.MinValue;

        /// <summary>
        /// GCS-side audit record for a runtime-routed actuation command. The
        /// runtime emits the authoritative machine-readable line on its own stderr; this
        /// companion Log line records the outcome where the operator and the
        /// plugin's log adapters can see it.
        /// </summary>
        private static void Audit(string command, bool accepted, string detail)
        {
            // Keep the command name aligned with the runtime audit line for correlation.
            var client = CreateCoreClient();
            var outcome = accepted ? "success" : FormatOutcome(client?.LastOutcome ?? NomadCoreRequestOutcome.NotAttempted);
            Log.Info($"audit command={command} result={outcome} {detail}");
        }

        /// <summary>
        /// Keep one client for output and gimbal callers during the
        /// current plugin configuration session.
        /// </summary>
        internal static void Initialize(NOMADConfig config)
        {
            _coreClient = config == null ? null :
                new NomadCoreClient(config.CoreClientCredential, config.CoreRuntimePort);
        }

        internal static NomadCoreClient CreateCoreClient()
        {
            return _coreClient;
        }

        /// <summary>
        /// Drive an ArduPilot servo channel to a PWM value through the core
        /// (MAV_CMD_DO_SET_SERVO, acknowledged by the core; physical effect is not verified).
        /// Fails closed on invalid input or an unavailable/refusing core.
        /// </summary>
        public static Task<bool> SendServoPwmAsync(int channel, int pwmUs)
        {
            return Task.FromResult(SendServoPwm(channel, pwmUs));
        }

        public static bool SendServoPwm(int channel, int pwmUs)
        {
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
        /// acknowledged by the core; physical effect is not verified). Fails closed when the core
        /// is not configured, refuses, or cannot reach the vehicle.
        /// </summary>
        public static bool TrySetRelay(int relayNumber, bool on)
        {
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

        internal static bool SendGimbalTarget(double pitchDeg, double rollDeg)
        {
            lock (GimbalRequestLock)
            {
                var client = CreateCoreClient();
                if (client == null)
                {
                    ReportGimbalFailure("NOMAD core client is not configured; no target was sent.");
                    return false;
                }
                if (client.GimbalTarget(pitchDeg, rollDeg))
                {
                    ClearGimbalFailure();
                    return true;
                }

                ReportGimbalFailure(DescribeFailure("Gimbal target", client));
                return false;
            }
        }

        internal static bool ConfigureGimbal(int mountMode)
        {
            lock (GimbalRequestLock)
            {
                var client = CreateCoreClient();
                if (client == null)
                {
                    Log.Warn("Gimbal configure failed: NOMAD core client is not configured.");
                    return false;
                }
                if (client.GimbalConfigure(mountMode))
                {
                    return true;
                }
                Log.Warn(DescribeFailure("Gimbal configure", client));
                return false;
            }
        }

        private static void ReportGimbalFailure(string detail)
        {
            lock (GimbalFailureLock)
            {
                var now = DateTime.UtcNow;
                if (detail == _lastGimbalFailure && now - _lastGimbalFailureAt < TimeSpan.FromSeconds(5))
                {
                    return;
                }
                _lastGimbalFailure = detail;
                _lastGimbalFailureAt = now;
            }
            Log.Warn($"Gimbal target failed: {detail}");
        }

        private static void ClearGimbalFailure()
        {
            lock (GimbalFailureLock)
            {
                _lastGimbalFailure = "";
                _lastGimbalFailureAt = DateTime.MinValue;
            }
        }

        internal static string DescribeFailure(string action, NomadCoreClient client)
        {
            var evidence = client.LastOutcome switch
            {
                NomadCoreRequestOutcome.Rejected => "Command was not sent to the vehicle; runtime rejected it.",
                NomadCoreRequestOutcome.Failed => "Operation was attempted and NOMAD obtained a definite failure.",
                NomadCoreRequestOutcome.Interrupted => "Authority or session changed during execution; "
                    + "final vehicle state is unknown. Do not retry blindly.",
                NomadCoreRequestOutcome.UnknownOutcome => "Request may have been transmitted; "
                    + "final vehicle state is unknown. Do not retry blindly.",
                _ => "No runtime mutation request was sent.",
            };
            return $"{action}: {evidence} {client.LastErrorCode}: {client.LastMessage}";
        }

        internal static string DescribeLastFailure(string action)
        {
            var client = CreateCoreClient();
            return client == null ? $"{action}: NOMAD core is not configured; no request was sent."
                : DescribeFailure(action, client);
        }

        private static string FormatOutcome(NomadCoreRequestOutcome outcome)
        {
            return outcome switch
            {
                NomadCoreRequestOutcome.Succeeded => "success",
                NomadCoreRequestOutcome.Rejected => "rejected",
                NomadCoreRequestOutcome.Failed => "failed",
                NomadCoreRequestOutcome.Interrupted => "interrupted",
                NomadCoreRequestOutcome.UnknownOutcome => "unknown",
                NomadCoreRequestOutcome.FailedBeforeSend => "failed-before-send",
                _ => "not-attempted",
            };
        }

        private static string FailureReason(NomadCoreClient client)
        {
            return client.LastOutcome.ToString();
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
