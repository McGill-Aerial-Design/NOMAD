// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Threading;
using System.Threading.Tasks;
using NOMAD.MissionPlanner.Connectivity;

namespace NOMAD.MissionPlanner
{
    internal static class OutputController
    {
        private static NomadCoreClient _coreClient;
        private static readonly SemaphoreSlim GimbalRequests = new SemaphoreSlim(1, 1);
        private static readonly SemaphoreSlim[] PayloadRequests = CreatePayloadGates();
        private static readonly SemaphoreSlim[] PayloadStops = CreatePayloadGates();
        private static readonly object GimbalFailureLock = new object();
        private static string _lastGimbalFailure = "";
        private static DateTime _lastGimbalFailureAt = DateTime.MinValue;

        private static SemaphoreSlim[] CreatePayloadGates()
        {
            // Runtime supports 16 servos and 16 relays; the last gate covers invalid inputs.
            var gates = new SemaphoreSlim[33];
            for (var index = 0; index < gates.Length; index++)
            {
                gates[index] = new SemaphoreSlim(1, 1);
            }
            return gates;
        }
        internal static void Initialize(NOMADConfig config)
        {
            _coreClient = config == null ? null :
                new NomadCoreClient(config.CoreClientCredential, config.CoreRuntimePort);
        }

        internal static NomadCoreClient CreateCoreClient() => _coreClient;

        // Nonwaiting gates reject overlapping input instead of collecting stale commands.
        private static NomadCoreRequestResult NotSent(string code, string message)
        {
            return new NomadCoreRequestResult(NomadCoreRequestOutcome.NotAttempted, code, message);
        }

        public static Task<NomadCoreRequestResult> SendServoPwmAsync(int channel, int pwmUs)
        {
            return SendPayloadAsync(CreateCoreClient(), GetServoGate(channel), "servo",
                $"channel={channel} pwm_us={pwmUs}",
                client => client.ServoAsync(channel, pwmUs));
        }

        private static int GetServoGate(int channel) => channel >= 1 && channel <= 16 ? channel - 1 : 32;
        private static int GetRelayGate(int relay) => relay >= 0 && relay <= 15 ? relay + 16 : 32;

        // Only one explicit stop may wait behind its channel's current command.
        public static async Task<NomadCoreRequestResult> SendServoStopAsync(int channel, int pwmUs)
        {
            var capturedClient = CreateCoreClient();
            var gateIndex = GetServoGate(channel);
            var stopGate = PayloadStops[gateIndex];
            if (!await stopGate.WaitAsync(0).ConfigureAwait(false))
            {
                return NotSent("request_in_progress", "A stop is already pending; no additional request was sent.");
            }
            try
            {
                return await SendPayloadAsync(capturedClient, gateIndex, "servo", $"channel={channel} pwm_us={pwmUs}",
                    client => client.ServoAsync(channel, pwmUs), explicitStop: true).ConfigureAwait(false);
            }
            finally
            {
                stopGate.Release();
            }
        }

        public static Task<NomadCoreRequestResult> SetRelayAsync(int relayNumber, bool on)
        {
            return SendPayloadAsync(CreateCoreClient(), GetRelayGate(relayNumber), "relay",
                $"relay={relayNumber} state={(on ? 1 : 0)}",
                client => client.SetRelayAsync(relayNumber, on));
        }

        private static async Task<NomadCoreRequestResult> SendPayloadAsync(NomadCoreClient client,
            int gateIndex, string command, string detail,
            Func<NomadCoreClient, Task<NomadCoreRequestResult>> send, bool explicitStop = false)
        {
            var gate = PayloadRequests[gateIndex];
            if (!explicitStop && PayloadStops[gateIndex].CurrentCount == 0)
            {
                return NotSent("request_in_progress", "An explicit stop is pending; no new command was sent.");
            }
            if (explicitStop)
            {
                await gate.WaitAsync().ConfigureAwait(false);
            }
            else if (!await gate.WaitAsync(0).ConfigureAwait(false))
            {
                return NotSent("request_in_progress", "Another payload command is in progress; no request was sent.");
            }
            try
            {
                var result = client == null ? NotSent("core_not_configured", "NOMAD core is not configured.")
                    : await send(client).ConfigureAwait(false);
                ReportPayloadResult(command, detail, result);
                return result;
            }
            finally
            {
                gate.Release();
            }
        }

        private static void ReportPayloadResult(string command, string detail, NomadCoreRequestResult result)
        {
            if (!result.Succeeded)
            {
                Log.Warn(DescribeFailure(command + " command", result));
            }
            Log.Info($"audit command={command} result={FormatOutcome(result.Outcome)} {detail}");
        }

        internal static async Task<NomadCoreRequestResult> SendGimbalTargetAsync(double pitchDeg, double rollDeg)
        {
            var result = await SendGimbalAsync(client => client.GimbalTargetAsync(pitchDeg, rollDeg))
                .ConfigureAwait(false);
            if (result.Succeeded)
            {
                ClearGimbalFailure();
            }
            else
            {
                ReportGimbalFailure(DescribeFailure("Gimbal target", result));
            }
            return result;
        }

        internal static async Task<NomadCoreRequestResult> ConfigureGimbalAsync(int mountMode)
        {
            var result = await SendGimbalAsync(client => client.GimbalConfigureAsync(mountMode)).ConfigureAwait(false);
            if (!result.Succeeded)
            {
                Log.Warn(DescribeFailure("Gimbal configure", result));
            }
            return result;
        }

        private static async Task<NomadCoreRequestResult> SendGimbalAsync(
            Func<NomadCoreClient, Task<NomadCoreRequestResult>> send)
        {
            if (!await GimbalRequests.WaitAsync(0).ConfigureAwait(false))
            {
                return NotSent("request_in_progress", "Another gimbal request is in progress; no request was sent.");
            }
            try
            {
                var client = CreateCoreClient();
                return client == null ? NotSent("core_not_configured", "NOMAD core is not configured.")
                    : await send(client).ConfigureAwait(false);
            }
            finally
            {
                GimbalRequests.Release();
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

        internal static string DescribeFailure(string action, NomadCoreRequestResult result)
        {
            var evidence = result.Outcome switch
            {
                NomadCoreRequestOutcome.Rejected => "Command was not sent to the vehicle; runtime rejected it.",
                NomadCoreRequestOutcome.Failed => "Operation was attempted and NOMAD obtained a definite failure.",
                NomadCoreRequestOutcome.Interrupted => "Authority or session changed during execution; "
                    + "final vehicle state is unknown. Do not retry blindly.",
                NomadCoreRequestOutcome.UnknownOutcome => "Request may have been transmitted; "
                    + "final vehicle state is unknown. Do not retry blindly.",
                _ => "No runtime mutation request was sent.",
            };
            return $"{action}: {evidence} {result.ErrorCode}: {result.Message}";
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

        // Hold the payload gate for the whole pulse; neither edge is automatically retried.
        public static Task<NomadCoreRequestResult> FireRelayAsync(int relayNumber, int durationMs)
        {
            durationMs = Math.Max(50, Math.Min(durationMs, 5000));
            return SendPayloadAsync(CreateCoreClient(), GetRelayGate(relayNumber), "relay",
                $"relay={relayNumber} pulse_ms={durationMs}", async client =>
            {
                var started = await client.SetRelayAsync(relayNumber, true).ConfigureAwait(false);
                if (!started.Succeeded)
                {
                    return started;
                }
                await Task.Delay(durationMs).ConfigureAwait(false);
                return await client.SetRelayAsync(relayNumber, false).ConfigureAwait(false);
            });
        }
    }
}
