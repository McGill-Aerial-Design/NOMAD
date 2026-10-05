// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Globalization;
using System.Threading;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner.Connectivity
{
    /// <summary>
    /// Sends typed requests to the persistent NOMAD runtime over local IPC.
    /// </summary>
    public sealed partial class NomadCoreClient
    {
        private const string ProcessSource = "mission-planner";

        public const int DefaultRuntimePort = 14611;

        public int RuntimePort { get; }
        // Legacy synchronous API snapshot; async callers must use their returned result.
        public NomadCoreRequestOutcome LastOutcome { get; private set; }
        public string LastErrorCode { get; private set; } = "";
        public string LastMessage { get; private set; } = "";
        public bool? LastAcknowledged { get; private set; }

        private readonly NomadRuntimeClient _runtimeClient;

        // apiKey retains named-call compatibility; it is the runtime client credential, not NOMAD_API_KEY.
        public NomadCoreClient(string apiKey, int runtimePort = DefaultRuntimePort)
        {
            RuntimePort = runtimePort >= 1 && runtimePort <= 65535 ? runtimePort : DefaultRuntimePort;
            _runtimeClient = new NomadRuntimeClient(RuntimePort, apiKey ?? "", ProcessSource);
        }

        public Task<NomadCoreRequestResult> AdmitAuthorityAsync(CancellationToken cancellationToken = default) =>
            RunCoreAsync("admit", cancellationToken);
        public Task<NomadCoreRequestResult> RevokeAuthorityAsync(CancellationToken cancellationToken = default) =>
            RunCoreAsync("revoke", cancellationToken);
        public Task<NomadCoreRequestResult> HandbackAuthorityAsync(CancellationToken cancellationToken = default) =>
            RunCoreAsync("handback", cancellationToken);

        /// <summary>
        /// Drive an ArduPilot servo channel through the runtime.
        /// Fails closed on out-of-range input or unavailable runtime.
        /// </summary>
        public async Task<NomadCoreRequestResult> ServoAsync(int channel, int pwmUs,
            CancellationToken cancellationToken = default)
        {
            if (channel < 1 || pwmUs < 500 || pwmUs > 2500)
            {
                return RejectLocal("Servo channel and PWM are invalid.");
            }
            return await RunCoreAsync("servo", cancellationToken, channel.ToString(CultureInfo.InvariantCulture),
                           pwmUs.ToString(CultureInfo.InvariantCulture)).ConfigureAwait(false);
        }

        /// <summary>
        /// Toggle an ArduPilot relay through the runtime.
        /// </summary>
        public async Task<NomadCoreRequestResult> SetRelayAsync(int relayNumber, bool on,
            CancellationToken cancellationToken = default)
        {
            if (relayNumber < 0 || relayNumber > 15)
            {
                return RejectLocal("Relay must be between 0 and 15.");
            }
            return await RunCoreAsync("relay", cancellationToken,
                relayNumber.ToString(CultureInfo.InvariantCulture), on ? "1" : "0").ConfigureAwait(false);
        }

        /// <summary>
        /// Run a motor test through the runtime. PWM is 500..2500 us or 0 to stop;
        /// the timeout is clamped to 0.05..3.0 seconds.
        /// </summary>
        public async Task<NomadCoreRequestResult> MotorTestAsync(int motorInstance, int pwmUs, double timeoutSeconds,
            CancellationToken cancellationToken = default)
        {
            if (motorInstance < 1 || (pwmUs != 0 && (pwmUs < 500 || pwmUs > 2500)) || !IsFinite(timeoutSeconds))
            {
                return RejectLocal("Motor instance, PWM or timeout is invalid.");
            }
            var clamped = Math.Max(0.05, Math.Min(timeoutSeconds, 3.0));
            return await RunCoreAsync(
                "motor-test", cancellationToken,
                motorInstance.ToString(CultureInfo.InvariantCulture),
                pwmUs.ToString(CultureInfo.InvariantCulture),
                clamped.ToString("F2", CultureInfo.InvariantCulture)).ConfigureAwait(false);
        }

        /// <summary>
        /// Select the gimbal mount mode through the runtime.
        /// </summary>
        public async Task<NomadCoreRequestResult> GimbalConfigureAsync(int mountMode,
            CancellationToken cancellationToken = default)
        {
            if (mountMode < 0 || mountMode > 4)
            {
                return RejectLocal("Gimbal mount mode must be between 0 and 4.");
            }
            return await RunCoreAsync("gimbal-config", cancellationToken,
                mountMode.ToString(CultureInfo.InvariantCulture)).ConfigureAwait(false);
        }

        /// <summary>
        /// Set a finite absolute gimbal angle through the runtime.
        /// </summary>
        public async Task<NomadCoreRequestResult> GimbalTargetAsync(double pitchDeg, double rollDeg,
            CancellationToken cancellationToken = default)
        {
            if (!IsFinite(pitchDeg) || pitchDeg < -90.0 || pitchDeg > 90.0)
            {
                return RejectLocal("Gimbal pitch must be finite and between -90 and 90 degrees.");
            }
            if (!IsFinite(rollDeg) || rollDeg < -30.0 || rollDeg > 30.0)
            {
                return RejectLocal("Gimbal roll must be finite and between -30 and 30 degrees.");
            }
            return await RunCoreAsync(
                "gimbal-target", cancellationToken,
                pitchDeg.ToString("R", CultureInfo.InvariantCulture),
                rollDeg.ToString("R", CultureInfo.InvariantCulture)).ConfigureAwait(false);
        }

        private static NomadCoreRequestResult RejectLocal(string message)
        {
            return new NomadCoreRequestResult(NomadCoreRequestOutcome.NotAttempted, "invalid_argument", message);
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }

        private Task<NomadCoreRequestResult> RunCoreAsync(string verb, CancellationToken cancellationToken,
                                                         params string[] values)
        {
            return _runtimeClient.RunAsync(verb, values, cancellationToken);
        }

        // Compatibility only. Production callers await the immutable request result.
        private bool CompleteLegacy(Task<NomadCoreRequestResult> request)
        {
            var result = request.GetAwaiter().GetResult();
            LastOutcome = result.Outcome;
            LastErrorCode = result.ErrorCode;
            LastMessage = result.Message;
            LastAcknowledged = result.Acknowledged;
            return result.Succeeded;
        }

        public bool AdmitAuthority() => CompleteLegacy(AdmitAuthorityAsync());
        public bool RevokeAuthority() => CompleteLegacy(RevokeAuthorityAsync());
        public bool HandbackAuthority() => CompleteLegacy(HandbackAuthorityAsync());
        public bool Servo(int channel, int pwmUs) => CompleteLegacy(ServoAsync(channel, pwmUs));
        public bool SetRelay(int relayNumber, bool on) => CompleteLegacy(SetRelayAsync(relayNumber, on));
        public bool MotorTest(int motorInstance, int pwmUs, double timeoutSeconds) =>
            CompleteLegacy(MotorTestAsync(motorInstance, pwmUs, timeoutSeconds));
        public bool GimbalConfigure(int mountMode) => CompleteLegacy(GimbalConfigureAsync(mountMode));
        public bool GimbalTarget(double pitchDeg, double rollDeg) =>
            CompleteLegacy(GimbalTargetAsync(pitchDeg, rollDeg));
    }
}
