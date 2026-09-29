// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Globalization;

namespace NOMAD.MissionPlanner.Connectivity
{
    public enum NomadCoreRequestOutcome
    {
        NotAttempted,
        Succeeded,
        Rejected,
        FailedBeforeSend,
        UnknownOutcome
    }

    /// <summary>
    /// Sends typed requests to the persistent NOMAD runtime over local IPC.
    /// </summary>
    public sealed class NomadCoreClient
    {
        private static readonly string ProcessSource = "mission-planner:" + Guid.NewGuid().ToString("N");

        public const int DefaultRuntimePort = 14611;

        public int RuntimePort { get; }
        public NomadCoreRequestOutcome LastOutcome { get; private set; }
        public string LastErrorCode { get; private set; } = "";
        public string LastMessage { get; private set; } = "";

        private readonly NomadRuntimeClient _runtimeClient;

        public NomadCoreClient(string apiKey, int runtimePort = DefaultRuntimePort)
        {
            RuntimePort = runtimePort >= 1 && runtimePort <= 65535 ? runtimePort : DefaultRuntimePort;
            _runtimeClient = new NomadRuntimeClient(RuntimePort, apiKey ?? "", ProcessSource);
        }

        public bool AdmitAuthority() => RequestAuthority("admit");
        public bool RevokeAuthority() => RequestAuthority("revoke");
        public bool HandbackAuthority() => RequestAuthority("handback");

        private bool RequestAuthority(string verb)
        {
            var result = _runtimeClient.Run(verb, Array.Empty<string>());
            CopyLastResult();
            return result == 0;
        }

        /// <summary>
        /// Drive an ArduPilot servo channel through the runtime.
        /// Fails closed on out-of-range input or unavailable runtime.
        /// </summary>
        public bool Servo(int channel, int pwmUs)
        {
            if (channel < 1 || pwmUs < 500 || pwmUs > 2500)
            {
                return false;
            }
            return RunCore("servo", channel.ToString(CultureInfo.InvariantCulture),
                           pwmUs.ToString(CultureInfo.InvariantCulture)) == 0;
        }

        /// <summary>
        /// Toggle an ArduPilot relay through the runtime.
        /// </summary>
        public bool SetRelay(int relayNumber, bool on)
        {
            if (relayNumber < 0 || relayNumber > 15)
            {
                return false;
            }
            return RunCore("relay", relayNumber.ToString(CultureInfo.InvariantCulture), on ? "1" : "0") == 0;
        }

        /// <summary>
        /// Run a motor test through the runtime. PWM is 500..2500 us or 0 to stop;
        /// the timeout is clamped to 0.05..3.0 seconds.
        /// </summary>
        public bool MotorTest(int motorInstance, int pwmUs, double timeoutSeconds)
        {
            if (motorInstance < 1 || (pwmUs != 0 && (pwmUs < 500 || pwmUs > 2500)) || !IsFinite(timeoutSeconds))
            {
                return false;
            }
            var clamped = Math.Max(0.05, Math.Min(timeoutSeconds, 3.0));
            return RunCore(
                "motor-test",
                motorInstance.ToString(CultureInfo.InvariantCulture),
                pwmUs.ToString(CultureInfo.InvariantCulture),
                clamped.ToString("F2", CultureInfo.InvariantCulture)) == 0;
        }

        /// <summary>
        /// Select the gimbal mount mode through the runtime.
        /// </summary>
        public bool GimbalConfigure(int mountMode)
        {
            if (mountMode < 0 || mountMode > 4)
            {
                return false;
            }
            return RunCore("gimbal-config", mountMode.ToString(CultureInfo.InvariantCulture)) == 0;
        }

        /// <summary>
        /// Set a finite absolute gimbal angle through the runtime.
        /// </summary>
        public bool GimbalTarget(double pitchDeg, double rollDeg)
        {
            if (!IsFinite(pitchDeg) || pitchDeg < -90.0 || pitchDeg > 90.0)
            {
                LastOutcome = NomadCoreRequestOutcome.Rejected;
                LastErrorCode = "invalid_argument";
                LastMessage = "Gimbal pitch must be finite and between -90 and 90 degrees.";
                return false;
            }
            if (!IsFinite(rollDeg) || rollDeg < -30.0 || rollDeg > 30.0)
            {
                LastOutcome = NomadCoreRequestOutcome.Rejected;
                LastErrorCode = "invalid_argument";
                LastMessage = "Gimbal roll must be finite and between -30 and 30 degrees.";
                return false;
            }
            return RunCore(
                "gimbal-target",
                pitchDeg.ToString("R", CultureInfo.InvariantCulture),
                rollDeg.ToString("R", CultureInfo.InvariantCulture)) == 0;
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }

        private int RunCore(string verb, params string[] values)
        {
            var result = _runtimeClient.Run(verb, values);
            CopyLastResult();
            return result;
        }

        private void CopyLastResult()
        {
            LastOutcome = _runtimeClient.LastOutcome;
            LastErrorCode = _runtimeClient.LastErrorCode;
            LastMessage = _runtimeClient.LastMessage;
        }
    }
}
