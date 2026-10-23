// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Gimbal Controller (Mission Planner host)
// ============================================================
// The floating window and physical DirectInput stick share one target-angle
// integrator. Target requests use the selected Mission Planner connection.
// ============================================================

using System;
using System.Threading;
using System.Threading.Tasks;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Shared gimbal input state and native Mission Planner command helpers.
    /// All state is static so multiple input sources see the same target.
    /// </summary>
    public static class GimbalController
    {
        // Mount limits — single source of truth in GimbalCommand; re-exposed here
        // so existing callers (joystick window, services) keep their references.
        public const float PITCH_MIN_DEG = GimbalCommand.PITCH_MIN_DEG;
        public const float PITCH_MAX_DEG = GimbalCommand.PITCH_MAX_DEG;
        public const float ROLL_MIN_DEG = GimbalCommand.ROLL_MIN_DEG;
        public const float ROLL_MAX_DEG = GimbalCommand.ROLL_MAX_DEG;
        public const float YAW_MIN_DEG = GimbalCommand.YAW_MIN_DEG;
        public const float YAW_MAX_DEG = GimbalCommand.YAW_MAX_DEG;

        // Integrated target angles in deg. Visible to UI components that want
        // to display the current target alongside their own controls.
        public static float TargetPitchDeg
        {
            get; private set;
        }
        public static float TargetRollDeg
        {
            get; private set;
        }
        public static float TargetYawDeg
        {
            get; private set;
        }

        // Desired input preset, not an observed or confirmed vehicle mount mode.
        public static MountMode SelectedMode
        {
            get; private set;
        } = MountMode.MavlinkTargeting;

        // Shared rate limit for ALL stick-driven inputs (floating window + physical
        // joystick service). Lives here so changing it in one UI is immediately
        // honored by the other — no code duplication and no config round-trip.
        public const float DEFAULT_MAX_RATE_DEG_SEC = GimbalCommand.DEFAULT_MAX_RATE_DEG_SEC;
        public const float MIN_MAX_RATE_DEG_SEC = GimbalCommand.MIN_MAX_RATE_DEG_SEC;
        public const float MAX_MAX_RATE_DEG_SEC = GimbalCommand.MAX_MAX_RATE_DEG_SEC;
        private static float _maxRateDegSec = DEFAULT_MAX_RATE_DEG_SEC;
        public static float MaxRateDegSec
        {
            get => _maxRateDegSec;
            set
            {
                float v = GimbalCommand.ClampRate(value);
                if (Math.Abs(_maxRateDegSec - v) < 0.001f) return;
                _maxRateDegSec = v;
                MaxRateChanged?.Invoke(v);
            }
        }
        public static event Action<float> MaxRateChanged;

        /// <summary>
        /// Fires whenever an input source moves the integrated target. Subscribers
        /// (e.g. GimbalJoystickWindow) update their on-screen readouts.
        /// </summary>
        public static event Action<float, float> TargetChanged;

        /// <summary>Fires when the desired mount mode preset changes; vehicle acceptance is separate.</summary>
        public static event Action<MountMode> ModeChanged;

        private static int _inflight;
        private static long _modeRevision;
        private static long _manualRevision;
        private static long _trackingClaim;
        internal static long CurrentTrackingClaim => Interlocked.Read(ref _trackingClaim);
        internal static long ClaimTracking() => Interlocked.Increment(ref _trackingClaim);
        internal static long ManualRevision => Interlocked.Read(ref _manualRevision);
        internal static event Action ManualInput;
        internal static string LastCommandStatus
        {
            get; private set;
        } = "Idle";
        internal static void ApplyProfile(NOMADConfig config)
        {
            InvalidateInput();
            SelectedMode = (MountMode)config.GimbalMountMode;
            MaxRateDegSec = config.JoystickGimbalMaxRateDegSec;
            SetTargetAnglesCore(0, 0, 0);
            ModeChanged?.Invoke(SelectedMode);
        }
        internal static void InvalidateInput()
        {
            Interlocked.Increment(ref _modeRevision);
            TargetPitchDeg = 0;
            TargetRollDeg = 0;
            TargetYawDeg = 0;
        }
        public static void SetTargetAngles(float pitchDeg, float rollDeg)
        {
            if (!Finite(pitchDeg) || !Finite(rollDeg))
            {
                return;
            }
            RecordManualInput();
            SetTargetAnglesCore(GimbalCommand.ClampPitch(pitchDeg), GimbalCommand.ClampRoll(rollDeg), 0);
        }
        private static void SetTargetAnglesCore(float pitch, float roll, float yaw)
        {
            TargetPitchDeg = pitch;
            TargetRollDeg = roll;
            TargetYawDeg = yaw;
            TargetChanged?.Invoke(TargetPitchDeg, TargetRollDeg);
        }
        public static void NudgeTarget(float pitchDeltaDeg, float rollDeltaDeg)
        {
            _ = SetModeAndTargetAsync(MountMode.MavlinkTargeting, TargetPitchDeg + pitchDeltaDeg, TargetRollDeg + rollDeltaDeg);
        }
        public static void ApplyStick(float stickX, float stickY, float dt, bool send) =>
            ApplyStick(stickX, stickY, _maxRateDegSec, dt, send);
        public static void ApplyStick(float stickX, float stickY, float rate, float dt, bool send)
        {
            if (!Finite(stickX) || !Finite(stickY) || !Finite(rate) || !Finite(dt) ||
                Math.Abs(stickX) > 1 || Math.Abs(stickY) > 1 || dt <= 0 || dt > 0.5f ||
                (stickX == 0 && stickY == 0))
                {
                    return;
                }
            long generation = OutputController.Generation;
            long revision = RecordManualInput();
            GimbalCommand.IntegrateStick(TargetPitchDeg, TargetRollDeg, stickX, stickY,
                GimbalCommand.ClampRate(rate), dt, out float pitch, out float roll);
            SetTargetAnglesCore(pitch, roll, 0);
            if (send)
            {
                _ = SendAsync(SelectedMode, pitch, roll, 0, generation, revision, null);
            }
        }
        public static void SetMode(MountMode mode)
        {
            _ = SetModeAsync(mode);
        }
        internal static Task<GimbalRequestResult> SetModeAsync(MountMode mode) => SendManualAsync(mode, null, null);
        public static void RequestPitchRollTarget(float pitchDeg, float rollDeg)
        {
            _ = RequestPitchRollTargetAsync(pitchDeg, rollDeg);
        }
        internal static Task<GimbalRequestResult> RequestPitchRollTargetAsync(float pitch, float roll) =>
            SendManualAsync(SelectedMode, pitch, roll);
        internal static Task<GimbalRequestResult> RequestManualPitchYawAsync(float pitch, float yaw, Func<bool> current)
        {
            long generation = OutputController.Generation;
            long revision = RecordManualInput();
            return SendAsync(MountMode.MavlinkTargeting, pitch, 0, yaw, generation, revision, current);
        }
        internal static Task<GimbalRequestResult> SetModeAndTargetAsync(MountMode mode, float pitch, float roll) =>
            SendManualAsync(mode, pitch, roll);
        internal static Task<GimbalRequestResult> RequestTrackingTargetAsync(float pitch, float yaw,
            long generation, long manualRevision, Func<bool> inputStillCurrent) =>
            SendAsync(MountMode.MavlinkTargeting, pitch, 0, yaw, generation, Interlocked.Read(ref _modeRevision),
                () => manualRevision == ManualRevision && inputStillCurrent?.Invoke() == true);
        private static long RecordManualInput()
        {
            Interlocked.Increment(ref _manualRevision);
            long revision = Interlocked.Increment(ref _modeRevision);
            ManualInput?.Invoke();
            return revision;
        }
        private static Task<GimbalRequestResult> SendManualAsync(MountMode mode, float? pitch, float? roll)
        {
            long generation = OutputController.Generation;
            long revision = RecordManualInput();
            return SendAsync(mode, pitch, roll, 0, generation, revision, null);
        }
        private static async Task<GimbalRequestResult> SendAsync(MountMode mode, float? pitch, float? roll,
            float yaw, long generation, long revision, Func<bool> inputStillCurrent)
        {
            bool Current() => revision == Interlocked.Read(ref _modeRevision) && inputStillCurrent?.Invoke() != false;
            if (!Current())
            {
                return new GimbalRequestResult(GimbalRequestOutcome.NotSent, "Gimbal input was retired; no command was sent.");
            }
            if (Interlocked.Exchange(ref _inflight, 1) != 0)
            {
                return new GimbalRequestResult(GimbalRequestOutcome.Busy, "A gimbal command is already in progress.");
            }
            try
            {
                if (pitch.HasValue && mode != MountMode.MavlinkTargeting)
                {
                    return new GimbalRequestResult(GimbalRequestOutcome.NotSent, "Angle targets require MAVLink targeting.");
                }
                var result = await OutputController.SendGimbalTargetAsync((int)mode, pitch, roll, yaw, generation,
                    Current).ConfigureAwait(false);
                LastCommandStatus = result.Message;
                if (!result.Submitted)
                {
                    if (result.Outcome != GimbalRequestOutcome.Busy) { Log.Warn(result.Message); }
                }
                else if (generation == OutputController.Generation && Current())
                {
                    SelectedMode = mode;
                    if (pitch.HasValue)
                    {
                        SetTargetAnglesCore(pitch.Value, roll.Value, yaw);
                    }
                    ModeChanged?.Invoke(mode);
                }
                return result;
            }
            finally
            {
                Interlocked.Exchange(ref _inflight, 0);
            }
        }
        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
