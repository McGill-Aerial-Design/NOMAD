// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Gimbal Controller (shared runtime client)
// ============================================================
// The floating window and physical DirectInput stick share one target-angle
// integrator. Target requests go through the typed NOMAD runtime client.
// ============================================================

using System;
using System.Threading.Tasks;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Process-wide gimbal target state and runtime request helpers.
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

        // Integrated target angles in deg. Visible to UI components that want
        // to display the current target alongside their own controls.
        public static float TargetPitchDeg { get; private set; }
        public static float TargetRollDeg { get; private set; }

        // Desired input preset, not an observed or confirmed vehicle mount mode.
        public static MountMode SelectedMode { get; private set; } = MountMode.MavlinkTargeting;

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

        private static int _inflight; // 0/1 — Interlocked.Exchange guards.

        public static void SetTargetAngles(float pitchDeg, float rollDeg)
        {
            TargetPitchDeg = GimbalCommand.ClampPitch(pitchDeg);
            TargetRollDeg = GimbalCommand.ClampRoll(rollDeg);
            TargetChanged?.Invoke(TargetPitchDeg, TargetRollDeg);
        }

        /// <summary>
        /// Apply a discrete keyboard nudge and send the resulting target. Keyboard
        /// input always selects MAVLink targeting so the angle command is honored.
        /// </summary>
        public static void NudgeTarget(float pitchDeltaDeg, float rollDeltaDeg)
        {
            if (SelectedMode != MountMode.MavlinkTargeting)
                SetMode(MountMode.MavlinkTargeting);

            SetTargetAngles(TargetPitchDeg + pitchDeltaDeg, TargetRollDeg + rollDeltaDeg);
            RequestPitchRollTarget(TargetPitchDeg, TargetRollDeg);
        }

        /// <summary>
        /// Integrate a normalized stick reading over dt using the shared
        /// <see cref="MaxRateDegSec"/> and (optionally) push the new angle to the
        /// mount.
        /// </summary>
        public static void ApplyStick(float stickX, float stickY, float dt, bool send)
            => ApplyStick(stickX, stickY, _maxRateDegSec, dt, send);

        public static void ApplyStick(float stickX, float stickY, float maxRateDegSec, float dt, bool send)
        {
            GimbalCommand.IntegrateStick(
                TargetPitchDeg, TargetRollDeg, stickX, stickY, maxRateDegSec, dt,
                out float p, out float r);
            if (p == TargetPitchDeg && r == TargetRollDeg) return;
            TargetPitchDeg = p;
            TargetRollDeg = r;
            TargetChanged?.Invoke(p, r);
            if (send) RequestPitchRollTarget(p, r);
        }

        public static void SetMode(MountMode mode)
        {
            SelectedMode = mode;
            ModeChanged?.Invoke(mode);
            _ = ConfigureModeAsync(mode);
        }

        /// <summary>
        /// Request an absolute pitch/roll target through the runtime. Drops a
        /// new request while the previous one is in flight so stick motion never queues.
        /// </summary>
        public static void RequestPitchRollTarget(float pitchDeg, float rollDeg)
        {
            if (System.Threading.Interlocked.Exchange(ref _inflight, 1) == 1) return;

            _ = SendTargetAsync(pitchDeg, rollDeg);
        }

        private static async Task ConfigureModeAsync(MountMode mode)
        {
            try
            {
                await OutputController.ConfigureGimbalAsync((int)mode).ConfigureAwait(false);
            }
            catch (Exception error)
            {
                Log.Warn($"Gimbal configure failed: {error.Message}; mode was not confirmed.");
            }
        }

        private static async Task SendTargetAsync(float pitchDeg, float rollDeg)
        {
            try
            {
                await OutputController.SendGimbalTargetAsync(pitchDeg, rollDeg).ConfigureAwait(false);
            }
            catch (Exception error)
            {
                Log.Warn($"Gimbal target request failed: {error.Message}; target was not confirmed.");
            }
            finally
            {
                System.Threading.Interlocked.Exchange(ref _inflight, 0);
            }
        }
    }
}
