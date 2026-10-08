// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Gimbal Controller (shared runtime client)
// ============================================================
// The floating window and physical DirectInput stick share one target-angle
// integrator. Target requests go through the typed NOMAD runtime client.
// ============================================================

using System;
using System.Threading;
using System.Threading.Tasks;
using MissionPlanner;
using NOMAD.MissionPlanner.Connectivity;

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

        private static readonly object ModeLock = new object();
        private static int _inflight;
        private static MountMode? _configuredMode;
        private static bool _targetModeBlocked;
        private static long _modeRevision;
        private static Task<NomadCoreRequestResult> _pendingModeRequest;

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
            {
                SetModeAndTargetAsync(MountMode.MavlinkTargeting,
                    TargetPitchDeg + pitchDeltaDeg, TargetRollDeg + rollDeltaDeg);
                return;
            }

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
            _ = SetModeAsync(mode);
        }

        internal static Task<NomadCoreRequestResult> SetModeAsync(MountMode mode)
        {
            long revision;
            Task<NomadCoreRequestResult> request;
            lock (ModeLock)
            {
                SelectedMode = mode;
                _modeRevision++;
                revision = _modeRevision;
                _configuredMode = null;
                _targetModeBlocked = false;
                request = ConfigureModeAsync(mode, revision);
                _pendingModeRequest = request;
            }
            ModeChanged?.Invoke(mode);
            return request;
        }

        /// <summary>
        /// Request an absolute target. If a mode selection is still in flight,
        /// wait for its result before sending the target; only one target can wait.
        /// </summary>
        public static void RequestPitchRollTarget(float pitchDeg, float rollDeg)
        {
            _ = RequestPitchRollTargetAsync(pitchDeg, rollDeg);
        }

        internal static Task<NomadCoreRequestResult> RequestPitchRollTargetAsync(float pitchDeg, float rollDeg)
        {
            MountMode mode;
            long revision;
            bool blocked;
            Task<NomadCoreRequestResult> pendingMode;
            lock (ModeLock)
            {
                mode = SelectedMode;
                revision = _modeRevision;
                blocked = _targetModeBlocked;
                pendingMode = _pendingModeRequest;
            }
            if (mode != MountMode.MavlinkTargeting)
            {
                return Task.FromResult(NotAttempted("selected_mode", "Angle targets require MAVLink targeting mode."));
            }
            if (blocked)
            {
                return Task.FromResult(NotAttempted("gimbal_mode_not_confirmed",
                    "The selected mode was not confirmed. Select the mode again before sending a target."));
            }
            if (Interlocked.Exchange(ref _inflight, 1) == 1)
            {
                return Task.FromResult(NotAttempted("request_in_progress", "A gimbal target is already in progress."));
            }
            return SendTargetAsync(mode, pitchDeg, rollDeg, revision, pendingMode);
        }

        /// <summary>
        /// Send one explicit mode-and-target action. The runtime configures the
        /// mount before applying the new angle within this single mutation.
        /// </summary>
        internal static Task<NomadCoreRequestResult> SetModeAndTargetAsync(
            MountMode mode, float pitchDeg, float rollDeg)
        {
            float targetPitch = GimbalCommand.ClampPitch(pitchDeg);
            float targetRoll = GimbalCommand.ClampRoll(rollDeg);
            Task<NomadCoreRequestResult> pendingMode;
            long revision;
            lock (ModeLock)
            {
                pendingMode = _pendingModeRequest;
                SelectedMode = mode;
                _modeRevision++;
                revision = _modeRevision;
                _configuredMode = null;
                _targetModeBlocked = false;
                _pendingModeRequest = null;
            }
            ModeChanged?.Invoke(mode);
            SetTargetAngles(targetPitch, targetRoll);
            if (Interlocked.Exchange(ref _inflight, 1) == 1)
            {
                return Task.FromResult(NotAttempted("request_in_progress", "A gimbal target is already in progress."));
            }
            return SendModeAndTargetAsync(mode, targetPitch, targetRoll, revision, pendingMode);
        }

        private static async Task<NomadCoreRequestResult> ConfigureModeAsync(MountMode mode, long revision)
        {
            try
            {
                var result = await OutputController.ConfigureGimbalAsync((int)mode,
                    () => IsCurrentMode(mode, revision)).ConfigureAwait(false);
                lock (ModeLock)
                {
                    if (revision == _modeRevision)
                    {
                        _configuredMode = result.Succeeded ? mode : null;
                        _targetModeBlocked = !result.Succeeded;
                    }
                }
                return result;
            }
            catch (Exception error)
            {
                Log.Warn($"Gimbal configure failed: {error.Message}; mode was not confirmed.");
                BlockMode(revision);
                return UnknownOutcome("Gimbal mode outcome is unknown. Do not retry blindly.");
            }
        }

        private static async Task<NomadCoreRequestResult> SendTargetAsync(MountMode mode, float pitchDeg, float rollDeg,
            long revision, Task<NomadCoreRequestResult> pendingMode)
        {
            try
            {
                if (pendingMode != null)
                {
                    var modeResult = await pendingMode.ConfigureAwait(false);
                    if (!modeResult.Succeeded)
                    {
                        BlockMode(revision);
                        return modeResult;
                    }
                }
                if (!IsCurrentMode(mode, revision))
                {
                    return NotAttempted("stale_gimbal_mode", "Gimbal mode changed before target transmission.");
                }
                var configuredMode = GetConfiguredMode();
                var result = configuredMode == mode
                    ? await OutputController.SendGimbalTargetAsync(pitchDeg, rollDeg,
                        () => IsCurrentMode(mode, revision)).ConfigureAwait(false)
                    : await OutputController.ConfigureGimbalAndSendTargetAsync((int)mode, pitchDeg, rollDeg,
                        () => IsCurrentMode(mode, revision))
                        .ConfigureAwait(false);
                RecordTargetResult(mode, result, revision);
                return result;
            }
            catch (Exception error)
            {
                Log.Warn($"Gimbal target request failed: {error.Message}; target was not confirmed.");
                BlockMode(revision);
                return UnknownOutcome("Gimbal target outcome is unknown. Do not retry blindly.");
            }
            finally
            {
                Interlocked.Exchange(ref _inflight, 0);
            }
        }

        private static async Task<NomadCoreRequestResult> SendModeAndTargetAsync(
            MountMode mode, float pitchDeg, float rollDeg, long revision,
            Task<NomadCoreRequestResult> pendingMode)
        {
            try
            {
                if (pendingMode != null)
                {
                    await pendingMode.ConfigureAwait(false);
                }
                if (!IsCurrentMode(mode, revision))
                {
                    return NotAttempted("stale_gimbal_mode", "Gimbal mode changed before command transmission.");
                }
                var result = await OutputController.ConfigureGimbalAndSendTargetAsync((int)mode, pitchDeg, rollDeg,
                    () => IsCurrentMode(mode, revision)).ConfigureAwait(false);
                RecordTargetResult(mode, result, revision);
                return result;
            }
            catch (Exception error)
            {
                Log.Warn($"Gimbal mode and target failed: {error.Message}; outcome was not confirmed.");
                BlockMode(revision);
                return UnknownOutcome("Gimbal mode and target outcome is unknown. Do not retry blindly.");
            }
            finally
            {
                Interlocked.Exchange(ref _inflight, 0);
            }
        }

        private static bool IsCurrentMode(MountMode mode, long revision)
        {
            lock (ModeLock) { return SelectedMode == mode && _modeRevision == revision; }
        }

        private static MountMode? GetConfiguredMode()
        {
            lock (ModeLock) { return _configuredMode; }
        }

        private static void RecordTargetResult(MountMode mode, NomadCoreRequestResult result, long revision)
        {
            lock (ModeLock)
            {
                if (revision != _modeRevision) { return; }
                _configuredMode = result.Succeeded ? mode : null;
                _targetModeBlocked = !result.Succeeded && result.Outcome != NomadCoreRequestOutcome.NotAttempted &&
                    result.Outcome != NomadCoreRequestOutcome.FailedBeforeSend;
            }
        }

        private static void BlockMode(long revision)
        {
            lock (ModeLock)
            {
                if (revision == _modeRevision) { _targetModeBlocked = true; }
            }
        }

        private static NomadCoreRequestResult NotAttempted(string code, string message)
        {
            return new NomadCoreRequestResult(NomadCoreRequestOutcome.NotAttempted, code, message);
        }

        private static NomadCoreRequestResult UnknownOutcome(string message)
        {
            return new NomadCoreRequestResult(NomadCoreRequestOutcome.UnknownOutcome, "unknown_outcome", message);
        }
    }
}
