// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Gimbal Controller (shared MAVLink sender)
// ============================================================
// Extracted from GimbalJoystickWindow so any input source — the floating
// joystick window, the NomadJoystickService driven by a physical DirectInput
// stick, or a future scripted automation — can drive the same target-angle
// integrator and serialize MAVLink access against PayloadControlPanel via
// CubeOutputController.MavlinkLock.
//
// The pure command construction + kinematics live in GimbalCommand (Mission
// Planner-free, unit-tested by tests/gimbal); this class is the thin send
// adapter that maps a GimbalFrame onto MAVLink.MAV_CMD and ships it.
// ============================================================

using System;
using System.Threading.Tasks;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Process-wide gimbal target state and MAVLink send helpers.
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

        public static MountMode CurrentMode { get; private set; } = MountMode.MavlinkTargeting;

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

        /// <summary>Fires when the mount mode preset changes.</summary>
        public static event Action<MountMode> ModeChanged;

        private static int _inflight; // 0/1 — Interlocked.Exchange guards.

        public static void SetTargetAngles(float pitchDeg, float rollDeg)
        {
            TargetPitchDeg = GimbalCommand.ClampPitch(pitchDeg);
            TargetRollDeg = GimbalCommand.ClampRoll(rollDeg);
            TargetChanged?.Invoke(TargetPitchDeg, TargetRollDeg);
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
            if (send) SendPitchRollAngle(p, r);
        }

        public static void SetMode(MountMode mode)
        {
            CurrentMode = mode;
            ModeChanged?.Invoke(mode);
            SendMountConfigure(mode);
        }

        /// <summary>
        /// Send DO_MOUNT_CONTROL with absolute pitch/roll. Drops if a previous
        /// command is still in flight so fast stick motion never queues.
        /// </summary>
        public static void SendPitchRollAngle(float pitchDeg, float rollDeg)
        {
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return;
            if (System.Threading.Interlocked.Exchange(ref _inflight, 1) == 1) return;

            byte sysid = MainV2.comPort.MAV.sysid;
            byte compid = MainV2.comPort.MAV.compid;
            var frame = GimbalCommand.BuildMountControl(pitchDeg, rollDeg);

            Task.Run(async () =>
            {
                bool acquired = false;
                try
                {
                    acquired = await CubeOutputController.MavlinkLock
                        .WaitAsync(1000).ConfigureAwait(false);
                    if (!acquired) return;
                    await SendFrameAsync(sysid, compid, frame).ConfigureAwait(false);
                }
                catch { }
                finally
                {
                    if (acquired) CubeOutputController.MavlinkLock.Release();
                    System.Threading.Interlocked.Exchange(ref _inflight, 0);
                }
            });
        }

        public static void SendMountConfigure(MountMode mode)
        {
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return;

            byte sysid = MainV2.comPort.MAV.sysid;
            byte compid = MainV2.comPort.MAV.compid;
            var frame = GimbalCommand.BuildMountConfigure(mode);

            Task.Run(async () =>
            {
                bool acquired = false;
                try
                {
                    acquired = await CubeOutputController.MavlinkLock
                        .WaitAsync(2000).ConfigureAwait(false);
                    if (!acquired) return;
                    await SendFrameAsync(sysid, compid, frame).ConfigureAwait(false);
                }
                catch { }
                finally
                {
                    if (acquired) CubeOutputController.MavlinkLock.Release();
                }
            });
        }

        // Ship a GimbalFrame as a COMMAND_LONG. The frame's numeric command id is
        // the MAVLink MAV_CMD value (pinned by GimbalCommand + tests/gimbal), so
        // the cast is exact.
        private static Task SendFrameAsync(byte sysid, byte compid, GimbalFrame frame)
            => MainV2.comPort.doCommandAsync(
                sysid, compid,
                (MAVLink.MAV_CMD)frame.Command,
                frame.P1, frame.P2, frame.P3, frame.P4, frame.P5, frame.P6, frame.P7,
                requireack: false, uicallback: null);
    }
}
