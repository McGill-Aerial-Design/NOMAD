// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Gimbal command construction + kinematics (Mission Planner-free)
// ============================================================
// The pure, dependency-free core of the gimbal control: the MAVLink command
// frames sent to ArduPilot (DO_MOUNT_CONTROL / DO_MOUNT_CONFIGURE) plus the
// stick-integration and angle-clamping math. Extracted from GimbalController so
// it can be unit-tested offline with the csc harness (no Mission Planner /
// MAVLink assemblies), mirroring PayloadReleaseInterlock and GeoMath.
// GimbalController maps GimbalFrame.Command onto MAVLink.MAV_CMD and performs
// the actual send; the autopilot side is exercised by tests/sitl.
// ============================================================

using System;

namespace NOMAD.MissionPlanner
{
    /// <summary>Mount targeting mode. Values are the MAVLink <c>MAV_MOUNT_MODE</c> enum.</summary>
    public enum MountMode
    {
        Retract = 0,
        Neutral = 1,
        MavlinkTargeting = 2,
        RcTargeting = 3,
    }

    /// <summary>
    /// An ArduPilot <c>COMMAND_LONG</c> payload: a MAV_CMD id plus its seven float
    /// params, exactly as handed to MAVLink. Lets tests assert the wire-level
    /// command without a live link.
    /// </summary>
    public readonly struct GimbalFrame
    {
        public readonly int Command;
        public readonly float P1, P2, P3, P4, P5, P6, P7;

        public GimbalFrame(int command, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
        {
            Command = command;
            P1 = p1;
            P2 = p2;
            P3 = p3;
            P4 = p4;
            P5 = p5;
            P6 = p6;
            P7 = p7;
        }
    }

    /// <summary>
    /// Pure gimbal command + kinematics — no Mission Planner / MAVLink deps, so it
    /// unit-tests in isolation. <see cref="GimbalController"/> consumes it.
    /// </summary>
    public static class GimbalCommand
    {
        // MAVLink MAV_CMD ids (ArduPilot mount control).
        public const int DO_MOUNT_CONFIGURE = 204;
        public const int DO_MOUNT_CONTROL = 205;

        // Default mount travel limits (deg). Typical brushless-gimbal range, kept
        // generic so this drives any DO_MOUNT_CONTROL mount — not a specific brand.
        public const float PITCH_MIN_DEG = -90f;
        public const float PITCH_MAX_DEG = 90f;
        public const float ROLL_MIN_DEG = -30f;
        public const float ROLL_MAX_DEG = 30f;

        // Shared stick rate limits (deg/s) for all stick-driven inputs.
        public const float DEFAULT_MAX_RATE_DEG_SEC = 60f;
        public const float MIN_MAX_RATE_DEG_SEC = 5f;
        public const float MAX_MAX_RATE_DEG_SEC = 200f;

        public static float Clamp(float v, float min, float max)
            => v < min ? min : v > max ? max : v;

        public static float ClampPitch(float deg) => Clamp(deg, PITCH_MIN_DEG, PITCH_MAX_DEG);

        public static float ClampRoll(float deg) => Clamp(deg, ROLL_MIN_DEG, ROLL_MAX_DEG);

        public static float ClampRate(float degPerSec) => Clamp(degPerSec, MIN_MAX_RATE_DEG_SEC, MAX_MAX_RATE_DEG_SEC);

        /// <summary>Zero a normalized stick axis when it is inside the deadzone.</summary>
        public static float ApplyDeadzone(float value, float deadzone)
            => Math.Abs(value) < deadzone ? 0f : value;

        /// <summary>
        /// Integrate a normalized stick reading over <paramref name="dt"/> at
        /// <paramref name="maxRateDegSec"/> and return the new clamped target.
        /// Convention: +stickY raises pitch; +stickX rolls right (negative roll),
        /// matching the on-screen joystick pad.
        /// </summary>
        public static void IntegrateStick(
            float curPitch, float curRoll, float stickX, float stickY,
            float maxRateDegSec, float dt, out float newPitch, out float newRoll)
        {
            float dPitch = stickY * maxRateDegSec * dt;
            float dRoll = -stickX * maxRateDegSec * dt;
            newPitch = ClampPitch(curPitch + dPitch);
            newRoll = ClampRoll(curRoll + dRoll);
        }

        /// <summary>
        /// Build the <c>DO_MOUNT_CONTROL</c> frame for an absolute pitch/roll target.
        /// Angles are clamped to the mount limits; P7 = MAVLINK_TARGETING (2) so the
        /// mount honors the absolute angle.
        /// </summary>
        public static GimbalFrame BuildMountControl(float pitchDeg, float rollDeg)
            => new GimbalFrame(
                DO_MOUNT_CONTROL,
                ClampPitch(pitchDeg), ClampRoll(rollDeg),
                0f, 0f, 0f, 0f, (float)MountMode.MavlinkTargeting);

        /// <summary>Build the <c>DO_MOUNT_CONFIGURE</c> frame selecting a mount mode.</summary>
        public static GimbalFrame BuildMountConfigure(MountMode mode)
            => new GimbalFrame(
                DO_MOUNT_CONFIGURE,
                (float)mode, 1f, 1f, 1f, 2f, 2f, 2f);
    }
}
