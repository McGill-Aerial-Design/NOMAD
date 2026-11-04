// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Gimbal input math (Mission Planner-free)
// ============================================================
// The pure, dependency-free core of the gimbal control: stick integration,
// deadzone and angle/rate clamping. The Native OutputController owns all MAVLink sends.
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
    /// Pure gimbal kinematics — no Mission Planner / MAVLink dependencies, so it
    /// unit-tests in isolation.
    /// </summary>
    public static class GimbalCommand
    {
        // Configured target-angle limits, independent of the MAVLink transport.
        public const float PITCH_MIN_DEG = -90f;
        public const float PITCH_MAX_DEG = 90f;
        public const float ROLL_MIN_DEG = -30f;
        public const float ROLL_MAX_DEG = 30f;
        public const float YAW_MIN_DEG = -180f;
        public const float YAW_MAX_DEG = 180f;

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

        internal static readonly string[] ScreenAxes = { "None", "Pitch", "Roll", "Yaw" };

        internal static bool ValidScreenAxes(string horizontal, string vertical) =>
            Array.IndexOf(ScreenAxes, horizontal) >= 0 && Array.IndexOf(ScreenAxes, vertical) >= 0 &&
            (horizontal == "None" || horizontal != vertical);

        internal static bool TryMapScreenAxes(string horizontal, string vertical, bool invertHorizontal,
            bool invertVertical, float x, float y, out float pitch, out float roll, out float yaw)
        {
            pitch = roll = yaw = 0;
            if (!ValidScreenAxes(horizontal, vertical) || float.IsNaN(x) || float.IsInfinity(x) ||
                float.IsNaN(y) || float.IsInfinity(y) || Math.Abs(x) > 1 || Math.Abs(y) > 1) { return false; }
            if (invertHorizontal) { x = -x; }
            if (invertVertical) { y = -y; }
            pitch = (horizontal == "Pitch" ? x : 0) + (vertical == "Pitch" ? y : 0);
            roll = (horizontal == "Roll" ? x : 0) + (vertical == "Roll" ? y : 0);
            yaw = (horizontal == "Yaw" ? x : 0) + (vertical == "Yaw" ? y : 0);
            return true;
        }

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
    }
}
