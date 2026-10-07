// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// GimbalCommand unit tests
// ============================================================
// Compiled together with src/Control/GimbalCommand.cs by
// scripts/build/test_plugin_gimbal.ps1 (plain csc, no test framework — exits
// non-zero on failure). Run via `pixi run test-plugin-gimbal`.
//
// Covers the UI-side MAV_MOUNT_MODE values and stick-integration /
// angle-clamping math. The C++ runtime tests own the MAVLink contract.
// ============================================================

using System;
using NOMAD.MissionPlanner;

internal static class GimbalCommandTests
{
    private static int _failures;
    private const float Eps = 1e-4f;

    private static int Main()
    {
        MountMode_ValuesMatchMavMountMode();
        Clamp_PitchRollRate();
        Deadzone_ZerosInsideKeepsOutside();

        IntegrateStick_AppliesRateOverTime();
        IntegrateStick_SignConventions();
        IntegrateStick_ClampsAtLimits();

        Console.WriteLine(_failures == 0
            ? "All gimbal-command tests passed."
            : $"{_failures} gimbal-command test(s) FAILED.");
        return _failures == 0 ? 0 : 1;
    }

    // ============================================================
    // Wire-level constants (must match the MAVLink / ArduPilot spec)
    // ============================================================

    private static void MountMode_ValuesMatchMavMountMode()
    {
        // MAV_MOUNT_MODE: RETRACT=0, NEUTRAL=1, MAVLINK_TARGETING=2, RC_TARGETING=3.
        AssertEqual(0, (int)MountMode.Retract, "MountMode.Retract == 0");
        AssertEqual(1, (int)MountMode.Neutral, "MountMode.Neutral == 1");
        AssertEqual(2, (int)MountMode.MavlinkTargeting, "MountMode.MavlinkTargeting == 2");
        AssertEqual(3, (int)MountMode.RcTargeting, "MountMode.RcTargeting == 3");
    }

    // ============================================================
    // Clamps + deadzone
    // ============================================================

    private static void Clamp_PitchRollRate()
    {
        AssertNear(GimbalCommand.PITCH_MAX_DEG, GimbalCommand.ClampPitch(1000f), "ClampPitch high");
        AssertNear(GimbalCommand.PITCH_MIN_DEG, GimbalCommand.ClampPitch(-1000f), "ClampPitch low");
        AssertNear(15f, GimbalCommand.ClampPitch(15f), "ClampPitch in range");

        AssertNear(GimbalCommand.ROLL_MAX_DEG, GimbalCommand.ClampRoll(1000f), "ClampRoll high");
        AssertNear(GimbalCommand.ROLL_MIN_DEG, GimbalCommand.ClampRoll(-1000f), "ClampRoll low");

        AssertNear(GimbalCommand.MAX_MAX_RATE_DEG_SEC, GimbalCommand.ClampRate(9999f), "ClampRate high");
        AssertNear(GimbalCommand.MIN_MAX_RATE_DEG_SEC, GimbalCommand.ClampRate(0f), "ClampRate low");
    }

    private static void Deadzone_ZerosInsideKeepsOutside()
    {
        AssertNear(0f, GimbalCommand.ApplyDeadzone(0.05f, 0.06f), "deadzone zeros small +");
        AssertNear(0f, GimbalCommand.ApplyDeadzone(-0.05f, 0.06f), "deadzone zeros small -");
        AssertNear(0.5f, GimbalCommand.ApplyDeadzone(0.5f, 0.06f), "deadzone passes large +");
        AssertNear(-0.5f, GimbalCommand.ApplyDeadzone(-0.5f, 0.06f), "deadzone passes large -");
    }

    // ============================================================
    // Stick integration
    // ============================================================

    private static void IntegrateStick_AppliesRateOverTime()
    {
        // full +pitch stick, 60 deg/s, 0.5 s -> +30 deg from 0.
        GimbalCommand.IntegrateStick(0f, 0f, 0f, 1f, 60f, 0.5f, out float p, out float r);
        AssertNear(30f, p, "integrate: pitch = rate * dt");
        AssertNear(0f, r, "integrate: roll unchanged with no x");
    }

    private static void IntegrateStick_SignConventions()
    {
        // +stickY raises pitch.
        GimbalCommand.IntegrateStick(0f, 0f, 0f, 1f, 10f, 1f, out float pUp, out _);
        Assert(pUp > 0f, "integrate: +stickY -> +pitch");

        // +stickX rolls right == NEGATIVE roll.
        GimbalCommand.IntegrateStick(0f, 0f, 1f, 0f, 10f, 1f, out _, out float rRight);
        Assert(rRight < 0f, "integrate: +stickX -> -roll (roll right)");
    }

    private static void IntegrateStick_ClampsAtLimits()
    {
        // Push far past the limits in one big step; must stop at the limits.
        GimbalCommand.IntegrateStick(80f, 25f, -1f, -1f, 200f, 5f, out float p, out float r);
        AssertNear(GimbalCommand.PITCH_MIN_DEG, p, "integrate: pitch clamps to min");
        AssertNear(GimbalCommand.ROLL_MAX_DEG, r, "integrate: roll clamps to max");
    }

    // ============================================================
    // Assertion helpers
    // ============================================================

    private static void Assert(bool condition, string name)
    {
        if (condition)
        {
            Console.WriteLine($"  PASS  {name}");
        }
        else
        {
            Console.WriteLine($"  FAIL  {name}");
            _failures++;
        }
    }

    private static void AssertEqual(int expected, int actual, string name)
    {
        Assert(expected == actual, $"{name} (expected {expected}, got {actual})");
    }

    private static void AssertNear(float expected, float actual, string name)
    {
        Assert(Math.Abs(expected - actual) <= Eps, $"{name} (expected {expected}, got {actual})");
    }
}
