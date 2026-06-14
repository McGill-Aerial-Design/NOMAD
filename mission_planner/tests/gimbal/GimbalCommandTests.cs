// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// GimbalCommand unit tests
// ============================================================
// Compiled together with src/Control/GimbalCommand.cs by
// scripts/build/test_plugin_gimbal.ps1 (plain csc, no test framework — exits
// non-zero on failure). Run via `pixi run test-plugin-gimbal`.
//
// Pins the exact MAVLink the gimbal control emits to ArduPilot: the
// DO_MOUNT_CONTROL / DO_MOUNT_CONFIGURE command ids + parameter layout, the
// MAV_MOUNT_MODE enum values, and the stick-integration / angle-clamping math.
// The companion tests/sitl/gimbal_mount_control.py proves these same commands
// actually point a real ArduPilot mount.
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
        CommandIds_MatchMavlinkSpec();

        MountControl_MapsPitchRollAndTargetingMode();
        MountControl_ClampsToLimits();

        MountConfigure_MapsModeToParam1();

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

    private static void CommandIds_MatchMavlinkSpec()
    {
        AssertEqual(205, GimbalCommand.DO_MOUNT_CONTROL, "DO_MOUNT_CONTROL == 205");
        AssertEqual(204, GimbalCommand.DO_MOUNT_CONFIGURE, "DO_MOUNT_CONFIGURE == 204");
    }

    // ============================================================
    // DO_MOUNT_CONTROL frame
    // ============================================================

    private static void MountControl_MapsPitchRollAndTargetingMode()
    {
        var f = GimbalCommand.BuildMountControl(-30f, 12f);
        AssertEqual(205, f.Command, "mount-control command id");
        AssertNear(-30f, f.P1, "mount-control P1 = pitch");
        AssertNear(12f, f.P2, "mount-control P2 = roll");
        AssertNear(0f, f.P3, "mount-control P3 = 0");
        AssertNear(0f, f.P4, "mount-control P4 = 0");
        AssertNear(0f, f.P5, "mount-control P5 = 0");
        AssertNear(0f, f.P6, "mount-control P6 = 0");
        // P7 selects MAVLINK_TARGETING so the mount honors the absolute angle.
        AssertNear(2f, f.P7, "mount-control P7 = MAVLINK_TARGETING (2)");
    }

    private static void MountControl_ClampsToLimits()
    {
        var hi = GimbalCommand.BuildMountControl(200f, 200f);
        AssertNear(GimbalCommand.PITCH_MAX_DEG, hi.P1, "mount-control clamps pitch to max");
        AssertNear(GimbalCommand.ROLL_MAX_DEG, hi.P2, "mount-control clamps roll to max");

        var lo = GimbalCommand.BuildMountControl(-200f, -200f);
        AssertNear(GimbalCommand.PITCH_MIN_DEG, lo.P1, "mount-control clamps pitch to min");
        AssertNear(GimbalCommand.ROLL_MIN_DEG, lo.P2, "mount-control clamps roll to min");
    }

    // ============================================================
    // DO_MOUNT_CONFIGURE frame
    // ============================================================

    private static void MountConfigure_MapsModeToParam1()
    {
        foreach (MountMode mode in new[] { MountMode.Retract, MountMode.Neutral, MountMode.MavlinkTargeting, MountMode.RcTargeting })
        {
            var f = GimbalCommand.BuildMountConfigure(mode);
            AssertEqual(204, f.Command, $"mount-configure command id ({mode})");
            AssertNear((float)mode, f.P1, $"mount-configure P1 = mode ({mode})");
        }
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
