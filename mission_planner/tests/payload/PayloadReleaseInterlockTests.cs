// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// PayloadReleaseInterlock unit tests (tier SC, SR-PAY-03)
// ============================================================
// Compiled together with src/Payload/PayloadReleaseInterlock.cs by
// scripts/build/test_plugin_interlock.ps1 (plain csc, no test framework —
// exits non-zero on failure). Run via `pixi run test-plugin-interlock`.
//
// The ground-station counterpart of tests/test_safety_payload.py: the pure
// arm/confirm decision behind every multi-click-armed payload release (drop
// servos, momentary relay / pump fire). Time is injected, so every window and
// clock-anomaly path is exercised deterministically.
// ============================================================

using System;
using NOMAD.MissionPlanner;

internal static class PayloadReleaseInterlockTests
{
    private static int _failures;
    private const int Window = 3000;

    private static int Main()
    {
        Ctor_RejectsClicksRequiredBelowOne();
        Ctor_RejectsNegativeWindow();
        NewInterlock_IsNotArming();

        ThreeClick_ArmsTwiceThenFires();
        Fire_DisarmsSoNextSequenceStartsFresh();
        TwoClick_ArmsThenFires();
        SingleClickRequired_FiresImmediately();

        ClickAfterWindowLapses_RestartsTheCount();
        ClickExactlyAtWindowEdge_StillCounts();
        BackwardsClock_RestartsTheCount();

        IsExpired_FalseWhenIdle();
        IsExpired_FalseWithinWindow_TrueAfter();
        IsExpired_TrueWhenClockMovedBackwards();

        Reset_ClearsArmingState();

        Console.WriteLine(_failures == 0
            ? "All payload-interlock tests passed."
            : $"{_failures} payload-interlock test(s) FAILED.");
        return _failures == 0 ? 0 : 1;
    }

    // ============================================================
    // Construction
    // ============================================================

    private static void Ctor_RejectsClicksRequiredBelowOne()
    {
        AssertThrows<ArgumentOutOfRangeException>(
            () => new PayloadReleaseInterlock(0, Window), "ctor rejects clicksRequired = 0");
        AssertThrows<ArgumentOutOfRangeException>(
            () => new PayloadReleaseInterlock(-1, Window), "ctor rejects clicksRequired < 0");
    }

    private static void Ctor_RejectsNegativeWindow()
    {
        AssertThrows<ArgumentOutOfRangeException>(
            () => new PayloadReleaseInterlock(3, -1), "ctor rejects negative window");
    }

    private static void NewInterlock_IsNotArming()
    {
        var il = new PayloadReleaseInterlock(3, Window);
        Assert(!il.IsArming, "fresh interlock is not arming");
        AssertEqual(0, il.ClickCount, "fresh interlock click count is 0");
    }

    // ============================================================
    // Drop: three-click confirm
    // ============================================================

    private static void ThreeClick_ArmsTwiceThenFires()
    {
        var il = new PayloadReleaseInterlock(3, Window);

        var first = il.RegisterClick(0);
        Assert(first.Outcome == ReleaseInterlockOutcome.Arming, "3-click: 1st click arms");
        AssertEqual(1, first.ClickCount, "3-click: 1st click count");
        AssertEqual(2, first.ClicksRemaining, "3-click: 1st click remaining");
        Assert(il.IsArming, "3-click: arming after 1st click");

        var second = il.RegisterClick(500);
        Assert(second.Outcome == ReleaseInterlockOutcome.Arming, "3-click: 2nd click arms");
        AssertEqual(2, second.ClickCount, "3-click: 2nd click count");
        AssertEqual(1, second.ClicksRemaining, "3-click: 2nd click remaining");

        var third = il.RegisterClick(1000);
        Assert(third.Outcome == ReleaseInterlockOutcome.Fire, "3-click: 3rd click fires");
        AssertEqual(3, third.ClickCount, "3-click: fire click count");
        AssertEqual(0, third.ClicksRemaining, "3-click: fire remaining is 0");
    }

    private static void Fire_DisarmsSoNextSequenceStartsFresh()
    {
        var il = new PayloadReleaseInterlock(3, Window);
        il.RegisterClick(0);
        il.RegisterClick(100);
        var fire = il.RegisterClick(200);
        Assert(fire.Outcome == ReleaseInterlockOutcome.Fire, "fire on 3rd click");

        Assert(!il.IsArming, "interlock is safe after firing");
        AssertEqual(0, il.ClickCount, "click count cleared after firing");

        var afterFire = il.RegisterClick(300);
        Assert(afterFire.Outcome == ReleaseInterlockOutcome.Arming, "post-fire click arms a fresh sequence");
        AssertEqual(1, afterFire.ClickCount, "post-fire click count restarts at 1");
    }

    // ============================================================
    // Relay: two-click arm -> confirm
    // ============================================================

    private static void TwoClick_ArmsThenFires()
    {
        var il = new PayloadReleaseInterlock(2, Window);

        var arm = il.RegisterClick(0);
        Assert(arm.Outcome == ReleaseInterlockOutcome.Arming, "2-click: 1st click arms");
        AssertEqual(1, arm.ClicksRemaining, "2-click: 1 remaining after arm");

        var fire = il.RegisterClick(1000);
        Assert(fire.Outcome == ReleaseInterlockOutcome.Fire, "2-click: 2nd click fires");
    }

    private static void SingleClickRequired_FiresImmediately()
    {
        var il = new PayloadReleaseInterlock(1, Window);
        var r = il.RegisterClick(0);
        Assert(r.Outcome == ReleaseInterlockOutcome.Fire, "1-click interlock fires immediately");
        AssertEqual(0, r.ClicksRemaining, "1-click: 0 remaining");
    }

    // ============================================================
    // Rolling window expiry
    // ============================================================

    private static void ClickAfterWindowLapses_RestartsTheCount()
    {
        var il = new PayloadReleaseInterlock(3, Window);
        il.RegisterClick(0);
        il.RegisterClick(1000);

        var stale = il.RegisterClick(1000 + Window + 1);
        Assert(stale.Outcome == ReleaseInterlockOutcome.Arming, "stale click does not fire");
        AssertEqual(1, stale.ClickCount, "stale click restarts count at 1");
        AssertEqual(2, stale.ClicksRemaining, "stale click remaining back to 2");
    }

    private static void ClickExactlyAtWindowEdge_StillCounts()
    {
        var il = new PayloadReleaseInterlock(3, Window);
        il.RegisterClick(0);
        var second = il.RegisterClick(Window); // boundary is inclusive
        AssertEqual(2, second.ClickCount, "click at the window edge still counts");
    }

    private static void BackwardsClock_RestartsTheCount()
    {
        var il = new PayloadReleaseInterlock(3, Window);
        il.RegisterClick(5000);
        var back = il.RegisterClick(4000);
        Assert(back.Outcome == ReleaseInterlockOutcome.Arming, "backwards clock does not fire");
        AssertEqual(1, back.ClickCount, "backwards clock restarts count at 1");
    }

    // ============================================================
    // IsExpired (drives the panel's visual revert)
    // ============================================================

    private static void IsExpired_FalseWhenIdle()
    {
        var il = new PayloadReleaseInterlock(3, Window);
        Assert(!il.IsExpired(0), "idle interlock is never expired (t=0)");
        Assert(!il.IsExpired(1_000_000), "idle interlock is never expired (t large)");
    }

    private static void IsExpired_FalseWithinWindow_TrueAfter()
    {
        var il = new PayloadReleaseInterlock(3, Window);
        il.RegisterClick(0);
        Assert(!il.IsExpired(Window), "not expired at the window edge");
        Assert(il.IsExpired(Window + 1), "expired just past the window");
    }

    private static void IsExpired_TrueWhenClockMovedBackwards()
    {
        var il = new PayloadReleaseInterlock(3, Window);
        il.RegisterClick(5000);
        Assert(il.IsExpired(4000), "expired when the clock moved backwards");
    }

    // ============================================================
    // Reset
    // ============================================================

    private static void Reset_ClearsArmingState()
    {
        var il = new PayloadReleaseInterlock(3, Window);
        il.RegisterClick(0);
        il.RegisterClick(100);
        Assert(il.IsArming, "arming before reset");

        il.Reset();

        Assert(!il.IsArming, "not arming after reset");
        AssertEqual(0, il.ClickCount, "click count cleared by reset");

        var r = il.RegisterClick(200);
        AssertEqual(1, r.ClickCount, "click after reset starts a new sequence");
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

    private static void AssertThrows<TException>(Action action, string name) where TException : Exception
    {
        try
        {
            action();
            Assert(false, $"{name} (expected {typeof(TException).Name}, none thrown)");
        }
        catch (TException)
        {
            Assert(true, name);
        }
        catch (Exception ex)
        {
            Assert(false, $"{name} (expected {typeof(TException).Name}, got {ex.GetType().Name})");
        }
    }
}
