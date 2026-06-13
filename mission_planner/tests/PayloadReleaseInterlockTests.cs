// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Unit tests for PayloadReleaseInterlock (tier SC, SR-PAY-03)
// ============================================================
// The ground-station counterpart of tests/test_safety_payload.py: the pure
// arm/confirm decision behind every multi-click-armed payload release (drop
// servos, momentary relay / pump fire). Time is injected, so every window and
// clock-anomaly path is exercised deterministically.
// ============================================================

using System;
using NOMAD.MissionPlanner;
using Xunit;

namespace NOMAD.MissionPlanner.Tests
{
    public class PayloadReleaseInterlockTests
    {
        private const int Window = 3000;

        // ---- Construction ----

        [Theory]
        [InlineData(0)]
        [InlineData(-1)]
        public void Ctor_RejectsClicksRequiredBelowOne(int clicks)
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => new PayloadReleaseInterlock(clicks, Window));
        }

        [Fact]
        public void Ctor_RejectsNegativeWindow()
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => new PayloadReleaseInterlock(3, -1));
        }

        [Fact]
        public void NewInterlock_IsNotArming()
        {
            var il = new PayloadReleaseInterlock(3, Window);
            Assert.False(il.IsArming);
            Assert.Equal(0, il.ClickCount);
        }

        // ---- Drop: three-click confirm ----

        [Fact]
        public void ThreeClick_ArmsTwiceThenFires()
        {
            var il = new PayloadReleaseInterlock(3, Window);

            var first = il.RegisterClick(0);
            Assert.Equal(ReleaseInterlockOutcome.Arming, first.Outcome);
            Assert.Equal(1, first.ClickCount);
            Assert.Equal(2, first.ClicksRemaining);
            Assert.True(il.IsArming);

            var second = il.RegisterClick(500);
            Assert.Equal(ReleaseInterlockOutcome.Arming, second.Outcome);
            Assert.Equal(2, second.ClickCount);
            Assert.Equal(1, second.ClicksRemaining);

            var third = il.RegisterClick(1000);
            Assert.Equal(ReleaseInterlockOutcome.Fire, third.Outcome);
            Assert.Equal(3, third.ClickCount);
            Assert.Equal(0, third.ClicksRemaining);
        }

        [Fact]
        public void Fire_DisarmsSoNextSequenceStartsFresh()
        {
            var il = new PayloadReleaseInterlock(3, Window);
            il.RegisterClick(0);
            il.RegisterClick(100);
            var fire = il.RegisterClick(200);
            Assert.Equal(ReleaseInterlockOutcome.Fire, fire.Outcome);

            // After firing the interlock is safe again.
            Assert.False(il.IsArming);
            Assert.Equal(0, il.ClickCount);

            var afterFire = il.RegisterClick(300);
            Assert.Equal(ReleaseInterlockOutcome.Arming, afterFire.Outcome);
            Assert.Equal(1, afterFire.ClickCount);
        }

        // ---- Relay: two-click arm -> confirm ----

        [Fact]
        public void TwoClick_ArmsThenFires()
        {
            var il = new PayloadReleaseInterlock(2, Window);

            var arm = il.RegisterClick(0);
            Assert.Equal(ReleaseInterlockOutcome.Arming, arm.Outcome);
            Assert.Equal(1, arm.ClicksRemaining);

            var fire = il.RegisterClick(1000);
            Assert.Equal(ReleaseInterlockOutcome.Fire, fire.Outcome);
        }

        // ---- Single-click degenerate case ----

        [Fact]
        public void SingleClickRequired_FiresImmediately()
        {
            var il = new PayloadReleaseInterlock(1, Window);
            var r = il.RegisterClick(0);
            Assert.Equal(ReleaseInterlockOutcome.Fire, r.Outcome);
            Assert.Equal(0, r.ClicksRemaining);
        }

        // ---- Rolling window expiry ----

        [Fact]
        public void ClickAfterWindowLapses_RestartsTheCount()
        {
            var il = new PayloadReleaseInterlock(3, Window);
            il.RegisterClick(0);                 // count 1
            il.RegisterClick(1000);              // count 2

            // Gap exceeds the window -> this click is treated as a fresh start.
            var stale = il.RegisterClick(1000 + Window + 1);
            Assert.Equal(ReleaseInterlockOutcome.Arming, stale.Outcome);
            Assert.Equal(1, stale.ClickCount);
            Assert.Equal(2, stale.ClicksRemaining);
        }

        [Fact]
        public void ClickExactlyAtWindowEdge_StillCounts()
        {
            var il = new PayloadReleaseInterlock(3, Window);
            il.RegisterClick(0);
            // Exactly Window ms later is within the window (boundary is inclusive).
            var second = il.RegisterClick(Window);
            Assert.Equal(2, second.ClickCount);
        }

        [Fact]
        public void BackwardsClock_RestartsTheCount()
        {
            var il = new PayloadReleaseInterlock(3, Window);
            il.RegisterClick(5000);              // count 1
            var back = il.RegisterClick(4000);   // clock moved backwards -> restart
            Assert.Equal(ReleaseInterlockOutcome.Arming, back.Outcome);
            Assert.Equal(1, back.ClickCount);
        }

        // ---- IsExpired (drives the panel's visual revert) ----

        [Fact]
        public void IsExpired_FalseWhenIdle()
        {
            var il = new PayloadReleaseInterlock(3, Window);
            Assert.False(il.IsExpired(0));
            Assert.False(il.IsExpired(1_000_000));
        }

        [Fact]
        public void IsExpired_FalseWithinWindow_TrueAfter()
        {
            var il = new PayloadReleaseInterlock(3, Window);
            il.RegisterClick(0);
            Assert.False(il.IsExpired(Window));          // edge: still valid
            Assert.True(il.IsExpired(Window + 1));        // lapsed
        }

        [Fact]
        public void IsExpired_TrueWhenClockMovedBackwards()
        {
            var il = new PayloadReleaseInterlock(3, Window);
            il.RegisterClick(5000);
            Assert.True(il.IsExpired(4000));
        }

        // ---- Reset ----

        [Fact]
        public void Reset_ClearsArmingState()
        {
            var il = new PayloadReleaseInterlock(3, Window);
            il.RegisterClick(0);
            il.RegisterClick(100);
            Assert.True(il.IsArming);

            il.Reset();

            Assert.False(il.IsArming);
            Assert.Equal(0, il.ClickCount);

            // A click after Reset starts a brand new sequence.
            var r = il.RegisterClick(200);
            Assert.Equal(1, r.ClickCount);
        }
    }
}
