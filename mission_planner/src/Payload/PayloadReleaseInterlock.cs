// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// PayloadReleaseInterlock — pure SC confirm/arm state machine
// ============================================================
// Tier SC (safety-critical). The "is this actuation authorized yet?" decision
// for the multi-click-armed payload-release actions on the ground station: drop
// servos and momentary relay / water-pump fire. It owns ONLY the arm/confirm
// decision — no WinForms, no MAVLink, no timers — so it is unit-testable in
// isolation, mirroring the edge-side interlock in edge_core/safety/payload.py
// (requirement SR-PAY-03, hazard H-06). PayloadControlPanel is then pure chrome:
// it renders the returned outcome and drives the visual revert with its own
// WinForms timer, but makes no release decision itself.
//
// The operator must click an actuator button ClicksRequired times within a
// rolling WindowMs window; the final click authorizes exactly one actuation and
// disarms. A click after the window has lapsed — or a clock that moved backwards
// — restarts the count from one, so an actuation always requires a fresh,
// deliberate sequence. Time is injected (monotonic milliseconds) so the machine
// is deterministic under test.
// ============================================================

using System;

namespace NOMAD.MissionPlanner
{
    /// <summary>Outcome of registering a click on a multi-click-armed payload actuator.</summary>
    public enum ReleaseInterlockOutcome
    {
        /// <summary>More deliberate clicks are required before the actuation may fire.</summary>
        Arming,

        /// <summary>The confirm sequence is complete — actuate exactly once now.</summary>
        Fire,
    }

    /// <summary>
    /// Result of <see cref="PayloadReleaseInterlock.RegisterClick"/>: the outcome plus,
    /// while <see cref="ReleaseInterlockOutcome.Arming"/>, the click bookkeeping the
    /// panel needs to render the armed visual.
    /// </summary>
    public readonly struct ReleaseInterlockResult
    {
        public ReleaseInterlockResult(ReleaseInterlockOutcome outcome, int clickCount, int clicksRemaining)
        {
            Outcome = outcome;
            ClickCount = clickCount;
            ClicksRemaining = clicksRemaining;
        }

        /// <summary>Whether this click fired the actuation or only advanced the arm.</summary>
        public ReleaseInterlockOutcome Outcome { get; }

        /// <summary>Clicks registered so far in the current sequence (1-based; equals ClicksRequired on Fire).</summary>
        public int ClickCount { get; }

        /// <summary>Clicks still required before Fire (0 on Fire).</summary>
        public int ClicksRemaining { get; }
    }

    /// <summary>
    /// A deliberate-confirm interlock for ground-station payload release: require
    /// <see cref="ClicksRequired"/> clicks within a rolling <see cref="WindowMs"/>
    /// window to authorize exactly one actuation. Pure and time-injected; holds no
    /// UI or transport state.
    /// </summary>
    public sealed class PayloadReleaseInterlock
    {
        private int _clickCount;
        private long _lastClickAtMs;
        private bool _hasClick;

        /// <param name="clicksRequired">Clicks needed to fire (e.g. 3 for a drop, 2 for an arm→confirm relay fire).</param>
        /// <param name="windowMs">Maximum gap, in milliseconds, allowed between consecutive clicks.</param>
        public PayloadReleaseInterlock(int clicksRequired, int windowMs)
        {
            if (clicksRequired < 1)
                throw new ArgumentOutOfRangeException(nameof(clicksRequired), "at least one click is required");
            if (windowMs < 0)
                throw new ArgumentOutOfRangeException(nameof(windowMs), "window must be non-negative");

            ClicksRequired = clicksRequired;
            WindowMs = windowMs;
        }

        /// <summary>Clicks needed to fire.</summary>
        public int ClicksRequired { get; }

        /// <summary>Maximum allowed gap between consecutive clicks, in milliseconds.</summary>
        public int WindowMs { get; }

        /// <summary>Clicks registered so far in the current (not-yet-fired) sequence.</summary>
        public int ClickCount => _clickCount;

        /// <summary>True while a confirm sequence is part-way through (one or more clicks, not yet fired).</summary>
        public bool IsArming => _clickCount > 0;

        /// <summary>
        /// Register an actuator-button click at <paramref name="nowMs"/> (monotonic
        /// milliseconds). If the previous click is older than <see cref="WindowMs"/>,
        /// or the clock moved backwards, the sequence restarts from one. Returns
        /// <see cref="ReleaseInterlockOutcome.Fire"/> (and disarms) once
        /// <see cref="ClicksRequired"/> deliberate clicks land inside the window.
        /// </summary>
        public ReleaseInterlockResult RegisterClick(long nowMs)
        {
            if (!_hasClick || nowMs < _lastClickAtMs || nowMs - _lastClickAtMs > WindowMs)
                _clickCount = 0;

            _clickCount++;
            _lastClickAtMs = nowMs;
            _hasClick = true;

            if (_clickCount >= ClicksRequired)
            {
                int fired = _clickCount;
                Reset();
                return new ReleaseInterlockResult(ReleaseInterlockOutcome.Fire, fired, 0);
            }

            return new ReleaseInterlockResult(
                ReleaseInterlockOutcome.Arming, _clickCount, ClicksRequired - _clickCount);
        }

        /// <summary>
        /// True if an in-progress sequence has gone stale at <paramref name="nowMs"/>
        /// (the window lapsed with no further click, or the clock moved backwards).
        /// The panel can poll this from its revert timer to clear the armed visual.
        /// </summary>
        public bool IsExpired(long nowMs)
            => _clickCount > 0 && (nowMs < _lastClickAtMs || nowMs - _lastClickAtMs > WindowMs);

        /// <summary>Clear all arming state — on timeout, cancel, or after a fire.</summary>
        public void Reset()
        {
            _clickCount = 0;
            _lastClickAtMs = 0;
            _hasClick = false;
        }
    }
}
