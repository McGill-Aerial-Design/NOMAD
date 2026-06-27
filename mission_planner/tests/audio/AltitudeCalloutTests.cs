// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// AltitudeCallout unit tests
// ============================================================
// Compiled together with src/Notifications/AltitudeCallout.cs by
// scripts/build/test_plugin_audio.ps1 (plain csc, no test
// framework — exits non-zero on failure).
// Run via `pixi run test-plugin-audio`.
// ============================================================

using System;
using NOMAD.MissionPlanner;

internal static class AltitudeCalloutTests
{
    private static int _failures;

    private static int Main()
    {
        FirstReadPrimesSilently();
        ClimbAnnouncesEachThresholdCrossed();
        DescentAnnouncesThresholdCrossed();
        JitterOnThresholdStaysSilent();
        FastDescentAnnouncesNearestLowThreshold();
        DisarmResetReprimesSilently();

        Console.WriteLine(_failures == 0
            ? "All altitude-callout tests passed."
            : $"{_failures} altitude-callout test(s) FAILED.");
        return _failures == 0 ? 0 : 1;
    }

    private static void FirstReadPrimesSilently()
    {
        int band = -1;
        // Powering up already at 12 m must not blurt a callout.
        AssertNull(AltitudeCallout.Next(12.0, ref band), "first read primes silently");
        AssertEqual(3, band, "primed band above 3/5/10");
    }

    private static void ClimbAnnouncesEachThresholdCrossed()
    {
        int band = -1;
        AltitudeCallout.Next(0.0, ref band);                        // prime on the ground
        AssertNull(AltitudeCallout.Next(2.0, ref band), "below first threshold");
        AssertEqual("3 meters", AltitudeCallout.Next(4.0, ref band), "cross 3");
        AssertEqual("5 meters", AltitudeCallout.Next(6.0, ref band), "cross 5");
        AssertEqual("10 meters", AltitudeCallout.Next(11.0, ref band), "cross 10");
        AssertNull(AltitudeCallout.Next(11.2, ref band), "no new threshold");
    }

    private static void DescentAnnouncesThresholdCrossed()
    {
        int band = -1;
        AltitudeCallout.Next(25.0, ref band);                       // prime above 20
        AssertEqual("20 meters", AltitudeCallout.Next(19.0, ref band), "drop below 20");
        AssertEqual("10 meters", AltitudeCallout.Next(9.0, ref band), "drop below 10");
    }

    private static void JitterOnThresholdStaysSilent()
    {
        // Hover wobbling within the ±Margin deadband around 10 m never fires —
        // the Schmitt deadband swallows jitter that never truly clears a threshold.
        int b = -1;
        AltitudeCallout.Next(10.2, ref b);                          // prime above 10
        int silent = 0;
        foreach (var a in new[] { 9.9, 10.4, 9.6, 10.2 })           // all inside (9.5, 20.5)
            if (AltitudeCallout.Next(a, ref b) != null) silent++;
        AssertEqual(0, silent, "hover inside deadband never callouts");

        // One genuine crossing (clearing 10 + Margin) fires exactly once; later
        // wobble that stays above the lower edge does not re-announce.
        int band = -1;
        AltitudeCallout.Next(9.4, ref band);                        // prime just under 10
        int spoken = 0;
        foreach (var a in new[] { 10.6, 9.9, 10.3, 9.7, 10.1 })
            if (AltitudeCallout.Next(a, ref band) != null) spoken++;
        AssertEqual(1, spoken, "one real crossing then jitter deadbands");
    }

    private static void FastDescentAnnouncesNearestLowThreshold()
    {
        int band = -1;
        AltitudeCallout.Next(55.0, ref band);                       // prime above 50
        // One 1 Hz tick drops from 55 m to 2 m (crosses 50/30/20/10/5/3). Announce
        // the threshold nearest the ground — the most relevant — not a burst.
        AssertEqual("3 meters", AltitudeCallout.Next(2.0, ref band), "multi-band descent picks lowest crossed");
    }

    private static void DisarmResetReprimesSilently()
    {
        // The monitor sets band back to -1 on disarm; the next flight's first read
        // must prime in silence even though the vehicle is already off the ground.
        int band = -1;
        AssertNull(AltitudeCallout.Next(8.0, ref band), "re-primes silently after reset");
    }

    // ---- assertions ----
    private static void AssertEqual(object expected, object actual, string what)
    {
        if (!Equals(expected, actual))
        {
            _failures++;
            Console.WriteLine($"  FAIL: {what} — expected '{expected}', got '{actual}'");
        }
    }

    private static void AssertNull(object actual, string what)
    {
        if (actual != null)
        {
            _failures++;
            Console.WriteLine($"  FAIL: {what} — expected null, got '{actual}'");
        }
    }
}
