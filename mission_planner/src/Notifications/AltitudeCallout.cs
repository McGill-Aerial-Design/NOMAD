// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Altitude Callouts
// ============================================================
// Airbus-style spoken altitude callouts: as the vehicle crosses
// a threshold altitude — climbing OR descending — it announces
// the altitude reached. Pure (no MissionPlanner / MAVLink deps)
// so the crossing logic unit-tests standalone via the csc harness.
// ============================================================

using System;

namespace NOMAD.MissionPlanner
{
    public static class AltitudeCallout
    {
        // Crossing any of these (up or down) triggers one spoken callout.
        // Tuned for the sub-122 m competition envelope — this array is the
        // calibration knob if the flight ceiling changes.
        public static readonly double[] Thresholds = { 3, 5, 10, 20, 30, 50, 75, 100, 120 };

        // Hysteresis (m): altitude must clear a threshold by this margin before a
        // crossing registers, so jitter sitting exactly on a threshold can't chatter.
        public const double Margin = 0.5;

        /// <summary>
        /// Decide the next callout from the current altitude (m, above home) and the
        /// caller's running band index. Returns the phrase to speak ("&lt;n&gt; meters"),
        /// or null when no threshold was crossed. Updates <paramref name="bandIndex"/>.
        ///
        /// <paramref name="bandIndex"/> starts at -1 (unprimed): the first call primes
        /// it to the current band and stays silent, so we never announce on startup or
        /// right after arming. Otherwise it is the count of thresholds the vehicle is
        /// currently above.
        /// </summary>
        public static string Next(double altM, ref int bandIndex)
        {
            if (bandIndex < 0) { bandIndex = BandFrom(altM); return null; }

            int band = bandIndex;
            // Climb a band only once altitude exceeds the next threshold by Margin;
            // drop a band only once it falls below the current one by Margin. The gap
            // between these is the deadband that absorbs jitter.
            while (band < Thresholds.Length && altM >= Thresholds[band] + Margin) band++;
            while (band > 0 && altM < Thresholds[band - 1] - Margin) band--;

            if (band == bandIndex) return null;

            // The threshold just crossed: going up it's the one we rose above
            // (band-1); going down it's the one we fell below (band, the old floor).
            // On a multi-band jump in one tick this picks the boundary nearest the
            // new altitude — the most relevant callout.
            int crossed = band > bandIndex ? band - 1 : band;
            bandIndex = band;
            return $"{(int)Thresholds[crossed]} meters";
        }

        // Band with no hysteresis — used only to prime bandIndex on the first read.
        private static int BandFrom(double altM)
        {
            int b = 0;
            while (b < Thresholds.Length && altM >= Thresholds[b]) b++;
            return b;
        }
    }
}
