// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Diagnostics;

namespace NOMAD.MissionPlanner
{
    internal sealed class RouterClock
    {
        // Negative infinity means no observation; zero is a valid monotonic timestamp.
        internal const double Unset = double.NegativeInfinity;
        internal readonly Func<double> Seconds;
        internal readonly Func<DateTime> UtcNow;

        internal RouterClock(Func<double> seconds = null, Func<DateTime> utcNow = null)
        {
            Seconds = seconds ?? (() => Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency);
            UtcNow = utcNow ?? (() => DateTime.UtcNow);
        }
    }
}
