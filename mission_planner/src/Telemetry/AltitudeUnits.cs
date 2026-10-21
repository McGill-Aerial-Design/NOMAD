// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;

namespace NOMAD.MissionPlanner
{
    internal static class AltitudeUnits
    {
        // MP alt/sonarrange use its display multiplier; NOMAD's references and callouts use meters.
        internal static double? ToMeters(float displayValue, float multiplier)
        {
            if (float.IsNaN(displayValue) || float.IsInfinity(displayValue) ||
                float.IsNaN(multiplier) || float.IsInfinity(multiplier) || multiplier <= 0)
            {
                return null;
            }
            // Restore the host's float SI precision before comparing exact meter thresholds.
            float meters = displayValue / multiplier;
            return float.IsNaN(meters) || float.IsInfinity(meters) ? (double?)null : meters;
        }
    }
}
