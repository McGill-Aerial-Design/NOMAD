// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;

namespace NOMAD.MissionPlanner
{
    internal static class FlightModeVisuals
    {
        private static readonly Color[] FallbackPalette =
        {
            Color.FromArgb(46, 134, 222),
            Color.FromArgb(22, 160, 133),
            Color.FromArgb(241, 196, 15),
            Color.FromArgb(142, 68, 173),
            Color.FromArgb(230, 126, 34),
            Color.FromArgb(26, 188, 156),
            Color.FromArgb(192, 57, 43),
            Color.FromArgb(127, 140, 141),
        };

        public static Color ColorFor(string mode)
        {
            string normalized = Normalize(mode);
            if (normalized.Contains("STABILIZE")) return Color.FromArgb(46, 134, 222);
            if (normalized.Contains("LOITER") || normalized.Contains("POSHOLD"))
                return Color.FromArgb(22, 160, 133);
            if (normalized.Contains("LAND")) return Color.FromArgb(230, 126, 34);
            if (normalized.Contains("RTL")) return Color.FromArgb(192, 57, 43);
            if (normalized.Contains("AUTO")) return Color.FromArgb(142, 68, 173);
            if (normalized.Contains("GUIDED")) return Color.FromArgb(39, 174, 96);
            if (normalized.Contains("ALTHOLD") || normalized.Contains("ALT_HOLD"))
                return Color.FromArgb(241, 196, 15);
            if (normalized.Contains("ACRO")) return Color.FromArgb(211, 84, 145);
            if (normalized.Contains("BRAKE")) return Color.FromArgb(52, 152, 219);
            if (normalized.Contains("CIRCLE")) return Color.FromArgb(26, 188, 156);

            int hash = normalized.GetHashCode() & int.MaxValue;
            return FallbackPalette[hash % FallbackPalette.Length];
        }

        private static string Normalize(string mode)
            => string.IsNullOrWhiteSpace(mode) ? "UNKNOWN" : mode.Trim().ToUpperInvariant();
    }
}
