// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;

namespace NOMAD.MissionPlanner
{
    internal static class MissionPlannerVersion
    {
        public static string GetWarning(Version actual, string reviewedTarget)
        {
            if (!Version.TryParse(reviewedTarget, out var reviewed))
            {
                return "This development build has no reviewed Mission Planner target metadata.";
            }
            if (actual == null)
            {
                return $"Could not determine Mission Planner version. Reviewed target: {reviewedTarget}.";
            }
            if (actual.Major == reviewed.Major && actual.Minor == reviewed.Minor && actual.Build == reviewed.Build)
            {
                return null;
            }
            return $"Untested Mission Planner version {actual}. Reviewed target: {reviewedTarget}. " +
                "Some features may not work correctly.";
        }
    }
}
