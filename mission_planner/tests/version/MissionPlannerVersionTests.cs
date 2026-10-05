// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using NOMAD.MissionPlanner;

internal static class MissionPlannerVersionTests
{
    private static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new Exception(message);
        }
    }

    private static int Main()
    {
        const string target = "2.3.4";
        Require(MissionPlannerVersion.GetWarning(new Version(target), target) == null,
            "Reviewed version unexpectedly warned");
        Require(MissionPlannerVersion.GetWarning(new Version("2.3.4.7"), target) == null,
            "Assembly revision changed the reviewed product version");
        foreach (string version in new[] { "2.3.3", "2.3.5", "3.0.0" })
        {
            string warning = MissionPlannerVersion.GetWarning(new Version(version), target);
            Require(warning != null && warning.Contains(target) && warning.Contains(version),
                $"Unreviewed version {version} did not identify actual and reviewed targets");
        }
        Require(MissionPlannerVersion.GetWarning(null, target).Contains(target),
            "Missing actual version omitted reviewed target");
        foreach (string targetMetadata in new[] { "", "invalid", NomadRelease.MissionPlannerTarget })
        {
            Require(MissionPlannerVersion.GetWarning(new Version(target), targetMetadata)
                .Contains("no reviewed Mission Planner target"), "Fallback impersonated a reviewed target");
        }
        Console.WriteLine("Mission Planner version warning tests passed.");
        return 0;
    }
}
