// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using NOMAD.MissionPlanner;

internal static partial class NomadCoreClientTests
{
    private static void Termination_ReportsUnavailableWithoutVehicleDispatch()
    {
        Expect(!FlightModeController.RequestTermination(), "unconfigured termination must fail");
        Expect(Log.LastError.Contains("Termination unavailable"), "operator must see unavailable termination");

        Log.LastError = "";
        Expect(!FlightModeController.RequestTermination(), "configured core cannot qualify termination");
        Expect(Log.LastError.Contains("aircraft mechanism"), "failure must identify the missing aircraft mechanism");
        Expect(!FlightModeController.RequestTermination(), "repeated activation cannot report success");
    }
}

// Deliberately no Mission Planner transport assembly: direct mode/parameter writes cannot compile here.
namespace NOMAD.MissionPlanner
{
    internal static class OutputController
    {
        internal static Connectivity.NomadCoreClient CreateCoreClient() => null;
    }

    public static class Log
    {
        public static string LastError = "";
        public static void Error(string message) { LastError = message; }
        public static void Warn(string message) { }
    }
}
