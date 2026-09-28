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

    private static void GuidedGoto_ReportsUnavailableWithoutDispatch()
    {
        OutputController.CreateClientCalls = 0;
        Log.LastWarning = "";

        Expect(!FlightModeController.GuidedGoto(45.0, 9.0, 5.0), "GuidedGoto remains unavailable");
        Expect(Log.LastWarning.Contains("runtime protocol v1 does not support navigation requests"),
            "operator feedback names the missing runtime capability");
        Expect(Log.LastWarning.Contains("No vehicle command was sent"),
            "operator feedback confirms that no command was sent");
        Expect(OutputController.CreateClientCalls == 0, "unavailable GuidedGoto does not create a runtime client");
    }
}

// Deliberately no Mission Planner transport assembly: direct mode/parameter writes cannot compile here.
namespace NOMAD.MissionPlanner
{
    internal static class OutputController
    {
        internal static int CreateClientCalls;

        internal static Connectivity.NomadCoreClient CreateCoreClient()
        {
            CreateClientCalls++;
            return null;
        }
    }

    public static class Log
    {
        public static string LastError = "";
        public static string LastWarning = "";
        public static void Error(string message) { LastError = message; }
        public static void Warn(string message) { LastWarning = message; }
    }
}
