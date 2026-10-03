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
        OutputController.Initialize(null);
        Log.LastWarning = "";

        Expect(!FlightModeController.GuidedGoto(45.0, 9.0, 5.0), "GuidedGoto remains unavailable");
        Expect(Log.LastWarning.Contains("runtime protocol v1 does not support navigation requests"),
            "operator feedback names the missing runtime capability");
        Expect(Log.LastWarning.Contains("No vehicle command was sent"),
            "operator feedback confirms that no command was sent");
        Expect(OutputController.CreateCoreClient() == null, "unavailable GuidedGoto does not create a runtime client");
    }
}
