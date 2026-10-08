// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using NOMAD.MissionPlanner;

internal static partial class NomadCoreClientTests
{
    private static void Termination_ReportsUnavailableWithoutVehicleDispatch()
    {
        Expect(!TerminationControl.RequestTermination(), "unconfigured termination must fail");
        Expect(Log.LastError.Contains("Termination unavailable"), "operator must see unavailable termination");

        Log.LastError = "";
        Expect(!TerminationControl.RequestTermination(), "configured core cannot qualify termination");
        Expect(Log.LastError.Contains("aircraft mechanism"), "failure must identify the missing aircraft mechanism");
        Expect(!TerminationControl.RequestTermination(), "repeated activation cannot report success");
    }

}
