// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using NOMAD.MissionPlanner;

// In-memory host data only; no Mission Planner constructor, stream or device is used.
namespace MissionPlanner
{
    public static class MainV2 { public static VehicleLink comPort; }
    public sealed class VehicleLink { public Vehicle MAV; }
    public sealed class Vehicle
    {
        public State cs = new State();
        public Dictionary<string, Parameter> param = new Dictionary<string, Parameter>();
    }
    public sealed class State { public bool armed; public double battery_voltage = 11.5, battery_usedmah = 100; }
    public sealed class Parameter { public object Value { get; set; } }
}

internal static class BatteryHealthTests
{
    private static MissionPlanner.Vehicle Aircraft(double low, double critical, double capacity)
    {
        var vehicle = new MissionPlanner.Vehicle();
        vehicle.param["BATT_MONITOR"] = new MissionPlanner.Parameter { Value = 1 };
        vehicle.param["BATT_LOW_VOLT"] = new MissionPlanner.Parameter { Value = low };
        vehicle.param["BATT_CRT_VOLT"] = new MissionPlanner.Parameter { Value = critical };
        vehicle.param["BATT_CAPACITY"] = new MissionPlanner.Parameter { Value = capacity };
        return vehicle;
    }

    private static void Require(bool condition, string message)
    {
        if (!condition) { throw new InvalidOperationException(message); }
    }

    private static int Main()
    {
        var first = Aircraft(11, 10, 1000);
        var second = Aircraft(14, 12, 2000);
        MissionPlanner.MainV2.comPort = new MissionPlanner.VehicleLink { MAV = first };
        var initial = BatteryHealth.Read(1);
        Require(initial.LowVoltage == 11 && initial.CapacityMah == 1000 && initial.Severity == 0,
            "Initial aircraft battery state did not use its configured thresholds.");
        MissionPlanner.MainV2.comPort.MAV = second;
        var selected = BatteryHealth.Read(1);
        Require(selected.LowVoltage == 14 && selected.CriticalVoltage == 12 && selected.CapacityMah == 2000 &&
            selected.Severity == 2, "Aircraft selection reused another aircraft's battery parameters or severity.");
        second.param["BATT_LOW_VOLT"].Value = 15;
        Require(BatteryHealth.GetBattParam(second, 1, "LOW_VOLT") == 15,
            "A native in-memory parameter update was hidden by a second cache.");
        second.param.Remove("BATT_LOW_VOLT");
        Require(BatteryHealth.GetBattParam(second, 1, "LOW_VOLT") == 0,
            "Missing battery parameter retained a previous value.");
        foreach (object invalid in new object[]
            { "invalid", double.NaN, double.PositiveInfinity, double.NegativeInfinity })
        {
            second.param["BATT_LOW_VOLT"] = new MissionPlanner.Parameter { Value = invalid };
            Require(BatteryHealth.GetBattParam(second, 1, "LOW_VOLT") == 0,
                "Malformed or nonfinite battery parameter was presented as a threshold.");
        }
        second.param = null;
        Require(BatteryHealth.GetBattParam(second, 1, "LOW_VOLT") == 0 && BatteryHealth.Read(1) == null,
            "Unavailable parameter bag was presented as a configured battery monitor.");
        Require(BatteryHealth.GetBattParam(null, 1, "LOW_VOLT") == 0,
            "Missing aircraft did not return the established unavailable-parameter value.");
        Console.WriteLine("Battery health tests passed " +
            "(aircraft selection, live parameter updates and unavailable values).");
        return 0;
    }
}
