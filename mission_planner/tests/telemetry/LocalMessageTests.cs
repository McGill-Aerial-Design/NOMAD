// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

// Only observation/UI surfaces exist in this fixture; vehicle command APIs are absent.
namespace MAVLink
{
    public enum MAV_SEVERITY { CRITICAL = 2, ERROR = 3, WARNING = 4, INFO = 6 }
}

namespace MissionPlanner
{
    public sealed class CurrentState
    {
        public readonly List<(DateTime time, string text)> messages = new List<(DateTime, string)>();
    }
    public sealed class Vehicle { public readonly CurrentState cs = new CurrentState(); }
    public sealed class Port { public readonly Vehicle MAV = new Vehicle(); }
    public sealed class MainV2 : Form
    {
        public static MainV2 instance;
        public static Port comPort = new Port();
    }
}

internal static class LocalMessageTests
{
    private static void Require(bool condition, string description)
    {
        if (!condition)
        {
            throw new Exception(description);
        }
    }

    [STAThread]
    private static int Main()
    {
        using (var owner = new MissionPlanner.MainV2())
        {
            MissionPlanner.MainV2.instance = owner;
            var injector = new TelemetryInjector();
            var messages = MissionPlanner.MainV2.comPort.MAV.cs.messages;
            injector.InjectStatusText(null);
            injector.InjectStatusText("  ");
            Require(messages.Count == 0, "Blank local notifications must be ignored");
            foreach (var severity in new[] { MAVLink.MAV_SEVERITY.WARNING, MAVLink.MAV_SEVERITY.CRITICAL })
            {
                string text = "Unicode notification: \u00e9 " + new string('x', 80);
                injector.InjectStatusText(text, severity);
                Require(messages[messages.Count - 1].text == $"NOMAD [{severity}]: {text}",
                    "Local text must retain severity, Unicode and the complete message");
            }
            injector.SendCustomStatus("Available");
            Require(messages[messages.Count - 1].text == "NOMAD [INFO]: Available", "Prefix must appear once");
            MissionPlanner.MainV2.comPort = null;
            injector.InjectStatusText("Disconnected", MAVLink.MAV_SEVERITY.ERROR);
            Require(messages.Count == 3, "Unavailable local display must not add a message or throw");
            MissionPlanner.MainV2.instance = null;
        }
        Console.WriteLine("Local notification tests passed");
        return 0;
    }
}
