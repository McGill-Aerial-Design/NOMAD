// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;

// Only unrelated Mission Planner/configuration chrome is replaced. Tests compile
// the production output controller, payload actions and panel command handlers.
namespace MissionPlanner { }
namespace MissionPlanner.Utilities { }

namespace NOMAD.MissionPlanner
{
    internal static class AudioAlerts { public static void Speak(string message, string component) { } }
    internal static class Log
    {
        public static readonly List<string> Messages = new List<string>();
        public static void Info(string message) { lock (Messages) { Messages.Add(message); } }
        public static string LastError = "";
        public static string LastWarning = "";
        public static void Warn(string message) { lock (Messages) { LastWarning = message; Messages.Add(message); } }
        public static void Error(string message) { lock (Messages) { LastError = message; Messages.Add(message); } }
    }

    public sealed partial class NOMADConfig
    {
        public string CoreClientCredential = "test-key";
        public int CoreRuntimePort;
        public int[] JoystickButtonIndices = new[] { 0, 1, 2, 3, 4, 5 };
        public string JoystickPositionActuatorId = "";
        public string JoystickSw1UpAction = "None";
        public string JoystickSw1DownAction = "None";
        public string JoystickSw2UpAction = "None";
        public string JoystickSw2DownAction = "None";
        public string JoystickSw3UpAction = "None";
        public string JoystickSw3DownAction = "None";
        public int JoystickTerminationButtonIndex = 6;
        public bool JoystickKillSwitchEnabled;
    }
    internal static class UiAsync
    {
        public static void RunSync(Control control, Action action, string context) => action();
    }

}
