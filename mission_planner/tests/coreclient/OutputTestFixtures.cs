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
    internal static class Log
    {
        public static readonly List<string> Messages = new List<string>();
        public static void Info(string message) { lock (Messages) { Messages.Add(message); } }
        public static string LastError = "";
        public static string LastWarning = "";
        public static void Warn(string message) { lock (Messages) { LastWarning = message; Messages.Add(message); } }
        public static void Error(string message) { lock (Messages) { LastError = message; Messages.Add(message); } }
    }

    public enum PayloadKind { Drop, Slider, Relay, Reel }

    public sealed class PayloadControl
    {
        public string Name = "Test payload";
        public PayloadKind Kind = PayloadKind.Drop;
        public int Channel = 8;
        public int PwmMin = 1000;
        public int PwmMax = 2000;
        public int PwmNeutral = 1500;
        public int PulseMs;
        public int HoldSafetyS = 10;
        public int FullDurationS = 80;
        public bool Reversed;
    }

    public sealed class NOMADConfig
    {
        public const int MaxPayloads = 8;
        public string CoreClientCredential = "test-key";
        public int CoreRuntimePort;
        public readonly List<PayloadControl> Payloads = new List<PayloadControl> { new PayloadControl() };
        public List<PayloadControl> EnabledPayloads() => Payloads;
        public List<PayloadControl> DropPayloads() => Payloads.FindAll(p => p.Kind == PayloadKind.Drop);
        public PayloadControl ReelAt(int index)
        {
            var reels = Payloads.FindAll(p => p.Kind == PayloadKind.Reel);
            return index >= 0 && index < reels.Count ? reels[index] : null;
        }
        public PayloadControl WaterPump() => Payloads.Find(p => p.Kind == PayloadKind.Relay);
        public string JoystickSw1UpAction = "DropToggleP1";
        public string JoystickSw1DownAction = "None";
        public string JoystickSw2UpAction = "None";
        public string JoystickSw2DownAction = "None";
        public string JoystickSw3UpAction = "None";
        public string JoystickSw3DownAction = "FireWaterPump";
        public bool JoystickKillSwitchEnabled;
    }

    internal static class UiAsync
    {
        public static void RunSync(Control control, Action action, string context) => action();
    }

    public partial class PayloadControlPanel : UserControl
    {
        private readonly NOMADConfig _config;
        private const int ROW_H = 30;
        private const int ROW_GAP = 5;
        private const int TILT_SETTLE_MS = 100;
        private static readonly Color TEXT_SECONDARY = Color.Gray;
        private static readonly Color TEXT_PRIMARY = Color.White;
        private static readonly Color CARD_BG = Color.Black;
        private static readonly Color WARNING_COLOR = Color.Orange;
        private static readonly Color SUCCESS_COLOR = Color.Green;
        private static readonly Color ERROR_COLOR = Color.Red;
        private readonly bool[] _reelActive = new bool[2];
        private readonly System.Windows.Forms.Timer[] _reelSafetyTimers = new System.Windows.Forms.Timer[2];
        private readonly Button[] _fullReelButtons = new Button[4];
        private readonly bool[] _fullReelActive = new bool[4];
        private readonly int[] _fullReelRemainingMs = new int[4];
        private readonly int[] _fullReelClickCount = new int[4];
        private readonly System.Windows.Forms.Timer[] _fullReelClickReset = new System.Windows.Forms.Timer[4];
        private readonly System.Windows.Forms.Timer[] _fullReelCountdown = new System.Windows.Forms.Timer[4];
        private const int FULL_REEL_CLICKS_REQUIRED = 3;
        private const int FULL_REEL_CLICK_RESET_MS = 3000;
        private PayloadControl ReelPayload(int index) => _config.Payloads[0];
        private string ReelName(int index) => "Test reel";
        internal string TestStatus { get; private set; }

        internal PayloadControlPanel(NOMADConfig config)
        {
            _config = config;
            var y = 0;
            BuildPayloadRows(ref y);
        }

        private static Button MakeButton(string text, Color color, int width, int height)
        {
            return new Button { Text = text, BackColor = color, Width = width, Height = height };
        }

        private void SetStatus(string message, Color color) => TestStatus = message;
        internal void TestActivateReelPrerequisite(int kind)
        {
            if (kind == 0)
            {
                _fullReelActive[0] = true;
                _fullReelActive[1] = true;
            }
            else if (kind == 1)
            {
                _fullReelActive[1] = true;
                _reelActive[0] = true;
            }
            else
            {
                _reelActive[0] = true;
            }
        }

        internal bool TestHasReelTimers => _reelSafetyTimers[0] != null || _fullReelCountdown[0] != null;
        internal System.Threading.Tasks.Task TestStartReel() => StartReel(0, 2000);
        internal System.Threading.Tasks.Task TestStopReel() => StopReel(0);
        internal System.Threading.Tasks.Task TestStartFullReel() => StartFullReel(0);
        internal System.Threading.Tasks.Task TestStopFullReel() => StopFullReel(0, false);
        internal System.Threading.Tasks.Task TestDrop()
        {
            var interlock = DropInterlock(0);
            interlock.RegisterClick(NowMs());
            interlock.RegisterClick(NowMs());
            return ExecuteDrop(0, interlock.RegisterClick(NowMs()).Authorization);
        }
        internal void TestDropClick() => OnDropClick(0);
        internal System.Threading.Tasks.Task TestUnauthorizedDrop() => ExecuteDrop(0, null);
        internal System.Threading.Tasks.Task TestRetract() => ExecuteRetract(0);
        internal System.Threading.Tasks.Task TestToggleRelay(PayloadControl payload, Button button) => ToggleRelay(payload, button);
    }
}
