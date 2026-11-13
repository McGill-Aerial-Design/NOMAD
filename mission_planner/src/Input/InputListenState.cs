// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using IMyJoystickState = MissionPlanner.Joystick.IMyJoystickState;
namespace NOMAD.MissionPlanner
{
    // Detection has no output/controller dependency and never dispatches commands.
    internal sealed class InputListenState
    {
        private bool[] _previous;
        private readonly Dictionary<string, float> _baseline = new Dictionary<string, float>();
        private readonly long _startedTicks;
        internal bool Active { get; private set; } = true;
        internal int? Button { get; private set; }
        internal string Axis { get; private set; }
        internal string Status { get; private set; } = "Move one button, toggle or axis.";
        internal InputListenState(long now) { _startedTicks = now; }
        internal void Stop(string status) { Active = false; Status = status; }
        internal void Sample(IMyJoystickState sample, bool connected, long now)
        {
            if (!Active) { return; }
            if (connected) { Stop("Disconnect the aircraft before listening."); return; }
            if ((now - _startedTicks) / (double)Stopwatch.Frequency >= 15)
            { Stop("Listen timed out. Try again."); return; }
            if (sample == null) { Stop("Input device is unavailable."); return; }
            try
            {
                var buttons = sample.GetButtons();
                if (buttons == null || (_previous != null && buttons.Length != _previous.Length))
                { Stop("Input device changed or is unavailable."); return; }
                if (_previous != null)
                {
                    for (int index = 0; index < Math.Min(128, buttons.Length); index++)
                    {
                        if (buttons[index] && !_previous[index])
                        { Button = index; Stop("Detected " + ButtonLabel(index) + "."); return; }
                    }
                }
                foreach (string axis in InputBinding.Axes)
                {
                    if (!NomadJoystickService.TryReadAxisNorm(sample, axis, false, 0, out float value)) { continue; }
                    if (_previous == null) { _baseline[axis] = value; }
                    else if (_baseline.TryGetValue(axis, out float baseline) && Math.Abs(value - baseline) >= 0.35f)
                    { Axis = axis; Stop("Detected axis " + axis + ". Choose continuous or a 2/3-position switch."); return; }
                }
                _previous = (bool[])buttons.Clone();
            }
            catch { Stop("Input device is unavailable."); }
        }
        internal static string ButtonLabel(int index) => "Button " + (index + 1);
    }
}
