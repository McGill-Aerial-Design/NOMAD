// SPDX-License-Identifier: Apache-2.0
using System;
using IMyJoystickState = MissionPlanner.Joystick.IMyJoystickState;
namespace NOMAD.MissionPlanner
{
    // One physical control must return to neutral after retirement before it can act again.
    internal sealed class InputControlState
    {
        internal int Direction { get; private set; }
        internal bool Ready { get; private set; }
        internal float Value { get; private set; }
        internal bool Changed { get; private set; }
        // Retire authority without manufacturing a change in the next identical physical sample.
        internal void RequireNeutral() { Ready = false; }
        internal void Reset() { Direction = 0; Ready = false; Value = 0; Changed = false; }
        internal bool Sample(InputBinding binding, IMyJoystickState sample)
        {
            int direction = 0;
            float value = 0;
            bool valid = sample != null;
            try
            {
                if (binding.Source == "Button" || binding.Source == "ButtonPair")
                {
                    var buttons = sample?.GetButtons();
                    valid = buttons != null && binding.Button < buttons.Length &&
                        (binding.Source != "ButtonPair" || binding.OtherButton < buttons.Length);
                    if (valid)
                    {
                        bool first = buttons[binding.Button];
                        bool second = binding.Source == "ButtonPair" && buttons[binding.OtherButton];
                        direction = first && second ? 2 : first ? 1 : second ? -1 : 0;
                    }
                }
                else
                {
                    valid = NomadJoystickService.TryReadAxisNorm(sample, binding.Axis, binding.Invert,
                        binding.Continuous ? binding.Deadzone : 0, out value);
                    if (binding.Continuous) { direction = value == 0 ? 0 : value > 0 ? 1 : -1; }
                    else if (binding.Source == "AxisSwitch2")
                    { direction = value <= -0.5f ? 0 : value >= 0.5f ? 1 : Direction; }
                    else
                    { direction = Math.Abs(value) <= 0.25f ? 0 : value >= 0.5f ? 1 : value <= -0.5f ? -1 : Direction; }
                }
            }
            catch { valid = false; }
            if (!valid) { Reset(); return false; }
            Changed = direction != Direction || (binding.Continuous && Math.Abs(value - Value) >= 0.001f);
            if (direction == 0 && (binding.Source != "AxisSwitch2" || value <= -0.5f)) { Ready = true; }
            else if (direction == 2 || (!binding.Continuous && Direction != 0 && direction != Direction)) { Ready = false; }
            Direction = direction;
            Value = value;
            return true;
        }
    }
}
