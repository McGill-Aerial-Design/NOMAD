// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Reflection;
using System.Threading.Tasks;
using System.Threading;
using IMyJoystickState = MissionPlanner.Joystick.IMyJoystickState;

namespace NOMAD.MissionPlanner
{
    public sealed partial class NomadJoystickService
    {
        private static Dictionary<string, PropertyInfo> BuildAxisMap()
        {
            var result = new Dictionary<string, PropertyInfo>(StringComparer.OrdinalIgnoreCase);
            foreach (var name in new[]
            {
                "X", "Y", "Z", "Rx", "Ry", "Rz"
            }
            )
            {
                var property = typeof(IMyJoystickState).GetProperty(name);
                if (property != null)
                {
                    result[name] = property;
                }
            }
            return result;
        }

        internal static bool TryReadAxisNorm(IMyJoystickState state, string name, bool invert,
            float deadzone, out float value)
        {
            value = 0;
            if (state == null || string.IsNullOrWhiteSpace(name) || float.IsNaN(deadzone) ||
                float.IsInfinity(deadzone) || deadzone < 0 || deadzone >= 1) { return false; }
            try
            {
                int raw;
                if (name.Equals("Slider1", StringComparison.OrdinalIgnoreCase) ||
                    name.Equals("Slider2", StringComparison.OrdinalIgnoreCase))
                {
                    int index = name.Equals("Slider1", StringComparison.OrdinalIgnoreCase) ? 0 : 1;
                    var sliders = state.GetSlider();
                    if (sliders == null || sliders.Length <= index)
                    {
                        return false;
                    }
                    raw = sliders[index];
                }
                else if (AxisProps.TryGetValue(name, out var property))
                {
                    raw = (int)property.GetValue(state, null);
                }
                else
                {
                    return false;
                }
                if (raw < 0 || raw > 65535)
                {
                    return false;
                }
                // The unsigned axis midpoint falls between these two integer samples.
                if (raw == 32767 || raw == 32768) { return true; }
                float normalized = (raw - 32767.5f) / 32767.5f;
                if (Math.Abs(normalized) < deadzone)
                {
                    return true;
                }
                float signed = (normalized < 0 ? -1 : 1) * (Math.Abs(normalized) - deadzone) / (1 - deadzone);
                value = invert ? -signed : signed;
                return true;
            }
            catch
            {
                return false;
            }
        }

    }
}
