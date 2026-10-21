// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System.Collections.Generic;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADConfig
    {
        internal string GetInputMappingError()
        {
            if (JoystickButtonIndices == null || JoystickButtonIndices.Length != 6)
            { return "Direct USB HID mappings require six button indices."; }
            var active = new HashSet<int>();
            var bindings = new[] { JoystickSw1UpAction, JoystickSw1DownAction, JoystickSw2UpAction,
                JoystickSw2DownAction, JoystickSw3UpAction, JoystickSw3DownAction };
            for (int slot = 0; slot < bindings.Length; slot++)
            {
                int index = JoystickButtonIndices[slot];
                if (index < 0 || index > 127) { return "Direct USB HID button indices must be between 0 and 127."; }
                string binding = bindings[slot];
                if (string.IsNullOrEmpty(binding) || binding == "None") { continue; }
                int separator = binding.LastIndexOf(':');
                if (separator <= 0 || separator == binding.Length - 1)
                { return "Joystick actions must refer to payload IDs and operations. Review unsupported mappings before loading."; }
                if (!active.Add(index)) { return "Active actuator bindings must use unique physical HID button indices."; }
            }
            return null;
        }
    }
}
