// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    internal sealed partial class FenceParameters
    {
        private TerminationController _termination;
        private string _descentLimit;
        internal string TerminationStatus { get; private set; } = "Requires the advertised native extension.";

        internal async Task ReadTerminationAsync(TerminationController termination)
        {
            _termination = termination;
            var mav = _target.Connection.MAVlist[_target.System, _target.Component];
            string firmware = mav.cs.firmware.ToString();
            _descentLimit = firmware == "ArduCopter2" ? "WP_SPD_DN" : firmware == "ArduPlane" ? "Q_WP_SPD_DN" : null;
            if (termination == null || _descentLimit == null || !mav.param.ContainsKey(_descentLimit) ||
                new[] { "AFS_ENABLE", "AFS_TERM_ACTION", "AFS_TERM_RATE" }.Any(name => !mav.param.ContainsKey(name)))
            { TerminationStatus = "Unavailable: matching native TERMINATE extension and descent capability required."; return; }
            try
            {
                EnsureCurrentRead();
                if (!await termination.CheckConfigurationAsync().ConfigureAwait(false))
                { TerminationStatus = "Unavailable: no selectable native TERMINATE advertisement."; return; }
                float limit = await _target.ReadParameterAsync(_descentLimit).ConfigureAwait(false);
                EnsureCurrentRead();
                if (!Finite(limit) || limit < 2)
                { TerminationStatus = "Unavailable: native descent capability is below 2 m/s or invalid."; return; }
                var settings = new List<FenceParameter>();
                foreach (string name in new[] { "AFS_TERM_ACTION", "AFS_TERM_RATE", "AFS_RC_FAIL_TIME",
                    "AFS_RC", "AFS_GEOFENCE", "AFS_ENABLE" })
                {
                    if (!mav.param.ContainsKey(name)) { continue; }
                    var parameter = await ReadParameterAsync(name, firmware).ConfigureAwait(false);
                    ConstrainTermination(parameter, limit);
                    settings.Add(parameter);
                }
                Parameters.AddRange(settings);
                TerminationStatus = $"Separate descent rate: 2–{limit} m/s ({_descentLimit}). Normal LAND/QLAND is unchanged.";
            }
            catch (Exception error) { TerminationStatus = "Termination settings unavailable: " + error.Message; }
        }

        private static void ConstrainTermination(FenceParameter parameter, float limit)
        {
            parameter.Bits.Clear();
            parameter.Options.Clear();
            parameter.Description = "Private controlled-termination extension. Review aircraft commissioning before flight.";
            switch (parameter.Name)
            {
                case "AFS_TERM_ACTION":
                    parameter.DisplayName = "Termination response";
                    parameter.Options.Add(new KeyValuePair<int, string>(44, "Controlled termination (44)"));
                    break;
                case "AFS_TERM_RATE":
                    parameter.DisplayName = "Termination descent speed";
                    parameter.Units = "m/s";
                    parameter.HasRange = true; parameter.Minimum = 2; parameter.Maximum = limit;
                    break;
                case "AFS_RC_FAIL_TIME":
                    parameter.DisplayName = "RC loss timeout";
                    parameter.Units = "s";
                    parameter.HasRange = true; parameter.Minimum = 0.1; parameter.Maximum = 4.5;
                    break;
                default:
                    parameter.DisplayName = parameter.Name == "AFS_ENABLE" ? "Advanced failsafe enabled" :
                        parameter.Name == "AFS_RC" ? "Terminate on RC loss" : "Terminate on fence breach";
                    parameter.Options.Add(new KeyValuePair<int, string>(0, "Disabled"));
                    parameter.Options.Add(new KeyValuePair<int, string>(1, "Enabled"));
                    break;
            }
        }

        private async Task<bool> CheckTerminationChangesAsync(IDictionary<string, float> changes, bool verifySettings)
        {
            if (_termination == null || !CanApply || !_termination.CanConfigure(_target.Generation)) { return false; }
            float? Candidate(string name) => changes.TryGetValue(name, out float value) ? value :
                Parameters.Where(p => p.Name == name).Select(p => (float?)p.Value).FirstOrDefault();
            float? action = Candidate("AFS_TERM_ACTION"), rate = Candidate("AFS_TERM_RATE");
            if (action != 44 || !rate.HasValue || !Finite(rate.Value) || rate < 2) { return false; }
            if (Candidate("AFS_ENABLE") == 1)
            {
                foreach (string name in new[] { "AFS_RC", "AFS_GEOFENCE" })
                {
                    float? value = Candidate(name);
                    if (value != 1) { return false; }
                }
                float? timeout = Candidate("AFS_RC_FAIL_TIME");
                if (!timeout.HasValue || !Finite(timeout.Value) || timeout <= 0 || timeout > 4.5) { return false; }
            }
            if (verifySettings)
            {
                foreach (var parameter in Parameters.Where(p => p.Name.StartsWith("AFS_", StringComparison.Ordinal)))
                {
                    if (await _target.ReadParameterAsync(parameter.Name).ConfigureAwait(false) != parameter.Value) { return false; }
                }
            }
            float limit = await _target.ReadParameterAsync(_descentLimit).ConfigureAwait(false);
            return CanApply && _termination.CanConfigure(_target.Generation) && Finite(limit) && limit >= rate;
        }

        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
