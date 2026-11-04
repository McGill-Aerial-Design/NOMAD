// SPDX-License-Identifier: Apache-2.0
using System;
using System.IO;
using System.Linq;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    internal sealed class SimulationSetup
    {
        internal string[] Distributions;
        internal string Distribution, Workspace, Python;
        internal bool Prepared;

        internal static async Task<SimulationSetup> DetectAsync(string distribution, string workspace)
        {
            string installed = await SimulationSession.WslQueryAsync("--list --quiet").ConfigureAwait(false);
            if (string.IsNullOrWhiteSpace(installed))
            { throw new InvalidOperationException("No WSL distributions found. Prepare a Linux simulator workspace first."); }
            // Read only the selected/default distribution; do not start unrelated distributions.
            string metadata = await SimulationSession.QueryAsync(distribution, "/bin/sh", "-c",
                "printf '%s\\n' \"$WSL_DISTRO_NAME\" \"${1:-$HOME/nomad-simulation}\"; " +
                "command -v python3 || printf '\\n'; " +
                "if [ -f \"${1:-$HOME/nomad-simulation}/.nomad-simulation-owned\" ]; then printf 'prepared\\n'; fi",
                "nomad-detect", workspace ?? "").ConfigureAwait(false);
            return Parse(installed, metadata);
        }

        internal static SimulationSetup Parse(string installed, string metadata)
        {
            string[] names = installed.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
            string[] values = metadata.Split('\n').Select(value => value.TrimEnd('\r')).ToArray();
            if (values.Length < 2 || !names.Contains(values[0]) || !values[1].StartsWith("/", StringComparison.Ordinal))
            { throw new InvalidDataException("WSL did not report its distribution and default workspace."); }
            return new SimulationSetup { Distributions = names, Distribution = values[0], Workspace = values[1],
                Python = values.Length > 2 ? values[2] : "", Prepared = values.Length > 3 && values[3] == "prepared" };
        }
    }
}
