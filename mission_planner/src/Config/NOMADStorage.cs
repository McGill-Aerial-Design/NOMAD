// SPDX-License-Identifier: Apache-2.0
using System;
using System.IO;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    internal static class NOMADStorage
    {
        internal static string PluginsDirectory
        {
            get
            {
                string custom = Settings.CustomUserDataDirectory;
                if (string.IsNullOrEmpty(custom))
                {
                    return Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
                        "Mission Planner", "plugins");
                }
                // An invalid explicit host directory must never fall back to operator data.
                if (!Path.IsPathRooted(custom) || Path.GetPathRoot(custom).Length < 3)
                { throw new ArgumentException("Mission Planner's custom user data directory must be absolute."); }
                return Path.Combine(Path.GetFullPath(Settings.GetUserDataDirectory()), "plugins");
            }
        }
    }
}
