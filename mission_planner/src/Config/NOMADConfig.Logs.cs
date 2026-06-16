// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

namespace NOMAD.MissionPlanner
{
    public partial class NOMADConfig
    {
        /// <summary>Default local directory for flight log open/export dialogs.</summary>
        public string DefaultLogDirectory { get; set; } = "";

        /// <summary>Remote directory searched by the Jetson Latest action.</summary>
        public string JetsonLogDirectory { get; set; } = "~/NOMAD/logs";

        public double LogVibrationWarning { get; set; } = 30;
        public double LogVibrationCritical { get; set; } = 60;
        public double LogHdopWarning { get; set; } = 2;
        public double LogHdopCritical { get; set; } = 4;
        public int LogMinimumSatellites { get; set; } = 8;
        public double LogTuneRmsWarning { get; set; } = 5;
        public double LogTuneRmsCritical { get; set; } = 10;
        public double LogEkfVarianceWarning { get; set; } = 0.8;
        public double LogEkfVarianceCritical { get; set; } = 1.0;
        public int LogLiveBufferPoints { get; set; } = 600;
        public bool LogInjectAlertsToHud { get; set; } = false;
    }
}
