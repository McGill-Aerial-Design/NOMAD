// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;

namespace NOMAD.MissionPlanner
{
    public sealed class LogAnalysisThresholds
    {
        public double VibrationWarning { get; set; } = 30;
        public double VibrationCritical { get; set; } = 60;
        public double HdopWarning { get; set; } = 2;
        public double HdopCritical { get; set; } = 4;
        public int MinimumSatellites { get; set; } = 8;
        public double AttitudeRmsWarning { get; set; } = 5;
        public double AttitudeRmsCritical { get; set; } = 10;
        public double EkfVarianceWarning { get; set; } = 0.8;
        public double EkfVarianceCritical { get; set; } = 1.0;
    }

    public sealed class IssueSnapshot
    {
        public double TimeSeconds { get; set; }
        public double VibrationX { get; set; }
        public double VibrationY { get; set; }
        public double VibrationZ { get; set; }
        public double ClipCount { get; set; }
        public double Hdop { get; set; }
        public int Satellites { get; set; }
        public double AttitudeError { get; set; }
        public double EkfVariance { get; set; }
    }

    public static class IssueRules
    {
        public static List<LogAnomaly> Evaluate(IssueSnapshot snapshot, LogAnalysisThresholds thresholds)
        {
            var issues = new List<LogAnomaly>();
            thresholds = thresholds ?? new LogAnalysisThresholds();

            double vibration = Math.Max(snapshot.VibrationX, Math.Max(snapshot.VibrationY, snapshot.VibrationZ));
            if (vibration >= thresholds.VibrationCritical)
            {
                issues.Add(Issue("vibration-critical", snapshot.TimeSeconds, LogVerdict.Critical,
                    "Critical vibration", $"Peak axis is {vibration:F1} m/s2."));
            }
            else if (vibration >= thresholds.VibrationWarning)
            {
                issues.Add(Issue("vibration-warning", snapshot.TimeSeconds, LogVerdict.Warning,
                    "High vibration", $"Peak axis is {vibration:F1} m/s2."));
            }

            if (snapshot.ClipCount > 0)
            {
                issues.Add(Issue("vibration-clipping", snapshot.TimeSeconds, LogVerdict.Critical,
                    "IMU clipping", $"{snapshot.ClipCount:F0} clipped sample(s) were recorded."));
            }

            if (snapshot.Hdop >= thresholds.HdopCritical)
            {
                issues.Add(Issue("gps-hdop-critical", snapshot.TimeSeconds, LogVerdict.Critical,
                    "Poor GPS geometry", $"HDOP reached {snapshot.Hdop:F1}."));
            }
            else if (snapshot.Hdop >= thresholds.HdopWarning)
            {
                issues.Add(Issue("gps-hdop-warning", snapshot.TimeSeconds, LogVerdict.Warning,
                    "Weak GPS geometry", $"HDOP reached {snapshot.Hdop:F1}."));
            }

            if (snapshot.Satellites > 0 && snapshot.Satellites < thresholds.MinimumSatellites)
            {
                issues.Add(Issue("gps-satellites", snapshot.TimeSeconds, LogVerdict.Warning,
                    "Low satellite count", $"Minimum satellite count was {snapshot.Satellites}."));
            }

            if (snapshot.AttitudeError >= thresholds.AttitudeRmsCritical)
            {
                issues.Add(Issue("tune-critical", snapshot.TimeSeconds, LogVerdict.Critical,
                    "Poor attitude tracking", $"RMS tracking error is {snapshot.AttitudeError:F1} deg."));
            }
            else if (snapshot.AttitudeError >= thresholds.AttitudeRmsWarning)
            {
                issues.Add(Issue("tune-warning", snapshot.TimeSeconds, LogVerdict.Warning,
                    "Attitude tracking needs review", $"RMS tracking error is {snapshot.AttitudeError:F1} deg."));
            }

            if (snapshot.EkfVariance >= thresholds.EkfVarianceCritical)
            {
                issues.Add(Issue("ekf-critical", snapshot.TimeSeconds, LogVerdict.Critical,
                    "EKF variance critical", $"Peak variance is {snapshot.EkfVariance:F2}."));
            }
            else if (snapshot.EkfVariance >= thresholds.EkfVarianceWarning)
            {
                issues.Add(Issue("ekf-warning", snapshot.TimeSeconds, LogVerdict.Warning,
                    "EKF variance elevated", $"Peak variance is {snapshot.EkfVariance:F2}."));
            }

            return issues;
        }

        private static LogAnomaly Issue(
            string id,
            double timeSeconds,
            LogVerdict verdict,
            string title,
            string detail)
        {
            return new LogAnomaly
            {
                Id = id,
                TimeSeconds = timeSeconds,
                Verdict = verdict,
                Title = title,
                Detail = detail,
            };
        }
    }
}
