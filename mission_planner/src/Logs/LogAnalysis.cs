// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    public static class LogAnalysis
    {
        public static LogSummary Analyze(IFlightLogData log, LogAnalysisThresholds thresholds = null)
        {
            if (log == null) throw new ArgumentNullException(nameof(log));
            thresholds = thresholds ?? new LogAnalysisThresholds();

            var summary = new LogSummary
            {
                SourceName = string.IsNullOrWhiteSpace(log.SourcePath)
                    ? "Sample flight"
                    : Path.GetFileName(log.SourcePath),
            };

            var bounds = FindTimeBounds(log);
            summary.LogDurationSeconds = Math.Max(0, bounds.End - bounds.Start);

            AnalyzeFlightTime(log, summary, bounds);
            AnalyzeBattery(log, summary);
            AnalyzeGps(log, summary, thresholds);
            AnalyzeVibration(log, summary, thresholds);
            AnalyzeTune(log, summary, thresholds);
            AnalyzeEkf(log, summary, thresholds);
            AnalyzeThrottle(log, summary);
            AnalyzeMessages(log, summary);
            AnalyzeModes(log, summary, bounds);

            summary.Anomalies.Sort((a, b) => a.TimeSeconds.CompareTo(b.TimeSeconds));
            summary.OverallVerdict = summary.Anomalies.Any(a => a.Verdict == LogVerdict.Critical)
                ? LogVerdict.Critical
                : summary.Anomalies.Any(a => a.Verdict == LogVerdict.Warning)
                    ? LogVerdict.Warning
                    : summary.Anomalies.Count > 0 ? LogVerdict.Info : LogVerdict.Good;
            return summary;
        }

        private static (double Start, double End) FindTimeBounds(IFlightLogData log)
        {
            double start = double.MaxValue;
            double end = double.MinValue;
            foreach (string type in PreferredTypes(log, "GPS", "ATT", "BAT", "EV", "MODE", "VIBE", "MSG"))
            {
                foreach (LogRecord row in log.Records(type))
                {
                    start = Math.Min(start, row.TimeSeconds);
                    end = Math.Max(end, row.TimeSeconds);
                }
                if (start < double.MaxValue && end > start) break;
            }
            return start == double.MaxValue ? (0, 0) : (start, end);
        }

        private static void AnalyzeFlightTime(
            IFlightLogData log,
            LogSummary summary,
            (double Start, double End) bounds)
        {
            var events = Rows(log, "EV").ToList();
            bool armed = false;
            double armedAt = 0;
            foreach (LogRecord row in events)
            {
                int id = (int)row.GetDouble(-1, "Id", "ID", "Event");
                if (id == 10 && !armed)
                {
                    armed = true;
                    armedAt = row.TimeSeconds;
                }
                else if (id == 11 && armed)
                {
                    summary.ArmedDurationSeconds += Math.Max(0, row.TimeSeconds - armedAt);
                    armed = false;
                }
            }
            if (armed) summary.ArmedDurationSeconds += Math.Max(0, bounds.End - armedAt);

            bool airborne = false;
            double airborneAt = 0;
            double homeAlt = double.NaN;
            foreach (LogRecord row in Rows(log, "GPS"))
            {
                double speed = row.GetDouble(0, "Spd", "Speed", "GroundSpeed");
                double alt = NormalizeAltitude(row.GetDouble(double.NaN, "Alt", "RelAlt"));
                if (double.IsNaN(homeAlt) && !double.IsNaN(alt)) homeAlt = alt;
                bool flying = speed > 1.5
                    || (!double.IsNaN(alt) && !double.IsNaN(homeAlt) && Math.Abs(alt - homeAlt) > 2);
                if (flying && !airborne)
                {
                    airborne = true;
                    airborneAt = row.TimeSeconds;
                    summary.TakeoffCount++;
                }
                else if (!flying && airborne)
                {
                    summary.AirborneDurationSeconds += Math.Max(0, row.TimeSeconds - airborneAt);
                    airborne = false;
                }
            }
            if (airborne) summary.AirborneDurationSeconds += Math.Max(0, bounds.End - airborneAt);

            summary.Metrics.Add(new LogMetric
            {
                Label = "Flight time",
                Value = FormatDuration(summary.ArmedDurationSeconds > 0
                    ? summary.ArmedDurationSeconds
                    : summary.LogDurationSeconds),
                Verdict = LogVerdict.Info,
                Detail = $"{FormatDuration(summary.AirborneDurationSeconds)} airborne, " +
                    $"{summary.TakeoffCount} takeoff(s)",
            });
        }

        private static void AnalyzeBattery(IFlightLogData log, LogSummary summary)
        {
            int rowCount = 0;
            int voltageCount = 0;
            int currentCount = 0;
            double voltageSum = 0;
            double currentSum = 0;
            double peakCurrent = 0;
            double startVoltage = 0;
            double endVoltage = 0;
            double firstTotal = double.NaN;
            double lastTotal = double.NaN;
            double integratedMah = 0;
            double previousTime = 0;
            double previousCurrent = 0;
            bool havePreviousCurrent = false;

            foreach (LogRecord row in Rows(log, "BAT"))
            {
                rowCount++;
                if (row.TryGetDouble(out double voltage, "Volt", "Voltage") && voltage > 0)
                {
                    if (voltageCount == 0) startVoltage = voltage;
                    endVoltage = voltage;
                    voltageSum += voltage;
                    voltageCount++;
                }
                if (row.TryGetDouble(out double current, "Curr", "Current") && current >= 0)
                {
                    currentSum += current;
                    currentCount++;
                    peakCurrent = Math.Max(peakCurrent, current);
                    if (havePreviousCurrent)
                    {
                        double dtHours = Math.Max(0, row.TimeSeconds - previousTime) / 3600;
                        integratedMah += (previousCurrent + current) / 2 * dtHours * 1000;
                    }
                    previousTime = row.TimeSeconds;
                    previousCurrent = current;
                    havePreviousCurrent = true;
                }
                if (row.TryGetDouble(out double total, "CurrTot", "ConsumedMah", "CurrentConsumed")
                    && total >= 0)
                {
                    if (double.IsNaN(firstTotal)) firstTotal = total;
                    lastTotal = total;
                }
            }
            if (rowCount == 0) return;

            double usedMah = !double.IsNaN(firstTotal) && !double.IsNaN(lastTotal)
                ? Math.Max(0, lastTotal - firstTotal)
                : integratedMah;
            double avgCurrent = currentCount > 0 ? currentSum / currentCount : 0;
            double avgVoltage = voltageCount > 0 ? voltageSum / voltageCount : 0;
            double capacity = FirstPositive(
                log.Parameter("BATT_CAPACITY"),
                log.Parameter("BATT1_CAPACITY"),
                log.Parameter("BATT_CAPACITY_MAH"));
            double enduranceMinutes = avgCurrent > 0 && capacity > 0 ? capacity / (avgCurrent * 1000) * 60 : 0;
            double measuredDuration = summary.ArmedDurationSeconds > 0
                ? summary.ArmedDurationSeconds
                : summary.LogDurationSeconds;
            double flightMinutes = Math.Max(1d / 60, measuredDuration / 60);
            double usedWh = usedMah / 1000 * avgVoltage;

            summary.Metrics.Add(new LogMetric
            {
                Label = "Battery",
                Value = usedMah > 0 ? $"{usedMah:F0} mAh" : $"{avgCurrent:F1} A avg",
                Unit = "",
                Verdict = LogVerdict.Info,
                Detail = voltageCount > 0
                    ? $"{startVoltage:F1} -> {endVoltage:F1} V, {peakCurrent:F1} A peak, {usedWh:F1} Wh"
                    : $"{avgCurrent:F1} A average, {peakCurrent:F1} A peak",
            });
            summary.Metrics.Add(new LogMetric
            {
                Label = "Endurance",
                Value = enduranceMinutes > 0 ? $"~{enduranceMinutes:F1} min" : "Unavailable",
                Verdict = LogVerdict.Info,
                Detail = usedMah > 0
                    ? $"{usedMah / flightMinutes:F0} mAh/min, {(avgVoltage * avgCurrent):F0} W average"
                    : "Set BATT_CAPACITY and log current to estimate endurance",
            });
        }

        private static void AnalyzeGps(
            IFlightLogData log,
            LogSummary summary,
            LogAnalysisThresholds thresholds)
        {
            int rowCount = 0;
            int speedCount = 0;
            double speedSum = 0;
            double maxSpeed = 0;
            int minimumSatellites = int.MaxValue;
            double maximumHdop = 0;
            double minimumAltitude = double.MaxValue;
            double maximumAltitude = double.MinValue;
            double lastTime = 0;
            double distance = 0;
            double climb = 0;
            double descent = 0;
            bool havePrevious = false;
            double previousLat = 0, previousLon = 0, previousAlt = 0;

            foreach (LogRecord row in Rows(log, "GPS"))
            {
                rowCount++;
                lastTime = row.TimeSeconds;
                if (row.TryGetDouble(out double speed, "Spd", "Speed", "GroundSpeed") && speed >= 0)
                {
                    speedSum += speed;
                    speedCount++;
                    maxSpeed = Math.Max(maxSpeed, speed);
                }
                if (row.TryGetDouble(out double satellites, "NSats", "Satellites", "SatCount")
                    && satellites > 0)
                {
                    minimumSatellites = Math.Min(minimumSatellites, (int)satellites);
                }
                if (row.TryGetDouble(out double hdop, "HDop", "HDOP", "Eph") && hdop > 0)
                    maximumHdop = Math.Max(maximumHdop, NormalizeHdop(hdop));

                double lat = NormalizeCoordinate(row.GetDouble(double.NaN, "Lat", "Latitude"));
                double lon = NormalizeCoordinate(row.GetDouble(double.NaN, "Lng", "Lon", "Longitude"));
                double alt = NormalizeAltitude(row.GetDouble(double.NaN, "Alt", "RelAlt"));
                if (!double.IsNaN(alt))
                {
                    minimumAltitude = Math.Min(minimumAltitude, alt);
                    maximumAltitude = Math.Max(maximumAltitude, alt);
                }
                if (double.IsNaN(lat) || double.IsNaN(lon) || Math.Abs(lat) > 90 || Math.Abs(lon) > 180)
                    continue;

                if (havePrevious)
                {
                    double step = HaversineMeters(previousLat, previousLon, lat, lon);
                    if (step < 500) distance += step;
                    if (!double.IsNaN(alt) && !double.IsNaN(previousAlt))
                    {
                        double delta = alt - previousAlt;
                        if (delta > 0) climb += delta;
                        else descent -= delta;
                    }
                }
                previousLat = lat;
                previousLon = lon;
                previousAlt = alt;
                havePrevious = true;
            }
            if (rowCount == 0) return;

            double altitudeSpan = minimumAltitude < double.MaxValue && maximumAltitude > double.MinValue
                ? maximumAltitude - minimumAltitude
                : 0;
            summary.Metrics.Add(new LogMetric
            {
                Label = "Distance travelled",
                Value = FormatDistance(distance),
                Verdict = LogVerdict.Info,
                Detail = speedCount > 0
                    ? $"Horizontal GPS path; {speedSum / speedCount:F1} m/s average ground speed, " +
                        $"{maxSpeed:F1} m/s max, " +
                        $"{altitudeSpan:F1} m altitude span, {climb:F0}/{descent:F0} m climb/descent"
                    : $"Horizontal GPS path; {climb:F0} m climb, {descent:F0} m descent",
            });

            if (minimumSatellites == int.MaxValue) minimumSatellites = 0;
            var gpsIssues = IssueRules.Evaluate(new IssueSnapshot
            {
                TimeSeconds = lastTime,
                Satellites = minimumSatellites,
                Hdop = maximumHdop,
            }, thresholds);
            summary.Anomalies.AddRange(gpsIssues.FindAll(i => i.Id.StartsWith("gps-", StringComparison.Ordinal)));
            summary.Metrics.Add(new LogMetric
            {
                Label = "GPS health",
                Value = minimumSatellites > 0 ? $"{minimumSatellites} sats min" : "No satellite data",
                Verdict = VerdictFor(gpsIssues),
                Detail = maximumHdop > 0 ? $"{maximumHdop:F1} max HDOP" : "No HDOP data",
            });
        }

        private static void AnalyzeVibration(
            IFlightLogData log,
            LogSummary summary,
            LogAnalysisThresholds thresholds)
        {
            int rowCount = 0;
            int highSamples = 0;
            double peakX = 0;
            double peakY = 0;
            double peakZ = 0;
            double lastTime = 0;
            var clip0 = new CounterRange();
            var clip1 = new CounterRange();
            var clip2 = new CounterRange();
            foreach (LogRecord row in Rows(log, "VIBE"))
            {
                rowCount++;
                lastTime = row.TimeSeconds;
                double x = Math.Abs(row.GetDouble(0, "VibeX", "X"));
                double y = Math.Abs(row.GetDouble(0, "VibeY", "Y"));
                double z = Math.Abs(row.GetDouble(0, "VibeZ", "Z"));
                peakX = Math.Max(peakX, x);
                peakY = Math.Max(peakY, y);
                peakZ = Math.Max(peakZ, z);
                if (Math.Max(x, Math.Max(y, z)) >= thresholds.VibrationWarning) highSamples++;
                clip0.Add(row.GetDouble(0, "Clip0"));
                clip1.Add(row.GetDouble(0, "Clip1"));
                clip2.Add(row.GetDouble(0, "Clip2"));
            }
            if (rowCount == 0) return;

            double clips = clip0.Delta + clip1.Delta + clip2.Delta;
            var issues = IssueRules.Evaluate(new IssueSnapshot
            {
                TimeSeconds = lastTime,
                VibrationX = peakX,
                VibrationY = peakY,
                VibrationZ = peakZ,
                ClipCount = clips,
            }, thresholds);
            summary.Anomalies.AddRange(issues.FindAll(i => i.Id.StartsWith("vibration", StringComparison.Ordinal)));
            summary.Metrics.Add(new LogMetric
            {
                Label = "Vibration",
                Value = $"{Math.Max(peakX, Math.Max(peakY, peakZ)):F1} m/s2 peak",
                Verdict = VerdictFor(issues),
                Detail = $"X/Y/Z {peakX:F1}/{peakY:F1}/{peakZ:F1}; " +
                    $"{100d * highSamples / rowCount:F1}% high; {clips:F0} clips",
            });
        }

        private static void AnalyzeTune(
            IFlightLogData log,
            LogSummary summary,
            LogAnalysisThresholds thresholds)
        {
            double rollSquared = 0;
            double pitchSquared = 0;
            double yawSquared = 0;
            int rollCount = 0;
            int pitchCount = 0;
            int yawCount = 0;
            double lastTime = 0;
            foreach (LogRecord row in Rows(log, "ATT"))
            {
                lastTime = row.TimeSeconds;
                AccumulateError(row, "DesRoll", "Roll", false, ref rollSquared, ref rollCount);
                AccumulateError(row, "DesPitch", "Pitch", false, ref pitchSquared, ref pitchCount);
                AccumulateError(row, "DesYaw", "Yaw", true, ref yawSquared, ref yawCount);
            }

            double roll = Rms(rollSquared, rollCount);
            double pitch = Rms(pitchSquared, pitchCount);
            double yaw = Rms(yawSquared, yawCount);
            var valid = new[] { roll, pitch, yaw }.Where(v => !double.IsNaN(v)).ToList();
            if (valid.Count == 0) return;
            double rms = valid.Max();
            double score = Math.Max(0, 100 - rms * 7.5);
            var issues = IssueRules.Evaluate(new IssueSnapshot
            {
                TimeSeconds = lastTime,
                AttitudeError = rms,
            }, thresholds);
            summary.Anomalies.AddRange(issues.FindAll(i => i.Id.StartsWith("tune-", StringComparison.Ordinal)));
            summary.Metrics.Add(new LogMetric
            {
                Label = "Tune quality",
                Value = $"{score:F0}/100",
                Verdict = VerdictFor(issues),
                Detail = $"RMS roll/pitch/yaw {FormatMaybe(roll)}/{FormatMaybe(pitch)}/{FormatMaybe(yaw)} deg",
            });
        }

        private static void AnalyzeEkf(
            IFlightLogData log,
            LogSummary summary,
            LogAnalysisThresholds thresholds)
        {
            string[] fields = { "SV", "SP", "SH", "SM", "SVT", "pos_horiz_variance", "pos_vert_variance",
                "velocity_variance", "compass_variance" };
            int rowCount = 0;
            double peak = 0;
            double lastTime = 0;
            foreach (string type in PreferredTypes(log, "XKF4", "NKF4", "EKF", "XKF3", "NKF3"))
            {
                foreach (LogRecord row in Rows(log, type))
                {
                    rowCount++;
                    lastTime = row.TimeSeconds;
                    foreach (string field in fields)
                        peak = Math.Max(peak, Math.Abs(row.GetDouble(0, field)));
                }
            }
            if (rowCount == 0) return;
            var issues = IssueRules.Evaluate(new IssueSnapshot
            {
                TimeSeconds = lastTime,
                EkfVariance = peak,
            }, thresholds);
            summary.Anomalies.AddRange(issues.FindAll(i => i.Id.StartsWith("ekf-", StringComparison.Ordinal)));
            summary.Metrics.Add(new LogMetric
            {
                Label = "EKF health",
                Value = peak > 0 ? $"{peak:F2} peak" : "Available",
                Verdict = VerdictFor(issues),
                Detail = issues.Count == 0 ? "No elevated variance detected" : issues[0].Detail,
            });
        }

        private static void AnalyzeThrottle(IFlightLogData log, LogSummary summary)
        {
            int count = 0;
            double sum = 0;
            double peak = 0;
            double lastTime = 0;
            foreach (LogRecord row in Rows(log, "CTUN"))
            {
                lastTime = row.TimeSeconds;
                if (!row.TryGetDouble(out double value, "ThO", "ThrOut", "Throttle")) continue;
                double throttle = NormalizeThrottle(value);
                sum += throttle;
                count++;
                peak = Math.Max(peak, throttle);
            }
            if (count == 0) return;
            double average = sum / count;
            double peakHeadroom = Math.Max(0, 1 - peak);
            summary.Metrics.Add(new LogMetric
            {
                Label = "Throttle headroom",
                Value = $"{100 * peakHeadroom:F0}% at peak",
                Verdict = peak > 0.9 ? LogVerdict.Warning : LogVerdict.Info,
                Detail = $"{100 * average:F0}% average throttle, {100 * peak:F0}% peak throttle",
            });
            if (peak > 0.95)
            {
                summary.Anomalies.Add(new LogAnomaly
                {
                    Id = "throttle-margin",
                    TimeSeconds = lastTime,
                    Verdict = LogVerdict.Warning,
                    Title = "Low thrust margin",
                    Detail = $"Throttle reached {100 * peak:F0}%.",
                });
            }
        }

        private static void AnalyzeMessages(IFlightLogData log, LogSummary summary)
        {
            foreach (LogRecord row in Rows(log, "ERR"))
            {
                string subsystem = row.GetString("Subsys", "Subsystem");
                string code = row.GetString("ECode", "Code");
                summary.Anomalies.Add(new LogAnomaly
                {
                    Id = $"err-{subsystem}-{code}",
                    TimeSeconds = row.TimeSeconds,
                    Verdict = LogVerdict.Warning,
                    Title = "Flight controller error",
                    Detail = $"Subsystem {subsystem}, code {code}",
                });
            }

            string[] criticalWords = { "crash", "failsafe", "ekf reset", "gps glitch", "vibration compensation" };
            foreach (LogRecord row in Rows(log, "MSG"))
            {
                string text = row.GetString("Message", "Msg", "Text");
                if (string.IsNullOrWhiteSpace(text)) continue;
                string match = criticalWords.FirstOrDefault(word =>
                    text.IndexOf(word, StringComparison.OrdinalIgnoreCase) >= 0);
                if (match == null) continue;
                summary.Anomalies.Add(new LogAnomaly
                {
                    Id = "msg-" + match.Replace(" ", "-"),
                    TimeSeconds = row.TimeSeconds,
                    Verdict = match == "crash" ? LogVerdict.Critical : LogVerdict.Warning,
                    Title = "Logged event",
                    Detail = text,
                });
            }
        }

        private static void AnalyzeModes(
            IFlightLogData log,
            LogSummary summary,
            (double Start, double End) bounds)
        {
            var rows = Rows(log, "MODE").OrderBy(r => r.TimeSeconds).ToList();
            for (int i = 0; i < rows.Count; i++)
            {
                string mode = rows[i].GetString("Mode", "ModeNum", "Name");
                if (string.IsNullOrWhiteSpace(mode)) mode = "Unknown";
                summary.Modes.Add(new ModeSpan
                {
                    Mode = mode,
                    StartSeconds = rows[i].TimeSeconds,
                    EndSeconds = i + 1 < rows.Count ? rows[i + 1].TimeSeconds : bounds.End,
                });
            }
        }

        private static IEnumerable<LogRecord> Rows(IFlightLogData log, string messageType)
            => log.MessageTypes.Contains(messageType, StringComparer.OrdinalIgnoreCase)
                ? log.Records(messageType)
                : Enumerable.Empty<LogRecord>();

        private static IEnumerable<string> PreferredTypes(IFlightLogData log, params string[] types)
            => types.Where(type => log.MessageTypes.Contains(type, StringComparer.OrdinalIgnoreCase));

        private static void AccumulateError(
            LogRecord row,
            string desired,
            string actual,
            bool wrapAngle,
            ref double squaredSum,
            ref int count)
        {
            if (!row.TryGetDouble(out double desiredValue, desired)
                || !row.TryGetDouble(out double actualValue, actual))
                return;
            double error = desiredValue - actualValue;
            if (wrapAngle)
            {
                error %= 360;
                if (error > 180) error -= 360;
                if (error < -180) error += 360;
            }
            squaredSum += error * error;
            count++;
        }

        private static double Rms(double squaredSum, int count)
            => count == 0 ? double.NaN : Math.Sqrt(squaredSum / count);

        private static double HaversineMeters(double lat1, double lon1, double lat2, double lon2)
        {
            const double radius = 6371000;
            double dLat = (lat2 - lat1) * Math.PI / 180;
            double dLon = (lon2 - lon1) * Math.PI / 180;
            double a = Math.Sin(dLat / 2) * Math.Sin(dLat / 2)
                + Math.Cos(lat1 * Math.PI / 180) * Math.Cos(lat2 * Math.PI / 180)
                * Math.Sin(dLon / 2) * Math.Sin(dLon / 2);
            return radius * 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));
        }

        private static double NormalizeCoordinate(double value)
            => Math.Abs(value) > 180 ? value / 1e7 : value;

        private static double NormalizeAltitude(double value)
            => Math.Abs(value) > 10000 ? value / 100 : value;

        private static double NormalizeHdop(double value)
            => value > 100 ? value / 100 : value;

        private static double NormalizeThrottle(double value)
            => value > 1.5 ? value / 100 : Math.Max(0, value);

        private static double FirstPositive(params double[] values)
            => values.FirstOrDefault(value => value > 0);

        private static string FormatDuration(double seconds)
            => TimeSpan.FromSeconds(Math.Max(0, seconds)).ToString(seconds >= 3600 ? @"h\:mm\:ss" : @"m\:ss");

        private static string FormatDistance(double meters)
            => meters < 1000 ? $"{meters:F0} m" : $"{meters / 1000:F2} km";

        private static string FormatMaybe(double value)
            => double.IsNaN(value) ? "n/a" : value.ToString("F1");

        private static LogVerdict VerdictFor(IEnumerable<LogAnomaly> issues)
        {
            if (issues.Any(i => i.Verdict == LogVerdict.Critical)) return LogVerdict.Critical;
            if (issues.Any(i => i.Verdict == LogVerdict.Warning)) return LogVerdict.Warning;
            return LogVerdict.Good;
        }

        private sealed class CounterRange
        {
            private int _count;
            private double _minimum = double.MaxValue;
            private double _maximum = double.MinValue;

            public double Delta => _count == 0
                ? 0
                : _count == 1 ? _maximum : Math.Max(0, _maximum - _minimum);

            public void Add(double value)
            {
                _count++;
                _minimum = Math.Min(_minimum, value);
                _maximum = Math.Max(_maximum, value);
            }
        }
    }
}
