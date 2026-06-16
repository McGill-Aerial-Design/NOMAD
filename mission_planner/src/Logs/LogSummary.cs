// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    public enum LogVerdict
    {
        Good,
        Info,
        Warning,
        Critical
    }

    public sealed class LogMetric
    {
        public string Label { get; set; } = "";
        public string Value { get; set; } = "";
        public string Unit { get; set; } = "";
        public LogVerdict Verdict { get; set; } = LogVerdict.Info;
        public string Detail { get; set; } = "";
    }

    public sealed class LogAnomaly
    {
        public string Id { get; set; } = "";
        public double TimeSeconds { get; set; }
        public LogVerdict Verdict { get; set; }
        public string Title { get; set; } = "";
        public string Detail { get; set; } = "";
    }

    public sealed class ModeSpan
    {
        public string Mode { get; set; } = "";
        public double StartSeconds { get; set; }
        public double EndSeconds { get; set; }
        public double DurationSeconds => Math.Max(0, EndSeconds - StartSeconds);
    }

    public sealed class LogSummary
    {
        public string SourceName { get; set; } = "";
        public double LogDurationSeconds { get; set; }
        public double ArmedDurationSeconds { get; set; }
        public double AirborneDurationSeconds { get; set; }
        public int TakeoffCount { get; set; }
        public LogVerdict OverallVerdict { get; set; } = LogVerdict.Good;
        public List<LogMetric> Metrics { get; } = new List<LogMetric>();
        public List<LogAnomaly> Anomalies { get; } = new List<LogAnomaly>();
        public List<ModeSpan> Modes { get; } = new List<ModeSpan>();

        public string OverallText
        {
            get
            {
                int critical = Anomalies.Count(a => a.Verdict == LogVerdict.Critical);
                int warnings = Anomalies.Count(a => a.Verdict == LogVerdict.Warning);
                if (critical > 0) return $"{critical} critical issue(s), {warnings} warning(s)";
                if (warnings > 0) return $"{warnings} warning(s) detected";
                return Anomalies.Count > 0 ? $"{Anomalies.Count} advisory item(s)" : "No significant issues detected";
            }
        }
    }

    public sealed class LogRecord
    {
        private readonly IReadOnlyDictionary<string, string> _fields;

        public LogRecord(string messageType, double timeSeconds, IReadOnlyDictionary<string, string> fields)
        {
            MessageType = messageType ?? "";
            TimeSeconds = timeSeconds;
            _fields = fields ?? new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        }

        public string MessageType { get; }
        public double TimeSeconds { get; }
        public IReadOnlyDictionary<string, string> Fields => _fields;

        public string GetString(params string[] names)
        {
            foreach (string name in names)
            {
                if (name != null && _fields.TryGetValue(name, out string value))
                    return value?.Trim() ?? "";
            }
            return "";
        }

        public bool TryGetDouble(out double value, params string[] names)
        {
            foreach (string name in names)
            {
                if (name == null || !_fields.TryGetValue(name, out string text))
                    continue;

                if (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value)
                    || double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out value))
                {
                    return !double.IsNaN(value) && !double.IsInfinity(value);
                }
            }

            value = 0;
            return false;
        }

        public double GetDouble(double fallback, params string[] names)
            => TryGetDouble(out double value, names) ? value : fallback;
    }

    public interface IFlightLogData : IDisposable
    {
        string SourcePath { get; }
        IReadOnlyCollection<string> MessageTypes { get; }
        IReadOnlyList<string> Fields(string messageType);
        IEnumerable<LogRecord> Records(string messageType);
        string Unit(string messageType, string field);
        double Parameter(string name, double fallback = 0);
    }
}
