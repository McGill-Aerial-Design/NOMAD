// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using NOMAD.MissionPlanner;

internal static class LogAnalysisTests
{
    private static int _failures;

    private static void Main()
    {
        Run("computes armed time and endurance", ComputesFlightAndEndurance);
        Run("computes GPS distance", ComputesDistance);
        Run("reports peak throttle headroom", ReportsThrottleHeadroom);
        Run("detects vibration clips", DetectsVibration);
        Run("decodes ERR rows", DecodesErrors);
        Run("handles absent message types", HandlesMissingTypes);
        Run("shares live issue thresholds", EvaluatesIssueRules);

        if (_failures > 0)
        {
            Console.Error.WriteLine($"{_failures} log analysis test(s) failed.");
            Environment.Exit(1);
        }
        Console.WriteLine("All log analysis tests passed.");
    }

    private static void ComputesFlightAndEndurance()
    {
        using (var log = new FakeLog())
        {
            log.Param("BATT_CAPACITY", 6000);
            log.Add("EV", 10, ("Id", 10));
            log.Add("EV", 70, ("Id", 11));
            log.Add("BAT", 10, ("Volt", 25.0), ("Curr", 10.0), ("CurrTot", 0));
            log.Add("BAT", 70, ("Volt", 23.0), ("Curr", 10.0), ("CurrTot", 1000));
            LogSummary summary = LogAnalysis.Analyze(log);
            AssertNear(60, summary.ArmedDurationSeconds, 0.001, "armed time");
            LogMetric endurance = summary.Metrics.Single(metric => metric.Label == "Endurance");
            Assert(endurance.Value.Contains("36.0"), "capacity/current endurance should be 36 minutes");
        }
    }

    private static void ComputesDistance()
    {
        using (var log = new FakeLog())
        {
            log.Add("GPS", 0, ("Lat", 43.0), ("Lng", -79.0), ("Alt", 100), ("Spd", 5));
            log.Add("GPS", 10, ("Lat", 43.001), ("Lng", -79.0), ("Alt", 110), ("Spd", 6));
            LogSummary summary = LogAnalysis.Analyze(log);
            LogMetric distance = summary.Metrics.Single(metric => metric.Label == "Distance travelled");
            Assert(distance.Value.Contains("111"), "one millidegree latitude should be about 111 m");
            Assert(distance.Detail.Contains("Horizontal GPS path"), "distance metric should explain its meaning");
        }
    }

    private static void ReportsThrottleHeadroom()
    {
        using (var log = new FakeLog())
        {
            log.Add("CTUN", 0, ("ThO", 0.25));
            log.Add("CTUN", 1, ("ThO", 1.0));
            LogSummary summary = LogAnalysis.Analyze(log);
            LogMetric throttle = summary.Metrics.Single(metric => metric.Label == "Throttle headroom");
            Assert(throttle.Value == "0% at peak", "peak headroom should be based on peak throttle");
            Assert(throttle.Detail.Contains("63% average throttle"), "average throttle should be labeled once");
            Assert(throttle.Detail.Contains("100% peak throttle"), "peak throttle should be explicit");
        }
    }

    private static void DetectsVibration()
    {
        using (var log = new FakeLog())
        {
            log.Add("VIBE", 1, ("VibeX", 10), ("VibeY", 12), ("VibeZ", 65),
                ("Clip0", 1), ("Clip1", 0), ("Clip2", 0));
            LogSummary summary = LogAnalysis.Analyze(log);
            Assert(summary.Anomalies.Any(issue => issue.Id == "vibration-critical"), "critical vibration missing");
            Assert(summary.Anomalies.Any(issue => issue.Id == "vibration-clipping"), "clip issue missing");
        }
    }

    private static void DecodesErrors()
    {
        using (var log = new FakeLog())
        {
            log.Add("ERR", 12, ("Subsys", 3), ("ECode", 2));
            LogSummary summary = LogAnalysis.Analyze(log);
            Assert(summary.Anomalies.Any(issue => issue.Id == "err-3-2"), "ERR row was not decoded");
        }
    }

    private static void HandlesMissingTypes()
    {
        using (var log = new FakeLog())
        {
            log.Add("MSG", 0, ("Message", "Boot"));
            LogSummary summary = LogAnalysis.Analyze(log);
            Assert(summary != null, "summary should be returned");
            Assert(summary.Metrics.Count == 1, "flight-time fallback should still be present");
        }
    }

    private static void EvaluatesIssueRules()
    {
        var issues = IssueRules.Evaluate(new IssueSnapshot
        {
            TimeSeconds = 5,
            VibrationZ = 45,
            Hdop = 2.5,
            Satellites = 6,
        }, new LogAnalysisThresholds());
        Assert(issues.Any(issue => issue.Id == "vibration-warning"), "live vibration rule missing");
        Assert(issues.Any(issue => issue.Id == "gps-hdop-warning"), "live HDOP rule missing");
        Assert(issues.Any(issue => issue.Id == "gps-satellites"), "live satellite rule missing");
    }

    private static void Run(string name, Action test)
    {
        try
        {
            test();
            Console.WriteLine($"PASS {name}");
        }
        catch (Exception ex)
        {
            _failures++;
            Console.Error.WriteLine($"FAIL {name}: {ex.Message}");
        }
    }

    private static void Assert(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }

    private static void AssertNear(double expected, double actual, double tolerance, string message)
    {
        if (Math.Abs(expected - actual) > tolerance)
            throw new InvalidOperationException($"{message}: expected {expected}, got {actual}");
    }

    private sealed class FakeLog : IFlightLogData
    {
        private readonly Dictionary<string, List<LogRecord>> _rows =
            new Dictionary<string, List<LogRecord>>(StringComparer.OrdinalIgnoreCase);
        private readonly Dictionary<string, double> _parameters =
            new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);

        public string SourcePath => "fixture.log";
        public IReadOnlyCollection<string> MessageTypes => _rows.Keys.ToList();

        public void Add(string type, double time, params (string Name, object Value)[] fields)
        {
            if (!_rows.TryGetValue(type, out var rows))
            {
                rows = new List<LogRecord>();
                _rows[type] = rows;
            }
            rows.Add(new LogRecord(type, time, fields.ToDictionary(
                field => field.Name,
                field => Convert.ToString(field.Value, CultureInfo.InvariantCulture),
                StringComparer.OrdinalIgnoreCase)));
        }

        public void Param(string name, double value) => _parameters[name] = value;
        public IReadOnlyList<string> Fields(string messageType) => Array.Empty<string>();
        public IEnumerable<LogRecord> Records(string messageType)
            => _rows.TryGetValue(messageType, out var rows) ? rows : Enumerable.Empty<LogRecord>();
        public string Unit(string messageType, string field) => "";
        public double Parameter(string name, double fallback = 0)
            => _parameters.TryGetValue(name, out double value) ? value : fallback;
        public void Dispose() { }
    }
}
