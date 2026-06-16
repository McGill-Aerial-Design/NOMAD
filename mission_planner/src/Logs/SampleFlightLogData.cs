// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    internal sealed class SampleFlightLogData : IFlightLogData
    {
        private readonly Dictionary<string, List<LogRecord>> _records =
            new Dictionary<string, List<LogRecord>>(StringComparer.OrdinalIgnoreCase);

        public SampleFlightLogData()
        {
            Add("EV", 0, ("Id", 10));
            Add("MODE", 0, ("Mode", "STABILIZE"));
            Add("MODE", 20, ("Mode", "LOITER"));
            Add("MODE", 80, ("Mode", "RTL"));
            Add("EV", 100, ("Id", 11));

            for (int second = 0; second <= 100; second += 2)
            {
                double phase = second / 100d;
                Add("BAT", second,
                    ("Volt", 25.2 - 3.0 * phase),
                    ("Curr", 8 + 5 * Math.Sin(phase * Math.PI)),
                    ("CurrTot", 700 * phase));
                Add("GPS", second,
                    ("Lat", 43.65 + second * 0.000002),
                    ("Lng", -79.38 + second * 0.000003),
                    ("Alt", 100 + 15 * Math.Sin(phase * Math.PI)),
                    ("Spd", second < 8 || second > 94 ? 0.5 : 7 + Math.Sin(phase * 8)),
                    ("NSats", second == 54 ? 6 : 13),
                    ("HDop", second == 54 ? 2.6 : 1.1));
                Add("VIBE", second,
                    ("VibeX", 12 + 3 * Math.Sin(phase * 9)),
                    ("VibeY", 14 + 4 * Math.Cos(phase * 7)),
                    ("VibeZ", second == 60 ? 42 : 17 + 4 * Math.Sin(phase * 11)),
                    ("Clip0", 0),
                    ("Clip1", 0),
                    ("Clip2", 0));
                Add("ATT", second,
                    ("DesRoll", 8 * Math.Sin(phase * 10)),
                    ("Roll", 8 * Math.Sin(phase * 10) - 1.5),
                    ("DesPitch", 5 * Math.Cos(phase * 8)),
                    ("Pitch", 5 * Math.Cos(phase * 8) + 1),
                    ("DesYaw", 90 + second),
                    ("Yaw", 89 + second));
                Add("CTUN", second, ("ThO", 0.48 + 0.1 * Math.Sin(phase * Math.PI)));
            }

            Add("MSG", 54, ("Message", "GPS glitch cleared"));
        }

        public string SourcePath => "";
        public IReadOnlyCollection<string> MessageTypes => _records.Keys.ToList();

        public IReadOnlyList<string> Fields(string messageType)
        {
            return _records.TryGetValue(messageType, out var rows) && rows.Count > 0
                ? rows.SelectMany(row => row.Fields.Keys).Distinct(StringComparer.OrdinalIgnoreCase).ToList()
                : Array.Empty<string>();
        }

        public IEnumerable<LogRecord> Records(string messageType)
            => _records.TryGetValue(messageType, out var rows) ? rows : Enumerable.Empty<LogRecord>();

        public string Unit(string messageType, string field)
        {
            if (field.IndexOf("Volt", StringComparison.OrdinalIgnoreCase) >= 0) return "V";
            if (field.IndexOf("Curr", StringComparison.OrdinalIgnoreCase) >= 0) return "A";
            if (field.IndexOf("Vibe", StringComparison.OrdinalIgnoreCase) >= 0) return "m/s2";
            if (field.IndexOf("Alt", StringComparison.OrdinalIgnoreCase) >= 0) return "m";
            if (field.IndexOf("Spd", StringComparison.OrdinalIgnoreCase) >= 0) return "m/s";
            return "";
        }

        public double Parameter(string name, double fallback = 0)
            => name == "BATT_CAPACITY" ? 5000 : fallback;

        public void Dispose()
        {
        }

        private void Add(string type, double timeSeconds, params (string Name, object Value)[] fields)
        {
            if (!_records.TryGetValue(type, out var rows))
            {
                rows = new List<LogRecord>();
                _records[type] = rows;
            }

            rows.Add(new LogRecord(type, timeSeconds, fields.ToDictionary(
                field => field.Name,
                field => Convert.ToString(field.Value, System.Globalization.CultureInfo.InvariantCulture),
                StringComparer.OrdinalIgnoreCase)));
        }
    }
}
