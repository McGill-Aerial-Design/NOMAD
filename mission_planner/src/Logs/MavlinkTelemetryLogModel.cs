// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;

namespace NOMAD.MissionPlanner
{
    internal sealed class MavlinkTelemetryLogModel : IFlightLogData
    {
        private readonly Dictionary<string, List<LogRecord>> _records =
            new Dictionary<string, List<LogRecord>>(StringComparer.OrdinalIgnoreCase);
        private readonly Dictionary<string, HashSet<string>> _fields =
            new Dictionary<string, HashSet<string>>(StringComparer.OrdinalIgnoreCase);
        private bool? _armed;
        private double? _lastMode;
        private DateTime? _firstTimestamp;
        private long _packetIndex;

        public MavlinkTelemetryLogModel(string path)
        {
            if (string.IsNullOrWhiteSpace(path)) throw new ArgumentException("Log path is required.", nameof(path));
            if (!File.Exists(path)) throw new FileNotFoundException("Telemetry log was not found.", path);
            SourcePath = path;
            Read(path);
        }

        public string SourcePath { get; }
        public IReadOnlyCollection<string> MessageTypes => _records.Keys.OrderBy(type => type).ToList();

        public IReadOnlyList<string> Fields(string messageType)
            => _fields.TryGetValue(messageType, out var fields)
                ? fields.OrderBy(field => field).ToList()
                : Array.Empty<string>();

        public IEnumerable<LogRecord> Records(string messageType)
            => _records.TryGetValue(messageType, out var rows) ? rows : Enumerable.Empty<LogRecord>();

        public string Unit(string messageType, string field)
        {
            if (field.IndexOf("Volt", StringComparison.OrdinalIgnoreCase) >= 0) return "V";
            if (field.IndexOf("Curr", StringComparison.OrdinalIgnoreCase) >= 0) return "A";
            if (field.IndexOf("Vibe", StringComparison.OrdinalIgnoreCase) >= 0) return "m/s2";
            if (field.IndexOf("Alt", StringComparison.OrdinalIgnoreCase) >= 0) return "m";
            if (field.IndexOf("Spd", StringComparison.OrdinalIgnoreCase) >= 0) return "m/s";
            if (field.IndexOf("Roll", StringComparison.OrdinalIgnoreCase) >= 0
                || field.IndexOf("Pitch", StringComparison.OrdinalIgnoreCase) >= 0
                || field.IndexOf("Yaw", StringComparison.OrdinalIgnoreCase) >= 0)
                return "deg";
            return "";
        }

        public double Parameter(string name, double fallback = 0) => fallback;
        public void Dispose() { }

        private void Read(string path)
        {
            var parser = new MAVLink.MavlinkParse(true);
            using (var stream = File.OpenRead(path))
            {
                while (stream.Position < stream.Length)
                {
                    MAVLink.MAVLinkMessage message;
                    try { message = parser.ReadPacket(stream); }
                    catch (EndOfStreamException) { break; }
                    catch (IOException) { break; }
                    if (message == null || message.data == null) continue;
                    Process(message);
                }
            }
        }

        private void Process(MAVLink.MAVLinkMessage message)
        {
            double time = RelativeSeconds(message.rxtime);
            string type = NormalizeTypeName(message.msgtypename, message.data.GetType().Name);
            var raw = ObjectFields(message.data);
            Add(type, time, raw);

            switch (type)
            {
                case "HEARTBEAT":
                    if (message.compid == 1) ProcessHeartbeat(time, raw);
                    break;
                case "GPS_RAW_INT":
                    Add("GPS", time,
                        ("Lat", Scale(Get(raw, "lat"), 1e7)),
                        ("Lng", Scale(Get(raw, "lon"), 1e7)),
                        ("Alt", Scale(Get(raw, "alt"), 1000)),
                        ("Spd", Scale(Get(raw, "vel"), 100)),
                        ("NSats", Get(raw, "satellites_visible")),
                        ("HDop", Scale(Get(raw, "eph"), 100)));
                    break;
                case "GLOBAL_POSITION_INT":
                    Add("GPS", time,
                        ("Lat", Scale(Get(raw, "lat"), 1e7)),
                        ("Lng", Scale(Get(raw, "lon"), 1e7)),
                        ("Alt", Scale(Get(raw, "relative_alt", "alt"), 1000)),
                        ("Spd", HorizontalSpeed(raw)));
                    break;
                case "VFR_HUD":
                    Add("GPS", time,
                        ("Alt", Get(raw, "alt")),
                        ("Spd", Get(raw, "groundspeed")));
                    Add("CTUN", time, ("ThO", Scale(Get(raw, "throttle"), 100)));
                    break;
                case "SYS_STATUS":
                    Add("BAT", time,
                        ("Volt", Scale(Get(raw, "voltage_battery"), 1000)),
                        ("Curr", Scale(Get(raw, "current_battery"), 100)));
                    break;
                case "VIBRATION":
                    Add("VIBE", time,
                        ("VibeX", Get(raw, "vibration_x")),
                        ("VibeY", Get(raw, "vibration_y")),
                        ("VibeZ", Get(raw, "vibration_z")),
                        ("Clip0", Get(raw, "clipping_0")),
                        ("Clip1", Get(raw, "clipping_1")),
                        ("Clip2", Get(raw, "clipping_2")));
                    break;
                case "ATTITUDE":
                    Add("ATT", time,
                        ("Roll", RadiansToDegrees(Get(raw, "roll"))),
                        ("Pitch", RadiansToDegrees(Get(raw, "pitch"))),
                        ("Yaw", RadiansToDegrees(Get(raw, "yaw"))));
                    break;
                case "EKF_STATUS_REPORT":
                    Add("EKF", time,
                        ("SV", Get(raw, "velocity_variance")),
                        ("SP", Get(raw, "pos_horiz_variance")),
                        ("SH", Get(raw, "pos_vert_variance")),
                        ("SM", Get(raw, "compass_variance")));
                    break;
                case "STATUSTEXT":
                    Add("MSG", time, ("Message", DecodeText(raw.TryGetValue("text", out string text) ? text : "")));
                    break;
            }
        }

        private void ProcessHeartbeat(double time, IReadOnlyDictionary<string, string> fields)
        {
            int baseMode = (int)Get(fields, "base_mode");
            bool armed = (baseMode & 128) != 0;
            if (!_armed.HasValue || _armed.Value != armed)
            {
                Add("EV", time, ("Id", armed ? 10 : 11));
                _armed = armed;
            }
            double mode = Get(fields, "custom_mode");
            if (!_lastMode.HasValue || Math.Abs(_lastMode.Value - mode) > double.Epsilon)
            {
                Add("MODE", time, ("Mode", mode.ToString(CultureInfo.InvariantCulture)));
                _lastMode = mode;
            }
        }

        private double RelativeSeconds(DateTime timestamp)
        {
            _packetIndex++;
            if (timestamp == default(DateTime))
                return (_packetIndex - 1) * 0.1;
            if (!_firstTimestamp.HasValue) _firstTimestamp = timestamp;
            return Math.Max(0, (timestamp - _firstTimestamp.Value).TotalSeconds);
        }

        private static Dictionary<string, string> ObjectFields(object data)
        {
            var values = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            Type type = data.GetType();
            foreach (FieldInfo field in type.GetFields(BindingFlags.Public | BindingFlags.Instance))
                values[field.Name] = FormatValue(field.GetValue(data));
            foreach (PropertyInfo property in type.GetProperties(BindingFlags.Public | BindingFlags.Instance))
            {
                if (!property.CanRead || property.GetIndexParameters().Length > 0) continue;
                try { values[property.Name] = FormatValue(property.GetValue(data)); } catch { }
            }
            return values;
        }

        private void Add(string type, double time, IReadOnlyDictionary<string, string> values)
        {
            if (!_records.TryGetValue(type, out var rows))
            {
                rows = new List<LogRecord>();
                _records[type] = rows;
                _fields[type] = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            }
            foreach (string field in values.Keys) _fields[type].Add(field);
            rows.Add(new LogRecord(type, time, values));
        }

        private void Add(string type, double time, params (string Name, object Value)[] values)
        {
            Add(type, time, values.ToDictionary(
                item => item.Name,
                item => FormatValue(item.Value),
                StringComparer.OrdinalIgnoreCase));
        }

        private static string NormalizeTypeName(string name, string fallback)
        {
            string value = string.IsNullOrWhiteSpace(name) ? fallback : name;
            value = value.Replace("mavlink_", "").Replace("_t", "");
            return value.ToUpperInvariant();
        }

        private static string FormatValue(object value)
        {
            if (value == null) return "";
            if (value is byte[] bytes) return System.Text.Encoding.ASCII.GetString(bytes).TrimEnd('\0');
            if (value is IEnumerable enumerable && !(value is string))
            {
                var items = new List<string>();
                foreach (object item in enumerable) items.Add(Convert.ToString(item, CultureInfo.InvariantCulture));
                return string.Join(",", items);
            }
            return Convert.ToString(value, CultureInfo.InvariantCulture) ?? "";
        }

        private static double Get(IReadOnlyDictionary<string, string> fields, params string[] names)
        {
            foreach (string name in names)
            {
                if (!fields.TryGetValue(name, out string value)) continue;
                if (double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out double parsed))
                    return parsed;
            }
            return 0;
        }

        private static double Scale(double value, double divisor) => divisor == 0 ? value : value / divisor;
        private static double RadiansToDegrees(double value) => value * 180 / Math.PI;

        private static double HorizontalSpeed(IReadOnlyDictionary<string, string> fields)
        {
            double vx = Get(fields, "vx") / 100;
            double vy = Get(fields, "vy") / 100;
            return Math.Sqrt(vx * vx + vy * vy);
        }

        private static string DecodeText(string text) => (text ?? "").TrimEnd('\0');
    }
}
