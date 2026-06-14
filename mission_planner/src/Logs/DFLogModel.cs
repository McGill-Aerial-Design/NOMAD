// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    internal sealed class DFLogModel : IFlightLogData
    {
        private readonly object _sync = new object();
        private readonly DFLogBuffer _buffer;
        private readonly Dictionary<string, string> _typeNames;
        private bool _disposed;

        public DFLogModel(string path)
        {
            if (string.IsNullOrWhiteSpace(path)) throw new ArgumentException("Log path is required.", nameof(path));
            if (!File.Exists(path)) throw new FileNotFoundException("Flight log was not found.", path);

            SourcePath = path;
            _buffer = new DFLogBuffer(path);
            _typeNames = _buffer.SeenMessageTypes
                .Distinct(StringComparer.OrdinalIgnoreCase)
                .ToDictionary(type => type, type => type, StringComparer.OrdinalIgnoreCase);
        }

        public string SourcePath { get; }
        public IReadOnlyCollection<string> MessageTypes => _typeNames.Keys.OrderBy(type => type).ToList();

        public IReadOnlyList<string> Fields(string messageType)
        {
            if (!_typeNames.TryGetValue(messageType, out string actualType))
                return Array.Empty<string>();

            lock (_sync)
            {
                ThrowIfDisposed();
                foreach (var definition in _buffer.FMT.Values)
                {
                    if (!string.Equals(definition.Item2, actualType, StringComparison.OrdinalIgnoreCase))
                        continue;
                    return (definition.Item4 ?? "")
                        .Split(new[] { ',' }, StringSplitOptions.RemoveEmptyEntries)
                        .Select(field => field.Trim())
                        .Where(field => field.Length > 0)
                        .ToList();
                }
            }
            return Array.Empty<string>();
        }

        public IEnumerable<LogRecord> Records(string messageType)
        {
            if (!_typeNames.TryGetValue(messageType, out string actualType))
                return Enumerable.Empty<LogRecord>();
            return EnumerateRecords(actualType);
        }

        private IEnumerable<LogRecord> EnumerateRecords(string actualType)
        {
            lock (_sync)
            {
                ThrowIfDisposed();
                IReadOnlyList<string> fields = Fields(actualType);
                foreach (DFLog.DFItem item in _buffer.GetEnumeratorType(actualType))
                {
                    var values = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
                    foreach (string field in fields)
                    {
                        try
                        {
                            values[field] = item[field] ?? "";
                        }
                        catch
                        {
                            object raw = null;
                            try { raw = item.GetRaw(field); } catch { }
                            values[field] = Convert.ToString(raw, CultureInfo.InvariantCulture) ?? "";
                        }
                    }
                    yield return new LogRecord(actualType, item.timems / 1000d, values);
                }
            }
        }

        public string Unit(string messageType, string field)
        {
            lock (_sync)
            {
                ThrowIfDisposed();
                try
                {
                    var unit = _buffer.GetUnit(messageType, field);
                    return unit?.Item1 ?? "";
                }
                catch
                {
                    return "";
                }
            }
        }

        public double Parameter(string name, double fallback = 0)
        {
            foreach (LogRecord row in Records("PARM"))
            {
                string parameterName = row.GetString("Name", "Param", "Parameter");
                if (!string.Equals(parameterName, name, StringComparison.OrdinalIgnoreCase))
                    continue;
                return row.GetDouble(fallback, "Value", "Val");
            }
            return fallback;
        }

        public void Dispose()
        {
            lock (_sync)
            {
                if (_disposed) return;
                _disposed = true;
                _buffer.Dispose();
            }
        }

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(DFLogModel));
        }
    }
}
