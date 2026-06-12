// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Logging Helper
// ============================================================
// Single entry point for all plugin diagnostics. Replaces scattered
// Console.WriteLine("NOMAD: ...") calls so log levels can be gated
// centrally and output redirected without touching call sites.
//
// Output goes to the console AND to a rolling file under
// %LOCALAPPDATA%\Mission Planner\plugins\NOMAD\nomad.log — Mission
// Planner does not capture plugin console output in its own log, so
// the file is the only way to diagnose issues in a Release build.
// ============================================================

using System;
using System.Diagnostics;
using System.IO;

namespace NOMAD.MissionPlanner
{
    internal static class Log
    {
        private static readonly object _fileLock = new object();
        private static string _logPath;
        private static bool _fileFailed;
        private const long MaxLogBytes = 5 * 1024 * 1024;

        public static void Info(string message) => Write(message);

        public static void Warn(string message) => Write($"WARNING — {message}");

        public static void Error(string message) => Write($"ERROR — {message}");

        [Conditional("DEBUG")]
        public static void Debug(string message) => Write(message);

        private static void Write(string message)
        {
            Console.WriteLine($"NOMAD: {message}");
            try
            {
                lock (_fileLock)
                {
                    if (_fileFailed) return;
                    if (_logPath == null)
                    {
                        var dir = Path.Combine(
                            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
                            "Mission Planner", "plugins", "NOMAD");
                        Directory.CreateDirectory(dir);
                        _logPath = Path.Combine(dir, "nomad.log");
                        // Simple rollover: keep one previous generation.
                        var info = new FileInfo(_logPath);
                        if (info.Exists && info.Length > MaxLogBytes)
                        {
                            var old = _logPath + ".old";
                            if (File.Exists(old)) File.Delete(old);
                            File.Move(_logPath, old);
                        }
                    }
                    File.AppendAllText(_logPath, $"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff} {message}{Environment.NewLine}");
                }
            }
            catch
            {
                // Never let logging take the plugin down; stop trying on failure.
                _fileFailed = true;
            }
        }
    }
}
