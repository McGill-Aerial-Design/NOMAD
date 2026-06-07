// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Logging Helper
// ============================================================
// Single entry point for all plugin diagnostics. Replaces scattered
// Console.WriteLine("NOMAD: ...") calls so log levels can be gated
// centrally and output redirected without touching call sites.
// ============================================================

using System;
using System.Diagnostics;

namespace NOMAD.MissionPlanner
{
    internal static class Log
    {
        public static void Info(string message)
        {
            Console.WriteLine($"NOMAD: {message}");
        }

        public static void Warn(string message)
        {
            Console.WriteLine($"NOMAD: WARNING — {message}");
        }

        public static void Error(string message)
        {
            Console.WriteLine($"NOMAD: ERROR — {message}");
        }

        [Conditional("DEBUG")]
        public static void Debug(string message)
        {
            Console.WriteLine($"NOMAD: {message}");
        }
    }
}
