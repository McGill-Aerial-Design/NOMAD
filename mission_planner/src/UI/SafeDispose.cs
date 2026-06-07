// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD SafeDispose Helper
// ============================================================
// Replaces repeated try/catch/dispose blocks throughout the
// codebase with a single, consistent pattern.
// ============================================================

using System;

namespace NOMAD.MissionPlanner
{
    internal static class SafeDispose
    {
        public static void Dispose(IDisposable obj)
        {
            if (obj == null) return;
            try { obj.Dispose(); }
            catch { /* best-effort cleanup */ }
        }

        public static void StopAndDispose<T>(T obj) where T : class, IDisposable
        {
            if (obj == null) return;
            try
            {
                if (obj is System.Timers.Timer t) t.Stop();
                obj.Dispose();
            }
            catch { /* best-effort cleanup */ }
        }
    }
}
