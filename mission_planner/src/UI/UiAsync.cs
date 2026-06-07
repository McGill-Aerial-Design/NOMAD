// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD UI Async Guard
// ============================================================
// WinForms event handlers must be void. These helpers keep UI-thread
// marshalling observed so exceptions are logged instead of escaping
// onto the UI context.
//
//   UiAsync.Run(owner, async () => { ... }, "name") — async fire-and-forget
//   UiAsync.RunSync(owner, () => { ... }, "name")   — synchronous, does
//     the InvokeRequired check internally.
// ============================================================

using System;
using System.Diagnostics;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    internal static class UiAsync
    {
        public static void Run(Control owner, Func<Task> action, string operationName)
        {
            _ = RunObservedAsync(owner, action, operationName);
        }

        /// <summary>
        /// Synchronous UI-thread marshal.  Replaces the common pattern
        ///   if (control.InvokeRequired) { control.BeginInvoke(...); return; }
        /// with a single call that also catches disposal errors.
        /// </summary>
        public static void RunSync(Control owner, Action action, string operationName)
        {
            if (owner == null) return;
            try
            {
                if (owner.InvokeRequired)
                {
                    owner.BeginInvoke(action);
                    return;
                }
                action();
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException ex) when (owner.IsDisposed || !owner.IsHandleCreated)
            {
                Debug.WriteLine($"NOMAD UI sync ignored after dispose ({operationName}): {ex.Message}");
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"NOMAD UI sync error ({operationName}): {ex}");
            }
        }

        private static async Task RunObservedAsync(Control owner, Func<Task> action, string operationName)
        {
            try
            {
                await action().ConfigureAwait(true);
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException ex) when (owner == null || owner.IsDisposed || !owner.IsHandleCreated)
            {
                Debug.WriteLine($"NOMAD UI async ignored after dispose ({operationName}): {ex.Message}");
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"NOMAD UI async error ({operationName}): {ex}");
            }
        }
    }
}
