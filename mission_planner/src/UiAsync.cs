// ============================================================
// NOMAD UI Async Guard
// ============================================================
// WinForms event handlers must be void. This helper keeps their async work
// observed so exceptions are logged instead of escaping onto the UI context.
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
