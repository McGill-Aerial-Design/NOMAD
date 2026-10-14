// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Runtime.InteropServices;
using System.Threading;
using System.Text;
using System.Windows.Forms;

// Independent native-dialog contract probe. It never constructs MP or opens operator configuration.
public static class NativeFenceDialogProbe
{
    private delegate IntPtr Hook(int code, IntPtr window, IntPtr data);
    [DllImport("user32.dll")] private static extern IntPtr SetWindowsHookEx(int kind, Hook hook, IntPtr module, uint thread);
    [DllImport("user32.dll")] private static extern bool UnhookWindowsHookEx(IntPtr hook);
    [DllImport("user32.dll")] private static extern IntPtr CallNextHookEx(IntPtr hook, int code, IntPtr window, IntPtr data);
    [DllImport("user32.dll")] private static extern bool PostMessage(IntPtr window, uint message, IntPtr wparam, IntPtr lparam);
    [DllImport("user32.dll")] private static extern IntPtr GetDlgItem(IntPtr window, int item);
    [DllImport("user32.dll", CharSet = CharSet.Unicode)] private static extern int GetWindowText(
        IntPtr window, StringBuilder text, int count);
    [DllImport("kernel32.dll")] private static extern uint GetCurrentThreadId();

    public static void Run()
    {
        Console.WriteLine("Native modal framework: " + typeof(MessageBox).Assembly.Location + "; CLR " + Environment.Version);
        foreach (string action in new[] { "cancel", "escape", "close" })
        {
            DialogResult result = DialogResult.None;
            Exception failure = null;
            IntPtr ownedWindow = IntPtr.Zero;
            var thread = new Thread(() =>
            {
                Hook callback = (code, window, data) =>
                {
                    var caption = new StringBuilder(128);
                    if (code == 5 && GetWindowText(window, caption, caption.Capacity) > 0 &&
                        caption.ToString() == "NOMAD choice test")
                    {
                        ownedWindow = window;
                        if (GetDlgItem(window, 6) == IntPtr.Zero || GetDlgItem(window, 7) == IntPtr.Zero ||
                            GetDlgItem(window, 2) == IntPtr.Zero)
                        {
                            failure = new Exception("Native choice lost Yes, No or Cancel.");
                        }
                        if (action == "escape")
                        {
                            PostMessage(window, 0x100, new IntPtr(27), IntPtr.Zero);
                            PostMessage(window, 0x101, new IntPtr(27), IntPtr.Zero);
                        }
                        else { PostMessage(window, action == "close" ? 0x10u : 0x111u, new IntPtr(2), IntPtr.Zero); }
                    }
                    return CallNextHookEx(IntPtr.Zero, code, window, data);
                };
                using (var owner = new Form())
                using (var control = new UserControl())
                {
                    owner.Controls.Add(control);
                    var ownerHandle = control.Handle;
                    IntPtr hook = SetWindowsHookEx(5, callback, IntPtr.Zero, GetCurrentThreadId());
                    try
                    {
                        if (hook == IntPtr.Zero) { throw new Exception("Native dialog observation hook unavailable."); }
                        result = MessageBox.Show(control, "Isolated cached-import cancellation qualification.", "NOMAD choice test",
                            MessageBoxButtons.YesNoCancel, MessageBoxIcon.Question, MessageBoxDefaultButton.Button3);
                    }
                    catch (Exception error) { failure = error; }
                    finally { UnhookWindowsHookEx(hook); GC.KeepAlive(callback); }
                }
            });
            thread.IsBackground = true;
            thread.SetApartmentState(ApartmentState.STA);
            thread.Start();
            if (!thread.Join(3000))
            {
                if (ownedWindow != IntPtr.Zero) { PostMessage(ownedWindow, 0x10, IntPtr.Zero, IntPtr.Zero); }
                thread.Join(1000);
                throw new Exception("Native cancellation probe exceeded its deadline: " + action);
            }
            if (failure != null) { throw failure; }
            if (result != DialogResult.Cancel) { throw new Exception(action + " returned " + result); }
            Console.WriteLine("PASS native WinForms YesNoCancel: " + action);
        }
    }
}
