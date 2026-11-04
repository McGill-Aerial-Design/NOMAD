// SPDX-License-Identifier: Apache-2.0
using System;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
namespace NOMAD.MissionPlanner
{
    internal static class GStreamerNative
    {
        internal const ulong GST_SECOND = 1000000000;
        internal enum GstState { GST_STATE_NULL = 1, GST_STATE_PLAYING = 4 }
        internal enum GstStateChangeReturn { GST_STATE_CHANGE_FAILURE = 0, Success = 1, Async = 2, NoPreroll = 3 }
        internal enum GstMessageType { GST_MESSAGE_ERROR = 2 }
        internal enum GstMapFlags { GST_MAP_READ = 1 }
        internal static class WinNativeMethods { internal const string lib = "libgstreamer-1.0-0.dll"; }
        private static readonly object Gate = new object();
        private static string _bin;
        internal static void Configure(string configured)
        {
            lock (Gate)
            {
                if (_bin != null) { return; }
                string data = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.CommonApplicationData),
                    "Mission Planner", "gstreamer", "1.0", "x86_64");
                string bin = null;
                var candidates = new[] { configured, Environment.GetEnvironmentVariable("GSTREAMER_1_0_ROOT_MSVC_X86_64"),
                    Environment.GetEnvironmentVariable("GSTREAMER_1_0_ROOT_X86_64"), data,
                    @"C:\gstreamer\1.0\msvc_x86_64", @"C:\gstreamer\1.0\x86_64" };
                foreach (string candidate in candidates.Where(value => !string.IsNullOrWhiteSpace(value)))
                {
                    string path = Path.GetFullPath(candidate);
                    if (File.Exists(path)) { path = Path.GetDirectoryName(path); }
                    if (!File.Exists(Path.Combine(path, WinNativeMethods.lib))) { path = Path.Combine(path, "bin"); }
                    if (File.Exists(Path.Combine(path, WinNativeMethods.lib))) { bin = path; break; }
                }
                if (bin == null) { throw new IOException("GStreamer is unavailable."); }
                string prefix = Path.GetDirectoryName(bin);
                Environment.SetEnvironmentVariable("PATH", bin + ";" + Environment.GetEnvironmentVariable("PATH"));
                Environment.SetEnvironmentVariable("GST_PLUGIN_PATH", Path.Combine(prefix, "lib", "gstreamer-1.0"));
                Environment.SetEnvironmentVariable("GST_PLUGIN_SYSTEM_PATH", Path.Combine(prefix, "lib", "gstreamer-1.0"));
                Environment.SetEnvironmentVariable("GST_PLUGIN_SCANNER",
                    Path.Combine(prefix, "libexec", "gstreamer-1.0", "gst-plugin-scanner.exe"));
                if (!SetDefaultDllDirectories(0x1000) || AddDllDirectory(bin) == IntPtr.Zero ||
                    LoadLibraryEx(Path.Combine(bin, WinNativeMethods.lib), IntPtr.Zero, 0x100 | 0x1000) == IntPtr.Zero ||
                    LoadLibraryEx(Path.Combine(bin, "libgstapp-1.0-0.dll"), IntPtr.Zero, 0x100 | 0x1000) == IntPtr.Zero)
                { throw new IOException("GStreamer native runtime could not load."); }
                bool initialized = NativeMethods.gst_init_check(IntPtr.Zero, IntPtr.Zero, out IntPtr error);
                if (error != IntPtr.Zero) { g_error_free(error); throw new IOException("GStreamer initialization failed."); }
                if (!initialized) { throw new IOException("GStreamer initialization failed."); }
                _bin = bin;
            }
        }
        [DllImport("libglib-2.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void g_error_free(IntPtr error);
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern IntPtr AddDllDirectory(string directory);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool SetDefaultDllDirectories(uint flags);
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern IntPtr LoadLibraryEx(string path, IntPtr file, uint flags);
        internal static class NativeMethods
        {
            internal static void gst_sample_unref(IntPtr sample) => gst_mini_object_unref(sample);
            [DllImport("libgstreamer-1.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
            internal static extern bool gst_init_check(IntPtr argc, IntPtr argv, out IntPtr error);
            [DllImport("libgstreamer-1.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
            internal static extern IntPtr gst_bin_get_by_name(IntPtr bin, [MarshalAs(UnmanagedType.LPUTF8Str)] string name);
            [DllImport("libgstreamer-1.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
            internal static extern IntPtr gst_element_get_bus(IntPtr element);
            [DllImport("libgstreamer-1.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
            internal static extern GstStateChangeReturn gst_element_set_state(IntPtr element, GstState state);
            [DllImport("libgstreamer-1.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
            internal static extern IntPtr gst_bus_timed_pop_filtered(IntPtr bus, ulong timeout, int types);
            [DllImport("libgstreamer-1.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
            internal static extern void gst_mini_object_unref(IntPtr value);
            [DllImport("libgstreamer-1.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
            internal static extern IntPtr gst_sample_get_buffer(IntPtr sample);
            [DllImport("libgstreamer-1.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
            internal static extern IntPtr gst_sample_get_caps(IntPtr sample);
            [DllImport("libgstreamer-1.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
            internal static extern IntPtr gst_caps_get_structure(IntPtr caps, uint index);
            [DllImport("libgstreamer-1.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
            internal static extern bool gst_structure_get_int(IntPtr structure, [MarshalAs(UnmanagedType.LPUTF8Str)] string name, out int value);
            [DllImport("libgstapp-1.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
            internal static extern void gst_app_sink_set_drop(IntPtr sink, bool drop);
            [DllImport("libgstapp-1.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
            internal static extern void gst_app_sink_set_max_buffers(IntPtr sink, uint count);
            [DllImport("libgstapp-1.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
            internal static extern bool gst_app_sink_is_eos(IntPtr sink);
            [DllImport("libgstapp-1.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
            internal static extern IntPtr gst_app_sink_try_pull_sample(IntPtr sink, ulong timeout);
        }
    }
}
