// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void CaptureNativeNull(string runtime)
    {
        runtime = Path.GetFullPath(runtime);
        string bin = Path.Combine(runtime, "bin");
        Check(File.Exists(Path.Combine(bin, "libgstreamer-1.0-0.dll")), "Capture probe needs an existing GStreamer runtime");
        string storage = MissionPlanner.Utilities.Settings.GetDataDirectory();
        Environment.SetEnvironmentVariable("GST_REGISTRY_1_0", Path.Combine(storage, "capture-registry.bin"));
        Environment.SetEnvironmentVariable("GST_PLUGIN_SYSTEM_PATH_1_0", "");
        Environment.SetEnvironmentVariable("GST_PLUGIN_PATH_1_0", Path.Combine(runtime, "lib/gstreamer-1.0"));
        Environment.SetEnvironmentVariable("GST_PLUGIN_SCANNER_1_0", Path.Combine(runtime, "libexec/gstreamer-1.0/gst-plugin-scanner.exe"));
        Environment.SetEnvironmentVariable("PATH", bin + ";" + Environment.GetEnvironmentVariable("PATH"));
        Check(SetDllDirectory(bin), "Could not select the probe's existing native runtime");
        IntPtr pipeline = IntPtr.Zero, source = IntPtr.Zero, property = IntPtr.Zero, error = IntPtr.Zero;
        try
        {
            Check(capture_gst_init_check(IntPtr.Zero, IntPtr.Zero, out error) && error == IntPtr.Zero,
                "Existing GStreamer could not initialize for the NULL-only capture probe");
            string path = @"\\?\usb#synthetic_日本語#quote"" ! fakesink name=unexpected";
            string description = VideoPipeline.Build(CaptureInput(path)).Replace("ksvideosrc ", "ksvideosrc name=capture ");
            pipeline = capture_gst_parse_launch(description, out error);
            Check(pipeline != IntPtr.Zero && error == IntPtr.Zero, "The actual capture pipeline did not parse");
            source = capture_gst_bin_get_by_name(pipeline, "capture");
            Check(source != IntPtr.Zero, "Parsed pipeline has no selected capture source");
            capture_g_object_get(source, "device-path", out property, IntPtr.Zero);
            Check(property != IntPtr.Zero, "Capture source lost its explicit device path");
            int length = 0;
            while (length < 8192 && Marshal.ReadByte(property, length) != 0) { length++; }
            Check(length < 8192, "Invalid native device path length");
            var bytes = new byte[length]; Marshal.Copy(property, bytes, 0, length);
            Check(Encoding.UTF8.GetString(bytes) == path, "Native capture source changed the UTF-8/quoted device path");
            capture_gst_element_get_state(pipeline, out int state, out _, 0);
            Check(state == 1, "Capture probe must remain in NULL; no camera may open");
            var unexpected = capture_gst_bin_get_by_name(pipeline, "unexpected");
            if (unexpected != IntPtr.Zero) { capture_gst_object_unref(unexpected); }
            Check(unexpected == IntPtr.Zero, "Device path injected another pipeline element");
            var devices = VideoCaptureDevice.Enumerate();
            Check(devices.All(device => !string.IsNullOrWhiteSpace(device.Name) &&
                VideoCaptureDevice.QuotedPath(device.Path).Length > 2), "Passive enumeration returned invalid metadata");
            Console.WriteLine("Passive Windows capture enumeration: " + devices.Length + " devices; no capture opened.");
        }
        finally
        {
            if (property != IntPtr.Zero) { capture_g_free(property); }
            if (source != IntPtr.Zero) { capture_gst_object_unref(source); }
            if (pipeline != IntPtr.Zero) { capture_gst_object_unref(pipeline); }
            if (error != IntPtr.Zero) { capture_g_error_free(error); }
        }
    }

    [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    private static extern bool SetDllDirectory(string directory);
    [DllImport("libgstreamer-1.0-0.dll", EntryPoint = "gst_parse_launch", CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr capture_gst_parse_launch([MarshalAs(UnmanagedType.LPUTF8Str)] string pipeline, out IntPtr error);
    [DllImport("libgstreamer-1.0-0.dll", EntryPoint = "gst_init_check", CallingConvention = CallingConvention.Cdecl)]
    private static extern bool capture_gst_init_check(IntPtr argc, IntPtr argv, out IntPtr error);
    [DllImport("libgstreamer-1.0-0.dll", EntryPoint = "gst_bin_get_by_name", CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr capture_gst_bin_get_by_name(IntPtr pipeline, string name);
    [DllImport("libgstreamer-1.0-0.dll", EntryPoint = "gst_element_get_state", CallingConvention = CallingConvention.Cdecl)]
    private static extern int capture_gst_element_get_state(IntPtr pipeline, out int state, out int pending, ulong timeout);
    [DllImport("libgstreamer-1.0-0.dll", EntryPoint = "gst_object_unref", CallingConvention = CallingConvention.Cdecl)]
    private static extern void capture_gst_object_unref(IntPtr value);
    [DllImport("libgobject-2.0-0.dll", EntryPoint = "g_object_get", CallingConvention = CallingConvention.Cdecl)]
    private static extern void capture_g_object_get(IntPtr value, string name, out IntPtr property, IntPtr end);
    [DllImport("libglib-2.0-0.dll", EntryPoint = "g_free", CallingConvention = CallingConvention.Cdecl)]
    private static extern void capture_g_free(IntPtr value);
    [DllImport("libglib-2.0-0.dll", EntryPoint = "g_error_free", CallingConvention = CallingConvention.Cdecl)]
    private static extern void capture_g_error_free(IntPtr value);
}
