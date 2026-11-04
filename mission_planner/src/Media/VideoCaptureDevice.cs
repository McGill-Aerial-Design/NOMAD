// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;

namespace NOMAD.MissionPlanner
{
    internal sealed class VideoCaptureDevice
    {
        public string Name { get; }
        internal string Path { get; }
        internal VideoCaptureDevice(string name, string path) { Name = name; Path = path; }

        internal static string QuotedPath(string path)
        {
            if (string.IsNullOrWhiteSpace(path) || path.Length > 1024 || path.Any(char.IsControl) ||
                !path.StartsWith(@"\\?\", StringComparison.Ordinal))
            { throw new FormatException("Choose a local camera or capture card device."); }
            return "\"" + path.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";
        }

        // SetupAPI reads interface metadata without opening cameras to probe their capture formats.
        internal static VideoCaptureDevice[] Enumerate()
        {
            var video = new Guid("6994AD05-93EF-11D0-A3CC-00A0C9223196");
            var capture = new Guid("65E8773D-8F56-11D0-A3B9-00A0C9223196");
            var set = SetupDiGetClassDevs(ref video, null, IntPtr.Zero, 0x12);
            if (set == new IntPtr(-1)) { throw new Win32Exception(); }
            try
            {
                var devices = new List<VideoCaptureDevice>();
                for (uint index = 0; ; index++)
                {
                    var data = new DeviceInterface { Size = Marshal.SizeOf(typeof(DeviceInterface)) };
                    if (!SetupDiEnumDeviceInterfaces(set, IntPtr.Zero, ref video, index, ref data))
                    {
                        if (Marshal.GetLastWin32Error() == 259) { break; }
                        throw new Win32Exception();
                    }
                    var alias = new DeviceInterface { Size = data.Size };
                    if (!SetupDiGetDeviceInterfaceAlias(set, ref data, ref capture, ref alias))
                    {
                        int error = Marshal.GetLastWin32Error();
                        if (error == unchecked((int)0xE0000225)) { continue; } // ERROR_NO_SUCH_DEVICE_INTERFACE
                        throw new Win32Exception(error);
                    }
                    var info = new DeviceInfo { Size = Marshal.SizeOf(typeof(DeviceInfo)) };
                    SetupDiGetDeviceInterfaceDetail(set, ref data, IntPtr.Zero, 0, out int required, ref info);
                    if (Marshal.GetLastWin32Error() != 122) { throw new Win32Exception(); }
                    if (required < 8 || required > 65536)
                    { throw new InvalidOperationException("Windows returned an invalid capture device path size."); }
                    var detail = Marshal.AllocHGlobal(required);
                    try
                    {
                        Marshal.WriteInt32(detail, IntPtr.Size == 8 ? 8 : 6);
                        if (!SetupDiGetDeviceInterfaceDetail(set, ref data, detail, required, out _, ref info))
                        { throw new Win32Exception(); }
                        // The Unicode path starts after the DWORD on both x86 and x64.
                        string path = Marshal.PtrToStringUni(IntPtr.Add(detail, 4));
                        QuotedPath(path);
                        string name = ReadName(set, ref info, 12) ?? ReadName(set, ref info, 0) ?? "Video capture device";
                        devices.Add(new VideoCaptureDevice(name, path));
                    }
                    finally { Marshal.FreeHGlobal(detail); }
                }
                return devices.OrderBy(device => device.Name, StringComparer.CurrentCultureIgnoreCase)
                    .ThenBy(device => device.Path, StringComparer.Ordinal).ToArray();
            }
            finally { SetupDiDestroyDeviceInfoList(set); }
        }

        private static string ReadName(IntPtr set, ref DeviceInfo info, uint property)
        {
            var bytes = new byte[2048];
            if (!SetupDiGetDeviceRegistryProperty(set, ref info, property, out uint type, bytes, bytes.Length, out _) ||
                type != 1) { return null; }
            string value = Encoding.Unicode.GetString(bytes).TrimEnd('\0').Trim();
            return value.Length == 0 ? null : value;
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct DeviceInterface
        {
            internal int Size;
            internal Guid Class;
            internal int Flags;
            internal UIntPtr Reserved;
        }
        [StructLayout(LayoutKind.Sequential)]
        private struct DeviceInfo
        {
            internal int Size;
            internal Guid Class;
            internal uint Instance;
            internal UIntPtr Reserved;
        }
        [DllImport("setupapi.dll", CharSet = CharSet.Unicode, SetLastError = true, EntryPoint = "SetupDiGetClassDevsW")]
        private static extern IntPtr SetupDiGetClassDevs(ref Guid category, string enumerator, IntPtr parent, uint flags);
        [DllImport("setupapi.dll", SetLastError = true)]
        private static extern bool SetupDiEnumDeviceInterfaces(IntPtr set, IntPtr info, ref Guid category,
            uint index, ref DeviceInterface data);
        [DllImport("setupapi.dll", SetLastError = true)]
        private static extern bool SetupDiGetDeviceInterfaceAlias(IntPtr set, ref DeviceInterface data,
            ref Guid category, ref DeviceInterface alias);
        [DllImport("setupapi.dll", CharSet = CharSet.Unicode, SetLastError = true, EntryPoint = "SetupDiGetDeviceInterfaceDetailW")]
        private static extern bool SetupDiGetDeviceInterfaceDetail(IntPtr set, ref DeviceInterface data,
            IntPtr detail, int size, out int required, ref DeviceInfo info);
        [DllImport("setupapi.dll", CharSet = CharSet.Unicode, SetLastError = true, EntryPoint = "SetupDiGetDeviceRegistryPropertyW")]
        private static extern bool SetupDiGetDeviceRegistryProperty(IntPtr set, ref DeviceInfo info,
            uint property, out uint type, byte[] bytes, int size, out int required);
        [DllImport("setupapi.dll", SetLastError = true)]
        private static extern bool SetupDiDestroyDeviceInfoList(IntPtr set);
    }
}
