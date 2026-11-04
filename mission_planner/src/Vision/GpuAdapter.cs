// SPDX-License-Identifier: Apache-2.0
using System;
using System.Runtime.InteropServices;

namespace NOMAD.MissionPlanner
{
    // DXGI adapter order is per-process (Windows graphics preference), so index 0 can be an integrated GPU.
    // DirectML's device id follows this EnumAdapters1 order; NOMAD selects the NVIDIA adapter explicitly.
    internal static class GpuAdapter
    {
        private const uint NvidiaVendor = 0x10DE;

        internal sealed class Selection
        {
            internal int Index;
            internal string Name;
            internal long DedicatedBytes;
        }

        internal static Selection FindNvidia()
        {
            var guid = typeof(IDXGIFactory1).GUID;
            Marshal.ThrowExceptionForHR(CreateDXGIFactory1(ref guid, out object created));
            var factory = (IDXGIFactory1)created;
            try
            {
                for (uint index = 0; ; index++)
                {
                    int result = factory.EnumAdapters1(index, out IDXGIAdapter1 adapter);
                    if (result == unchecked((int)0x887A0002)) { return null; } // DXGI_ERROR_NOT_FOUND
                    Marshal.ThrowExceptionForHR(result);
                    try
                    {
                        Marshal.ThrowExceptionForHR(adapter.GetDesc1(out AdapterDescription description));
                        bool software = (description.Flags & 2) != 0; // DXGI_ADAPTER_FLAG_SOFTWARE
                        if (description.VendorId == NvidiaVendor && !software)
                        {
                            return new Selection { Index = (int)index, Name = description.Description,
                                DedicatedBytes = (long)description.DedicatedVideoMemory };
                        }
                    }
                    finally { Marshal.ReleaseComObject(adapter); }
                }
            }
            finally { Marshal.ReleaseComObject(factory); }
        }

        [DllImport("dxgi.dll")]
        private static extern int CreateDXGIFactory1(ref Guid riid, [MarshalAs(UnmanagedType.IUnknown)] out object factory);

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        private struct AdapterDescription
        {
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)] internal string Description;
            internal uint VendorId, DeviceId, SubSysId, Revision;
            internal UIntPtr DedicatedVideoMemory, DedicatedSystemMemory, SharedSystemMemory;
            internal uint LuidLow;
            internal int LuidHigh;
            internal uint Flags;
        }

        // Vtable order: IDXGIObject (4), IDXGIFactory (5), IDXGIFactory1 (2).
        [ComImport, Guid("770aae78-f26f-4dba-a829-253c83d1b387"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
        private interface IDXGIFactory1
        {
            void SetPrivateData(); void SetPrivateDataInterface(); void GetPrivateData(); void GetParent();
            void EnumAdapters(); void MakeWindowAssociation(); void GetWindowAssociation(); void CreateSwapChain();
            void CreateSoftwareAdapter();
            [PreserveSig] int EnumAdapters1(uint index, out IDXGIAdapter1 adapter);
            [PreserveSig] bool IsCurrent();
        }

        // Vtable order: IDXGIObject (4), IDXGIAdapter (3), IDXGIAdapter1 (1).
        [ComImport, Guid("29038f61-3839-4626-91fd-086879011a05"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
        private interface IDXGIAdapter1
        {
            void SetPrivateData(); void SetPrivateDataInterface(); void GetPrivateData(); void GetParent();
            void EnumOutputs(); void GetDesc(); void CheckInterfaceSupport();
            [PreserveSig] int GetDesc1(out AdapterDescription description);
        }
    }
}
