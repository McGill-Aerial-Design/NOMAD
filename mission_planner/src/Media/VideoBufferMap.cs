// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Runtime.InteropServices;

namespace NOMAD.MissionPlanner
{
    // GstMapInfo uses pointer-sized gsize and preserves private map data until unmap.
    [StructLayout(LayoutKind.Sequential)]
    internal struct VideoBufferMap
    {
        public IntPtr Memory;
        public int Flags;
        public IntPtr Data;
        public UIntPtr Size;
        public UIntPtr MaxSize;
        public IntPtr UserData0;
        public IntPtr UserData1;
        public IntPtr UserData2;
        public IntPtr UserData3;
        public IntPtr Reserved0;
        public IntPtr Reserved1;
        public IntPtr Reserved2;
        public IntPtr Reserved3;
    }
}
