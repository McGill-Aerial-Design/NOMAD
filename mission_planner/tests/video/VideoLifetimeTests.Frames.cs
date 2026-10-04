// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using System.Runtime.InteropServices;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void BorrowedFrameLifetime()
    {
        Check(Marshal.SizeOf<VideoBufferMap>() == (IntPtr.Size == 8 ? 104 : 52), "Incorrect GstMapInfo ABI size");
        Check(Marshal.OffsetOf<VideoBufferMap>("Size").ToInt64() == 3 * IntPtr.Size,
            "Incorrect pointer-sized GstMapInfo size field");
        var data = Marshal.AllocHGlobal(4);
        Bitmap frame = null;
        try
        {
            Marshal.WriteInt32(data, unchecked((int)0xffff0000));
            var map = new VideoBufferMap { Data = data, Size = new UIntPtr(4) };
            frame = GStreamerVideoPipeline.CopyMappedFrame(new Size(1, 1), map);
            Marshal.WriteInt32(data, unchecked((int)0xff0000ff));
            Check(frame.GetPixel(0, 0).ToArgb() == Color.Red.ToArgb(), "Frame still borrowed native pixels");
            CheckTruncatedFrame(map);
        }
        finally
        {
            Marshal.FreeHGlobal(data);
        }
        using (frame)
        {
            Check(frame.GetPixel(0, 0).ToArgb() == Color.Red.ToArgb(), "Frame was invalid after native release");
        }
    }

    private static void CheckTruncatedFrame(VideoBufferMap map)
    {
        try
        {
            using (var invalid = GStreamerVideoPipeline.CopyMappedFrame(new Size(2, 2), map))
            {
                throw new Exception("Truncated native frame was accepted");
            }
        }
        catch (InvalidOperationException ex)
        {
            Check(ex.Message == "Truncated video frame", "Unexpected frame validation failure");
        }
    }
}
