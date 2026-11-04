// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using System.Runtime.InteropServices;
using System.Threading;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void RetainedFrameLifetime()
    {
        var pixels = SolidVideoImage(Color.CornflowerBlue);
        var frame = new VideoFrame(pixels, Array.Empty<VideoDetection>(), 7, 3,
            System.Diagnostics.Stopwatch.GetTimestamp(), DateTime.UtcNow, "owned model");
        var retained = frame.Retain();
        frame.Dispose(); frame.Dispose();
        Check(ReferenceEquals(retained.Image, pixels) && retained.Sequence == 7 && retained.Generation == 3 &&
            pixels.GetPixel(0, 0).ToArgb() == Color.CornflowerBlue.ToArgb(),
            "Releasing display invalidated retained pixels or copied the envelope identity");
        bool rejected = false;
        try { frame.Retain(); } catch (ObjectDisposedException) { rejected = true; }
        Check(rejected, "A released envelope acquired another reference");
        retained.Dispose(); retained.Dispose();
        CheckDisposed(pixels);
    }

    private static void FreshReceiptsDoNotEstablishFreshCapture()
    {
        var pipeline = new FakeVideoPipeline();
        using (var frozen = new Bitmap(16, 16))
        using (var graphics = Graphics.FromImage(frozen))
        using (var session = new VideoSession(() => pipeline))
        {
            graphics.Clear(Color.DarkBlue);
            session.Start("owned frozen-pixel fixture");
            long previousSequence = 0, previousReceipt = 0;
            for (int index = 0; index < 6; index++)
            {
                pipeline.Frames.Add((Bitmap)frozen.Clone());
                using (var frame = NextVideoFrame(session))
                {
                    Check(frame.Fresh && frame.Sequence > previousSequence && frame.ReceivedTicks > previousReceipt,
                        "Fresh decoded receipts must remain distinct even when upstream pixels repeat");
                    Check(frame.Image.GetPixel(8, 8).ToArgb() == Color.DarkBlue.ToArgb(),
                        "Frozen-pixel fixture changed its content");
                    previousSequence = frame.Sequence; previousReceipt = frame.ReceivedTicks;
                }
                Thread.Sleep(35);
            }
        }
    }

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
        try { Check(frame.GetPixel(0, 0).ToArgb() == Color.Red.ToArgb(), "Frame was invalid after native release"); }
        finally { MediaPixels.Dispose(frame); }
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
