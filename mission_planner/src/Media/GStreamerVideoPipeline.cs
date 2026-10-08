// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Threading;
using MissionPlanner.Utilities;
using Gst = MissionPlanner.Utilities.GStreamer;
using Native = MissionPlanner.Utilities.GStreamer.NativeMethods;

namespace NOMAD.MissionPlanner
{
    // All native calls and handles are confined to the VideoSession worker.
    internal sealed class GStreamerVideoPipeline : IVideoPipeline
    {
        private IntPtr _pipeline;
        private IntPtr _sink;
        private IntPtr _bus;

        public void Start(string pipeline, CancellationToken cancellation)
        {
            cancellation.ThrowIfCancellationRequested();
            GStreamer.GstLaunch = GStreamer.LookForGstreamer();
            if (!GStreamer.GstLaunchExists)
            {
                throw new InvalidOperationException("GStreamer is not available in Mission Planner");
            }
            cancellation.ThrowIfCancellationRequested();
            bool initialized = Native.gst_init_check(IntPtr.Zero, IntPtr.Zero, out var error);
            CheckError(error);
            if (!initialized)
            {
                throw new InvalidOperationException("GStreamer could not initialize");
            }
            _pipeline = Native.gst_parse_launch(pipeline, out error);
            CheckError(error);
            if (_pipeline == IntPtr.Zero)
            {
                throw new InvalidOperationException("Video pipeline is empty");
            }
            cancellation.ThrowIfCancellationRequested();
            _sink = Native.gst_bin_get_by_name(_pipeline, "outsink");
            _bus = Native.gst_element_get_bus(_pipeline);
            if (_sink == IntPtr.Zero || _bus == IntPtr.Zero)
            {
                throw new InvalidOperationException("Video pipeline has no appsink or bus");
            }
            Native.gst_app_sink_set_drop(_sink, true);
            Native.gst_app_sink_set_max_buffers(_sink, 1);
            cancellation.ThrowIfCancellationRequested();
            if (Native.gst_element_set_state(_pipeline, Gst.GstState.GST_STATE_PLAYING) ==
                Gst.GstStateChangeReturn.GST_STATE_CHANGE_FAILURE)
            {
                throw new InvalidOperationException("Video pipeline could not start");
            }
        }

        public Bitmap ReadFrame(CancellationToken cancellation)
        {
            while (!cancellation.IsCancellationRequested)
            {
                CheckBusError();
                if (Native.gst_app_sink_is_eos(_sink))
                {
                    return null;
                }
                // A bounded native read makes cancellation observable without sleeps or UI callbacks.
                var sample = Native.gst_app_sink_try_pull_sample(_sink, Gst.GST_SECOND / 10);
                if (sample == IntPtr.Zero)
                {
                    continue;
                }
                try
                {
                    cancellation.ThrowIfCancellationRequested();
                    return CopyFrame(sample);
                }
                finally
                {
                    Native.gst_sample_unref(sample);
                }
            }
            cancellation.ThrowIfCancellationRequested();
            return null;
        }

        private void CheckBusError()
        {
            var message = Native.gst_bus_timed_pop_filtered(_bus, 0, (int)Gst.GstMessageType.GST_MESSAGE_ERROR);
            if (message == IntPtr.Zero)
            {
                return;
            }
            try
            {
                throw new InvalidOperationException("Video pipeline reported a stream error");
            }
            finally
            {
                Native.gst_mini_object_unref(message);
            }
        }

        private static Bitmap CopyFrame(IntPtr sample)
        {
            var size = GetFrameSize(sample);
            var buffer = Native.gst_sample_get_buffer(sample);
            if (!gst_buffer_map(buffer, out var map, Gst.GstMapFlags.GST_MAP_READ))
            {
                throw new InvalidOperationException("Could not map video frame");
            }
            try
            {
                return CopyMappedFrame(size, map);
            }
            finally
            {
                gst_buffer_unmap(buffer, ref map);
            }
        }

        private static Size GetFrameSize(IntPtr sample)
        {
            var caps = Native.gst_sample_get_caps(sample);
            if (caps == IntPtr.Zero)
            {
                throw new InvalidOperationException("Video frame has no format");
            }
            var structure = Native.gst_caps_get_structure(caps, 0);
            if (structure == IntPtr.Zero)
            {
                throw new InvalidOperationException("Video frame has no format structure");
            }
            Native.gst_structure_get_int(structure, "width", out var width);
            Native.gst_structure_get_int(structure, "height", out var height);
            if (width <= 0 || height <= 0)
            {
                throw new InvalidOperationException("Invalid video frame size");
            }
            return new Size(width, height);
        }

        internal static Bitmap CopyMappedFrame(Size size, VideoBufferMap map)
        {
            int stride = checked((size.Width * 4 + 3) & ~3);
            if (map.Size.ToUInt64() < checked((ulong)stride * (ulong)size.Height))
            {
                throw new InvalidOperationException("Truncated video frame");
            }
            var frame = new Bitmap(size.Width, size.Height, PixelFormat.Format32bppArgb);
            try
            {
                CopyPixels(frame, map.Data, stride);
                return frame;
            }
            catch
            {
                frame.Dispose();
                throw;
            }
        }

        private static unsafe void CopyPixels(Bitmap frame, IntPtr source, int sourceStride)
        {
            var pixels = frame.LockBits(new Rectangle(0, 0, frame.Width, frame.Height),
                ImageLockMode.WriteOnly, PixelFormat.Format32bppArgb);
            try
            {
                for (int row = 0; row < frame.Height; ++row)
                {
                    Buffer.MemoryCopy((byte*)source + row * sourceStride,
                        (byte*)pixels.Scan0 + row * pixels.Stride, pixels.Stride, sourceStride);
                }
            }
            finally
            {
                frame.UnlockBits(pixels);
            }
        }

        private static void CheckError(IntPtr error)
        {
            if (error == IntPtr.Zero)
            {
                return;
            }
            try
            {
                throw new InvalidOperationException(Marshal.PtrToStructure<Gst.GError>(error).message);
            }
            finally
            {
                g_error_free(error);
            }
        }

        public void Dispose()
        {
            try
            {
                if (_pipeline != IntPtr.Zero)
                {
                    Native.gst_element_set_state(_pipeline, Gst.GstState.GST_STATE_NULL);
                }
            }
            finally
            {
                try
                {
                    ReleaseObject(ref _bus);
                }
                finally
                {
                    try
                    {
                        ReleaseObject(ref _sink);
                    }
                    finally
                    {
                        ReleaseObject(ref _pipeline);
                    }
                }
            }
        }

        private static void ReleaseObject(ref IntPtr resource)
        {
            var value = resource;
            resource = IntPtr.Zero;
            if (value != IntPtr.Zero)
            {
                gst_object_unref(value);
            }
        }

        [DllImport(Gst.WinNativeMethods.lib, CallingConvention = CallingConvention.Cdecl)]
        private static extern void gst_object_unref(IntPtr resource);

        [DllImport(Gst.WinNativeMethods.lib, CallingConvention = CallingConvention.Cdecl)]
        private static extern bool gst_buffer_map(IntPtr buffer, out VideoBufferMap map, Gst.GstMapFlags flags);

        [DllImport(Gst.WinNativeMethods.lib, CallingConvention = CallingConvention.Cdecl)]
        private static extern void gst_buffer_unmap(IntPtr buffer, ref VideoBufferMap map);

        [DllImport("libglib-2.0-0.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void g_error_free(IntPtr error);
    }
}
