// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Runtime.InteropServices;
using System.Security.AccessControl;
using System.Security.Principal;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    // Immutable pixels have bounded unmanaged storage; large frame arrays never enter MP's managed heap.
    internal static class MediaPixels
    {
        private const long MaximumAllocated = 512L * 1024 * 1024, MaximumPooled = 128L * 1024 * 1024;
        private static readonly object Gate = new object();
        private static readonly Dictionary<int, Stack<IntPtr>> Pool = new Dictionary<int, Stack<IntPtr>>();
        private static long _allocated, _pooled;
        private static bool _pooling = true;
        internal static long AllocatedBytes => Interlocked.Read(ref _allocated);

        private sealed class Pixels
        {
            internal readonly IntPtr Pointer;
            internal readonly int Width, Height, Bytes;
            internal readonly long Ticks;
            private int _references = 1;
            internal Pixels(int width, int height, long ticks)
            { Width = width; Height = height; Bytes = MediaWire.PixelBytes(width, height); Ticks = ticks; Pointer = Rent(Bytes); }
            internal void Retain()
            {
                int count;
                do { count = Volatile.Read(ref _references); if (count == 0) { throw new ObjectDisposedException("Pixels"); } }
                while (Interlocked.CompareExchange(ref _references, count + 1, count) != count);
            }
            internal void Release() { if (Interlocked.Decrement(ref _references) == 0) { Return(Pointer, Bytes); } }
            internal Bitmap Image()
            {
                try
                {
                    var image = new Bitmap(Width, Height, checked(Width * 4), PixelFormat.Format32bppArgb, Pointer);
                    image.Tag = this; return image;
                }
                catch { Release(); throw; }
            }
        }

        internal static void EnablePooling() { lock (Gate) { _pooling = true; } }
        internal static void ReleasePool()
        {
            lock (Gate)
            {
                _pooling = false;
                foreach (var bucket in Pool)
                { while (bucket.Value.Count != 0) { Marshal.FreeHGlobal(bucket.Value.Pop()); _allocated -= bucket.Key; } }
                Pool.Clear(); _pooled = 0;
            }
        }

        private static IntPtr Rent(int bytes)
        {
            lock (Gate)
            {
                if (Pool.TryGetValue(bytes, out var available) && available.Count != 0)
                { _pooled -= bytes; return available.Pop(); }
                if (_allocated + bytes > MaximumAllocated) { throw new InvalidOperationException("Media pixel budget exceeded."); }
                var result = Marshal.AllocHGlobal(bytes); _allocated += bytes; return result;
            }
        }

        private static void Return(IntPtr pointer, int bytes)
        {
            lock (Gate)
            {
                if (_pooling && _pooled + bytes <= MaximumPooled)
                {
                    if (!Pool.TryGetValue(bytes, out var available)) { Pool.Add(bytes, available = new Stack<IntPtr>()); }
                    available.Push(pointer); _pooled += bytes;
                }
                else { Marshal.FreeHGlobal(pointer); _allocated -= bytes; }
            }
        }

        internal static long ReceivedTicks(Bitmap image) => (image.Tag as Pixels)?.Ticks ?? 0;
        internal static Bitmap Clone(Bitmap image)
        {
            if (image.Tag is Pixels pixels) { pixels.Retain(); return pixels.Image(); }
            return (Bitmap)image.Clone();
        }

        internal static void Dispose(Bitmap image)
        {
            if (image == null) { return; }
            var pixels = image.Tag as Pixels;
            image.Tag = null; image.Dispose(); pixels?.Release();
        }

        internal static Bitmap Own(Bitmap image, long ticks)
        {
            if (image.Tag is Pixels) { return image; }
            var result = new Pixels(image.Width, image.Height, ticks);
            try
            {
                var bits = image.LockBits(new Rectangle(0, 0, image.Width, image.Height),
                    ImageLockMode.ReadOnly, PixelFormat.Format32bppArgb);
                try { CopyRows(bits.Scan0, bits.Stride, result.Pointer, checked(image.Width * 4), image.Width, image.Height); }
                finally { image.UnlockBits(bits); }
            }
            catch { result.Release(); throw; }
            image.Dispose(); return result.Image();
        }

        internal static Bitmap CopyMappedFrame(Size size, VideoBufferMap map)
        {
            int bytes = MediaWire.PixelBytes(size.Width, size.Height);
            if (map.Data == IntPtr.Zero || map.Size.ToUInt64() < (ulong)bytes)
            { throw new InvalidOperationException("Truncated video frame"); }
            var pixels = new Pixels(size.Width, size.Height, 0);
            CopyRows(map.Data, checked(size.Width * 4), pixels.Pointer, checked(size.Width * 4), size.Width, size.Height);
            return pixels.Image();
        }

        private static unsafe void CopyRows(IntPtr source, int sourceStride, IntPtr target, int targetStride,
            int width, int height)
        {
            int rowBytes = checked(width * 4);
            if (Math.Abs((long)sourceStride) < rowBytes || Math.Abs((long)targetStride) < rowBytes)
            { throw new InvalidDataException("Invalid media stride."); }
            for (int row = 0; row < height; row++)
            { Buffer.MemoryCopy((byte*)source + row * sourceStride, (byte*)target + row * targetStride, rowBytes, rowBytes); }
        }

        internal static Bitmap Read(MediaMessage message, string prefix)
        {
            int bytes = Validate(message, prefix);
            var pixels = new Pixels(message.Width, message.Height, message.Ticks);
            try
            {
                using (var map = MemoryMappedFile.OpenExisting(message.Mapping, MemoryMappedFileRights.Read))
                using (var view = map.CreateViewAccessor(0, bytes, MemoryMappedFileAccess.Read))
                { CopyView(view, pixels.Pointer, bytes, false); }
            }
            catch { pixels.Release(); throw; }
            return pixels.Image();
        }

        internal static void Write(Bitmap image, MediaMessage message, string prefix)
        {
            int bytes = Validate(message, prefix);
            if (image.Width != message.Width || image.Height != message.Height)
            { throw new InvalidDataException("Media dimensions changed."); }
            using (var map = MemoryMappedFile.OpenExisting(message.Mapping, MemoryMappedFileRights.ReadWrite))
            using (var view = map.CreateViewAccessor(0, bytes, MemoryMappedFileAccess.Write))
            {
                if (image.Tag is Pixels pixels) { CopyView(view, pixels.Pointer, bytes, true); }
                else
                {
                    var bits = image.LockBits(new Rectangle(0, 0, image.Width, image.Height),
                        ImageLockMode.ReadOnly, PixelFormat.Format32bppArgb);
                    try
                    {
                        unsafe
                        {
                            byte* pointer = null; view.SafeMemoryMappedViewHandle.AcquirePointer(ref pointer);
                            try { CopyRows(bits.Scan0, bits.Stride, (IntPtr)pointer, checked(image.Width * 4), image.Width, image.Height); }
                            finally { view.SafeMemoryMappedViewHandle.ReleasePointer(); }
                        }
                    }
                    finally { image.UnlockBits(bits); }
                }
            }
        }

        private static int Validate(MediaMessage message, string prefix)
        {
            int bytes = MediaWire.PixelBytes(message.Width, message.Height);
            if (message.Mapping == null || message.Mapping.Length > 180 ||
                !message.Mapping.StartsWith(prefix, StringComparison.Ordinal) || message.Capacity < bytes ||
                message.Capacity > MediaWire.MaximumPixels * 4)
            { throw new InvalidDataException("Invalid private media buffer."); }
            return bytes;
        }

        private static unsafe void CopyView(MemoryMappedViewAccessor view, IntPtr pixels, int bytes, bool write)
        {
            if (view.Capacity < bytes) { throw new InvalidDataException("Truncated private media buffer."); }
            byte* pointer = null; view.SafeMemoryMappedViewHandle.AcquirePointer(ref pointer);
            try
            {
                if (write) { Buffer.MemoryCopy((void*)pixels, pointer, bytes, bytes); }
                else { Buffer.MemoryCopy(pointer, (void*)pixels, bytes, bytes); }
            }
            finally { view.SafeMemoryMappedViewHandle.ReleasePointer(); }
        }

        internal static MemoryMappedFile Create(string name, int bytes)
        {
            if (bytes < 4 || bytes > MediaWire.MaximumPixels * 4) { throw new InvalidDataException("Invalid media capacity."); }
            var security = new MemoryMappedFileSecurity();
            security.SetAccessRuleProtection(true, false);
            security.AddAccessRule(new AccessRule<MemoryMappedFileRights>(
                WindowsIdentity.GetCurrent().User, MemoryMappedFileRights.FullControl, AccessControlType.Allow));
            return MemoryMappedFile.CreateNew(name, bytes, MemoryMappedFileAccess.ReadWrite,
                MemoryMappedFileOptions.None, security, System.IO.HandleInheritability.None);
        }
    }
}
