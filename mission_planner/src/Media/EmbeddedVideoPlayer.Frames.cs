// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
extern alias MPDrawing;

// ============================================================
// NOMAD Embedded Video Player - Frame buffer partial
// ============================================================
// Double-buffered frame handling for the GStreamer callback:
// buffer allocation/recycling, the new-image handler, center
// depth polling, and pushing frames to the PictureBox.
// ============================================================

using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Net.Http;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;
using MPBitmap = MPDrawing::System.Drawing.Bitmap;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        private void EnsureFrameBuffers(int width, int height)
        {
            Bitmap[] oldBuffers = null;
            lock (_frameBufferLock)
            {
                if (_frameBuffers != null && _frameBufferWidth == width && _frameBufferHeight == height)
                    return;

                oldBuffers = _frameBuffers;
                _frameBuffers = new Bitmap[FrameBufferCount];
                for (int i = 0; i < _frameBuffers.Length; i++)
                {
                    _frameBuffers[i] = new Bitmap(width, height, PixelFormat.Format32bppPArgb);
                }
                _frameBufferWidth = width;
                _frameBufferHeight = height;
                _displayBufferIndex = -1;
                _pendingBufferIndex = -1;
                _nextBufferIndex = 0;
            }

            if (oldBuffers != null)
            {
                DisposeOldFrameBuffersOnUi(oldBuffers);
            }
        }

        private bool TryAcquireFrameBuffer(int width, int height, out Bitmap bitmap, out int bufferIndex)
        {
            EnsureFrameBuffers(width, height);

            lock (_frameBufferLock)
            {
                bitmap = null;
                bufferIndex = -1;

                if (_frameBuffers == null || _pendingBufferIndex >= 0)
                    return false;

                for (int offset = 0; offset < _frameBuffers.Length; offset++)
                {
                    int candidate = (_nextBufferIndex + offset) % _frameBuffers.Length;
                    if (candidate == _displayBufferIndex)
                        continue;

                    _pendingBufferIndex = candidate;
                    _nextBufferIndex = (candidate + 1) % _frameBuffers.Length;
                    bitmap = _frameBuffers[candidate];
                    bufferIndex = candidate;
                    return true;
                }
            }

            return false;
        }

        private void ReleasePendingFrameBuffer(int bufferIndex)
        {
            lock (_frameBufferLock)
            {
                if (_pendingBufferIndex == bufferIndex)
                    _pendingBufferIndex = -1;
            }
        }

        private void DisposeOldFrameBuffersOnUi(Bitmap[] oldBuffers)
        {
            void DisposeNow()
            {
                try
                {
                    if (_fullscreenBox != null && !_fullscreenBox.IsDisposed &&
                        Array.IndexOf(oldBuffers, _fullscreenBox.Image as Bitmap) >= 0)
                    {
                        _fullscreenBox.Image = null;
                    }
                    if (_videoBox != null && !_videoBox.IsDisposed &&
                        Array.IndexOf(oldBuffers, _videoBox.Image as Bitmap) >= 0)
                    {
                        _videoBox.Image = null;
                    }
                    foreach (var old in oldBuffers)
                    {
                        old?.Dispose();
                    }
                }
                catch { }
            }

            UiAsync.RunSync(this, DisposeNow, "DisposeOldFrameBuffersOnUi");
        }

        private void ClearVideoDisplayAndDisposeBuffers()
        {
            UiAsync.RunSync(this, () =>
            {
                Bitmap[] buffers;
                lock (_frameBufferLock)
                {
                    buffers = _frameBuffers;
                    _frameBuffers = null;
                    _frameBufferWidth = 0;
                    _frameBufferHeight = 0;
                    _displayBufferIndex = -1;
                    _pendingBufferIndex = -1;
                    _nextBufferIndex = 0;
                }

                try
                {
                    if (_fullscreenBox != null && !_fullscreenBox.IsDisposed)
                        _fullscreenBox.Image = null;
                    if (_videoBox != null && !_videoBox.IsDisposed)
                        _videoBox.Image = null;
                    if (buffers != null)
                    {
                        foreach (var buffer in buffers)
                            buffer?.Dispose();
                    }
                }
                catch { }
            }, "ClearVideoDisplayAndDisposeBuffers");
        }

        private void OnGstNewImage(object sender, MPBitmap frame)
        {
            if (frame == null || frame.Width <= 0 || frame.Height <= 0) return;
            if (_stopping) return;

            var generation = _streamGeneration;
            _frameCount++;
            if (_frameCount % 30 == 1)
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Frame #{_frameCount} - {frame.Width}x{frame.Height}");

            if (!TryAcquireFrameBuffer(frame.Width, frame.Height, out var displayBitmap, out var bufferIndex))
                return;

            try
            {
                var lockData = frame.LockBits(Rectangle.Empty, null, SkiaSharp.SKColorType.Bgra8888);
                try
                {
                    var bmpData = displayBitmap.LockBits(
                        new Rectangle(0, 0, frame.Width, frame.Height),
                        System.Drawing.Imaging.ImageLockMode.WriteOnly,
                        PixelFormat.Format32bppPArgb);
                    try
                    {
                        var srcStride = 4 * frame.Width;
                        var dstStride = bmpData.Stride;
                        var rowBytes = Math.Min(srcStride, dstStride);
                        unsafe
                        {
                            byte* src = (byte*)lockData.Scan0;
                            byte* dst = (byte*)bmpData.Scan0;
                            for (int y = 0; y < frame.Height; y++)
                            {
                                Buffer.MemoryCopy(src + y * srcStride, dst + y * dstStride, dstStride, rowBytes);
                            }
                        }
                    }
                    finally
                    {
                        displayBitmap.UnlockBits(bmpData);
                    }
                }
                finally
                {
                    try { frame.UnlockBits(lockData); } catch { }
                }
            }
            catch
            {
                ReleasePendingFrameBuffer(bufferIndex);
                return;
            }

            var width = frame.Width;
            var height = frame.Height;

            UiAsync.RunSync(this, () => UpdateVideoDisplay(bufferIndex, width, height, generation), "OnGstNewImage");
        }

        private async Task PollCenterDepthAsync()
        {
            if (IsDisposed || _stopping) return;
            try
            {
                var resp = await JetsonApiService.ApiClient.GetAsync(
                    $"{_apiBaseUrl}/api/video/depth/center");
                if (!resp.IsSuccessStatusCode) return;
                var body = await resp.Content.ReadAsStringAsync();
                var json = JObject.Parse(body);
                var token = json["range_m"];
                if (token == null || token.Type == JTokenType.Null)
                {
                    _centerDepthM = null;
                }
                else
                {
                    double v = token.Value<double>();
                    _centerDepthM = (v > 0.0 && !double.IsNaN(v) && !double.IsInfinity(v)) ? (double?)v : null;
                }
                _centerDepthStamp = DateTime.UtcNow;
                if (!IsDisposed && _videoBox != null && _videoBox.IsHandleCreated)
                    _videoBox.Invalidate();
            }
            catch { }
        }

        private void UpdateVideoDisplay(int bufferIndex, int width, int height, int generation)
        {
            if (IsDisposed || _videoBox == null || generation != _streamGeneration || _stopping)
            {
                ReleasePendingFrameBuffer(bufferIndex);
                return;
            }

            Bitmap displayBitmap;
            lock (_frameBufferLock)
            {
                if (_frameBuffers == null ||
                    bufferIndex < 0 ||
                    bufferIndex >= _frameBuffers.Length ||
                    _frameBuffers[bufferIndex] == null)
                {
                    if (_pendingBufferIndex == bufferIndex)
                        _pendingBufferIndex = -1;
                    return;
                }

                displayBitmap = _frameBuffers[bufferIndex];
            }

            try
            {
                _videoBox.Image = displayBitmap;

                if (_fullscreenBox != null && !_fullscreenBox.IsDisposed)
                {
                    _fullscreenBox.Image = displayBitmap;
                }

                lock (_frameBufferLock)
                {
                    _displayBufferIndex = bufferIndex;
                    if (_pendingBufferIndex == bufferIndex)
                        _pendingBufferIndex = -1;
                }

                if (_frameCount % 30 == 1)
                {
                    _lblStatus.Text = $"Streaming {width}x{height}";
                    _lblStatus.ForeColor = Color.LimeGreen;
                }
            }
            catch (Exception ex)
            {
                lock (_frameBufferLock)
                {
                    if (_displayBufferIndex == bufferIndex)
                        _displayBufferIndex = -1;
                    if (_pendingBufferIndex == bufferIndex)
                        _pendingBufferIndex = -1;
                }
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Frame error - {ex.Message}");
            }
        }
    }
}
