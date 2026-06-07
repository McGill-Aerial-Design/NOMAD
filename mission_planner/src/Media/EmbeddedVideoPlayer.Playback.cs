// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
extern alias MPDrawing;

// ============================================================
// NOMAD Embedded Video Player - Playback partial
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Net.Http;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner.Utilities;
using Newtonsoft.Json.Linq;
using MPBitmap = MPDrawing::System.Drawing.Bitmap;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        private async System.Threading.Tasks.Task SyncOverlayStatusAsync()
        {
            try
            {
                var overlayJson = await JetsonApiService.ApiClient.GetStringAsync($"{_apiBaseUrl}/api/video/overlay/status");
                var overlayData = JObject.Parse(overlayJson);
                var overlayEnabled = overlayData["enabled"]?.Value<bool>() ?? false;

                _syncingOverlayState = true;
                _chkDetections.Checked = overlayEnabled;
                _syncingOverlayState = false;
                _overlayEnabled = overlayEnabled;
            }
            catch
            {
                _syncingOverlayState = false;
            }
        }

        private async System.Threading.Tasks.Task SetHsvModeAsync(bool enabled)
        {
            if (enabled)
            {
                var overlayResp = await JetsonApiService.LongRunClient.PostAsync(
                    $"{_apiBaseUrl}/api/video/overlay/enable", null);
                await EnsureRequestSucceededAsync(overlayResp, "overlay enable", requireSuccessField: true);
                return;
            }

            var overlayDisableResp = await JetsonApiService.LongRunClient.PostAsync(
                $"{_apiBaseUrl}/api/video/overlay/disable", null);
            await EnsureRequestSucceededAsync(overlayDisableResp, "overlay disable", requireSuccessField: true);
        }

        private async System.Threading.Tasks.Task EnsureRequestSucceededAsync(
            HttpResponseMessage response,
            string operation,
            bool requireSuccessField = false)
        {
            if (!response.IsSuccessStatusCode)
            {
                throw new InvalidOperationException(
                    $"{operation} returned {response.StatusCode}");
            }

            var payload = await response.Content.ReadAsStringAsync();
            if (string.IsNullOrWhiteSpace(payload))
            {
                if (requireSuccessField)
                    throw new InvalidOperationException($"{operation} returned empty response");
                return;
            }

            try
            {
                var json = JObject.Parse(payload);
                var successToken = json["success"];
                if (successToken != null && !successToken.Value<bool>())
                {
                    var detail = json["error"]?.ToString()
                        ?? json["message"]?.ToString()
                        ?? payload;
                    throw new InvalidOperationException($"{operation} failed: {detail}");
                }

                if (requireSuccessField && successToken == null)
                    throw new InvalidOperationException($"{operation} returned no success field");
            }
            catch (Newtonsoft.Json.JsonException) when (!requireSuccessField)
            {
            }
        }

        private async System.Threading.Tasks.Task RefreshTopicsAsync(bool autoSelectRgb = false)
        {
            try
            {
                _lblStatus.Text = "Refreshing topics...";
                _lblStatus.ForeColor = Color.Yellow;

                var json = await JetsonApiService.LongRunClient.GetStringAsync($"{_apiBaseUrl}/api/video/topics");
                var data = JObject.Parse(json);
                var arr = data["topics"] as JArray;

                _topics.Clear();
                if (arr != null)
                {
                    foreach (var t in arr)
                    {
                        var name = t["name"]?.ToString();
                        if (string.IsNullOrEmpty(name)) continue;

                        var lower = name.ToLowerInvariant();
                        if (lower.IndexOf("gray") >= 0 || lower.IndexOf("depth") >= 0)
                            continue;

                        bool isLeft  = lower.IndexOf("/left")  >= 0
                                     || lower.IndexOf("/rgb")  >= 0;
                        bool isRight = lower.IndexOf("/right") >= 0;
                        if (!isLeft && !isRight) continue;

                        bool isRaw  = lower.IndexOf("_raw") >= 0
                                    || lower.IndexOf("/raw") >= 0
                                    || lower.IndexOf("image_raw") >= 0;
                        bool isRect = lower.IndexOf("rect") >= 0;
                        if (!isRaw && !isRect) continue;

                        string side = isRight ? "Right" : "Left";
                        string kind = isRect ? "Rect" : "Raw";
                        _topics.Add((name, $"{side} {kind}"));
                    }

                    var order = new System.Collections.Generic.Dictionary<string, int>
                    {
                        { "Left Rect", 0 }, { "Left Raw", 1 },
                        { "Right Rect", 2 }, { "Right Raw", 3 },
                    };
                    _topics.Sort((a, b) =>
                    {
                        int ai, bi;
                        if (!order.TryGetValue(a.Display, out ai)) ai = 99;
                        if (!order.TryGetValue(b.Display, out bi)) bi = 99;
                        return ai.CompareTo(bi);
                    });

                    var seen = new System.Collections.Generic.HashSet<string>();
                    _topics.RemoveAll(t => !seen.Add(t.Display));
                }

                if (_topics.Count == 0)
                    _topics.Add(("/zed/zed_node/rgb/color/rect/image", "Left Rect"));

                PopulateTopics();
                _lblStatus.Text = $"Found {_topics.Count} topics";
                _lblStatus.ForeColor = Color.LimeGreen;

                if (autoSelectRgb && _topics.Count > 0)
                {
                    int rgbIndex = _topics.FindIndex(t =>
                        t.Name.IndexOf("rgb/color/rect/image", StringComparison.OrdinalIgnoreCase) >= 0 ||
                        t.Name.IndexOf("rgb_color_rect_image", StringComparison.OrdinalIgnoreCase) >= 0 ||
                        t.Name.IndexOf("rgb/image_rect_color", StringComparison.OrdinalIgnoreCase) >= 0 ||
                        t.Name.IndexOf("rgb_image_rect_color", StringComparison.OrdinalIgnoreCase) >= 0);

                    if (rgbIndex >= 0)
                    {
                        _suppressTopicChange = true;
                        _cmbTopic.SelectedIndex = rgbIndex;
                        _suppressTopicChange = false;
                        await SwitchTopicAsync();
                    }
                }
            }
            catch (Exception ex)
            {
                _lblStatus.Text = $"Refresh failed: {ex.Message}";
                _lblStatus.ForeColor = Color.Red;
            }
        }

        private async System.Threading.Tasks.Task SwitchTopicAsync()
        {
            if (_suppressTopicChange) return;
            if (_cmbTopic.SelectedIndex < 0 || _cmbTopic.SelectedIndex >= _topics.Count) return;

            var (name, display) = _topics[_cmbTopic.SelectedIndex];
            _lblStatus.Text = $"Switching to {display}...";
            _lblStatus.ForeColor = Color.Yellow;

            try
            {
                var wasPlaying = _isPlaying;

                if (wasPlaying)
                    StopStream();

                var resp = await JetsonApiService.ApiClient.PostAsync($"{_apiBaseUrl}/api/video/source?topic={Uri.EscapeDataString(name)}", null);
                if (!resp.IsSuccessStatusCode)
                {
                    _lblStatus.Text = $"Switch failed: {resp.StatusCode}";
                    _lblStatus.ForeColor = Color.Red;
                    return;
                }

                await System.Threading.Tasks.Task.Delay(500);
                if (wasPlaying && !IsDisposed)
                {
                    StartStream();
                }

                _lblStatus.Text = $"Streaming: {display}";
                _lblStatus.ForeColor = Color.LimeGreen;
            }
            catch (Exception ex)
            {
                _lblStatus.Text = $"Switch error: {ex.Message}";
                _lblStatus.ForeColor = Color.Red;
            }
        }

        private bool TryInitializeGStreamer()
        {
            try
            {
                var gstPath = GStreamer.LookForGstreamer();
                return !string.IsNullOrWhiteSpace(gstPath) && GStreamer.GstLaunchExists;
            }
            catch { return false; }
        }

        private string BuildGStreamerPipeline()
        {
            int queueBuffers = _latencyMs <= 100 ? 1 : Math.Min(_latencyMs / 50, 10);
            string leaky = _latencyMs <= 100 ? "leaky=2" : "leaky=0";
            string syncVal = _latencyMs <= 100 ? "false" : "true";

            if (_streamUrl.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
            {
                var port = ExtractUdpPort(_streamUrl);
                return $"udpsrc port={port} buffer-size=90000 ! " +
                       $"application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264 ! " +
                       $"decodebin3 ! queue max-size-buffers={queueBuffers} {leaky} ! " +
                       $"videoconvert ! video/x-raw,format=BGRA ! " +
                       $"appsink name=outsink sync={syncVal}";
            }

                 return $"rtspsrc location={_streamUrl} protocols=tcp latency={_latencyMs} do-retransmission=false ! " +
                     $"application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse disable-passthrough=true ! avdec_h264 ! " +
                     $"queue max-size-buffers={queueBuffers} {leaky} ! " +
                   $"videoconvert ! video/x-raw,format=BGRA ! " +
                   $"appsink name=outsink sync={syncVal}";
        }

        private async System.Threading.Tasks.Task RestartStreamAsync()
        {
            await _lifecycleLock.WaitAsync();
            try
            {
                StopStream();
                await System.Threading.Tasks.Task.Delay(500);
                if (!IsDisposed)
                    StartStream();
            }
            finally
            {
                _lifecycleLock.Release();
            }
        }

        public void StartStream()
        {
            if (_isPlaying) return;

            if (!TryInitializeGStreamer())
            {
                _lblStatus.Text = "GStreamer not available - use VLC";
                _lblStatus.ForeColor = Color.Orange;
                return;
            }

            GStreamer gst = null;
            try
            {
                _stopping = false;
                _streamGeneration++;

                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Starting stream to {_streamUrl}");
                gst = new GStreamer();
                gst.OnNewImage += OnGstNewImage;

                var pipeline = BuildGStreamerPipeline();
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Pipeline: {pipeline}");

                gst.Start(pipeline);
                _gst = gst;
                _isPlaying = true;
                gst = null;
                _lblStatus.Text = "Connecting...";
                _lblStatus.ForeColor = Color.Yellow;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Start error - {ex}");
                _lblStatus.Text = $"Error: {ex.Message}";
                _lblStatus.ForeColor = Color.Red;
            }
            finally
            {
                if (gst != null)
                {
                    try { gst.OnNewImage -= OnGstNewImage; } catch { }
                    try { gst.Stop(); } catch { }
                    try { (gst as IDisposable)?.Dispose(); } catch { }
                }
            }
        }

        public void StopStream()
        {
            if (!_isPlaying && _gst == null) return;

            _stopping = true;
            _streamGeneration++;

            var gst = _gst;
            _gst = null;
            _isPlaying = false;

            try
            {
                if (gst != null)
                {
                    try { gst.OnNewImage -= OnGstNewImage; } catch { }

                    try { gst.Stop(); } catch { }

                    System.Threading.Thread.Sleep(300);

                    try { (gst as IDisposable)?.Dispose(); } catch { }
                }
            }
            catch { }

            ClearVideoDisplayAndDisposeBuffers();

            _lblStatus.Text = "Stopped";
            _lblStatus.ForeColor = Color.Gray;
        }

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
