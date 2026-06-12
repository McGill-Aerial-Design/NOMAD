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

        // Frame-buffer handling lives in EmbeddedVideoPlayer.Frames.cs.
    }
}
