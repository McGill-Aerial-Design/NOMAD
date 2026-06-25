// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// SLAM3DView.Data.cs - WebSocket data handling and polling
// ============================================================

using System;
using System.Diagnostics;
using System.Net.Http;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;
using NOMAD.MissionPlanner.SLAM3D.Data;
using NOMAD.MissionPlanner.SLAM3D.Models;
using NOMAD.MissionPlanner.SLAM3D.Network;
using NOMAD.MissionPlanner.SLAM3D.Rendering;

namespace NOMAD.MissionPlanner
{
    public partial class SLAM3DView
    {
        private void HandleWebSocketStatusChanged(string status)
        {
            if (string.IsNullOrWhiteSpace(status))
                return;

            string text = status.StartsWith("Status:", StringComparison.OrdinalIgnoreCase)
                ? status
                : $"Status: {status}";
            UpdateStatusSafe(text);

        }

        private void HandleWebSocketError(string error)
        {
            if (string.IsNullOrWhiteSpace(error))
                return;
            UpdateStatusSafe($"Status: {error}");
        }

        // ==================== WebSocket Stream ====================

        private void StartUpdateLoop()
        {
            _webSocketClient.BaseUrl = JetsonApiService.BaseUrl ?? "http://localhost:8000";
            _webSocketClient.ApiKey = JetsonApiService.ApiKey;
            _webSocketClient.Start();
        }

        private void HandleSlamFrame(SlamFrame frame)
        {
            if (frame == null || !_autoUpdateEnabled)
                return;

            var frameJson = frame.RawJson ?? new JObject();
            // Canonical SLAM frame identifier end-to-end: "map" (REP-103 axes).
            // Bridge, ws_slam, and mesh endpoint all emit this exact string.
            string frameId = string.IsNullOrWhiteSpace(frame.FrameId) ? "map" : frame.FrameId;
            if (!string.Equals(frameId, "map", StringComparison.OrdinalIgnoreCase))
            {
                Debug.WriteLine($"[SLAM3D] Unexpected frame_id: {frameId} (expected map)");
            }

            bool hasPosePositionInFrame = false;
            float latestX = 0f, latestY = 0f, latestZ = 0f;

            lock (_poseLock)
            {
                if (TryReadFloatToken(frameJson["x"], out float xVal))
                {
                    _dronePosX = xVal;
                    hasPosePositionInFrame = true;
                }

                if (TryReadFloatToken(frameJson["y"], out float yVal))
                {
                    _dronePosY = yVal;
                    hasPosePositionInFrame = true;
                }

                if (TryReadFloatToken(frameJson["z"], out float zVal))
                {
                    _dronePosZ = zVal;
                    hasPosePositionInFrame = true;
                }

                // Do NOT reset PoseState when there is a gap since the last pose
                // frame: PoseState.Update() already snaps (instead of smoothing)
                // once its internal gap exceeds SnapAfterGapSec. Resetting here
                // would zero the render values and teleport the drone to (0,0,0)
                // on any websocket hiccup (GC pause, net jitter, etc.).

                // Server marks attitude_valid=false when it is replaying the
                // last known-good attitude because the live VIO attitude has
                // collapsed to zero/degraded. In that case do NOT push raw
                // values through the client pipeline: keep the previously
                // cached attitude and flag it as degraded so UI can surface
                // the state instead of silently drifting.
                bool attitudeValidFromFrame = frame.AttitudeValid;
                if (frameJson["attitude_valid"] != null)
                {
                    try { attitudeValidFromFrame = frameJson["attitude_valid"].Value<bool>(); }
                    catch { attitudeValidFromFrame = true; }
                }
                _attitudeValid = attitudeValidFromFrame;

                bool hasBodyRoll = TryReadFloatToken(frameJson["body_roll"], out float bodyRoll);
                bool hasBodyPitch = TryReadFloatToken(frameJson["body_pitch"], out float bodyPitch);
                bool hasBodyYaw = TryReadFloatToken(frameJson["body_yaw"], out float bodyYaw);

                bool hasFallbackRoll = TryReadFloatToken(frameJson["roll"], out float rollFallback);
                bool hasFallbackPitch = TryReadFloatToken(frameJson["pitch"], out float pitchFallback);
                bool hasFallbackYaw = TryReadFloatToken(frameJson["yaw"], out float yawFallback);

                bool hasRoll = hasBodyRoll || hasFallbackRoll;
                bool hasPitch = hasBodyPitch || hasFallbackPitch;
                bool hasYaw = hasBodyYaw || hasFallbackYaw;

                float nextRoll = hasBodyRoll ? bodyRoll : (hasFallbackRoll ? rollFallback : 0f);
                float nextPitch = hasBodyPitch ? bodyPitch : (hasFallbackPitch ? pitchFallback : 0f);
                float nextYaw = hasBodyYaw ? bodyYaw : (hasFallbackYaw ? yawFallback : 0f);

                if (hasBodyYaw)
                {
                    float prevYaw = _droneYawRaw;
                    float diff = nextYaw - prevYaw;
                    while (diff > Math.PI) diff -= (float)(2.0 * Math.PI);
                    while (diff < -Math.PI) diff += (float)(2.0 * Math.PI);
                    float yawJumpDeg = Math.Abs(diff * 180f / (float)Math.PI);

                    // Guard against sporadic magnetometer resets to exactly zero.
                    if (Math.Abs(nextYaw) < 1e-4f && yawJumpDeg > 45f)
                    {
                        nextYaw = prevYaw;
                    }
                }

                _hasBodyAttitude = hasBodyYaw;

                // Only commit a new attitude sample when the server reports it
                // as valid. When attitude_valid is false the server is replaying
                // the last-good value, and we prefer to leave our local cached
                // raw values untouched so the client does not count the replay
                // as a fresh observation (which would feed PoseState smoothing
                // and jump-rejection with stale data).
                if (attitudeValidFromFrame && hasRoll && hasPitch && hasYaw)
                {
                    _droneRollRaw = nextRoll;
                    _dronePitchRaw = nextPitch;
                    _droneYawRaw = nextYaw;
                }

                if (TryReadFloatToken(frameJson["vx"], out float vxVal))
                    _droneVelX = vxVal;
                if (TryReadFloatToken(frameJson["vy"], out float vyVal))
                    _droneVelY = vyVal;
                if (TryReadFloatToken(frameJson["vz"], out float vzVal))
                    _droneVelZ = vzVal;

                latestX = _dronePosX;
                latestY = _dronePosY;
                latestZ = _dronePosZ;
            }

            if (hasPosePositionInFrame)
            {
                _trajectoryRenderer.AddPointFromRos(latestX, latestY, latestZ);
                _hasPoseFrame = true;
            }

            UpdateDetectionMarkersFromFrame(frameJson);

            if (frame.Type == SlamFrameType.Mesh && frame.MeshToken != null)
            {
                var meshData = frame.MeshToken.ToObject<MeshDataModel>();
                if (meshData == null)
                    return;

                _voxelMeshBuilder.UpdateMesh(meshData);
                _meshUpdateCount++;

                _totalBlocks = meshData.TotalBlocks > 0 ? meshData.TotalBlocks : meshData.TotalVoxels;
                string mode = (meshData.Mode == "voxel" || meshData.Mode == "voxels") ? "voxels" : "blocks";
                string statsText = $"Mesh: {_totalBlocks:N0} {mode} ({_voxelMeshBuilder.VoxelCount:N0} cached)";

                UpdateStatusSafe($"Status: Connected (30Hz) | Updates: {_meshUpdateCount}");
                UpdateStatsSafe(statsText);
            }
        }

        private void UpdateDetectionMarkersFromFrame(JObject frameJson)
        {
            if (frameJson == null)
                return;

            var detectionsToken = frameJson["detections"] as JArray;
            if (detectionsToken == null)
            {
                if (frameJson.ContainsKey("detections"))
                    _detectionRenderer.Clear();
                return;
            }

            var markers = new System.Collections.Generic.List<DetectionMarkerData>(detectionsToken.Count);
            foreach (var detection in detectionsToken)
            {
                var dx = detection["x"]?.Value<double?>();
                var dy = detection["y"]?.Value<double?>();
                var dz = detection["z"]?.Value<double?>();
                if (dx == null || dy == null || dz == null)
                    continue;

                var (gx, gy, gz) = CoordinateConverter.RosToOpenGL((float)dx.Value, (float)dy.Value, (float)dz.Value);
                markers.Add(new DetectionMarkerData
                {
                    Label = detection["label"]?.ToString() ?? string.Empty,
                    X = gx,
                    Y = gy,
                    Z = gz,
                    Confidence = detection["confidence"]?.Value<float>() ?? 0f,
                    SeenCount = detection["seen_count"]?.Value<int>() ?? 1,
                    ColorMatch = detection["color_match"]?.Value<bool?>() ?? true,
                    NeedsReview = detection["needs_review"]?.Value<bool>() ?? false,
                });
            }
            _detectionRenderer.UpdateMarkers(markers);
        }

        private void UpdateLocalMapFocus()
        {
            if (!_hasPoseFrame)
                return;

            var (glX, glY, glZ) = CoordinateConverter.RosToOpenGL(_renderPosX, _renderPosY, _renderPosZ);
            _voxelMeshBuilder.SetFocusSphere(glX, glY, glZ, Math.Max(1f, _config.SlamMapRadiusM));
        }

        // ==================== Servo Polling ====================

        private void StartServoPolling()
        {
            _servoTimer = new System.Windows.Forms.Timer { Interval = 500 };
            _servoTimer.Tick += async (s, e) =>
            {
                try
                {
                    var response = await JetsonApiService.GetAsync("/api/servo/camera/tilt");
                    if (response.IsSuccessStatusCode)
                    {
                        var json = await response.Content.ReadAsStringAsync();
                        var obj = JObject.Parse(json);
                        // Prefer pwm_us from SERVO_OUTPUT_RAW feedback (actual FC output).
                        // Fall back to angle_deg → pulse conversion using calibration range.
                        int pulseUs;
                        var pwmToken = obj["pwm_us"];
                        if (pwmToken != null)
                        {
                            pulseUs = pwmToken.Value<int>();
                        }
                        else
                        {
                            float angleDeg = obj["angle"]?.Value<float>() ?? 90f;
                            // angle_deg is in 0-180 with 90=level; convert using calibration.
                            // Below neutral uses down-side slope, above uses up-side slope.
                            if (angleDeg <= 90f)
                                pulseUs = (int)Math.Round(ServoPulseLevelUs - (90f - angleDeg) / 45f * (ServoPulseLevelUs - ServoPulseDownUs));
                            else
                                pulseUs = (int)Math.Round(ServoPulseLevelUs + (angleDeg - 90f) / 45f * (ServoPulseUpUs - ServoPulseLevelUs));
                        }
                        lock (_poseLock) { _servoPulseUs = pulseUs; }
                    }
                }
                catch (Exception ex) { System.Diagnostics.Debug.WriteLine(ex); }
            };
            _servoTimer.Start();
        }

    }
}
