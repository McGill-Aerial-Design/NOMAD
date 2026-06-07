// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// SLAM3DView.Rendering.cs - OpenGL rendering pipeline
// ============================================================

using System;
using System.Diagnostics;
using System.Drawing;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using NOMAD.MissionPlanner.SLAM3D.Data;
using NOMAD.MissionPlanner.SLAM3D.Models;

namespace NOMAD.MissionPlanner
{
    public partial class SLAM3DView
    {
        // ==================== OpenGL Setup ====================

        private void GlControl_Load(object sender, EventArgs e)
        {
            GL.ClearColor(0.08f, 0.08f, 0.10f, 1.0f);
            GL.Enable(EnableCap.DepthTest);
            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
            GL.Enable(EnableCap.Lighting);
            GL.Enable(EnableCap.Light0);
            GL.Enable(EnableCap.Light1);
            GL.Enable(EnableCap.ColorMaterial);
            GL.ColorMaterial(MaterialFace.FrontAndBack, ColorMaterialParameter.AmbientAndDiffuse);

            // Main light from upper-right-front
            GL.Light(LightName.Light0, LightParameter.Position, new float[] { 1f, 2f, 1f, 0f });
            GL.Light(LightName.Light0, LightParameter.Diffuse, new float[] { 0.7f, 0.7f, 0.7f, 1f });
            GL.Light(LightName.Light0, LightParameter.Ambient, new float[] { 0.35f, 0.35f, 0.35f, 1f });

            // Fill light from opposite side
            GL.Light(LightName.Light1, LightParameter.Position, new float[] { -1f, 0.5f, -0.5f, 0f });
            GL.Light(LightName.Light1, LightParameter.Diffuse, new float[] { 0.25f, 0.25f, 0.3f, 1f });

            _glInitialized = true;
            GlControl_Resize(sender, e);
        }

        private void GlControl_Resize(object sender, EventArgs e)
        {
            if (!_glInitialized || _glControl == null) return;
            int w = Math.Max(1, _glControl.Width);
            int h = Math.Max(1, _glControl.Height);
            GL.Viewport(0, 0, w, h);
            ApplyProjectionMatrix(w, h);
        }

        // ==================== Main Render ====================

        private void GlControl_Paint(object sender, PaintEventArgs e)
        {
            if (!_glInitialized || _glControl == null) return;

            try
            {
                _glControl.MakeCurrent();
                ApplyProjectionMatrix(Math.Max(1, _glControl.Width), Math.Max(1, _glControl.Height));

                _voxelMeshBuilder.ProcessPendingRebuild();

                BlendRenderPose();

                GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

                GL.MatrixMode(MatrixMode.Modelview);
                GL.LoadIdentity();
                ApplyCameraControllerView();

                _gridRenderer.ShowGrid = _chkShowGrid != null && _chkShowGrid.Checked;
                _trajectoryRenderer.ShowTrajectory = _chkShowTrajectory != null && _chkShowTrajectory.Checked;

                _gridRenderer.Render();
                _voxelMeshBuilder.Render();
                _trajectoryRenderer.Render();
                DrawDroneModel();
                _detectionRenderer.Render();

                _glControl.SwapBuffers();

                // Draw HUD overlay (top-right)
                DrawHudOverlay(e.Graphics);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"SLAM3D render error: {ex.Message}");
            }
        }

        // ==================== Camera ====================

        /// <summary>
        /// Blends raw pose to render pose using PoseState's anti-jitter filtering.
        /// PoseState handles: jump rejection, zero-reset glitch rejection, and smoothing.
        /// </summary>
        private void BlendRenderPose()
        {
            // Read raw pose under lock
            float srcX, srcY, srcZ, srcRoll, srcPitch, srcYaw;
            bool hasBodyAttitude;
            lock (_poseLock)
            {
                srcX = _dronePosX;
                srcY = _dronePosY;
                srcZ = _dronePosZ;
                srcRoll = _droneRollRaw;
                srcPitch = _dronePitchRaw;
                srcYaw = _droneYawRaw;
                hasBodyAttitude = _hasBodyAttitude;
            }

            // Update PoseState (handles jump rejection, smoothing)
            // PoseState expects radians for angles (same as _droneRollRaw etc.)
            _poseState.Update(srcX, srcY, srcZ, srcRoll, srcPitch, srcYaw, hasBodyAttitude);

            // Read filtered output from PoseState (already in radians)
            _renderPosX = _poseState.X;
            _renderPosY = _poseState.Y;
            _renderPosZ = _poseState.Z;
            _renderRollRaw = _poseState.Roll;
            _renderPitchRaw = _poseState.Pitch;
            _renderYawRaw = _poseState.Yaw;
            UpdateLocalMapFocus();

            // Log pose state stats periodically (every 5 seconds)
            LogPoseStateStats();
        }

        private DateTime _lastPoseStatsLogUtc = DateTime.MinValue;

        private void LogPoseStateStats()
        {
            if ((DateTime.UtcNow - _lastPoseStatsLogUtc).TotalSeconds < 5.0)
                return;
            _lastPoseStatsLogUtc = DateTime.UtcNow;

            var (total, rejected, accepted, streak) = _poseState.GetStats();
            if (total > 0)
            {
                double rejectRate = (double)rejected / total * 100.0;
                System.Diagnostics.Debug.WriteLine(
                    $"[PoseState] Total: {total}, Rejected: {rejected} ({rejectRate:F1}%), Accepted: {accepted}, Streak: {streak}");
            }
        }

        private static bool TryReadFloatToken(JToken token, out float value)
        {
            value = 0f;
            if (token == null || (token.Type != JTokenType.Integer && token.Type != JTokenType.Float))
                return false;

            value = token.Value<float>();
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        // Note: AngleMagnitudeDeg and ShouldRejectAttitudeResetGlitch moved to PoseState.cs

        // Remap PWM us to the renderer's "90deg = level" space using the rig's
        // mechanical anchors. The mount can only tilt about 45deg either side
        // of horizontal — it cannot look straight down.
        //   700us  -> 45deg below horizontal  (renderer 45deg)
        //   1250us -> level                   (renderer 90deg)
        //   1450us -> 45deg above horizontal  (renderer 135deg)
        // Piecewise linear so each half compresses correctly given the
        // asymmetric pulse spans (down=550us, up=200us).
        private const float ServoTiltMaxDownDeg = 45f;
        private const float ServoTiltMaxUpDeg   = 45f;

        private static float ServoPulseUsToRenderDeg(int pulseUs)
        {
            if (pulseUs <= ServoPulseLevelUs)
            {
                float span = ServoPulseLevelUs - ServoPulseDownUs;
                if (span <= 0) return 90f;
                return (90f - ServoTiltMaxDownDeg) +
                       (pulseUs - ServoPulseDownUs) * ServoTiltMaxDownDeg / span;
            }
            else
            {
                float span = ServoPulseUpUs - ServoPulseLevelUs;
                if (span <= 0) return 90f;
                return 90f +
                       (pulseUs - ServoPulseLevelUs) * ServoTiltMaxUpDeg / span;
            }
        }

        private void ApplyCameraControllerView()
        {
            float servoDeg;
            lock (_poseLock) { servoDeg = ServoPulseUsToRenderDeg(_servoPulseUs); }

            var (glX, glY, glZ) = CoordinateConverter.RosToOpenGL(_renderPosX, _renderPosY, _renderPosZ);

            _cameraController.HeadingOffsetDeg = _config.SlamHeadingOffsetDeg;
            _cameraController.Update(glX, glY, glZ, _renderYawRaw, _renderPitchRaw, _renderRollRaw, servoDeg);

            var view = Matrix4.LookAt(
                _cameraController.EyeX,
                _cameraController.EyeY,
                _cameraController.EyeZ,
                _cameraController.TargetX,
                _cameraController.TargetY,
                _cameraController.TargetZ,
                _cameraController.UpX,
                _cameraController.UpY,
                _cameraController.UpZ);
            GL.LoadMatrix(ref view);
        }

        // ==================== Mouse Controls ====================

        private void GlControl_MouseDown(object sender, MouseEventArgs e)
        {
            _lastMousePos = e.Location;
            if (e.Button == MouseButtons.Left) _mouseRotating = true;
            if (e.Button == MouseButtons.Right || e.Button == MouseButtons.Middle) _mousePanning = true;
        }

        private void GlControl_MouseUp(object sender, MouseEventArgs e)
        {
            _mouseRotating = false;
            _mousePanning = false;
        }

        private void GlControl_MouseMove(object sender, MouseEventArgs e)
        {
            if (_cameraController.ViewMode != CameraViewMode.FreeOrbit) return;
            float dx = e.X - _lastMousePos.X;
            float dy = e.Y - _lastMousePos.Y;
            _lastMousePos = e.Location;

            if (_mouseRotating)
            {
                _cameraController.RotateOrbit(dx * 0.3f, dy * 0.3f);
            }

            if (_mousePanning)
            {
                _cameraController.Pan(dx, dy);
            }
        }

        private void GlControl_MouseWheel(object sender, MouseEventArgs e)
        {
            if (_cameraController.ViewMode != CameraViewMode.FreeOrbit) return;
            _cameraController.Zoom(-e.Delta * 0.01f);
        }

        private void DrawDroneModel()
        {
            if (_cameraController.ViewMode == CameraViewMode.FirstPerson)
                return;

            float servoDeg;
            lock (_poseLock) { servoDeg = ServoPulseUsToRenderDeg(_servoPulseUs); }

            var (glX, glY, glZ) = CoordinateConverter.RosToOpenGL(_renderPosX, _renderPosY, _renderPosZ);

            _droneRenderer.LengthM = _config.DroneLengthCm / 100f;
            _droneRenderer.WidthM = _config.DroneWidthCm / 100f;
            _droneRenderer.HeightM = _config.DroneHeightCm / 100f;
            _droneRenderer.FrameType = _config.DroneFrameType;
            _droneRenderer.HeadingOffsetDeg = _config.SlamHeadingOffsetDeg;
            _droneRenderer.CameraForwardOffsetM = _config.CameraForwardOffsetCm / 100f;
            _droneRenderer.CameraDownOffsetM = _config.CameraDownOffsetCm / 100f;

            _droneRenderer.Draw(glX, glY, glZ, _renderYawRaw, _renderPitchRaw, _renderRollRaw, servoDeg);
        }

        /// <summary>
        /// Draw a HUD overlay in the top-right corner showing pose and velocity.
        /// </summary>
        private void DrawHudOverlay(Graphics g)
        {
            float rollDeg, pitchDeg, yawDeg, vx, vy, vz;
            lock (_poseLock)
            {
                rollDeg = (float)(_droneRollRaw * 180.0 / Math.PI);
                pitchDeg = (float)(_dronePitchRaw * 180.0 / Math.PI);
                yawDeg = (float)(_droneYawRaw * 180.0 / Math.PI);
                vx = _droneVelX;
                vy = _droneVelY;
                vz = _droneVelZ;
            }

            int right = _glControl.Width - 12;
            int top = 12;
            int lineH = 16;
            int pad = 4;

            string[] lines = new string[7];
            lines[0] = "POSE (RAW ZED WS)";
            lines[1] = $"  Roll:  {rollDeg,8:F2}°";
            lines[2] = $"  Pitch: {pitchDeg,8:F2}°";
            lines[3] = $"  Yaw:   {yawDeg,8:F2}°";
            lines[4] = "VELOCITY (m/s)";
            lines[5] = $"  Vx: {vx,7:F3}";
            lines[6] = $"  Vy: {vy,7:F3}  Vz: {vz,7:F3}";

            using (var font = new Font("Consolas", 10f))
            using (var bgBrush = new SolidBrush(Color.FromArgb(140, 15, 15, 20)))
            using (var textBrush = new SolidBrush(Color.FromArgb(220, 220, 220)))
            using (var titleBrush = new SolidBrush(Color.FromArgb(0, 160, 230)))
            using (var pen = new Pen(Color.FromArgb(80, 80, 83), 1))
            {
                // Measure to size the background
                float maxWidth = 0;
                foreach (var line in lines)
                {
                    var sz = g.MeasureString(line, font);
                    if (sz.Width > maxWidth) maxWidth = sz.Width;
                }
                int boxW = (int)maxWidth + pad * 2;
                int boxH = lines.Length * lineH + pad * 2;

                int x = right - boxW;
                int y = top;

                // Background
                g.FillRectangle(bgBrush, x, y, boxW, boxH);
                g.DrawRectangle(pen, x, y, boxW, boxH);

                // Text
                int cy = y + pad;
                for (int i = 0; i < lines.Length; i++)
                {
                    var brush = (i == 0 || i == 4) ? titleBrush : textBrush;
                    g.DrawString(lines[i], font, brush, x + pad, cy);
                    cy += lineH;
                }
            }
        }
    }
}
