// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// CameraController.cs - 3D camera control for SLAM view
// ============================================================
// Supports FPV (with servo gimbal), attitude-based TPV, and
// free orbit camera modes. Computes eye/target/up for LookAt.
// ============================================================

using System;
using NOMAD.MissionPlanner.SLAM3D.Models;

namespace NOMAD.MissionPlanner.SLAM3D.Camera
{
    /// <summary>
    /// Controls 3D camera position and orientation for the SLAM view.
    /// All drone inputs are in ROS frame; GL position must be pre-converted.
    /// </summary>
    public class CameraController
    {
        // ==================== Configuration ====================

        /// <summary>Current camera view mode.</summary>
        public CameraViewMode ViewMode { get; set; } = CameraViewMode.ThirdPerson;

        /// <summary>Orbit distance from target (FreeOrbit mode).</summary>
        public float OrbitDistance { get; set; } = 12f;

        /// <summary>Orbit yaw angle (degrees).</summary>
        public float OrbitYaw { get; set; } = 45f;

        /// <summary>Orbit pitch angle (degrees, positive=looking down).</summary>
        public float OrbitPitch { get; set; } = 30f;

        /// <summary>Orbit center X (FreeOrbit mode).</summary>
        public float OrbitCenterX { get; set; }

        /// <summary>Orbit center Y (FreeOrbit mode).</summary>
        public float OrbitCenterY { get; set; }

        /// <summary>Orbit center Z (FreeOrbit mode).</summary>
        public float OrbitCenterZ { get; set; }

        /// <summary>Heading offset in degrees (camera vs drone orientation).</summary>
        public float HeadingOffsetDeg { get; set; } = 0f;

        /// <summary>Vertical FOV in degrees.</summary>
        public float FieldOfView { get; set; } = 60f;

        // ==================== Output (computed camera state) ====================

        /// <summary>Camera eye position X.</summary>
        public float EyeX { get; private set; }

        /// <summary>Camera eye position Y.</summary>
        public float EyeY { get; private set; }

        /// <summary>Camera eye position Z.</summary>
        public float EyeZ { get; private set; }

        /// <summary>Camera look-at target X.</summary>
        public float TargetX { get; private set; }

        /// <summary>Camera look-at target Y.</summary>
        public float TargetY { get; private set; }

        /// <summary>Camera look-at target Z.</summary>
        public float TargetZ { get; private set; }

        /// <summary>Camera up vector X.</summary>
        public float UpX { get; private set; }

        /// <summary>Camera up vector Y.</summary>
        public float UpY { get; private set; } = 1f;

        /// <summary>Camera up vector Z.</summary>
        public float UpZ { get; private set; }

        // ==================== Public Methods ====================

        /// <summary>
        /// Update camera based on drone pose. Position in GL frame, angles in ROS radians.
        /// </summary>
        /// <param name="glX">Drone GL X position</param>
        /// <param name="glY">Drone GL Y position</param>
        /// <param name="glZ">Drone GL Z position</param>
        /// <param name="yawRad">ROS yaw in radians</param>
        /// <param name="pitchRad">ROS pitch in radians</param>
        /// <param name="rollRad">ROS roll in radians</param>
        /// <param name="servoDeg">Servo angle in degrees (90=level, FPV only)</param>
        public void Update(float glX, float glY, float glZ,
                           float yawRad, float pitchRad, float rollRad,
                           float servoDeg = 90f)
        {
            // Heading: ROS yaw → GL Y-axis rotation
            float headingRad = -yawRad + HeadingOffsetDeg * PI / 180f;

            switch (ViewMode)
            {
                case CameraViewMode.FirstPerson:
                    UpdateFirstPerson(glX, glY, glZ, headingRad, pitchRad, rollRad, servoDeg);
                    break;
                case CameraViewMode.ThirdPerson:
                    UpdateThirdPerson(glX, glY, glZ, headingRad, pitchRad, rollRad);
                    break;
                default:
                    UpdateFreeOrbit();
                    break;
            }
        }

        /// <summary>Rotate the orbit camera by delta angles.</summary>
        public void RotateOrbit(float deltaYawDeg, float deltaPitchDeg)
        {
            OrbitYaw += deltaYawDeg;
            OrbitPitch = Math.Max(-89f, Math.Min(89f, OrbitPitch + deltaPitchDeg));
        }

        /// <summary>Zoom the orbit camera.</summary>
        public void Zoom(float delta)
        {
            OrbitDistance = Math.Max(1f, Math.Min(100f, OrbitDistance + delta));
        }

        /// <summary>Pan the orbit center using camera-relative vectors.</summary>
        public void Pan(float dx, float dy)
        {
            if (ViewMode != CameraViewMode.FreeOrbit) return;

            float scale = OrbitDistance * 0.002f;
            float fwdX = TargetX - EyeX, fwdY = TargetY - EyeY, fwdZ = TargetZ - EyeZ;
            float fwdLen = (float)Math.Sqrt(fwdX * fwdX + fwdY * fwdY + fwdZ * fwdZ);
            if (fwdLen < 0.001f) return;

            fwdX /= fwdLen; fwdY /= fwdLen; fwdZ /= fwdLen;

            // Right = cross(forward, (0,1,0)) simplified
            float rx = fwdZ, ry = 0f, rz = -fwdX;
            float rLen = (float)Math.Sqrt(rx * rx + rz * rz);
            if (rLen > 0.001f) { rx /= rLen; rz /= rLen; }

            // Up = cross(right, forward)
            float ux = ry * fwdZ - rz * fwdY;
            float uy = rz * fwdX - rx * fwdZ;
            float uz = rx * fwdY - ry * fwdX;

            OrbitCenterX -= rx * dx * scale + ux * dy * scale;
            OrbitCenterY -= ry * dx * scale + uy * dy * scale;
            OrbitCenterZ -= rz * dx * scale + uz * dy * scale;
        }

        /// <summary>Center orbit on a specific GL position.</summary>
        public void CenterOn(float glX, float glY, float glZ)
        {
            OrbitCenterX = glX;
            OrbitCenterY = glY;
            OrbitCenterZ = glZ;
        }

        /// <summary>Reset to default third-person view.</summary>
        public void Reset()
        {
            OrbitYaw = 45f;
            OrbitPitch = 30f;
            OrbitDistance = 12f;
            OrbitCenterX = OrbitCenterY = OrbitCenterZ = 0;
        }

        /// <summary>Cycle to next view mode.</summary>
        public CameraViewMode CycleViewMode()
        {
            ViewMode = ViewMode switch
            {
                CameraViewMode.ThirdPerson => CameraViewMode.FirstPerson,
                CameraViewMode.FirstPerson => CameraViewMode.FreeOrbit,
                CameraViewMode.FreeOrbit => CameraViewMode.ThirdPerson,
                _ => CameraViewMode.ThirdPerson
            };
            return ViewMode;
        }

        // ==================== Private: View Mode Implementations ====================

        private const float PI = 3.14159265358979323846f;

        private void UpdateFirstPerson(float glX, float glY, float glZ,
                                        float headingRad, float pitchRad, float rollRad,
                                        float servoDeg)
        {
            float pitchR = -pitchRad;
            float rollR = rollRad;
            float elevRad = (servoDeg - 90f) * PI / 180f + pitchR;

            float cosH = (float)Math.Cos(headingRad), sinH = (float)Math.Sin(headingRad);
            float cosE = (float)Math.Cos(elevRad), sinE = (float)Math.Sin(elevRad);
            float cosR = (float)Math.Cos(rollR), sinR = (float)Math.Sin(rollR);

            EyeX = glX; EyeY = glY; EyeZ = glZ;

            // Forward direction from heading + elevation
            float fwdX = sinH * cosE;
            float fwdY = sinE;
            float fwdZ = -cosH * cosE;

            TargetX = glX + fwdX;
            TargetY = glY + fwdY;
            TargetZ = glZ + fwdZ;

            // Build camera up vector with roll
            ComputeRolledUpVector(fwdX, fwdY, fwdZ, cosR, sinR);
        }

        private void UpdateThirdPerson(float glX, float glY, float glZ,
                                        float headingRad, float pitchRad, float rollRad)
        {
            float dist = 3f, height = 1.5f;
            float pitchR = -pitchRad;
            float rollR = rollRad;

            float cosH = (float)Math.Cos(headingRad), sinH = (float)Math.Sin(headingRad);
            float cosP = (float)Math.Cos(pitchR), sinP = (float)Math.Sin(pitchR);
            float cosR = (float)Math.Cos(rollR), sinR = (float)Math.Sin(rollR);

            // Forward from heading + pitch
            float fwdX = sinH * cosP;
            float fwdY = sinP;
            float fwdZ = -cosH * cosP;

            // Build up vector with roll
            ComputeRolledUpVector(fwdX, fwdY, fwdZ, cosR, sinR);

            // Camera behind forward + above in rolled up direction
            EyeX = glX - fwdX * dist + UpX * height;
            EyeY = glY - fwdY * dist + UpY * height;
            EyeZ = glZ - fwdZ * dist + UpZ * height;

            TargetX = glX; TargetY = glY; TargetZ = glZ;
        }

        private void UpdateFreeOrbit()
        {
            float yawRad = OrbitYaw * PI / 180f;
            float pitchRad = OrbitPitch * PI / 180f;
            float cosPitch = (float)Math.Cos(pitchRad);

            EyeX = OrbitCenterX + (float)Math.Sin(yawRad) * cosPitch * OrbitDistance;
            EyeY = OrbitCenterY + (float)Math.Sin(pitchRad) * OrbitDistance;
            EyeZ = OrbitCenterZ + (float)Math.Cos(yawRad) * cosPitch * OrbitDistance;

            TargetX = OrbitCenterX;
            TargetY = OrbitCenterY;
            TargetZ = OrbitCenterZ;

            UpX = 0; UpY = 1; UpZ = 0;
        }

        /// <summary>
        /// Compute an up vector from a forward direction and roll angle.
        /// Handles the near-vertical-forward degenerate case.
        /// </summary>
        private void ComputeRolledUpVector(float fwdX, float fwdY, float fwdZ,
                                            float cosR, float sinR)
        {
            // Reference up
            float refUpX = 0f, refUpY = 1f, refUpZ = 0f;
            if (Math.Abs(fwdY) > 0.98f)
            {
                refUpX = 1f; refUpY = 0f; refUpZ = 0f;
            }

            // Right = cross(forward, refUp)
            float rightX = fwdY * refUpZ - fwdZ * refUpY;
            float rightY = fwdZ * refUpX - fwdX * refUpZ;
            float rightZ = fwdX * refUpY - fwdY * refUpX;
            float rightLen = (float)Math.Sqrt(rightX * rightX + rightY * rightY + rightZ * rightZ);
            if (rightLen < 1e-6f)
            {
                UpX = 0; UpY = 1; UpZ = 0;
                return;
            }
            rightX /= rightLen; rightY /= rightLen; rightZ /= rightLen;

            // upNoRoll = cross(right, forward)
            float unX = rightY * fwdZ - rightZ * fwdY;
            float unY = rightZ * fwdX - rightX * fwdZ;
            float unZ = rightX * fwdY - rightY * fwdX;
            float unLen = (float)Math.Sqrt(unX * unX + unY * unY + unZ * unZ);
            if (unLen > 1e-6f) { unX /= unLen; unY /= unLen; unZ /= unLen; }

            // Apply roll: up = upNoRoll*cos(roll) + right*sin(roll)
            UpX = unX * cosR + rightX * sinR;
            UpY = unY * cosR + rightY * sinR;
            UpZ = unZ * cosR + rightZ * sinR;

            float upLen = (float)Math.Sqrt(UpX * UpX + UpY * UpY + UpZ * UpZ);
            if (upLen > 1e-6f) { UpX /= upLen; UpY /= upLen; UpZ /= upLen; }
        }
    }
}
