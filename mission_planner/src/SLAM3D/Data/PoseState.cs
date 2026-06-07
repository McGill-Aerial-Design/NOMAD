// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// PoseState.cs - Drone Pose Management with Anti-Jitter
// ============================================================
// Handles pose smoothing, jump rejection, and attitude filtering
// to prevent rubberbanding in 3D visualization.
// ============================================================

using System;
using System.Diagnostics;

namespace NOMAD.MissionPlanner.SLAM3D.Data
{
    /// <summary>
    /// Manages drone pose state with anti-jitter filtering to prevent rubberbanding.
    /// </summary>
    /// <remarks>
    /// The pose state tracks both raw incoming values and smoothed render values.
    /// It rejects sudden jumps that can occur from VIO tracking loss or network issues,
    /// while still allowing genuine large movements after sustained input.
    /// </remarks>
    public class PoseState
    {
        // ==================== Configuration ====================

        /// <summary>Maximum position jump (meters) before rejection.</summary>
        public float MaxPositionJumpM { get; set; } = 0.35f;

        /// <summary>Maximum yaw jump (degrees) before rejection.</summary>
        public float MaxYawJumpDeg { get; set; } = 25f;

        /// <summary>Maximum roll/pitch jump (degrees) before rejection.</summary>
        public float MaxTiltJumpDeg { get; set; } = 20f;

        /// <summary>Number of consecutive rejects before accepting (genuine large move).</summary>
        public int RejectStreakBeforeAccept { get; set; } = 5;

        /// <summary>Smoothing factor for position (0=no smooth, 1=instant).</summary>
        public float PositionSmoothFactor { get; set; } = 0.25f;

        /// <summary>Smoothing factor for attitude (0=no smooth, 1=instant).</summary>
        public float AttitudeSmoothFactor { get; set; } = 0.30f;

        /// <summary>Time threshold (seconds) after which pose snaps instead of blends.</summary>
        public double SnapAfterGapSec { get; set; } = 0.40;

        /// <summary>Near-zero threshold (degrees) for detecting reset-to-origin frames.</summary>
        public float NearZeroDeg { get; set; } = 8.0f;

        /// <summary>
        /// Parity mode: when true, all jump rejection and smoothing are bypassed
        /// so the client renders exactly what the server sends. Used for RViz
        /// parity validation runs where any client-side filtering would make the
        /// two visualizations diverge under aggressive maneuvers or relocalization.
        /// </summary>
        public bool ParityMode { get; set; } = false;

        // ==================== State ====================

        // Raw pose from latest accepted update
        private float _rawX, _rawY, _rawZ;
        private float _rawRoll, _rawPitch, _rawYaw;

        // Smoothed pose for rendering
        private float _smoothX, _smoothY, _smoothZ;
        private float _smoothRoll, _smoothPitch, _smoothYaw;

        // Velocity (optional, for prediction)
        private float _velX, _velY, _velZ;

        // Tracking
        private int _consecutiveRejects;
        private long _lastUpdateTicks;
        private bool _initialized;
        private readonly object _lock = new object();

        // Statistics
        private int _totalUpdates;
        private int _rejectedUpdates;
        private int _acceptedUpdates;
        private DateTime _lastRejectLogUtc = DateTime.MinValue;

        // ==================== Events ====================

        /// <summary>Fired when a pose update is rejected.</summary>
        public event Action<string> OnPoseRejected;

        // ==================== Properties ====================

        /// <summary>Current smoothed X position (meters).</summary>
        public float X { get { lock (_lock) return _smoothX; } }

        /// <summary>Current smoothed Y position (meters).</summary>
        public float Y { get { lock (_lock) return _smoothY; } }

        /// <summary>Current smoothed Z position (meters).</summary>
        public float Z { get { lock (_lock) return _smoothZ; } }

        /// <summary>Current smoothed roll (radians).</summary>
        public float Roll { get { lock (_lock) return _smoothRoll; } }

        /// <summary>Current smoothed pitch (radians).</summary>
        public float Pitch { get { lock (_lock) return _smoothPitch; } }

        /// <summary>Current smoothed yaw (radians).</summary>
        public float Yaw { get { lock (_lock) return _smoothYaw; } }

        /// <summary>Raw X position before smoothing.</summary>
        public float RawX { get { lock (_lock) return _rawX; } }

        /// <summary>Raw Y position before smoothing.</summary>
        public float RawY { get { lock (_lock) return _rawY; } }

        /// <summary>Raw Z position before smoothing.</summary>
        public float RawZ { get { lock (_lock) return _rawZ; } }

        /// <summary>Raw roll before smoothing (radians).</summary>
        public float RawRoll { get { lock (_lock) return _rawRoll; } }

        /// <summary>Raw pitch before smoothing (radians).</summary>
        public float RawPitch { get { lock (_lock) return _rawPitch; } }

        /// <summary>Raw yaw before smoothing (radians).</summary>
        public float RawYaw { get { lock (_lock) return _rawYaw; } }

        /// <summary>Last known velocity X (m/s).</summary>
        public float VelocityX { get { lock (_lock) return _velX; } }

        /// <summary>Last known velocity Y (m/s).</summary>
        public float VelocityY { get { lock (_lock) return _velY; } }

        /// <summary>Last known velocity Z (m/s).</summary>
        public float VelocityZ { get { lock (_lock) return _velZ; } }

        /// <summary>Number of consecutive rejected updates (for diagnostics).</summary>
        public int ConsecutiveRejects { get { lock (_lock) return _consecutiveRejects; } }

        /// <summary>Total number of updates received.</summary>
        public int TotalUpdates { get { lock (_lock) return _totalUpdates; } }

        /// <summary>Number of rejected updates.</summary>
        public int RejectedUpdates { get { lock (_lock) return _rejectedUpdates; } }

        // ==================== Public Methods ====================

        /// <summary>
        /// Update pose with new values from WebSocket.
        /// </summary>
        /// <param name="x">X position (meters)</param>
        /// <param name="y">Y position (meters)</param>
        /// <param name="z">Z position (meters)</param>
        /// <param name="roll">Roll angle (radians)</param>
        /// <param name="pitch">Pitch angle (radians)</param>
        /// <param name="yaw">Yaw angle (radians)</param>
        /// <param name="attitudeValid">Whether attitude values are valid</param>
        /// <returns>True if update was accepted, false if rejected</returns>
        public bool Update(float x, float y, float z, float roll, float pitch, float yaw, bool attitudeValid = true)
        {
            lock (_lock)
            {
                _totalUpdates++;
                long nowTicks = Stopwatch.GetTimestamp();

                // Parity mode short-circuit: store the raw values, snap smoothed
                // output directly to them, and skip all rejection/smoothing so
                // the view matches RViz's direct stream behavior one-for-one.
                if (ParityMode)
                {
                    _rawX = x; _rawY = y; _rawZ = z;
                    _smoothX = x; _smoothY = y; _smoothZ = z;
                    if (attitudeValid)
                    {
                        _rawRoll = roll; _rawPitch = pitch; _rawYaw = yaw;
                        _smoothRoll = roll; _smoothPitch = pitch; _smoothYaw = yaw;
                    }
                    _lastUpdateTicks = nowTicks;
                    _initialized = true;
                    _consecutiveRejects = 0;
                    _acceptedUpdates++;
                    return true;
                }

                // First update: initialize everything
                if (!_initialized)
                {
                    _rawX = x; _rawY = y; _rawZ = z;
                    _rawRoll = roll; _rawPitch = pitch; _rawYaw = yaw;
                    _smoothX = x; _smoothY = y; _smoothZ = z;
                    _smoothRoll = roll; _smoothPitch = pitch; _smoothYaw = yaw;
                    _lastUpdateTicks = nowTicks;
                    _initialized = true;
                    _acceptedUpdates++;
                    return true;
                }

                // Calculate time since last update
                double gapSec = (double)(nowTicks - _lastUpdateTicks) / Stopwatch.Frequency;
                bool shouldSnap = gapSec > SnapAfterGapSec;

                // Check for position jump
                float posJump = MathHelper.Sqrt(
                    (_rawX - x) * (_rawX - x) +
                    (_rawY - y) * (_rawY - y) +
                    (_rawZ - z) * (_rawZ - z)
                );

                // Check for attitude jumps (only if attitude is valid)
                float yawJump = 0f, tiltJump = 0f;
                if (attitudeValid)
                {
                    yawJump = MathHelper.Abs(AngleDiffRad(_rawYaw, yaw)) * 180f / MathHelper.PI;
                    float rollDiff = MathHelper.Abs(AngleDiffRad(_rawRoll, roll)) * 180f / MathHelper.PI;
                    float pitchDiff = MathHelper.Abs(AngleDiffRad(_rawPitch, pitch)) * 180f / MathHelper.PI;
                    tiltJump = MathHelper.Max(rollDiff, pitchDiff);
                }

                // Detect suspicious "reset to zero" frames
                bool suspiciousReset = false;
                if (attitudeValid)
                {
                    float rollDeg = MathHelper.Abs(roll) * 180f / MathHelper.PI;
                    float pitchDeg = MathHelper.Abs(pitch) * 180f / MathHelper.PI;
                    float yawDeg = MathHelper.Abs(yaw) * 180f / MathHelper.PI;

                    bool allNearZero = rollDeg < NearZeroDeg && pitchDeg < NearZeroDeg && yawDeg < NearZeroDeg;
                    bool prevHadAttitude = MathHelper.Abs(_rawRoll) > NearZeroDeg * MathHelper.PI / 180f ||
                                           MathHelper.Abs(_rawPitch) > NearZeroDeg * MathHelper.PI / 180f ||
                                           MathHelper.Abs(_rawYaw) > NearZeroDeg * MathHelper.PI / 180f;

                    // Suspicious if we suddenly reset to zero when we had real attitude before.
                    // Include yawJump so a flat drone (roll≈0, pitch≈0) with non-zero yaw that
                    // drops to (0,0,0) is also caught — tiltJump alone is near-zero in that case.
                    suspiciousReset = allNearZero && prevHadAttitude &&
                                      (tiltJump > MaxTiltJumpDeg || yawJump > NearZeroDeg);
                }

                // Decide whether to reject
                bool shouldReject = false;
                string rejectReason = null;

                if (!shouldSnap) // Only apply jump filters when not snapping
                {
                    if (posJump > MaxPositionJumpM && _consecutiveRejects < RejectStreakBeforeAccept)
                    {
                        shouldReject = true;
                        rejectReason = $"Position jump {posJump:F2}m > {MaxPositionJumpM:F2}m";
                    }
                    else if (attitudeValid && yawJump > MaxYawJumpDeg && _consecutiveRejects < RejectStreakBeforeAccept)
                    {
                        shouldReject = true;
                        rejectReason = $"Yaw jump {yawJump:F1}deg > {MaxYawJumpDeg:F1}deg";
                    }
                    else if (suspiciousReset && _consecutiveRejects < RejectStreakBeforeAccept)
                    {
                        shouldReject = true;
                        rejectReason = "Suspicious reset to zero attitude";
                    }
                }

                if (shouldReject)
                {
                    _consecutiveRejects++;
                    _rejectedUpdates++;

                    // Rate-limited logging
                    if ((DateTime.UtcNow - _lastRejectLogUtc).TotalSeconds > 1.0)
                    {
                        OnPoseRejected?.Invoke($"Rejected frame #{_consecutiveRejects}: {rejectReason}");
                        _lastRejectLogUtc = DateTime.UtcNow;
                    }

                    return false;
                }

                // Accept the update
                _consecutiveRejects = 0;
                _acceptedUpdates++;
                _lastUpdateTicks = nowTicks;

                // Update raw values
                _rawX = x; _rawY = y; _rawZ = z;
                if (attitudeValid)
                {
                    _rawRoll = roll;
                    _rawPitch = pitch;
                    _rawYaw = yaw;
                }

                // Apply smoothing (or snap if gap was too long)
                if (shouldSnap)
                {
                    _smoothX = x; _smoothY = y; _smoothZ = z;
                    if (attitudeValid)
                    {
                        _smoothRoll = roll;
                        _smoothPitch = pitch;
                        _smoothYaw = yaw;
                    }
                }
                else
                {
                    // Position smoothing
                    _smoothX = Lerp(_smoothX, x, PositionSmoothFactor);
                    _smoothY = Lerp(_smoothY, y, PositionSmoothFactor);
                    _smoothZ = Lerp(_smoothZ, z, PositionSmoothFactor);

                    // Attitude smoothing (with angle wrapping)
                    if (attitudeValid)
                    {
                        _smoothRoll = LerpAngle(_smoothRoll, roll, AttitudeSmoothFactor);
                        _smoothPitch = LerpAngle(_smoothPitch, pitch, AttitudeSmoothFactor);
                        _smoothYaw = LerpAngle(_smoothYaw, yaw, AttitudeSmoothFactor);
                    }
                }

                return true;
            }
        }

        /// <summary>
        /// Update velocity values (optional, for future prediction).
        /// </summary>
        public void UpdateVelocity(float vx, float vy, float vz)
        {
            lock (_lock)
            {
                _velX = vx;
                _velY = vy;
                _velZ = vz;
            }
        }

        /// <summary>
        /// Force-set the pose without smoothing or rejection (for initialization or reset).
        /// </summary>
        public void ForceSet(float x, float y, float z, float roll, float pitch, float yaw)
        {
            lock (_lock)
            {
                _rawX = x; _rawY = y; _rawZ = z;
                _rawRoll = roll; _rawPitch = pitch; _rawYaw = yaw;
                _smoothX = x; _smoothY = y; _smoothZ = z;
                _smoothRoll = roll; _smoothPitch = pitch; _smoothYaw = yaw;
                _consecutiveRejects = 0;
                _lastUpdateTicks = Stopwatch.GetTimestamp();
                _initialized = true;
            }
        }

        /// <summary>
        /// Reset the pose state (used when clearing the map).
        /// </summary>
        public void Reset()
        {
            lock (_lock)
            {
                _rawX = _rawY = _rawZ = 0;
                _rawRoll = _rawPitch = _rawYaw = 0;
                _smoothX = _smoothY = _smoothZ = 0;
                _smoothRoll = _smoothPitch = _smoothYaw = 0;
                _velX = _velY = _velZ = 0;
                _consecutiveRejects = 0;
                _initialized = false;
                _totalUpdates = 0;
                _rejectedUpdates = 0;
                _acceptedUpdates = 0;
            }
        }

        /// <summary>
        /// Get pose statistics for diagnostics.
        /// </summary>
        public (int total, int rejected, int accepted, int streak) GetStats()
        {
            lock (_lock)
            {
                return (_totalUpdates, _rejectedUpdates, _acceptedUpdates, _consecutiveRejects);
            }
        }

        // ==================== Helper Methods ====================

        private static float Lerp(float a, float b, float t)
        {
            return a + (b - a) * t;
        }

        private static float LerpAngle(float a, float b, float t)
        {
            float diff = AngleDiffRad(a, b);
            return a + diff * t;
        }

        /// <summary>
        /// Compute shortest angle difference between two angles (radians).
        /// Result is in [-PI, PI].
        /// </summary>
        private static float AngleDiffRad(float from, float to)
        {
            float diff = to - from;
            while (diff > MathHelper.PI) diff -= 2 * MathHelper.PI;
            while (diff < -MathHelper.PI) diff += 2 * MathHelper.PI;
            return diff;
        }
    }
}
