// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMADConfig — spray calibration settings (partial class)
// ============================================================
// Split out of NOMADConfig.cs to keep responsibilities small.
// Clamping/migration for these fields lives in NOMADConfig.cs.
// ============================================================

namespace NOMAD.MissionPlanner
{
    public partial class NOMADConfig
    {
        // ============================================================
        // Spray Calibration
        // ============================================================

        public float SprayTargetCameraRangeM { get; set; } = 3.8f;
        public float SprayRangeToleranceM { get; set; } = 0.25f;
        public float SprayTriggerMaxDistanceM { get; set; } = 5.5f;
        public int SprayAimPixelX { get; set; } = 640;
        public int SprayAimPixelY { get; set; } = 390;
        public int SprayAimTolerancePx { get; set; } = 25;
        public float SprayServoFireAngleDeg { get; set; } = 82.0f;
        public float SprayForwardGain { get; set; } = 0.45f;
        public float SprayLateralGain { get; set; } = 0.0010f;
        public float SprayAltitudeGain { get; set; } = 0.0010f;
        public float SprayYawGain { get; set; } = 0.0025f;
        public bool SprayUseYawAlignment { get; set; } = true;
        public float SprayMaxForwardSpeedMps { get; set; } = 0.45f;
        public float SprayMaxLateralSpeedMps { get; set; } = 0.25f;
        public float SprayMaxAltitudeSpeedMps { get; set; } = 0.20f;
        public float SprayMaxYawRateRadps { get; set; } = 0.35f;
        public int SprayLockHoldMs { get; set; } = 700;
        public float SprayAlignTimeoutS { get; set; } = 20.0f;
    }
}
