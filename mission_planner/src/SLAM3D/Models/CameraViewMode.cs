// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// CameraViewMode.cs - Camera view mode enumeration
// ============================================================

namespace NOMAD.MissionPlanner.SLAM3D.Models
{
    /// <summary>
    /// Camera view mode for 3D visualization.
    /// </summary>
    public enum CameraViewMode
    {
        /// <summary>First-person view from drone perspective.</summary>
        FirstPerson,

        /// <summary>Third-person view following the drone.</summary>
        ThirdPerson,

        /// <summary>Free orbit camera around a fixed point.</summary>
        FreeOrbit
    }
}
