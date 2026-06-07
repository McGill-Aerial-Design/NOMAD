// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Theme - Centralized Color Constants
// ============================================================
// Single source of truth for all UI colors across the plugin.
// Eliminates duplication and ensures visual consistency.
// ============================================================

using System.Drawing;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Centralized theme colors for NOMAD plugin UI consistency.
    /// </summary>
    public static class NOMADTheme
    {
        // Background colors
        public static readonly Color BG_DARK = Color.FromArgb(30, 30, 33);
        public static readonly Color CARD_BG = Color.FromArgb(40, 40, 45);
        public static readonly Color CARD_BORDER = Color.FromArgb(60, 60, 65);
        public static readonly Color INPUT_BG = Color.FromArgb(50, 50, 53);
        public static readonly Color BUTTON_BG = Color.FromArgb(60, 60, 65);

        // State colors
        public static readonly Color ACCENT = Color.FromArgb(0, 122, 204);
        public static readonly Color SUCCESS = Color.FromArgb(76, 175, 80);
        public static readonly Color WARNING = Color.FromArgb(255, 152, 0);
        public static readonly Color ERROR = Color.FromArgb(244, 67, 54);
        public static readonly Color INFO = Color.FromArgb(33, 150, 243);
        public static readonly Color FOCUS = Color.FromArgb(220, 50, 50);

        // Text colors
        public static readonly Color TEXT_PRIMARY = Color.White;
        public static readonly Color TEXT_SECONDARY = Color.FromArgb(150, 150, 150);
        public static readonly Color TEXT_MUTED = Color.FromArgb(120, 120, 120);

        // Sensor category colors (for notifications and health display)
        public static readonly Color GPS_COLOR = Color.FromArgb(33, 150, 243);
        public static readonly Color VIO_COLOR = Color.FromArgb(156, 39, 176);
        public static readonly Color OPTICAL_FLOW_COLOR = Color.FromArgb(0, 188, 212);
        public static readonly Color EKF_COLOR = Color.FromArgb(255, 193, 7);
        public static readonly Color BATTERY_COLOR = Color.FromArgb(244, 67, 54);
        public static readonly Color BOUNDARY_COLOR = Color.FromArgb(255, 87, 34);
        public static readonly Color LINK_COLOR = Color.FromArgb(76, 175, 80);
        public static readonly Color SYSTEM_COLOR = Color.FromArgb(96, 125, 139);

        // Button action colors
        public static readonly Color BTN_START = Color.FromArgb(0, 120, 60);
        public static readonly Color BTN_STOP = Color.FromArgb(150, 50, 50);
        public static readonly Color BTN_PRIMARY = Color.FromArgb(0, 122, 204);
    }
}
