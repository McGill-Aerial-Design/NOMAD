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
        // Background colors — black-first scheme
        public static readonly Color BG_DARK = Color.FromArgb(10, 10, 12);
        public static readonly Color CARD_BG = Color.FromArgb(24, 24, 27);
        public static readonly Color CARD_BORDER = Color.FromArgb(48, 48, 52);
        public static readonly Color INPUT_BG = Color.FromArgb(30, 30, 34);
        public static readonly Color BUTTON_BG = Color.FromArgb(42, 42, 46);

        // State colors — red accent
        public static readonly Color ACCENT = Color.FromArgb(220, 30, 40);
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
        public static readonly Color BTN_PRIMARY = Color.FromArgb(220, 30, 40);

        // Additional surfaces — named replacements for the two ad-hoc greys that
        // were copy-pasted across the views (combo/numeric/textbox backing, and the
        // slightly-lighter activity/log panel). Routing them through the theme keeps
        // inputs and panels uniform everywhere.
        public static readonly Color CONTROL_BG = Color.FromArgb(50, 50, 53);
        public static readonly Color PANEL_ALT = Color.FromArgb(38, 38, 42);

        // ============================================================
        // Typography — single source of truth for fonts + sizes
        // ============================================================
        // Every view built fonts inline with magic sizes ("Segoe UI", 9/10/14...).
        // These constants + helpers make sizing consistent and let a future font
        // change happen in one place. Callers own the returned Font (WinForms
        // controls dispose their own on assignment), matching prior inline usage.
        public const string FONT_FAMILY = "Segoe UI";
        public const string MONO_FAMILY = "Consolas";

        public const float SIZE_TITLE = 14f;   // page / window titles
        public const float SIZE_LARGE = 12f;   // prominent status values
        public const float SIZE_HEADING = 11f; // card / section headings
        public const float SIZE_BODY = 9f;     // default body text
        public const float SIZE_SMALL = 8f;    // captions, units, hints

        /// <summary>Themed UI font (Segoe UI) at the given size/style.</summary>
        public static Font Font(float size = SIZE_BODY, FontStyle style = FontStyle.Regular) =>
            new Font(FONT_FAMILY, size, style);

        /// <summary>Themed monospace font (Consolas) for readouts/logs.</summary>
        public static Font Mono(float size = SIZE_BODY, FontStyle style = FontStyle.Regular) =>
            new Font(MONO_FAMILY, size, style);

        // ============================================================
        // Layout metrics — consistent spacing across cards/rows
        // ============================================================
        public const int PAD = 12;  // card / view inner padding
        public const int GAP = 6;   // gap between stacked cards / controls
    }
}
