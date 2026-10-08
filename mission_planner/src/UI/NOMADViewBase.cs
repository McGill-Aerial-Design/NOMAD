// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD View Base Class
// ============================================================
// Common base class with shared styling for all NOMAD views
// ============================================================

using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Shared styling for NOMAD views.
    /// </summary>
    public abstract class NOMADViewBase : UserControl
    {
        /// <summary>Called after the view is shown in the content area. Default: no-op.</summary>
        /// <summary>Called before the view is swapped out (kept cached). Default: no-op.</summary>
        // Colors delegated to NOMADTheme for consistency
        protected static readonly Color CARD_BG = NOMADTheme.CARD_BG;
        protected static readonly Color ACCENT_COLOR = NOMADTheme.ACCENT;
        protected static readonly Color SUCCESS_COLOR = NOMADTheme.SUCCESS;
        protected static readonly Color WARNING_COLOR = NOMADTheme.WARNING;
        protected static readonly Color ERROR_COLOR = NOMADTheme.ERROR;
        protected static readonly Color INFO_COLOR = NOMADTheme.INFO;
        protected static readonly Color TEXT_PRIMARY = NOMADTheme.TEXT_PRIMARY;
        protected static readonly Color TEXT_SECONDARY = NOMADTheme.TEXT_SECONDARY;
        protected static readonly Color TEXT_MUTED = NOMADTheme.TEXT_MUTED;

        protected NOMADViewBase()
        {
            this.BackColor = NOMADTheme.BG_DARK;
            this.Dock = DockStyle.Fill;
            this.Padding = new Padding(20);
            this.AutoScroll = true;
        }

    }
}
