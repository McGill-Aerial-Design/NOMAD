// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD View Base Class
// ============================================================
// Common base class with shared styling for all NOMAD views
// ============================================================

using System.Drawing;
using System.Windows.Forms;
using NOMAD.MissionPlanner.Core;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Base class for NOMAD views with common styling. Implements the optional
    /// <see cref="INomadView"/> activation hooks as no-ops so a view only
    /// overrides the one(s) it needs (e.g. to pause work when swapped out).
    /// </summary>
    public abstract class NOMADViewBase : UserControl, INomadView
    {
        /// <summary>Called after the view is shown in the content area. Default: no-op.</summary>
        public virtual void OnActivated() { }

        /// <summary>Called before the view is swapped out (kept cached). Default: no-op.</summary>
        public virtual void OnDeactivated() { }

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

        protected Panel CreateCard(string title, int width = -1, int height = -1)
        {
            var card = new Panel
            {
                BackColor = CARD_BG,
                Margin = new Padding(5),
                Padding = new Padding(15),
            };

            if (width > 0) card.Width = width;
            if (height > 0) card.Height = height;

            var titleLabel = new Label
            {
                Text = title.ToUpper(),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(15, 15),
                AutoSize = true,
            };
            card.Controls.Add(titleLabel);

            return card;
        }

        protected Button CreateButton(string text, Color bgColor, int width = 150, int height = 45)
        {
            var btn = new Button
            {
                Text = text,
                Size = new Size(width, height),
                Margin = new Padding(5),
                FlatStyle = FlatStyle.Flat,
                BackColor = bgColor,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Cursor = Cursors.Hand,
            };
            btn.FlatAppearance.BorderSize = 0;
            return btn;
        }
    }
}
