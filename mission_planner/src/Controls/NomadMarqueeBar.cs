// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    internal sealed class NomadMarqueeBar : Control
    {
        private readonly Timer _timer;
        private int _offset;

        public NomadMarqueeBar()
        {
            DoubleBuffered = true;
            BackColor = NOMADTheme.INPUT_BG;
            ForeColor = NOMADTheme.ACCENT;
            SetStyle(ControlStyles.ResizeRedraw, true);
            _timer = new Timer { Interval = 30 };
            _timer.Tick += (s, e) =>
            {
                _offset = (_offset + 5) % Math.Max(1, ClientSize.Width * 2);
                Invalidate();
            };
        }

        protected override void OnVisibleChanged(EventArgs e)
        {
            base.OnVisibleChanged(e);
            if (Visible && IsHandleCreated)
                _timer.Start();
            else
                _timer.Stop();
        }

        protected override void OnHandleCreated(EventArgs e)
        {
            base.OnHandleCreated(e);
            if (Visible)
                _timer.Start();
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            Rectangle track = ClientRectangle;
            if (track.Width <= 2 || track.Height <= 2)
                return;

            using (var background = new SolidBrush(BackColor))
                e.Graphics.FillRectangle(background, track);
            int segmentWidth = Math.Max(20, track.Width / 3);
            int x = _offset % (track.Width + segmentWidth) - segmentWidth;
            using (var accent = new SolidBrush(ForeColor))
                e.Graphics.FillRectangle(accent, x, 1, segmentWidth, track.Height - 2);
            using (var border = new Pen(NOMADTheme.CARD_BORDER))
                e.Graphics.DrawRectangle(border, 0, 0, track.Width - 1, track.Height - 1);
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _timer.Stop();
                _timer.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
