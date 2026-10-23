// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.ComponentModel;
using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    internal sealed class NOMADMenuItem : ToolStripMenuItem
    {
        private readonly MenuStrip _strip;
        private readonly Func<bool> _isCurrent;
        private readonly Action _open;
        private ContextMenuStrip _hostContext;
        private bool _navigateOnRelease;

        internal NOMADMenuItem(MenuStrip strip, Func<bool> isCurrent, Action open) : base("NOMAD")
        {
            _strip = strip;
            _isCurrent = isCurrent;
            _open = open;
            ForeColor = Color.White;
            BackColor = NOMADTheme.ACCENT;
            _strip.ContextMenuStripChanged += HostContextChanged;
            HostContextChanged(this, EventArgs.Empty);
        }

        protected override void OnMouseDown(MouseEventArgs e)
        {
            _navigateOnRelease = e.Button == MouseButtons.Left && !_isCurrent();
            if (_navigateOnRelease) { return; }
            if (e.Button == MouseButtons.Right) { ShowDropDown(); return; }
            base.OnMouseDown(e);
        }

        protected override void OnMouseUp(MouseEventArgs e)
        {
            if (_navigateOnRelease)
            {
                _navigateOnRelease = false;
                if (e.Button == MouseButtons.Left && new Rectangle(Point.Empty, Size).Contains(e.Location))
                { OnClick(EventArgs.Empty); }
                return;
            }
            if (e.Button == MouseButtons.Right) { return; }
            base.OnMouseUp(e);
        }

        protected override void OnClick(EventArgs e)
        {
            if (!_isCurrent()) { _open(); }
            base.OnClick(e);
        }

        private void HostContextChanged(object sender, EventArgs args)
        {
            if (_hostContext != null) { _hostContext.Opening -= HostContextOpening; }
            _hostContext = _strip.ContextMenuStrip;
            if (_hostContext != null) { _hostContext.Opening += HostContextOpening; }
        }

        internal bool SuppressHostContext(Control source, Point screenPoint) =>
            ReferenceEquals(source, _strip) && Bounds.Contains(_strip.PointToClient(screenPoint));

        private void HostContextOpening(object sender, CancelEventArgs args)
        {
            if (SuppressHostContext(_hostContext.SourceControl, Cursor.Position)) { args.Cancel = true; }
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _strip.ContextMenuStripChanged -= HostContextChanged;
                if (_hostContext != null) { _hostContext.Opening -= HostContextOpening; }
            }
            base.Dispose(disposing);
        }
    }
}
