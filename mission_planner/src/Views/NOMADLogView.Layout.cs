// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADLogView
    {
        private readonly List<Control> _summaryMetricCards = new List<Control>();
        private readonly List<Control> _liveMetricCards = new List<Control>();
        private TableLayoutPanel _postFlightLayout;
        private FlowLayoutPanel _importToolbar;
        private TableLayoutPanel _summaryBody;
        private SplitContainer _plotSplit;
        private FlowLayoutPanel _plotToolbar;
        private TableLayoutPanel _liveLayout;
        private FlowLayoutPanel _liveToolbar;
        private FlowLayoutPanel _liveMetrics;
        private bool? _summaryStacked;
        private bool _applyingResponsiveLayout;

        private void ApplyResponsiveLayout()
        {
            if (_applyingResponsiveLayout || IsDisposed)
                return;

            _applyingResponsiveLayout = true;
            try
            {
                int width = ToLogicalPixels(Math.Max(
                    1,
                    _tabs?.SelectedTab?.ClientSize.Width ?? ClientSize.Width));
                ConfigureSummaryBody(width < 900);
                ConfigurePlotSplit(width < 820);
                ResizeSummaryCards();
                ResizeIssueColumns();
                ResizeLiveMetricCards();
                LimitToolbarStatusWidths();
            }
            finally
            {
                _applyingResponsiveLayout = false;
            }
        }

        private void ConfigureSummaryBody(bool stacked)
        {
            if (_summaryBody == null || _summaryStacked == stacked)
                return;

            _summaryBody.SuspendLayout();
            _summaryBody.Controls.Clear();
            _summaryBody.ColumnStyles.Clear();
            _summaryBody.RowStyles.Clear();
            _summaryBody.ColumnCount = stacked ? 1 : 2;
            _summaryBody.RowCount = stacked ? 2 : 1;

            if (stacked)
            {
                _summaryBody.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
                _summaryBody.RowStyles.Add(new RowStyle(SizeType.Percent, 50));
                _summaryBody.RowStyles.Add(new RowStyle(SizeType.Percent, 50));
                _summaryFlow.Margin = new Padding(0, 0, 0, 3);
                _issuesList.Margin = new Padding(0, 3, 0, 0);
                _summaryBody.Controls.Add(_summaryFlow, 0, 0);
                _summaryBody.Controls.Add(_issuesList, 0, 1);
            }
            else
            {
                _summaryBody.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 70));
                _summaryBody.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 30));
                _summaryBody.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
                _summaryFlow.Margin = new Padding(0, 0, 4, 0);
                _issuesList.Margin = new Padding(4, 0, 0, 0);
                _summaryBody.Controls.Add(_summaryFlow, 0, 0);
                _summaryBody.Controls.Add(_issuesList, 1, 0);
            }

            _summaryStacked = stacked;
            _summaryBody.ResumeLayout(true);
        }

        private void ConfigurePlotSplit(bool stacked)
        {
            if (_plotSplit == null)
                return;

            Orientation orientation = stacked ? Orientation.Horizontal : Orientation.Vertical;
            int targetExtent = stacked ? _plotSplit.ClientSize.Height : _plotSplit.ClientSize.Width;
            int currentExtent = _plotSplit.Orientation == Orientation.Horizontal
                ? _plotSplit.ClientSize.Height
                : _plotSplit.ClientSize.Width;
            int minimumExtent = ScalePixels(120);
            if (targetExtent <= minimumExtent || currentExtent <= minimumExtent)
                return;

            if (_plotSplit.Orientation != orientation)
            {
                _plotSplit.Panel1MinSize = 0;
                _plotSplit.Panel2MinSize = 0;
                _plotSplit.SplitterDistance = Math.Max(
                    ScalePixels(40),
                    Math.Min(
                        _plotSplit.SplitterDistance,
                        Math.Min(targetExtent, currentExtent) - ScalePixels(80)));
                _plotSplit.Orientation = orientation;
            }

            int target = stacked
                ? Math.Max(ScalePixels(90), Math.Min(ScalePixels(180), (int)(targetExtent * 0.28)))
                : Math.Max(ScalePixels(180), Math.Min(ScalePixels(300), (int)(targetExtent * 0.24)));
            _plotSplit.SplitterDistance = Math.Min(target, targetExtent - ScalePixels(80));
        }

        private void ResizeSummaryCards()
        {
            if (_summaryFlow == null)
                return;

            int available = Math.Max(ScalePixels(180), _summaryFlow.ClientSize.Width - ScalePixels(20));
            int logicalAvailable = ToLogicalPixels(available);
            int columns = logicalAvailable >= 720 ? 3 : logicalAvailable >= 440 ? 2 : 1;
            int width = Math.Max(ScalePixels(170), (available - columns * ScalePixels(8)) / columns);
            foreach (Control card in _summaryMetricCards)
            {
                card.Width = width;
                card.Height = ScalePixels(82);
            }
            if (_emptySummaryLabel != null)
                _emptySummaryLabel.Width = available;
        }

        private void ResizeIssueColumns()
        {
            if (_issuesList == null || _issuesList.Columns.Count < 2)
                return;

            int available = Math.Max(ScalePixels(180), _issuesList.ClientSize.Width - ScalePixels(4));
            int issueWidth = Math.Max(ScalePixels(105), (int)(available * 0.36));
            _issuesList.Columns[0].Width = issueWidth;
            _issuesList.Columns[1].Width = Math.Max(ScalePixels(70), available - issueWidth);
        }

        private void ResizeLiveMetricCards()
        {
            if (_liveMetrics == null || _liveLayout == null)
                return;

            int available = Math.Max(ScalePixels(170), _liveMetrics.ClientSize.Width - ScalePixels(20));
            int logicalAvailable = ToLogicalPixels(available);
            int columns = logicalAvailable >= 1120 ? 6 : logicalAvailable >= 720 ? 3
                : logicalAvailable >= 480 ? 2 : 1;
            int width = Math.Max(ScalePixels(160), (available - columns * ScalePixels(8)) / columns);
            foreach (Control card in _liveMetricCards)
            {
                card.Width = width;
                card.Height = ScalePixels(82);
            }

            int rows = Math.Max(1, (int)Math.Ceiling(_liveMetricCards.Count / (double)columns));
            int desiredHeight = rows * ScalePixels(90);
            int maxHeight = Math.Max(ScalePixels(90), _liveLayout.ClientSize.Height - ScalePixels(130));
            _liveLayout.RowStyles[1].Height = Math.Min(desiredHeight, maxHeight);
        }

        private void LimitToolbarStatusWidths()
        {
            if (_loadStatus != null && _importToolbar != null)
            {
                _loadStatus.MaximumSize = new Size(
                    Math.Max(ScalePixels(180), _importToolbar.ClientSize.Width - ScalePixels(24)),
                    0);
            }
            if (_liveStatus != null && _liveToolbar != null)
            {
                _liveStatus.MaximumSize = new Size(
                    Math.Max(ScalePixels(180), _liveToolbar.ClientSize.Width - ScalePixels(24)),
                    0);
            }
        }

        private int ScalePixels(int logicalPixels)
            => (int)Math.Round(logicalPixels * Math.Max(96, DeviceDpi) / 96d);

        private int ToLogicalPixels(int devicePixels)
            => (int)Math.Round(devicePixels * 96d / Math.Max(96, DeviceDpi));

        private static void AddNomadBorder(Control control, Color? color = null)
        {
            control.Paint += (s, e) =>
            {
                if (control.ClientSize.Width <= 1 || control.ClientSize.Height <= 1)
                    return;
                using (var pen = new Pen(color ?? NOMADTheme.CARD_BORDER))
                    e.Graphics.DrawRectangle(pen, 0, 0, control.ClientSize.Width - 1, control.ClientSize.Height - 1);
            };
        }

        private void DrawLogTab(object sender, DrawItemEventArgs e)
        {
            bool selected = e.Index == _tabs.SelectedIndex;
            Color background = selected ? NOMADTheme.ACCENT : NOMADTheme.CARD_BG;
            Color foreground = selected ? NOMADTheme.TEXT_PRIMARY : NOMADTheme.TEXT_SECONDARY;
            using (var brush = new SolidBrush(background))
                e.Graphics.FillRectangle(brush, e.Bounds);
            using (var font = NOMADTheme.Font(10, selected ? FontStyle.Bold : FontStyle.Regular))
            {
                TextRenderer.DrawText(
                    e.Graphics,
                    _tabs.TabPages[e.Index].Text,
                    font,
                    e.Bounds,
                    foreground,
                    TextFormatFlags.HorizontalCenter | TextFormatFlags.VerticalCenter);
            }
        }
    }
}
