// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADLogView
    {
        private Panel _verdictPanel;
        private Label _verdictLabel;
        private FlowLayoutPanel _summaryFlow;
        private ListView _issuesList;
        private Label _emptySummaryLabel;

        private Control CreateSummaryArea()
        {
            var area = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.BG_DARK,
            };

            _verdictPanel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 42,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(12, 7, 12, 7),
            };
            AddNomadBorder(_verdictPanel, NOMADTheme.ACCENT);
            _verdictLabel = new Label
            {
                Text = "NO LOG LOADED",
                Dock = DockStyle.Fill,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_HEADING, FontStyle.Bold),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                TextAlign = ContentAlignment.MiddleLeft,
            };
            _verdictPanel.Controls.Add(_verdictLabel);

            _summaryBody = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                Padding = new Padding(0, 6, 0, 0),
                BackColor = NOMADTheme.BG_DARK,
            };
            _summaryBody.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 70));
            _summaryBody.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 30));
            _summaryBody.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            _summaryFlow = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                AutoScroll = true,
                WrapContents = true,
                BackColor = NOMADTheme.BG_DARK,
                Margin = new Padding(0, 0, 4, 0),
            };
            _summaryFlow.Resize += (s, e) => ResizeSummaryCards();
            AddEmptySummaryMessage();

            _issuesList = new ListView
            {
                Dock = DockStyle.Fill,
                View = View.Details,
                FullRowSelect = true,
                HeaderStyle = ColumnHeaderStyle.Nonclickable,
                BackColor = CARD_BG,
                ForeColor = TEXT_PRIMARY,
                BorderStyle = BorderStyle.None,
                OwnerDraw = true,
                ShowItemToolTips = true,
                Margin = new Padding(4, 0, 0, 0),
            };
            _issuesList.Columns.Add("Issues", 125);
            _issuesList.Columns.Add("Detail", 260);
            _issuesList.DrawColumnHeader += DrawIssueHeader;
            _issuesList.DrawItem += DrawIssueItem;
            _issuesList.DrawSubItem += DrawIssueSubItem;
            _issuesList.Resize += (s, e) => ResizeIssueColumns();

            _summaryBody.Controls.Add(_summaryFlow, 0, 0);
            _summaryBody.Controls.Add(_issuesList, 1, 0);

            area.Controls.Add(_summaryBody);
            area.Controls.Add(_verdictPanel);
            return area;
        }

        private void RenderSummary(LogSummary summary)
        {
            if (summary == null) return;
            Color verdictColor = VerdictColor(summary.OverallVerdict);
            _verdictPanel.BackColor = NOMADTheme.CARD_BG;
            _verdictLabel.ForeColor = verdictColor;
            _verdictLabel.Text = $"{summary.OverallVerdict.ToString().ToUpperInvariant()}: {summary.OverallText}";

            _summaryFlow.SuspendLayout();
            _summaryFlow.Controls.Clear();
            _summaryMetricCards.Clear();
            foreach (LogMetric metric in summary.Metrics)
                _summaryFlow.Controls.Add(CreateMetricCard(metric));
            if (summary.Metrics.Count == 0) AddEmptySummaryMessage();
            _summaryFlow.ResumeLayout();
            ResizeSummaryCards();

            _issuesList.BeginUpdate();
            _issuesList.Items.Clear();
            foreach (LogAnomaly issue in summary.Anomalies
                .OrderByDescending(item => item.Verdict)
                .ThenBy(item => item.TimeSeconds))
            {
                var item = new ListViewItem($"{issue.Verdict}: {issue.Title}")
                {
                    ForeColor = VerdictColor(issue.Verdict),
                    ToolTipText = issue.Detail,
                };
                item.SubItems.Add(issue.Detail);
                _issuesList.Items.Add(item);
            }
            if (summary.Anomalies.Count == 0)
            {
                var item = new ListViewItem("No issues") { ForeColor = SUCCESS_COLOR };
                item.SubItems.Add("No configured rules were triggered.");
                _issuesList.Items.Add(item);
            }
            _issuesList.EndUpdate();
        }

        private Control CreateMetricCard(LogMetric metric)
        {
            var card = new TableLayoutPanel
            {
                Width = 228,
                Height = 82,
                ColumnCount = 1,
                RowCount = 3,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(12, 8, 12, 8),
                Margin = new Padding(4),
            };
            card.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            card.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            card.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            card.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            card.Controls.Add(new Label
            {
                Text = metric.Label.ToUpperInvariant(),
                AutoSize = true,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                Margin = new Padding(0),
            }, 0, 0);
            var value = new Label
            {
                Text = string.IsNullOrWhiteSpace(metric.Unit) ? metric.Value : $"{metric.Value} {metric.Unit}",
                Font = NOMADTheme.Font(NOMADTheme.SIZE_TITLE, FontStyle.Bold),
                ForeColor = VerdictColor(metric.Verdict),
                AutoSize = true,
                Margin = new Padding(0, 2, 0, 0),
            };
            var detail = new Label
            {
                Text = metric.Detail,
                Dock = DockStyle.Fill,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL),
                ForeColor = TEXT_SECONDARY,
                AutoEllipsis = true,
                TextAlign = ContentAlignment.MiddleLeft,
                Margin = new Padding(0),
            };
            card.Controls.Add(value, 0, 1);
            card.Controls.Add(detail, 0, 2);
            AddNomadBorder(card);
            _summaryMetricCards.Add(card);
            return card;
        }

        private void AddEmptySummaryMessage()
        {
            _emptySummaryLabel = new Label
            {
                Text = "Load a flight log or the sample flight to generate an automatic post-flight summary.",
                ForeColor = TEXT_SECONDARY,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_HEADING),
                AutoSize = false,
                Height = 80,
                TextAlign = ContentAlignment.MiddleCenter,
            };
            _summaryFlow.Controls.Add(_emptySummaryLabel);
        }

        private static void DrawIssueHeader(object sender, DrawListViewColumnHeaderEventArgs e)
        {
            using (var brush = new SolidBrush(NOMADTheme.BUTTON_BG))
                e.Graphics.FillRectangle(brush, e.Bounds);
            using (var font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL, FontStyle.Bold))
            {
                TextRenderer.DrawText(
                    e.Graphics,
                    e.Header.Text,
                    font,
                    e.Bounds,
                    NOMADTheme.TEXT_PRIMARY,
                    TextFormatFlags.Left | TextFormatFlags.VerticalCenter | TextFormatFlags.EndEllipsis);
            }
            using (var pen = new Pen(NOMADTheme.ACCENT))
                e.Graphics.DrawLine(pen, e.Bounds.Left, e.Bounds.Bottom - 1, e.Bounds.Right, e.Bounds.Bottom - 1);
        }

        private static void DrawIssueItem(object sender, DrawListViewItemEventArgs e)
        {
            if (((ListView)sender).View != View.Details)
                e.DrawDefault = true;
        }

        private static void DrawIssueSubItem(object sender, DrawListViewSubItemEventArgs e)
        {
            bool selected = e.Item.Selected;
            Color background = selected
                ? NOMADTheme.ACCENT
                : e.ItemIndex % 2 == 0 ? NOMADTheme.CARD_BG : NOMADTheme.PANEL_ALT;
            Color foreground = selected
                ? NOMADTheme.TEXT_PRIMARY
                : e.ColumnIndex == 0 ? e.Item.ForeColor : NOMADTheme.TEXT_SECONDARY;
            using (var brush = new SolidBrush(background))
                e.Graphics.FillRectangle(brush, e.Bounds);
            using (var font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL))
            {
                TextRenderer.DrawText(
                    e.Graphics,
                    e.SubItem.Text,
                    font,
                    e.Bounds,
                    foreground,
                    TextFormatFlags.Left | TextFormatFlags.VerticalCenter | TextFormatFlags.EndEllipsis);
            }
        }

        private Color VerdictColor(LogVerdict verdict)
        {
            switch (verdict)
            {
                case LogVerdict.Critical: return ERROR_COLOR;
                case LogVerdict.Warning: return WARNING_COLOR;
                case LogVerdict.Good: return SUCCESS_COLOR;
                default: return INFO_COLOR;
            }
        }
    }
}
