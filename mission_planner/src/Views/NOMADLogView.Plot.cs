// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADLogView
    {
        private TreeView _fieldTree;
        private TimeSeriesPlot _offlinePlot;
        private CheckBox _normalizePlot;

        private sealed class PlotField
        {
            public string MessageType;
            public string Field;
        }

        private Control CreatePlotArea()
        {
            _plotSplit = new SplitContainer
            {
                Dock = DockStyle.Fill,
                Orientation = Orientation.Vertical,
                SplitterDistance = 250,
                BackColor = NOMADTheme.BG_DARK,
                SplitterWidth = 5,
            };

            var left = new Panel { Dock = DockStyle.Fill, BackColor = CARD_BG };
            var title = new Label
            {
                Text = "LOG FIELDS",
                Dock = DockStyle.Top,
                Height = 30,
                Padding = new Padding(8, 7, 0, 0),
                Font = NOMADTheme.Font(NOMADTheme.SIZE_BODY, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
            };
            _fieldTree = new TreeView
            {
                Dock = DockStyle.Fill,
                CheckBoxes = true,
                BackColor = CARD_BG,
                ForeColor = TEXT_PRIMARY,
                BorderStyle = BorderStyle.None,
                LineColor = NOMADTheme.CARD_BORDER,
            };
            _fieldTree.AfterCheck += FieldTree_AfterCheck;
            left.Controls.Add(_fieldTree);
            left.Controls.Add(title);

            var right = new Panel { Dock = DockStyle.Fill, BackColor = NOMADTheme.BG_DARK };
            _plotToolbar = new FlowLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                FlowDirection = FlowDirection.LeftToRight,
                WrapContents = true,
                BackColor = NOMADTheme.BG_DARK,
            };
            _normalizePlot = new CheckBox
            {
                Text = "Normalized overlay",
                AutoSize = true,
                ForeColor = TEXT_SECONDARY,
                Font = NOMADTheme.Font(),
                Margin = new Padding(5, 7, 10, 0),
            };
            _normalizePlot.CheckedChanged += (s, e) =>
            {
                _offlinePlot.Normalized = _normalizePlot.Checked;
                _offlinePlot.Invalidate();
            };
            var reset = SmallButton("Reset Zoom", NOMADTheme.BUTTON_BG);
            reset.Width = 100;
            reset.Height = 28;
            reset.Click += (s, e) => _offlinePlot.ResetView();
            var clear = SmallButton("Clear", NOMADTheme.BUTTON_BG);
            clear.Width = 75;
            clear.Height = 28;
            clear.Click += (s, e) =>
            {
                _offlinePlot.ClearSeries();
                SetTreeChecks(_fieldTree.Nodes, false);
            };
            _plotToolbar.Controls.Add(_normalizePlot);
            _plotToolbar.Controls.Add(reset);
            _plotToolbar.Controls.Add(clear);

            _offlinePlot = new TimeSeriesPlot
            {
                Dock = DockStyle.Fill,
            };
            right.Controls.Add(_offlinePlot);
            right.Controls.Add(_plotToolbar);
            _plotSplit.Panel1.Controls.Add(left);
            _plotSplit.Panel2.Controls.Add(right);
            return _plotSplit;
        }

        private void PopulateFieldTree(IFlightLogData data)
        {
            _fieldTree.BeginUpdate();
            _fieldTree.Nodes.Clear();
            foreach (string type in data.MessageTypes.OrderBy(type => type))
            {
                var typeNode = new TreeNode(type);
                foreach (string field in data.Fields(type))
                {
                    typeNode.Nodes.Add(new TreeNode(field)
                    {
                        Tag = new PlotField { MessageType = type, Field = field },
                    });
                }
                if (typeNode.Nodes.Count > 0) _fieldTree.Nodes.Add(typeNode);
            }
            _fieldTree.EndUpdate();
        }

        private async void FieldTree_AfterCheck(object sender, TreeViewEventArgs e)
        {
            if (!(e.Node.Tag is PlotField field)) return;
            string key = $"{field.MessageType}.{field.Field}";
            if (!e.Node.Checked)
            {
                _offlinePlot.RemoveSeries(key);
                return;
            }

            IFlightLogData data = _logData;
            int generation = _loadGeneration;
            try
            {
                _loadStatus.Text = $"Loading {key}...";
                TimeSeriesData series = await Task.Run(() => BuildSeries(data, field, key))
                    .ConfigureAwait(false);
                if (generation != _loadGeneration || data != _logData) return;
                await RunOnUiThreadAsync(() =>
                {
                    if (generation != _loadGeneration || data != _logData)
                        return;
                    series.Color = _offlinePlot.NextColor;
                    _offlinePlot.SetSeries(series);
                    _loadStatus.Text = $"{key}: {series.Points.Count} samples.";
                }).ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                await RunOnUiThreadAsync(() =>
                {
                    e.Node.Checked = false;
                    _loadStatus.Text = $"Could not plot {key}: {ex.Message}";
                }).ConfigureAwait(false);
            }
        }

        private static TimeSeriesData BuildSeries(IFlightLogData data, PlotField field, string key)
        {
            const int maxPlotPoints = 25000;
            var series = new TimeSeriesData
            {
                Key = key,
                Label = key,
                Unit = data.Unit(field.MessageType, field.Field),
            };
            int rowIndex = 0;
            int sampleEvery = 1;
            foreach (LogRecord row in data.Records(field.MessageType))
            {
                rowIndex++;
                if (rowIndex % sampleEvery != 0) continue;
                if (row.TryGetDouble(out double value, field.Field))
                    series.Points.Add(new TimeSeriesPoint { Time = row.TimeSeconds, Value = value });
                if (series.Points.Count <= maxPlotPoints) continue;

                for (int read = 0, write = 0; read < series.Points.Count; read += 2, write++)
                    series.Points[write] = series.Points[read];
                series.Points.RemoveRange(
                    (series.Points.Count + 1) / 2,
                    series.Points.Count / 2);
                sampleEvery *= 2;
            }
            return series;
        }

        private static void SetTreeChecks(TreeNodeCollection nodes, bool value)
        {
            foreach (TreeNode node in nodes)
            {
                if (node.Tag is PlotField) node.Checked = value;
                if (node.Nodes.Count > 0) SetTreeChecks(node.Nodes, value);
            }
        }
    }
}
