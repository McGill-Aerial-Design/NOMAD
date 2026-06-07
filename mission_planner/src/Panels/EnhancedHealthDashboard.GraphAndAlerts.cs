// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Enhanced Health Dashboard — Graph & Alerts
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class EnhancedHealthDashboard
    {
        // ============================================================
        // Graph Drawing
        // ============================================================

        private void DrawGraph()
        {
            _graphBox?.Invalidate();
        }

        private void GraphBox_Paint(object sender, PaintEventArgs e)
        {
            var g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;

            var fullRect = _graphBox.ClientRectangle;

            g.Clear(Color.FromArgb(20, 20, 20));

            int leftMargin = 40;
            int bottomMargin = 25;
            int topMargin = 5;
            int rightMargin = 5;

            var rect = new Rectangle(
                leftMargin, topMargin,
                fullRect.Width - leftMargin - rightMargin,
                fullRect.Height - topMargin - bottomMargin);

            Queue<float> data1, data2;
            Color color1, color2;
            string label1, label2;
            string yAxisLabel;

            switch (_cmbGraphType?.SelectedIndex ?? 0)
            {
                case 0:
                    data1 = _cpuTempHistory;
                    data2 = _gpuTempHistory;
                    color1 = Color.Orange;
                    color2 = Color.OrangeRed;
                    label1 = "CPU";
                    label2 = "GPU";
                    yAxisLabel = "\u00b0C";
                    break;
                case 1:
                    data1 = _cpuLoadHistory;
                    data2 = _gpuLoadHistory;
                    color1 = Color.DodgerBlue;
                    color2 = Color.LimeGreen;
                    label1 = "CPU";
                    label2 = "GPU";
                    yAxisLabel = "%";
                    break;
                default:
                    data1 = _memoryHistory;
                    data2 = _memoryHistory;
                    color1 = Color.MediumPurple;
                    color2 = Color.MediumPurple;
                    label1 = "Memory";
                    label2 = "";
                    yAxisLabel = "%";
                    break;
            }

            using (var gridPen = new Pen(Color.FromArgb(40, 40, 40)))
            using (var axisPen = new Pen(Color.FromArgb(80, 80, 80)))
            using (var axisFont = new Font("Segoe UI", 7))
            {
                for (int i = 0; i <= 4; i++)
                {
                    int y = rect.Top + rect.Height * i / 4;
                    g.DrawLine(gridPen, rect.Left, y, rect.Right, y);

                    int value = 100 - (i * 25);
                    string yText = $"{value}{yAxisLabel}";
                    var textSize = g.MeasureString(yText, axisFont);
                    g.DrawString(yText, axisFont, Brushes.Gray,
                        rect.Left - textSize.Width - 3, y - textSize.Height / 2);
                }

                int totalSeconds = HISTORY_LENGTH * (_config.HealthPollInterval / 1000);
                for (int i = 0; i <= 4; i++)
                {
                    int x = rect.Left + rect.Width * i / 4;
                    g.DrawLine(gridPen, x, rect.Top, x, rect.Bottom);

                    int secsAgo = totalSeconds - (totalSeconds * i / 4);
                    string xText = secsAgo == 0 ? "now" : $"-{secsAgo}s";
                    var textSize = g.MeasureString(xText, axisFont);
                    g.DrawString(xText, axisFont, Brushes.Gray,
                        x - textSize.Width / 2, rect.Bottom + 3);
                }

                g.DrawLine(axisPen, rect.Left, rect.Top, rect.Left, rect.Bottom);
                g.DrawLine(axisPen, rect.Left, rect.Bottom, rect.Right, rect.Bottom);
            }

            DrawGraphLine(g, rect, data1, color1, 100);
            if (_cmbGraphType?.SelectedIndex != 2)
            {
                DrawGraphLine(g, rect, data2, color2, 100);
            }

            using (var brush1 = new SolidBrush(color1))
            using (var brush2 = new SolidBrush(color2))
            using (var font = new Font("Segoe UI", 8))
            {
                g.FillRectangle(brush1, fullRect.Width - 80, 5, 10, 10);
                g.DrawString(label1, font, Brushes.White, fullRect.Width - 65, 3);

                if (!string.IsNullOrEmpty(label2))
                {
                    g.FillRectangle(brush2, fullRect.Width - 80, 20, 10, 10);
                    g.DrawString(label2, font, Brushes.White, fullRect.Width - 65, 18);
                }
            }
        }

        private void DrawGraphLine(Graphics g, Rectangle rect, Queue<float> data, Color color, float maxValue)
        {
            if (data.Count < 2) return;

            var values = data.ToArray();
            var points = new PointF[values.Length];

            for (int i = 0; i < values.Length; i++)
            {
                float x = rect.Left + rect.Width * i / (float)(HISTORY_LENGTH - 1);
                float y = rect.Top + rect.Height - (rect.Height * values[i] / maxValue);
                points[i] = new PointF(x, Math.Max(rect.Top, Math.Min(rect.Bottom, y)));
            }

            using (var pen = new Pen(color, 2))
            {
                g.DrawLines(pen, points);
            }
        }

        // ============================================================
        // Alerts
        // ============================================================

        private void CheckAlerts(float cpuTemp, float gpuTemp, float memory, float disk)
        {
            var timestamp = DateTime.Now.ToString("HH:mm:ss");

            if (cpuTemp > _config.TempCriticalC)
            {
                AddAlert($"[{timestamp}] CRITICAL: CPU Temp {cpuTemp:F1}\u00b0C");
            }
            else if (cpuTemp > _config.TempWarningC)
            {
                AddAlert($"[{timestamp}] WARNING: CPU Temp {cpuTemp:F1}\u00b0C");
            }

            if (gpuTemp > _config.TempCriticalC)
            {
                AddAlert($"[{timestamp}] CRITICAL: GPU Temp {gpuTemp:F1}\u00b0C");
            }
            else if (gpuTemp > _config.TempWarningC)
            {
                AddAlert($"[{timestamp}] WARNING: GPU Temp {gpuTemp:F1}\u00b0C");
            }

            if (memory > 95)
            {
                AddAlert($"[{timestamp}] CRITICAL: Memory at {memory:F0}%");
            }

            if (disk > 95)
            {
                AddAlert($"[{timestamp}] WARNING: Disk at {disk:F0}%");
            }
        }

        private void AddAlert(string alert)
        {
            if (_alerts.Count > 0 && _alerts[_alerts.Count - 1].Contains(alert.Substring(alert.IndexOf(']') + 1)))
                return;

            _alerts.Add(alert);
            _lstAlerts.Items.Add(alert);

            while (_alerts.Count > 100)
            {
                _alerts.RemoveAt(0);
                _lstAlerts.Items.RemoveAt(0);
            }

            _lstAlerts.TopIndex = _lstAlerts.Items.Count - 1;
        }
    }
}
