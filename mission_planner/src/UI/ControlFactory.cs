// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public static class ControlFactory
    {
        public static Button Button(string text, EventHandler onClick = null, Color? backColor = null, int width = 100, int height = 25)
        {
            var btn = new Button
            {
                Text = text,
                FlatStyle = FlatStyle.Flat,
                BackColor = backColor ?? NOMADTheme.BUTTON_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Size = new Size(width, height),
                FlatAppearance = { BorderSize = 0 },
            };
            if (onClick != null)
                btn.Click += onClick;
            return btn;
        }

        public static Button ActionButton(string text, EventHandler onClick, Color backColor, int width = 70, int height = 25)
        {
            var btn = Button(text, onClick, backColor, width, height);
            btn.FlatAppearance.BorderSize = 0;
            return btn;
        }

        public static Label Label(string text, bool bold = false, Color? foreColor = null, float fontSize = 10, ContentAlignment textAlign = ContentAlignment.MiddleLeft)
        {
            return new Label
            {
                Text = text,
                Font = new Font("Segoe UI", fontSize, bold ? FontStyle.Bold : FontStyle.Regular),
                ForeColor = foreColor ?? NOMADTheme.TEXT_PRIMARY,
                AutoSize = true,
                TextAlign = textAlign,
            };
        }

        public static Label StatusLabel(string initialValue, Color foreColor, float fontSize = 10)
        {
            return new Label
            {
                Text = initialValue,
                Font = new Font("Segoe UI", fontSize, FontStyle.Bold),
                ForeColor = foreColor,
                AutoSize = true,
            };
        }

        public static Label KvLabel(string key, string value, Color valueColor, Font valueFont)
        {
            return new Label
            {
                Text = value,
                Font = valueFont,
                ForeColor = valueColor,
                AutoSize = true,
                Margin = new Padding(0, 2, 0, 2),
            };
        }

        public static Panel CardPanel(Color? backColor = null, int padding = 10)
        {
            return new Panel
            {
                BackColor = backColor ?? NOMADTheme.CARD_BG,
                Padding = new Padding(padding),
                Margin = new Padding(5),
                Dock = DockStyle.Fill,
            };
        }

        public static Panel CardPanelWithBorder(Color? backColor = null, Color? borderColor = null)
        {
            var panel = CardPanel(backColor);
            var border = borderColor ?? NOMADTheme.CARD_BORDER;
            panel.Paint += (s, e) =>
            {
                using (var pen = new Pen(border, 1))
                    e.Graphics.DrawRectangle(pen, 0, 0, panel.Width - 1, panel.Height - 1);
            };
            return panel;
        }

        public static GroupBox GroupBox(string text, Color? foreColor = null, Color? backColor = null, int padding = 8)
        {
            return new GroupBox
            {
                Text = text,
                ForeColor = foreColor ?? NOMADTheme.ACCENT,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                BackColor = backColor ?? Color.FromArgb(40, 40, 43),
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                Padding = new Padding(padding, 4, padding, padding),
                Margin = new Padding(0, 0, 0, 6),
                Dock = DockStyle.Top,
            };
        }

        public static FlowLayoutPanel FlowPanel(params Control[] children)
        {
            var panel = new FlowLayoutPanel
            {
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                WrapContents = true,
                FlowDirection = FlowDirection.LeftToRight,
                Margin = new Padding(0),
                Padding = new Padding(0),
            };
            foreach (var child in children)
                panel.Controls.Add(child);
            return panel;
        }

        public static TableLayoutPanel Table(int columns, int rows, params Control[] controls)
        {
            var table = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = columns,
                RowCount = rows,
                BackColor = Color.Transparent,
                Padding = new Padding(0),
                Margin = new Padding(0),
            };
            for (int i = 0; i < columns; i++)
                table.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f / columns));
            return table;
        }

        public static ComboBox Combo(string[] items, int selectedIndex = 0)
        {
            var cb = new ComboBox
            {
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
            };
            cb.Items.AddRange(items);
            cb.SelectedIndex = selectedIndex;
            return cb;
        }

        public static TextBox ReadOnlyLog(Color? foreColor = null)
        {
            return new TextBox
            {
                Dock = DockStyle.Fill,
                Multiline = true,
                ScrollBars = ScrollBars.Vertical,
                ReadOnly = true,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = foreColor ?? Color.LightGreen,
                Font = new Font("Consolas", 8),
            };
        }

        public static ProgressBar Progress(int width = 150, int height = 18)
        {
            return new ProgressBar
            {
                Size = new Size(width, height),
                Maximum = 100,
                Style = ProgressBarStyle.Continuous,
            };
        }

        public static void AddSeparator(Control parent, int x, int y, int width)
        {
            parent.Controls.Add(new Label
            {
                BorderStyle = BorderStyle.Fixed3D,
                Location = new Point(x, y),
                Size = new Size(width, 2),
            });
        }
    }
}
