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
                Font = NOMADTheme.Font(fontSize, bold ? FontStyle.Bold : FontStyle.Regular),
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
                Font = NOMADTheme.Font(fontSize, FontStyle.Bold),
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
                Font = NOMADTheme.Font(NOMADTheme.SIZE_BODY, FontStyle.Bold),
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
                Font = NOMADTheme.Mono(NOMADTheme.SIZE_SMALL),
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

        // ============================================================
        // Responsive builders (replace absolute-positioned layouts)
        // ============================================================
        // These produce controls that reflow with their container instead of
        // sitting at hardcoded coordinates, so panels fit any aspect ratio and
        // never overlap. Used by the view-level relayout work.

        /// <summary>Accent-colored card/section heading label.</summary>
        public static Label SectionTitle(string text, float fontSize = NOMADTheme.SIZE_HEADING)
        {
            return new Label
            {
                Text = text,
                Font = NOMADTheme.Font(fontSize, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                AutoSize = true,
                Margin = new Padding(0, 0, 0, NOMADTheme.GAP),
            };
        }

        /// <summary>
        /// A themed card that AutoSizes to its content (drop into a Dock=Top stack)
        /// or fills a parent table cell. Add your controls to <paramref name="body"/>,
        /// which is a single-column table that grows with whatever you put in it.
        /// </summary>
        public static TableLayoutPanel Card(string title, out TableLayoutPanel body)
        {
            var card = new TableLayoutPanel
            {
                ColumnCount = 1,
                RowCount = 2,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(NOMADTheme.PAD),
                Margin = new Padding(0, 0, 0, NOMADTheme.GAP),
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                Dock = DockStyle.Top,
            };
            card.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            card.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            card.RowStyles.Add(new RowStyle(SizeType.AutoSize));

            if (!string.IsNullOrEmpty(title))
                card.Controls.Add(SectionTitle(title), 0, 0);

            body = new TableLayoutPanel
            {
                ColumnCount = 1,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                BackColor = Color.Transparent,
                Margin = new Padding(0),
                Dock = DockStyle.Top,
            };
            body.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            card.Controls.Add(body, 0, 1);
            return card;
        }

        /// <summary>
        /// A reflowing "Label: [control] unit" row. The control stretches to fill
        /// remaining width; the label and optional unit hug their text.
        /// </summary>
        public static TableLayoutPanel LabeledRow(string labelText, Control input, string unit = null)
        {
            var row = new TableLayoutPanel
            {
                ColumnCount = unit == null ? 2 : 3,
                RowCount = 1,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                BackColor = Color.Transparent,
                Margin = new Padding(0, 0, 0, NOMADTheme.GAP),
                Dock = DockStyle.Top,
            };
            row.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            row.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            if (unit != null)
                row.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));

            var lbl = Label(labelText);
            lbl.Anchor = AnchorStyles.Left;
            lbl.Margin = new Padding(0, 0, NOMADTheme.GAP, 0);
            row.Controls.Add(lbl, 0, 0);

            input.Dock = DockStyle.Fill;
            input.Margin = new Padding(0);
            row.Controls.Add(input, 1, 0);

            if (unit != null)
            {
                var unitLbl = Label(unit, foreColor: NOMADTheme.TEXT_SECONDARY, fontSize: NOMADTheme.SIZE_SMALL);
                unitLbl.Anchor = AnchorStyles.Left;
                unitLbl.Margin = new Padding(NOMADTheme.GAP, 0, 0, 0);
                row.Controls.Add(unitLbl, 2, 0);
            }
            return row;
        }

        /// <summary>A row of buttons/controls that wraps when the width is tight.</summary>
        public static FlowLayoutPanel ButtonRow(params Control[] children)
        {
            var row = FlowPanel(children);
            row.Dock = DockStyle.Top;
            row.Margin = new Padding(0, 0, 0, NOMADTheme.GAP);
            return row;
        }

        public static CheckBox CheckBox(string text, bool isChecked = false, float fontSize = NOMADTheme.SIZE_BODY)
        {
            return new CheckBox
            {
                Text = text,
                Checked = isChecked,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = NOMADTheme.Font(fontSize),
                AutoSize = true,
                Margin = new Padding(0, 2, 0, 2),
            };
        }

        public static NumericUpDown Numeric(decimal min, decimal max, decimal value, decimal increment = 1, int decimals = 0, int width = 60)
        {
            return new NumericUpDown
            {
                Minimum = min,
                Maximum = max,
                Value = value < min ? min : value > max ? max : value,
                Increment = increment,
                DecimalPlaces = decimals,
                BackColor = NOMADTheme.CONTROL_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = NOMADTheme.Font(),
                Width = width,
            };
        }

        public static TabControl TabControl(float fontSize = 10f)
        {
            return new TabControl
            {
                Dock = DockStyle.Fill,
                Font = NOMADTheme.Font(fontSize),
            };
        }

        public static ListBox ListBox(bool mono = true)
        {
            return new ListBox
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(25, 25, 25),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = mono ? NOMADTheme.Mono(NOMADTheme.SIZE_BODY) : NOMADTheme.Font(),
                BorderStyle = BorderStyle.None,
                IntegralHeight = false,
            };
        }

        /// <summary>
        /// Dark themed two-column lat/lon grid (the boundary editors' grid skin).
        /// Columns auto-fill so the grid reflows with its card.
        /// </summary>
        public static DataGridView BoundaryGrid()
        {
            var dgv = new DataGridView
            {
                Dock = DockStyle.Fill,
                AllowUserToAddRows = false,
                AllowUserToDeleteRows = true,
                AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill,
                BackgroundColor = Color.FromArgb(30, 30, 30),
                BorderStyle = BorderStyle.None,
                CellBorderStyle = DataGridViewCellBorderStyle.SingleHorizontal,
                ColumnHeadersHeightSizeMode = DataGridViewColumnHeadersHeightSizeMode.AutoSize,
                DefaultCellStyle = new DataGridViewCellStyle
                {
                    BackColor = Color.FromArgb(40, 40, 43),
                    ForeColor = NOMADTheme.TEXT_PRIMARY,
                    SelectionBackColor = NOMADTheme.ACCENT,
                    SelectionForeColor = NOMADTheme.TEXT_PRIMARY,
                },
                EnableHeadersVisualStyles = false,
                GridColor = Color.FromArgb(60, 60, 63),
                RowHeadersVisible = false,
                SelectionMode = DataGridViewSelectionMode.FullRowSelect,
            };

            dgv.Columns.Add(new DataGridViewTextBoxColumn { Name = "Lat", HeaderText = "Latitude", FillWeight = 50 });
            dgv.Columns.Add(new DataGridViewTextBoxColumn { Name = "Lon", HeaderText = "Longitude", FillWeight = 50 });

            dgv.ColumnHeadersDefaultCellStyle = new DataGridViewCellStyle
            {
                BackColor = NOMADTheme.CONTROL_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_BODY, FontStyle.Bold),
            };

            return dgv;
        }
    }
}
