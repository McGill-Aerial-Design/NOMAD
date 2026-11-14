// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using Form = System.Windows.Forms.Form;
using MessageBox = System.Windows.Forms.MessageBox;
using DialogResult = System.Windows.Forms.DialogResult;
using FormStartPosition = System.Windows.Forms.FormStartPosition;
using MessageBoxButtons = System.Windows.Forms.MessageBoxButtons;
using MessageBoxIcon = System.Windows.Forms.MessageBoxIcon;
using TabPage = System.Windows.Controls.TabItem;
using TextBox = System.Windows.Controls.TextBox;
using CheckBox = System.Windows.Controls.CheckBox;
using ComboBox = System.Windows.Controls.ComboBox;
using Label = System.Windows.Controls.TextBlock;
using Button = System.Windows.Controls.Button;
using NumericUpDown = NOMAD.MissionPlanner.WpfNumberBox;

namespace NOMAD.MissionPlanner
{
    [global::MissionPlanner.Utilities.PreventTheming]
    public partial class NOMADSettingsForm : Form
    {
        private TabControl _tabControl;
        private TextBox _txtVideoUrl;
        private CheckBox _chkAutoStartHudVideo, _chkRouterClientEnabled, _chkDebugMode, _chkAudioAlerts, _chkAltitudeCallouts;
        private NumericUpDown _numRouterLocalPort, _numJoyGimbalMaxRate;
        private Label _profileContext;

        public NOMADProfiles Profiles { get; private set; }
        public NOMADConfig Config { get => Profiles.Selected.Config; private set => Profiles.Selected.Config = value; }

        public NOMADSettingsForm(NOMADProfiles profiles)
        {
            Profiles = (profiles ?? new NOMADProfiles(null)).Clone();
            InitializeComponents();
            LoadSettings();
        }

        private void InitializeComponents()
        {
            Text = "NOMAD Settings"; ClientSize = new System.Drawing.Size(780, 670);
            BackColor = NOMADTheme.BG_DARK; ForeColor = NOMADTheme.TEXT_PRIMARY; Font = NOMADTheme.Font();
            MinimumSize = new System.Drawing.Size(760, 600);
            StartPosition = FormStartPosition.CenterParent;
            var root = new DockPanel { Margin = new Thickness(12) };
            var context = new DockPanel { Margin = new Thickness(0, 0, 0, 8) };
            DockPanel.SetDock(context, System.Windows.Controls.Dock.Top);
            var hint = WpfUi.Text("· Save activates this profile");
            DockPanel.SetDock(hint, System.Windows.Controls.Dock.Right); context.Children.Add(hint);
            _profileContext = WpfUi.Section(""); _profileContext.Margin = new Thickness(0); context.Children.Add(_profileContext);
            root.Children.Add(context);
            var footer = new DockPanel(); DockPanel.SetDock(footer, System.Windows.Controls.Dock.Bottom);
            var actions = WpfUi.Action("Profile actions ▾", () => { });
            var menu = new ContextMenu();
            var reset = new MenuItem { Header = "Reset profile…" }; reset.Click += BtnReset_Click; menu.Items.Add(reset);
            menu.Items.Add(new Separator());
            var load = new MenuItem { Header = "Load configuration…" }; load.Click += BtnLoadConfig_Click; menu.Items.Add(load);
            var export = new MenuItem { Header = "Export configuration…" }; export.Click += BtnExportConfig_Click; menu.Items.Add(export);
            actions.ContextMenu = menu; actions.Click += (sender, args) => { menu.PlacementTarget = actions; menu.IsOpen = true; };
            DockPanel.SetDock(actions, System.Windows.Controls.Dock.Left); footer.Children.Add(actions);
            var buttons = new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Right };
            var save = WpfUi.Action("Save", () =>
            {
                try { SaveSettings(); DialogResult = DialogResult.OK; }
                catch (Exception error)
                {
                    DialogResult = DialogResult.None;
                    MessageBox.Show(this, error.Message, "Configuration requires review", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                }
            });
            save.IsDefault = true; buttons.Children.Add(save);
            var cancel = WpfUi.Action("Cancel", () => { DialogResult = DialogResult.Cancel; Close(); });
            cancel.IsCancel = true; buttons.Children.Add(cancel);
            footer.Children.Add(buttons); root.Children.Add(footer);
            _tabControl = new TabControl(); root.Children.Add(_tabControl);
            _tabControl.Items.Add(CreateAircraftTab()); _tabControl.Items.Add(CreateMultiLinkTab());
            _tabControl.Items.Add(CreateVideoTab()); _tabControl.Items.Add(CreateActuatorsTab());
            _tabControl.Items.Add(CreateJoystickTab()); _tabControl.Items.Add(CreatePreferencesTab());
            root.PreviewKeyDown += (sender, args) => { if (args.Key == System.Windows.Input.Key.Escape)
                { DialogResult = DialogResult.Cancel; Close(); args.Handled = true; } };
            Controls.Add(WpfUi.Host(root));
        }

        private void UpdateProfileContext()
        {
            string name = string.IsNullOrWhiteSpace(_txtAircraftName.Text) ? "(unnamed aircraft)" : _txtAircraftName.Text.Trim();
            _profileContext.Text = "Editing: " + name; _profileContext.ToolTip = _profileContext.Text;
        }

        private TabPage CreateTabPage(string text)
        {
            var canvas = new Canvas { MinWidth = 720, MinHeight = 480 };
            return new TabPage { Header = text, Content = new ScrollViewer { Content = canvas,
                VerticalScrollBarVisibility = ScrollBarVisibility.Auto, HorizontalScrollBarVisibility = ScrollBarVisibility.Auto } };
        }

        private static void AddAt(TabPage tab, FrameworkElement control, double x, double y,
            double width = double.NaN, double height = double.NaN)
        {
            var canvas = (Canvas)((ScrollViewer)tab.Content).Content;
            Canvas.SetLeft(control, x); Canvas.SetTop(control, y);
            control.Margin = new Thickness(0);
            if (!double.IsNaN(width)) { control.Width = width; }
            if (!double.IsNaN(height)) { control.Height = height; }
            canvas.MinWidth = Math.Max(canvas.MinWidth, x + (double.IsNaN(width) ? 150 : width) + 15);
            canvas.MinHeight = Math.Max(canvas.MinHeight, y + (double.IsNaN(height) ? 36 : height) + 15);
            canvas.Children.Add(control);
        }

        private static int SelectedRow(DataGrid list) => list.SelectedIndex;
        private static DataGrid CreateDefinitionList(params string[] columns)
        {
            var list = new DataGrid { IsReadOnly = true, AutoGenerateColumns = false, CanUserAddRows = false,
                CanUserDeleteRows = false, CanUserSortColumns = false, SelectionMode = DataGridSelectionMode.Single,
                SelectionUnit = DataGridSelectionUnit.FullRow, HeadersVisibility = DataGridHeadersVisibility.Column };
            var cellStyle = new Style(typeof(TextBlock));
            cellStyle.Setters.Add(new Setter(TextBlock.TextWrappingProperty, TextWrapping.Wrap));
            for (int i = 0; i < columns.Length; i++)
            { list.Columns.Add(new DataGridTextColumn { Header = columns[i], Binding = new Binding("[" + i + "]"),
                ElementStyle = cellStyle, Width = new DataGridLength(1, DataGridLengthUnitType.Star) }); }
            return list;
        }
        private static void AddDefinitionRow(DataGrid list, params object[] values) => list.Items.Add(values);

        private void AddSectionLabel(TabPage tab, string text, ref int y)
        { var label = WpfUi.Section(text); label.Margin = new Thickness(0); AddAt(tab, label, 10, y, 650); y += 25; }
        private Label AddLabel(TabPage tab, string text, int x, int y, System.Drawing.Color? color = null)
        { var label = WpfUi.Text(text); if (color.HasValue) { label.Foreground = ColorBrush(color.Value); }
            AddAt(tab, label, x, y + 3); return label; }
        private TextBox AddTextBox(TabPage tab, int x, int y, int width)
        { var field = new TextBox(); AddAt(tab, field, x, y, width, 26); return field; }
        private NumericUpDown AddNumericUpDown(TabPage tab, int x, int y, int width, decimal min, decimal max,
            decimal value, int decimals = 0)
        {
            var field = new NumericUpDown { Minimum = min, Maximum = max, DecimalPlaces = decimals,
                Value = Math.Max(min, Math.Min(max, value)) };
            AddAt(tab, field, x, y, width, 26); return field;
        }
        private CheckBox AddCheckBox(TabPage tab, string text, int x, int y, System.Drawing.Color? color = null)
        { var field = new CheckBox { Content = text }; if (color.HasValue) { field.Foreground = ColorBrush(color.Value); }
            AddAt(tab, field, x, y); return field; }
        private static System.Windows.Media.Brush ColorBrush(System.Drawing.Color color) =>
            new System.Windows.Media.SolidColorBrush(System.Windows.Media.Color.FromArgb(color.A, color.R, color.G, color.B));
        private ComboBox AddComboBox(TabPage tab, int x, int y, int width, string[] items)
        {
            var field = new ComboBox(); foreach (string item in items) { field.Items.Add(item); }
            if (field.Items.Count > 0) { field.SelectedIndex = 0; }
            AddAt(tab, field, x, y, width); return field;
        }

        private void BtnReset_Click(object sender, EventArgs e)
        {
            if (MessageBox.Show("Reset the selected aircraft's settings? Other profiles will stay available.",
                "Reset Settings", MessageBoxButtons.YesNo, MessageBoxIcon.Warning) != DialogResult.Yes) { return; }
            Config = new NOMADConfig { AircraftName = Config.AircraftName, ActuatorDefinitions = new Newtonsoft.Json.Linq.JArray() };
            LoadSettings();
        }
    }
}
