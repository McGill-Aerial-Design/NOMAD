// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Settings Form
// ============================================================
// Comprehensive configuration dialog for NOMAD plugin settings.
// Includes all settings from NOMADConfig organized in tab pages.
// ============================================================

using System;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm : Form
    {
        // ============================================================
        // Fields
        // ============================================================

        private TabControl _tabControl;

        // Connection Tab

        // Video Tab
        private TextBox _txtVideoUrl;
        private CheckBox _chkAutoStartHudVideo;

        // Multi-Link Tab
        private CheckBox _chkRouterClientEnabled;
        private NumericUpDown _numRouterLocalPort;

        // UI Tab
        private CheckBox _chkDebugMode;

        // Alerts Tab
        private CheckBox _chkAudioAlerts;
        private CheckBox _chkAltitudeCallouts;

        // Log Analysis Tab


        // Joystick Tab
        private CheckBox _chkJoyGimbalEnabled, _chkJoyPositionEnabled;
        private CheckBox _chkJoyGimbalPitchInvert, _chkJoyGimbalRollInvert, _chkJoyPositionInvert;
        private ComboBox _cmbJoyGimbalDevice, _cmbJoyPositionDevice;
        private ComboBox _cmbJoyGimbalPitchAxis, _cmbJoyGimbalRollAxis, _cmbJoyPositionAxis;
        private NumericUpDown _numJoyGimbalDeadzone, _numJoyPositionDeadzone;
        private NumericUpDown _numJoyGimbalMaxRate;
        private ComboBox _cmbJoyPositionPayload;
        private readonly NumericUpDown[] _switchButtons = new NumericUpDown[6];
        private Button _btnJoyRefreshDevices;
        private Label _lblJoyStatus;
        private Label _lblJoyPositionEligibility;
        // 3-position switch action mapping (6 slots: sw1/2/3 x up/down)
        private ComboBox _cmbSw1Up, _cmbSw1Down, _cmbSw2Up, _cmbSw2Down, _cmbSw3Up, _cmbSw3Down;
        private ComboBox _cmbSwitchDevice;

        // Buttons
        private Button _btnOK;
        private Button _btnCancel;
        private Label _profileContext;
        private readonly ToolTip _toolTips = new ToolTip { AutoPopDelay = 15000 };
        private readonly ContextMenuStrip _profileActions = new ContextMenuStrip();


        public NOMADProfiles Profiles { get; private set; }
        public NOMADConfig Config
        {
            get => Profiles.Selected.Config;
            private set => Profiles.Selected.Config = value;
        }

        // ============================================================
        // Constructor
        // ============================================================

        public NOMADSettingsForm(NOMADProfiles profiles)
        {
            Profiles = (profiles ?? new NOMADProfiles(null)).Clone();
            InitializeComponents();
            LoadSettings();
        }

        // ============================================================
        // UI Initialization
        // ============================================================

        private void InitializeComponents()
        {
            Text = "NOMAD Settings";
            ClientSize = new Size(780, 670);
            FormBorderStyle = FormBorderStyle.FixedDialog;
            MaximizeBox = false;
            MinimizeBox = false;
            StartPosition = FormStartPosition.CenterParent;
            BackColor = NOMADTheme.BG_DARK;
            ForeColor = NOMADTheme.TEXT_PRIMARY;
            Font = NOMADTheme.Font();

            _tabControl = new TabControl
            {
                Location = new Point(12, 58), Size = new Size(756, 540), BackColor = NOMADTheme.CARD_BG,
            };
            Controls.Add(_tabControl);
            _tabControl.TabPages.Add(CreateAircraftTab());
            _tabControl.TabPages.Add(CreateMultiLinkTab());
            _tabControl.TabPages.Add(CreateVideoTab());
            _tabControl.TabPages.Add(CreateActuatorsTab());
            _tabControl.TabPages.Add(CreateJoystickTab());
            _tabControl.TabPages.Add(CreatePreferencesTab());

            int clientWidth = CalculateSettingsClientWidth();
            ClientSize = new Size(clientWidth, 670);
            _tabControl.Width = clientWidth - 24;
            var context = new TableLayoutPanel
            {
                Location = new Point(12, 12), Size = new Size(clientWidth - 24, 32), ColumnCount = 2, RowCount = 1,
            };
            context.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            context.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            _profileContext = new Label
            {
                Dock = DockStyle.Fill, AutoEllipsis = true, TextAlign = ContentAlignment.MiddleLeft,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_BODY, FontStyle.Bold),
            };
            context.Controls.Add(_profileContext, 0, 0);
            context.Controls.Add(new Label
            {
                Text = "· Save activates this profile", AutoSize = true, Anchor = AnchorStyles.Right,
                ForeColor = NOMADTheme.TEXT_SECONDARY,
            }, 1, 0);
            Controls.Add(context);

            var actions = ControlFactory.Button("Profile actions ▾", width: 150, height: 34);
            actions.Location = new Point(12, 616);
            _profileActions.Items.Add(new ToolStripMenuItem("Reset profile…", null, BtnReset_Click));
            _profileActions.Items.Add(new ToolStripSeparator());
            _profileActions.Items.Add(new ToolStripMenuItem("Load configuration…", null, BtnLoadConfig_Click));
            _profileActions.Items.Add(new ToolStripMenuItem("Export configuration…", null, BtnExportConfig_Click));
            actions.Click += (sender, args) => _profileActions.Show(actions, new Point(0, actions.Height));
            Controls.Add(actions);

            _btnOK = ControlFactory.Button("Save", backColor: NOMADTheme.BTN_PRIMARY, width: 90, height: 34);
            _btnOK.Location = new Point(clientWidth - 204, 616);
            _btnOK.Click += (sender, args) =>
            {
                try { SaveSettings(); DialogResult = DialogResult.OK; }
                catch (Exception error)
                {
                    DialogResult = DialogResult.None;
                    MessageBox.Show(this, error.Message, "Configuration requires review", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                }
            };
            Controls.Add(_btnOK);
            _btnCancel = ControlFactory.Button("Cancel", width: 90, height: 34);
            _btnCancel.Location = new Point(clientWidth - 102, 616);
            _btnCancel.DialogResult = DialogResult.Cancel;
            Controls.Add(_btnCancel);
            AcceptButton = _btnOK;
            CancelButton = _btnCancel;
        }

        private void UpdateProfileContext()
        {
            string name = string.IsNullOrWhiteSpace(_txtAircraftName.Text) ? "(unnamed aircraft)" : _txtAircraftName.Text.Trim();
            _profileContext.Text = "Editing: " + name;
            _toolTips.SetToolTip(_profileContext, _profileContext.Text);
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _toolTips.Dispose();
                _profileActions.Dispose();
            }
            base.Dispose(disposing);
        }
        private int CalculateSettingsClientWidth()
        {
            int widestContent = _tabControl.TabPages
                .Cast<TabPage>()
                .SelectMany(tab => tab.Controls.Cast<Control>())
                .Select(control => control.Right)
                .DefaultIfEmpty(700)
                .Max();

            int tabHeadersWidth = _tabControl.TabPages
                .Cast<TabPage>()
                .Sum(tab => TextRenderer.MeasureText(tab.Text, _tabControl.Font).Width + 24);

            return Math.Max(760, Math.Max(widestContent + 45, tabHeadersWidth + 35));
        }

        // ============================================================
        // Helper Methods
        // ============================================================

        private TabPage CreateTabPage(string text)
        {
            var tab = new TabPage(text)
            {
                BackColor = NOMADTheme.CARD_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                AutoScroll = true,
                Padding = new Padding(5)
            };
            return tab;
        }

        private static int SelectedRow(DataGridView list) => list.CurrentRow?.Index ?? -1;
        private static DataGridView CreateDefinitionList(params string[] columns)
        {
            var list = new DataGridView
            {
                ReadOnly = true, AllowUserToAddRows = false, AllowUserToDeleteRows = false,
                AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill, RowHeadersVisible = false,
                MultiSelect = false, SelectionMode = DataGridViewSelectionMode.FullRowSelect,
                BackgroundColor = NOMADTheme.INPUT_BG, EnableHeadersVisualStyles = false,
                DefaultCellStyle = new DataGridViewCellStyle { BackColor = NOMADTheme.INPUT_BG,
                    ForeColor = NOMADTheme.TEXT_PRIMARY, SelectionBackColor = NOMADTheme.ACCENT },
                ColumnHeadersDefaultCellStyle = new DataGridViewCellStyle { BackColor = NOMADTheme.BUTTON_BG,
                    ForeColor = NOMADTheme.TEXT_PRIMARY },
            };
            foreach (string column in columns)
            {
                list.Columns.Add(column, column);
                list.Columns[column].SortMode = DataGridViewColumnSortMode.NotSortable;
            }
            return list;
        }

        private void AddSectionLabel(TabPage tab, string text, ref int y)
        {
            var label = new Label
            {
                Text = text,
                Location = new Point(10, y),
                AutoSize = true,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_BODY, FontStyle.Bold)
            };
            tab.Controls.Add(label);
            y += 25;
        }

        private Label AddLabel(TabPage tab, string text, int x, int y, Color? color = null)
        {
            var label = new Label
            {
                Text = text,
                Location = new Point(x, y + 3),
                AutoSize = true,
                ForeColor = color ?? NOMADTheme.TEXT_SECONDARY
            };
            tab.Controls.Add(label);
            return label;
        }

        private TextBox AddTextBox(TabPage tab, int x, int y, int width)
        {
            var textBox = new TextBox
            {
                Location = new Point(x, y),
                Size = new Size(width, 23),
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY
            };
            tab.Controls.Add(textBox);
            return textBox;
        }

        private NumericUpDown AddNumericUpDown(
            TabPage tab, int x, int y, int width, decimal min, decimal max, decimal value, int decimals = 0)
        {
            var num = new NumericUpDown
            {
                Location = new Point(x, y),
                Size = new Size(width, 23),
                Minimum = min,
                Maximum = max,
                Value = Math.Max(min, Math.Min(max, value)),
                DecimalPlaces = decimals,
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY
            };
            tab.Controls.Add(num);
            return num;
        }

        private CheckBox AddCheckBox(TabPage tab, string text, int x, int y, Color? color = null)
        {
            var checkBox = new CheckBox
            {
                Text = text,
                Location = new Point(x, y),
                AutoSize = true,
                ForeColor = color ?? NOMADTheme.TEXT_PRIMARY
            };
            tab.Controls.Add(checkBox);
            return checkBox;
        }

        private ComboBox AddComboBox(TabPage tab, int x, int y, int width, string[] items)
        {
            var combo = new ComboBox
            {
                Location = new Point(x, y),
                Size = new Size(width, 23),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY
            };
            combo.Items.AddRange(items);
            if (combo.Items.Count > 0)
                combo.SelectedIndex = 0;
            tab.Controls.Add(combo);
            return combo;
        }

        // LoadSettings/SaveSettings live in NOMADSettingsForm.Settings.cs.

        private void BtnReset_Click(object sender, EventArgs e)
        {
            var result = MessageBox.Show(
                "Reset the selected aircraft's settings? Other profiles will stay available.",
                "Reset Settings",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning
            );

            if (result == DialogResult.Yes)
            {
                Config = new NOMADConfig
                {
                    AircraftName = Config.AircraftName,
                    ActuatorDefinitions = new Newtonsoft.Json.Linq.JArray(),
                };
                LoadSettings();
            }
        }

    }
}
