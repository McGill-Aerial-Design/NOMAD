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
using System.IO.Ports;
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
        private NumericUpDown _numCoreRuntimePort;
        private TextBox _txtCoreClientCredential;

        // Video Tab
        private TextBox _txtVideoUrl;
        private NumericUpDown _numVideoCaching;
        private ComboBox _cmbVideoPlayer;
        private CheckBox _chkVideoAutoStart;
        private CheckBox _chkAutoStartHudVideo;

        // Dual Link Tab
        private CheckBox _chkRouterClientEnabled;
        private NumericUpDown _numRouterLocalPort;
        private NumericUpDown _numManagementPort;

        // UI Tab
        private CheckBox _chkDebugMode;
        private CheckBox _chkShowNotifications;
        private ComboBox _cmbDefaultTab;
        private CheckBox _chkDarkMode;
        private NumericUpDown _numSlamFov;
        private NumericUpDown _numSlamMapRadius;

        // Alerts Tab
        private NumericUpDown _numTempWarning;
        private NumericUpDown _numTempCritical;
        private CheckBox _chkAudioAlerts;
        private CheckBox _chkAltitudeCallouts;

        // Log Analysis Tab
        private TextBox _txtDefaultLogDirectory;
        private NumericUpDown _numLogVibrationWarning;
        private NumericUpDown _numLogVibrationCritical;
        private NumericUpDown _numLogHdopWarning;
        private NumericUpDown _numLogHdopCritical;
        private NumericUpDown _numLogMinimumSatellites;
        private NumericUpDown _numLogTuneWarning;
        private NumericUpDown _numLogTuneCritical;
        private NumericUpDown _numLogEkfWarning;
        private NumericUpDown _numLogEkfCritical;
        private NumericUpDown _numLogLiveBufferPoints;
        private CheckBox _chkLogInjectHud;


        // Joystick Tab
        private CheckBox _chkJoyGimbalEnabled, _chkJoyPositionEnabled;
        private CheckBox _chkJoyGimbalPitchInvert, _chkJoyGimbalRollInvert, _chkJoyPositionInvert;
        private ComboBox _cmbJoyGimbalDevice, _cmbJoyPositionDevice;
        private ComboBox _cmbJoyGimbalPitchAxis, _cmbJoyGimbalRollAxis, _cmbJoyPositionAxis;
        private NumericUpDown _numJoyGimbalDeadzone, _numJoyPositionDeadzone;
        private NumericUpDown _numJoyGimbalMaxRate;
        private TextBox _txtJoyPositionActuatorId;
        private readonly NumericUpDown[] _switchButtons = new NumericUpDown[6];
        private Button _btnJoyRefreshDevices;
        private Label _lblJoyStatus;
        private Label _lblJoyPositionEligibility;
        private NumericUpDown _numTerminationButton;
        // 3-position switch action mapping (6 slots: sw1/2/3 x up/down)
        private ComboBox _cmbSw1Up, _cmbSw1Down, _cmbSw2Up, _cmbSw2Down, _cmbSw3Up, _cmbSw3Down;
        private ComboBox _cmbSwitchDevice;
        private CheckBox _chkKillSwitchEnabled;

        // Buttons
        private Button _btnOK;
        private Button _btnCancel;
        private Button _btnTest;
        private Button _btnReset;
        private Button _btnLoadConfig;
        private Button _btnExportConfig;

        public NOMADConfig Config { get; private set; }

        // ============================================================
        // Constructor
        // ============================================================

        public NOMADSettingsForm(NOMADConfig config)
        {
            Config = (config ?? new NOMADConfig()).Clone();
            InitializeComponents();
            LoadSettings();
        }

        // ============================================================
        // UI Initialization
        // ============================================================

        private void InitializeComponents()
        {
            this.Text = "NOMAD Settings";
            this.ClientSize = new Size(760, 610);
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.StartPosition = FormStartPosition.CenterParent;
            this.BackColor = Color.FromArgb(45, 45, 48);
            this.ForeColor = Color.White;

            // Tab Control
            _tabControl = new TabControl
            {
                Location = new Point(10, 10),
                Size = new Size(740, 540),
                BackColor = Color.FromArgb(45, 45, 48),
            };
            this.Controls.Add(_tabControl);

            // Add all tab pages
            _tabControl.TabPages.Add(CreateConnectionTab());
            _tabControl.TabPages.Add(CreateVideoTab());
            _tabControl.TabPages.Add(CreateDualLinkTab());
            _tabControl.TabPages.Add(CreateUiTab());
            _tabControl.TabPages.Add(CreateAlertsTab());
            _tabControl.TabPages.Add(CreateLogsTab());
            _tabControl.TabPages.Add(CreateActuatorsTab());
            _tabControl.TabPages.Add(CreateJoystickTab());

            int clientWidth = CalculateSettingsClientWidth();
            this.ClientSize = new Size(clientWidth, 610);
            _tabControl.Width = clientWidth - 20;

            // Buttons at bottom
            int buttonY = 560;

            _btnTest = new Button
            {
                Text = "Test Connection",
                Location = new Point(10, buttonY),
                Size = new Size(120, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = NOMADTheme.ACCENT,
                ForeColor = Color.White
            };
            _btnTest.Click += BtnTest_Click;
            this.Controls.Add(_btnTest);

            _btnReset = new Button
            {
                Text = "Reset Defaults",
                Location = new Point(140, buttonY),
                Size = new Size(110, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(150, 80, 0),
                ForeColor = Color.White
            };
            _btnReset.Click += BtnReset_Click;
            this.Controls.Add(_btnReset);

            _btnLoadConfig = new Button
            {
                Text = "Load Config...",
                Location = new Point(260, buttonY),
                Size = new Size(110, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(70, 90, 130),
                ForeColor = Color.White
            };
            _btnLoadConfig.Click += BtnLoadConfig_Click;
            this.Controls.Add(_btnLoadConfig);

            _btnExportConfig = new Button
            {
                Text = "Export Config...",
                Location = new Point(380, buttonY),
                Size = new Size(120, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(70, 90, 130),
                ForeColor = Color.White
            };
            _btnExportConfig.Click += BtnExportConfig_Click;
            this.Controls.Add(_btnExportConfig);

            _btnOK = new Button
            {
                Text = "Save",
                Location = new Point(clientWidth - 200, buttonY),
                Size = new Size(90, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 150, 100),
                ForeColor = Color.White,
                DialogResult = DialogResult.None
            };
            _btnOK.Click += (s, e) =>
            {
                try
                {
                    SaveSettings(); DialogResult = DialogResult.OK;
                }
                catch (Exception ex)
                {
                    DialogResult = DialogResult.None;
                    MessageBox.Show(this, ex.Message, "Configuration requires review", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                }
            };
            this.Controls.Add(_btnOK);

            _btnCancel = new Button
            {
                Text = "Cancel",
                Location = new Point(clientWidth - 100, buttonY),
                Size = new Size(90, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(100, 100, 100),
                ForeColor = Color.White,
                DialogResult = DialogResult.Cancel
            };
            this.Controls.Add(_btnCancel);

            this.AcceptButton = _btnOK;
            this.CancelButton = _btnCancel;
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
                BackColor = Color.FromArgb(45, 45, 48),
                ForeColor = Color.White,
                AutoScroll = true,
                Padding = new Padding(5)
            };
            return tab;
        }

        private void AddSectionLabel(TabPage tab, string text, ref int y)
        {
            var label = new Label
            {
                Text = "-- " + text + " --",
                Location = new Point(10, y),
                AutoSize = true,
                ForeColor = Color.FromArgb(100, 180, 255),
                Font = new Font("Segoe UI", 9, FontStyle.Bold)
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
                ForeColor = color ?? Color.LightGray
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
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
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
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
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
                ForeColor = color ?? Color.White
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
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
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
                "Reset all settings to defaults?\n\nThis cannot be undone.",
                "Reset Settings",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning
            );

            if (result == DialogResult.Yes)
            {
                Config = new NOMADConfig();
                LoadSettings();
            }
        }

        private void BtnTest_Click(object sender, EventArgs e)
        {
            SaveSettings();
            bool configured = !string.IsNullOrWhiteSpace(Config.CoreClientCredential)
                && Config.CoreRuntimePort >= 1 && Config.CoreRuntimePort <= 65535;
            MessageBox.Show(
                configured
                    ? $"Runtime IPC is configured for 127.0.0.1:{Config.CoreRuntimePort}. " +
                      "The supervised runtime must be running to issue commands."
                    : "Configure the runtime IPC port and local actuation gate before issuing vehicle commands.",
                "NOMAD Runtime Configuration",
                MessageBoxButtons.OK,
                configured ? MessageBoxIcon.Information : MessageBoxIcon.Warning);
        }



    }
}
