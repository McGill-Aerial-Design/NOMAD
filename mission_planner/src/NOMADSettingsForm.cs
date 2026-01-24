// ============================================================
// NOMAD Settings Form
// ============================================================
// Configuration dialog for NOMAD plugin settings.
// ============================================================

using System;
using System.Drawing;
using System.IO.Ports;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Settings dialog for NOMAD plugin configuration.
    /// </summary>
    public class NOMADSettingsForm : Form
    {
        // ============================================================
        // Fields
        // ============================================================

        private TextBox _txtJetsonIP;
        private NumericUpDown _numPort;
        private TextBox _txtRtspZed;
        private NumericUpDown _numServoChannel;
        private CheckBox _chkUseELRS;
        private NumericUpDown _numTimeout;
        private CheckBox _chkDebug;
        private Button _btnOK;
        private Button _btnCancel;
        private Button _btnTest;
        
        // Dual Link Settings
        private CheckBox _chkDualLinkEnabled;
        private ComboBox _cmbRadioMasterConnType;
        private NumericUpDown _numRadioMasterPort;
        private ComboBox _cmbRadioMasterComPort;
        private ComboBox _cmbRadioMasterBaudRate;
        private CheckBox _chkAutoFailover;
        private ComboBox _cmbPreferredLink;
        private CheckBox _chkAutoReconnect;

        /// <summary>
        /// Gets the configured settings.
        /// </summary>
        public NOMADConfig Config { get; private set; }

        // ============================================================
        // Constructor
        // ============================================================

        public NOMADSettingsForm(NOMADConfig config)
        {
            Config = config ?? new NOMADConfig();
            InitializeComponents();
            LoadSettings();
        }

        // ============================================================
        // UI Initialization
        // ============================================================

        private void InitializeComponents()
        {
            this.Text = "NOMAD Settings";
            this.Size = new Size(500, 580);
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.StartPosition = FormStartPosition.CenterParent;
            this.BackColor = Color.FromArgb(45, 45, 48);
            this.ForeColor = Color.White;
            this.AutoScroll = true;

            int yOffset = 20;
            int labelWidth = 140;
            int inputX = labelWidth + 25;

            // Jetson IP
            AddLabel("Jetson IP:", 20, yOffset);
            _txtJetsonIP = new TextBox
            {
                Location = new Point(inputX, yOffset - 3),
                Size = new Size(200, 23),
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            this.Controls.Add(_txtJetsonIP);
            yOffset += 35;

            // Port
            AddLabel("API Port:", 20, yOffset);
            _numPort = new NumericUpDown
            {
                Location = new Point(inputX, yOffset - 3),
                Size = new Size(80, 23),
                Minimum = 1,
                Maximum = 65535,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            this.Controls.Add(_numPort);
            yOffset += 40;

            // Video Stream Section Header
            var lblVideoSection = new Label
            {
                Text = "-- Video Streams --",
                Location = new Point(20, yOffset),
                AutoSize = true,
                ForeColor = Color.FromArgb(200, 100, 200),
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                BackColor = Color.Transparent
            };
            this.Controls.Add(lblVideoSection);
            lblVideoSection.BringToFront();
            yOffset += 25;

            // ZED Camera RTSP URL
            AddLabel("ZED Camera URL:", 20, yOffset);
            _txtRtspZed = new TextBox
            {
                Location = new Point(inputX, yOffset - 3),
                Size = new Size(250, 23),
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            this.Controls.Add(_txtRtspZed);
            yOffset += 30;

            // Camera Tilt Servo Channel
            AddLabel("Tilt Servo Channel:", 20, yOffset);
            _numServoChannel = new NumericUpDown
            {
                Location = new Point(inputX, yOffset - 3),
                Size = new Size(60, 23),
                Minimum = 0,
                Maximum = 16,
                Value = 10,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            this.Controls.Add(_numServoChannel);
            
            var lblServoHelp = new Label
            {
                Text = "(0=off, 9-14=AUX1-6)",
                Location = new Point(inputX + 70, yOffset),
                AutoSize = true,
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
                BackColor = Color.Transparent
            };
            this.Controls.Add(lblServoHelp);
            lblServoHelp.BringToFront();
            yOffset += 40;

            // ============================================================
            // MAVLink Dual Link Section
            // ============================================================
            var lblDualLinkSection = new Label
            {
                Text = "-- MAVLink Dual Link (Failover) --",
                Location = new Point(20, yOffset),
                AutoSize = true,
                ForeColor = Color.FromArgb(0, 200, 150),
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                BackColor = Color.Transparent
            };
            this.Controls.Add(lblDualLinkSection);
            lblDualLinkSection.BringToFront();
            yOffset += 25;

            // Enable Dual Link
            _chkDualLinkEnabled = new CheckBox
            {
                Text = "Enable Dual Link Management",
                Location = new Point(20, yOffset),
                AutoSize = true,
                ForeColor = Color.LimeGreen
            };
            _chkDualLinkEnabled.CheckedChanged += (s, e) => UpdateDualLinkControlsState();
            this.Controls.Add(_chkDualLinkEnabled);
            yOffset += 28;

            // RadioMaster Connection Type
            AddLabel("RadioMaster Type:", 40, yOffset);
            _cmbRadioMasterConnType = new ComboBox
            {
                Location = new Point(inputX + 20, yOffset - 3),
                Size = new Size(80, 23),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            _cmbRadioMasterConnType.Items.AddRange(new object[] { "UDP", "COM" });
            _cmbRadioMasterConnType.SelectedIndexChanged += (s, e) => UpdateRadioMasterConnTypeState();
            this.Controls.Add(_cmbRadioMasterConnType);
            yOffset += 28;

            // RadioMaster Port (UDP)
            AddLabel("UDP Port:", 40, yOffset);
            _numRadioMasterPort = new NumericUpDown
            {
                Location = new Point(inputX + 20, yOffset - 3),
                Size = new Size(80, 23),
                Minimum = 1,
                Maximum = 65535,
                Value = 14550,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            this.Controls.Add(_numRadioMasterPort);
            
            var lblRadioHelp = new Label
            {
                Text = "(typically 14550)",
                Location = new Point(inputX + 110, yOffset),
                AutoSize = true,
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
                BackColor = Color.Transparent
            };
            this.Controls.Add(lblRadioHelp);
            lblRadioHelp.BringToFront();
            yOffset += 28;

            // RadioMaster COM Port
            AddLabel("COM Port:", 40, yOffset);
            _cmbRadioMasterComPort = new ComboBox
            {
                Location = new Point(inputX + 20, yOffset - 3),
                Size = new Size(80, 23),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            // Populate with available COM ports
            foreach (string port in System.IO.Ports.SerialPort.GetPortNames())
            {
                _cmbRadioMasterComPort.Items.Add(port);
            }
            if (_cmbRadioMasterComPort.Items.Count > 0)
                _cmbRadioMasterComPort.SelectedIndex = 0;
            this.Controls.Add(_cmbRadioMasterComPort);
            
            // Baud Rate
            _cmbRadioMasterBaudRate = new ComboBox
            {
                Location = new Point(inputX + 110, yOffset - 3),
                Size = new Size(90, 23),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            _cmbRadioMasterBaudRate.Items.AddRange(new object[] { "115200", "420000", "460800", "921600" });
            _cmbRadioMasterBaudRate.SelectedIndex = 1; // Default 420000
            this.Controls.Add(_cmbRadioMasterBaudRate);
            yOffset += 28;

            // Auto Failover
            _chkAutoFailover = new CheckBox
            {
                Text = "Enable Automatic Failover",
                Location = new Point(40, yOffset),
                AutoSize = true,
                ForeColor = Color.White
            };
            this.Controls.Add(_chkAutoFailover);
            yOffset += 28;

            // Preferred Link
            AddLabel("Preferred Link:", 40, yOffset);
            _cmbPreferredLink = new ComboBox
            {
                Location = new Point(inputX + 20, yOffset - 3),
                Size = new Size(130, 23),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            _cmbPreferredLink.Items.AddRange(new object[] { "LTE (Tailscale)", "RadioMaster", "None" });
            this.Controls.Add(_cmbPreferredLink);
            yOffset += 28;

            // Auto-reconnect to preferred
            _chkAutoReconnect = new CheckBox
            {
                Text = "Auto-reconnect to preferred link",
                Location = new Point(40, yOffset),
                AutoSize = true,
                ForeColor = Color.White
            };
            this.Controls.Add(_chkAutoReconnect);
            yOffset += 35;

            // ============================================================
            // Communication Section
            // ============================================================
            var lblCommSection = new Label
            {
                Text = "-- Communication --",
                Location = new Point(20, yOffset),
                AutoSize = true,
                ForeColor = Color.FromArgb(100, 150, 255),
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                BackColor = Color.Transparent
            };
            this.Controls.Add(lblCommSection);
            lblCommSection.BringToFront();
            yOffset += 25;

            // Use ELRS
            _chkUseELRS = new CheckBox
            {
                Text = "Use ELRS/MAVLink (instead of HTTP)",
                Location = new Point(20, yOffset),
                AutoSize = true,
                ForeColor = Color.Orange
            };
            this.Controls.Add(_chkUseELRS);
            yOffset += 35;

            // Timeout
            AddLabel("HTTP Timeout (s):", 20, yOffset);
            _numTimeout = new NumericUpDown
            {
                Location = new Point(inputX, yOffset - 3),
                Size = new Size(60, 23),
                Minimum = 1,
                Maximum = 30,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            this.Controls.Add(_numTimeout);
            yOffset += 35;

            // Debug Mode
            _chkDebug = new CheckBox
            {
                Text = "Enable Debug Logging",
                Location = new Point(20, yOffset),
                AutoSize = true,
                ForeColor = Color.Gray
            };
            this.Controls.Add(_chkDebug);
            yOffset += 45;

            // Buttons
            _btnTest = new Button
            {
                Text = "Test Connection",
                Location = new Point(20, yOffset),
                Size = new Size(120, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White
            };
            _btnTest.Click += BtnTest_Click;
            this.Controls.Add(_btnTest);

            _btnOK = new Button
            {
                Text = "OK",
                Location = new Point(200, yOffset),
                Size = new Size(80, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 150, 100),
                ForeColor = Color.White,
                DialogResult = DialogResult.OK
            };
            _btnOK.Click += (s, e) => SaveSettings();
            this.Controls.Add(_btnOK);

            _btnCancel = new Button
            {
                Text = "Cancel",
                Location = new Point(290, yOffset),
                Size = new Size(80, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(100, 100, 100),
                ForeColor = Color.White,
                DialogResult = DialogResult.Cancel
            };
            this.Controls.Add(_btnCancel);

            this.AcceptButton = _btnOK;
            this.CancelButton = _btnCancel;
        }

        private Label AddLabel(string text, int x, int y)
        {
            var label = new Label
            {
                Text = text,
                Location = new Point(x, y),
                AutoSize = true,
                ForeColor = Color.LightGray,
                BackColor = Color.Transparent  // Ensure transparency
            };
            this.Controls.Add(label);
            label.BringToFront();  // Ensure labels are on top of other controls
            return label;
        }

        // ============================================================
        // Settings Management
        // ============================================================

        private void LoadSettings()
        {
            _txtJetsonIP.Text = Config.JetsonIP;
            _numPort.Value = Config.JetsonPort;
            _txtRtspZed.Text = Config.VideoUrl;
            _numServoChannel.Value = Config.ZedServoChannel;
            _chkUseELRS.Checked = Config.UseELRS;
            _numTimeout.Value = Config.HttpTimeoutSeconds;
            _chkDebug.Checked = Config.DebugMode;
            
            // Dual Link settings
            _chkDualLinkEnabled.Checked = Config.DualLinkEnabled;
            _cmbRadioMasterConnType.SelectedIndex = Config.RadioMasterConnectionType == "COM" ? 1 : 0;
            _numRadioMasterPort.Value = Config.RadioMasterPort;
            
            // Set COM port
            var comPortIndex = _cmbRadioMasterComPort.Items.IndexOf(Config.RadioMasterComPort);
            if (comPortIndex >= 0)
                _cmbRadioMasterComPort.SelectedIndex = comPortIndex;
            else if (_cmbRadioMasterComPort.Items.Count > 0)
                _cmbRadioMasterComPort.SelectedIndex = 0;
            
            // Set baud rate
            var baudRateIndex = _cmbRadioMasterBaudRate.Items.IndexOf(Config.RadioMasterBaudRate.ToString());
            if (baudRateIndex >= 0)
                _cmbRadioMasterBaudRate.SelectedIndex = baudRateIndex;
            else
                _cmbRadioMasterBaudRate.SelectedIndex = 1; // Default 420000
            
            _chkAutoFailover.Checked = Config.AutoFailoverEnabled;
            _cmbPreferredLink.SelectedIndex = Config.PreferredMavlinkLink switch
            {
                "LTE" => 0,
                "RadioMaster" => 1,
                _ => 2
            };
            _chkAutoReconnect.Checked = Config.AutoReconnectToPreferred;
            
            UpdateDualLinkControlsState();
            UpdateRadioMasterConnTypeState();
        }

        private void SaveSettings()
        {
            Config.JetsonIP = _txtJetsonIP.Text.Trim();
            Config.JetsonPort = (int)_numPort.Value;
            Config.VideoUrl = _txtRtspZed.Text.Trim();
            Config.ZedServoChannel = (int)_numServoChannel.Value;
            Config.UseELRS = _chkUseELRS.Checked;
            Config.HttpTimeoutSeconds = (int)_numTimeout.Value;
            Config.DebugMode = _chkDebug.Checked;
            
            // Dual Link settings
            Config.DualLinkEnabled = _chkDualLinkEnabled.Checked;
            Config.RadioMasterConnectionType = _cmbRadioMasterConnType.SelectedIndex == 1 ? "COM" : "UDP";
            Config.RadioMasterPort = (int)_numRadioMasterPort.Value;
            Config.RadioMasterComPort = _cmbRadioMasterComPort.SelectedItem?.ToString() ?? "COM3";
            Config.RadioMasterBaudRate = int.TryParse(_cmbRadioMasterBaudRate.SelectedItem?.ToString(), out int baud) ? baud : 420000;
            Config.AutoFailoverEnabled = _chkAutoFailover.Checked;
            Config.PreferredMavlinkLink = _cmbPreferredLink.SelectedIndex switch
            {
                0 => "LTE",
                1 => "RadioMaster",
                _ => "None"
            };
            Config.AutoReconnectToPreferred = _chkAutoReconnect.Checked;
        }
        
        private void UpdateDualLinkControlsState()
        {
            bool enabled = _chkDualLinkEnabled.Checked;
            _cmbRadioMasterConnType.Enabled = enabled;
            _numRadioMasterPort.Enabled = enabled;
            _cmbRadioMasterComPort.Enabled = enabled;
            _cmbRadioMasterBaudRate.Enabled = enabled;
            _chkAutoFailover.Enabled = enabled;
            _cmbPreferredLink.Enabled = enabled;
            _chkAutoReconnect.Enabled = enabled;
            
            if (enabled)
            {
                UpdateRadioMasterConnTypeState();
            }
        }
        
        private void UpdateRadioMasterConnTypeState()
        {
            bool isUDP = _cmbRadioMasterConnType.SelectedIndex == 0;
            _numRadioMasterPort.Visible = isUDP;
            _cmbRadioMasterComPort.Visible = !isUDP;
            _cmbRadioMasterBaudRate.Visible = !isUDP;
        }

        // ============================================================
        // Event Handlers
        // ============================================================

        private async void BtnTest_Click(object sender, EventArgs e)
        {
            _btnTest.Enabled = false;
            _btnTest.Text = "Testing...";

            try
            {
                SaveSettings();

                using (var client = new System.Net.Http.HttpClient())
                {
                    client.Timeout = TimeSpan.FromSeconds(3);
                    var url = $"http://{Config.JetsonIP}:{Config.JetsonPort}/health";
                    var response = await client.GetAsync(url);

                    if (response.IsSuccessStatusCode)
                    {
                        MessageBox.Show(
                            $"Connection successful!\n\nJetson at {Config.JetsonIP}:{Config.JetsonPort} is reachable.",
                            "Success",
                            MessageBoxButtons.OK,
                            MessageBoxIcon.Information
                        );
                    }
                    else
                    {
                        MessageBox.Show(
                            $"Connection failed: HTTP {(int)response.StatusCode}",
                            "Error",
                            MessageBoxButtons.OK,
                            MessageBoxIcon.Warning
                        );
                    }
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(
                    $"Connection failed:\n{ex.Message}",
                    "Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error
                );
            }
            finally
            {
                _btnTest.Enabled = true;
                _btnTest.Text = "Test Connection";
            }
        }
    }
}
