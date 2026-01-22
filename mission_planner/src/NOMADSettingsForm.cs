// ============================================================
// NOMAD Settings Form
// ============================================================
// Configuration dialog for NOMAD plugin settings.
// ============================================================

using System;
using System.Drawing;
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
            this.Size = new Size(450, 420);
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.StartPosition = FormStartPosition.CenterParent;
            this.BackColor = Color.FromArgb(45, 45, 48);
            this.ForeColor = Color.White;

            int yOffset = 20;
            int labelWidth = 140;  // Increased from 130 for longer labels
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
