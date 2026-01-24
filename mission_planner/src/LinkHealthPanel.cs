// ============================================================
// MAVLink Link Health Panel - UI for Dual Link Management
// ============================================================
// Visual display and control for MAVLink dual link failover:
// - Real-time link status indicators
// - Health meters for both LTE and RadioMaster links
// - Manual link switching controls
// - Failover history log
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// UI Panel for displaying and controlling dual MAVLink link status.
    /// </summary>
    public class LinkHealthPanel : UserControl
    {
        // ============================================================
        // Fields
        // ============================================================

        private MAVLinkConnectionManager _connectionManager;
        private NOMADConfig _config;
        private System.Windows.Forms.Timer _updateTimer;

        // UI Controls - Main status
        private Panel _headerPanel;
        private Label _lblActiveLink;
        private Label _lblOverallStatus;

        // UI Controls - LTE Link
        private Panel _lteLinkPanel;
        private Label _lblLteStatus;
        private Label _lblLteLatency;
        private Label _lblLtePacketLoss;
        private ProgressBar _prgLteHealth;
        private Button _btnSwitchToLte;
        private PictureBox _picLteIndicator;

        // UI Controls - RadioMaster Link
        private Panel _radioMasterPanel;
        private Label _lblRadioMasterStatus;
        private Label _lblRadioMasterLatency;
        private Label _lblRadioMasterPacketLoss;
        private ProgressBar _prgRadioMasterHealth;
        private Button _btnSwitchToRadioMaster;
        private PictureBox _picRadioMasterIndicator;

        // UI Controls - Settings
        private Panel _settingsPanel;
        private CheckBox _chkAutoFailover;
        private ComboBox _cmbPreferredLink;
        private CheckBox _chkAutoReconnect;

        // UI Controls - Log
        private ListBox _lstFailoverLog;
        private readonly List<string> _failoverLog = new List<string>();

        // Colors
        private readonly Color _bgColor = Color.FromArgb(30, 30, 30);
        private readonly Color _panelColor = Color.FromArgb(45, 45, 48);
        private readonly Color _accentColor = Color.FromArgb(0, 122, 204);
        private readonly Color _textColor = Color.White;
        private readonly Color _subtextColor = Color.LightGray;

        // ============================================================
        // Constructor
        // ============================================================

        public LinkHealthPanel(MAVLinkConnectionManager connectionManager, NOMADConfig config)
        {
            _connectionManager = connectionManager ?? throw new ArgumentNullException(nameof(connectionManager));
            _config = config ?? throw new ArgumentNullException(nameof(config));

            InitializeUI();
            SubscribeToEvents();
            StartUpdateTimer();
        }

        // ============================================================
        // UI Initialization
        // ============================================================

        private void InitializeUI()
        {
            this.BackColor = _bgColor;
            this.Dock = DockStyle.Fill;
            this.AutoScroll = true;
            this.Padding = new Padding(10);

            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 5,
                BackColor = Color.Transparent,
            };

            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 70));   // Header
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 130));  // LTE Panel
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 130));  // RadioMaster Panel
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 100));  // Settings
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));   // Log

            // Header
            _headerPanel = CreateHeaderPanel();
            mainLayout.Controls.Add(_headerPanel, 0, 0);

            // LTE Link Panel
            _lteLinkPanel = CreateLinkPanel("LTE / Tailscale", LinkType.LTE, 
                ref _lblLteStatus, ref _lblLteLatency, ref _lblLtePacketLoss, 
                ref _prgLteHealth, ref _btnSwitchToLte, ref _picLteIndicator);
            mainLayout.Controls.Add(_lteLinkPanel, 0, 1);

            // RadioMaster Panel
            _radioMasterPanel = CreateLinkPanel("RadioMaster (14550)", LinkType.RadioMaster,
                ref _lblRadioMasterStatus, ref _lblRadioMasterLatency, ref _lblRadioMasterPacketLoss,
                ref _prgRadioMasterHealth, ref _btnSwitchToRadioMaster, ref _picRadioMasterIndicator);
            mainLayout.Controls.Add(_radioMasterPanel, 0, 2);

            // Settings Panel
            _settingsPanel = CreateSettingsPanel();
            mainLayout.Controls.Add(_settingsPanel, 0, 3);

            // Log Panel
            var logPanel = CreateLogPanel();
            mainLayout.Controls.Add(logPanel, 0, 4);

            this.Controls.Add(mainLayout);
        }

        private Panel CreateHeaderPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = _panelColor,
                Margin = new Padding(0, 0, 0, 5),
                Padding = new Padding(15, 10, 15, 10),
            };

            var title = new Label
            {
                Text = "MAVLink Dual Link Status",
                Font = new Font("Segoe UI", 14, FontStyle.Bold),
                ForeColor = _accentColor,
                Location = new Point(15, 8),
                AutoSize = true,
            };
            panel.Controls.Add(title);

            _lblActiveLink = new Label
            {
                Text = "Active: NONE",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = Color.Yellow,
                Location = new Point(300, 10),
                AutoSize = true,
            };
            panel.Controls.Add(_lblActiveLink);

            _lblOverallStatus = new Label
            {
                Text = "Monitoring...",
                Font = new Font("Segoe UI", 9),
                ForeColor = _subtextColor,
                Location = new Point(15, 38),
                AutoSize = true,
            };
            panel.Controls.Add(_lblOverallStatus);

            return panel;
        }

        private Panel CreateLinkPanel(string title, LinkType linkType,
            ref Label lblStatus, ref Label lblLatency, ref Label lblPacketLoss,
            ref ProgressBar prgHealth, ref Button btnSwitch, ref PictureBox picIndicator)
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = _panelColor,
                Margin = new Padding(0, 0, 0, 5),
                Padding = new Padding(10),
            };

            // Status indicator (colored circle)
            picIndicator = new PictureBox
            {
                Size = new Size(20, 20),
                Location = new Point(15, 15),
                BackColor = Color.Transparent,
            };
            picIndicator.Paint += (s, e) =>
            {
                var color = GetIndicatorColor(linkType);
                using (var brush = new SolidBrush(color))
                {
                    e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
                    e.Graphics.FillEllipse(brush, 2, 2, 16, 16);
                }
            };
            panel.Controls.Add(picIndicator);

            // Title
            var lblTitle = new Label
            {
                Text = title,
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = _textColor,
                Location = new Point(45, 12),
                AutoSize = true,
            };
            panel.Controls.Add(lblTitle);

            // Status
            lblStatus = new Label
            {
                Text = "Disconnected",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.Red,
                Location = new Point(45, 35),
                AutoSize = true,
            };
            panel.Controls.Add(lblStatus);

            // Metrics row
            lblLatency = new Label
            {
                Text = "Latency: --ms",
                Font = new Font("Segoe UI", 9),
                ForeColor = _subtextColor,
                Location = new Point(45, 58),
                Width = 120,
            };
            panel.Controls.Add(lblLatency);

            lblPacketLoss = new Label
            {
                Text = "Loss: --%",
                Font = new Font("Segoe UI", 9),
                ForeColor = _subtextColor,
                Location = new Point(170, 58),
                Width = 100,
            };
            panel.Controls.Add(lblPacketLoss);

            // Health progress bar
            var lblHealthLabel = new Label
            {
                Text = "Health:",
                Font = new Font("Segoe UI", 9),
                ForeColor = _subtextColor,
                Location = new Point(45, 82),
                AutoSize = true,
            };
            panel.Controls.Add(lblHealthLabel);

            prgHealth = new ProgressBar
            {
                Location = new Point(100, 80),
                Size = new Size(200, 18),
                Maximum = 100,
                Style = ProgressBarStyle.Continuous,
            };
            panel.Controls.Add(prgHealth);

            // Switch button
            btnSwitch = new Button
            {
                Text = "Switch",
                Location = new Point(320, 40),
                Size = new Size(80, 30),
                BackColor = _accentColor,
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Cursor = Cursors.Hand,
            };
            btnSwitch.FlatAppearance.BorderSize = 0;
            btnSwitch.Click += (s, e) => SwitchToLink(linkType);
            panel.Controls.Add(btnSwitch);

            return panel;
        }

        private Panel CreateSettingsPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = _panelColor,
                Margin = new Padding(0, 0, 0, 5),
                Padding = new Padding(10),
            };

            var title = new Label
            {
                Text = "⚙️ Failover Settings",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = _accentColor,
                Location = new Point(15, 10),
                AutoSize = true,
            };
            panel.Controls.Add(title);

            // Auto-failover checkbox
            _chkAutoFailover = new CheckBox
            {
                Text = "Enable Automatic Failover",
                Font = new Font("Segoe UI", 9),
                ForeColor = _textColor,
                Location = new Point(15, 38),
                AutoSize = true,
                Checked = _connectionManager.Config.AutoFailoverEnabled,
            };
            _chkAutoFailover.CheckedChanged += (s, e) =>
            {
                _connectionManager.Config.AutoFailoverEnabled = _chkAutoFailover.Checked;
            };
            panel.Controls.Add(_chkAutoFailover);

            // Preferred link
            var lblPreferred = new Label
            {
                Text = "Preferred Link:",
                Font = new Font("Segoe UI", 9),
                ForeColor = _textColor,
                Location = new Point(250, 38),
                AutoSize = true,
            };
            panel.Controls.Add(lblPreferred);

            _cmbPreferredLink = new ComboBox
            {
                Location = new Point(350, 35),
                Size = new Size(120, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                Font = new Font("Segoe UI", 9),
            };
            _cmbPreferredLink.Items.AddRange(new object[] { "LTE", "RadioMaster", "None" });
            _cmbPreferredLink.SelectedIndex = _connectionManager.Config.PreferredLink switch
            {
                LinkType.LTE => 0,
                LinkType.RadioMaster => 1,
                _ => 2
            };
            _cmbPreferredLink.SelectedIndexChanged += (s, e) =>
            {
                _connectionManager.Config.PreferredLink = _cmbPreferredLink.SelectedIndex switch
                {
                    0 => LinkType.LTE,
                    1 => LinkType.RadioMaster,
                    _ => LinkType.None
                };
            };
            panel.Controls.Add(_cmbPreferredLink);

            // Auto-reconnect to preferred
            _chkAutoReconnect = new CheckBox
            {
                Text = "Auto-reconnect to preferred when available",
                Font = new Font("Segoe UI", 9),
                ForeColor = _textColor,
                Location = new Point(15, 65),
                AutoSize = true,
                Checked = _connectionManager.Config.AutoReconnectPreferred,
            };
            _chkAutoReconnect.CheckedChanged += (s, e) =>
            {
                _connectionManager.Config.AutoReconnectPreferred = _chkAutoReconnect.Checked;
            };
            panel.Controls.Add(_chkAutoReconnect);

            return panel;
        }

        private Panel CreateLogPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = _panelColor,
                Margin = new Padding(0),
                Padding = new Padding(10),
            };

            var title = new Label
            {
                Text = "📋 Failover Log",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = _accentColor,
                Location = new Point(15, 10),
                AutoSize = true,
            };
            panel.Controls.Add(title);

            _lstFailoverLog = new ListBox
            {
                Location = new Point(15, 35),
                Size = new Size(450, 120),
                BackColor = Color.FromArgb(25, 25, 25),
                ForeColor = _textColor,
                Font = new Font("Consolas", 9),
                BorderStyle = BorderStyle.None,
            };
            panel.Controls.Add(_lstFailoverLog);

            // Clear button
            var btnClear = new Button
            {
                Text = "Clear",
                Location = new Point(380, 8),
                Size = new Size(60, 22),
                BackColor = Color.FromArgb(60, 60, 60),
                ForeColor = _textColor,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 8),
            };
            btnClear.FlatAppearance.BorderSize = 0;
            btnClear.Click += (s, e) =>
            {
                _failoverLog.Clear();
                _lstFailoverLog.Items.Clear();
            };
            panel.Controls.Add(btnClear);

            // Resize handler
            panel.Resize += (s, e) =>
            {
                _lstFailoverLog.Width = panel.Width - 30;
                _lstFailoverLog.Height = panel.Height - 50;
                btnClear.Location = new Point(panel.Width - 90, 8);
            };

            return panel;
        }

        // ============================================================
        // Event Handling
        // ============================================================

        private void SubscribeToEvents()
        {
            _connectionManager.LinkStatusChanged += OnLinkStatusChanged;
            _connectionManager.FailoverOccurred += OnFailoverOccurred;
            _connectionManager.ActiveLinkChanged += OnActiveLinkChanged;
        }

        private void OnLinkStatusChanged(object sender, LinkStatusChangedEventArgs e)
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke(new Action(() => OnLinkStatusChanged(sender, e)));
                return;
            }

            UpdateLinkDisplay(e.Link, e.Statistics, e.IsActive);
        }

        private void OnFailoverOccurred(object sender, FailoverEventArgs e)
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke(new Action(() => OnFailoverOccurred(sender, e)));
                return;
            }

            var logEntry = $"[{e.Timestamp:HH:mm:ss}] {e.FromLink} → {e.ToLink}: {e.Reason}";
            _failoverLog.Add(logEntry);
            _lstFailoverLog.Items.Add(logEntry);

            // Auto-scroll to bottom
            if (_lstFailoverLog.Items.Count > 0)
            {
                _lstFailoverLog.SelectedIndex = _lstFailoverLog.Items.Count - 1;
                _lstFailoverLog.SelectedIndex = -1;
            }
        }

        private void OnActiveLinkChanged(object sender, LinkType newLink)
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke(new Action(() => OnActiveLinkChanged(sender, newLink)));
                return;
            }

            UpdateActiveLinkDisplay(newLink);
        }

        // ============================================================
        // UI Updates
        // ============================================================

        private void StartUpdateTimer()
        {
            _updateTimer = new System.Windows.Forms.Timer
            {
                Interval = 500
            };
            _updateTimer.Tick += (s, e) => RefreshDisplay();
            _updateTimer.Start();
        }

        private void RefreshDisplay()
        {
            UpdateLinkDisplay(LinkType.LTE, _connectionManager.LteStatistics, 
                _connectionManager.ActiveLink == LinkType.LTE);
            UpdateLinkDisplay(LinkType.RadioMaster, _connectionManager.RadioMasterStatistics,
                _connectionManager.ActiveLink == LinkType.RadioMaster);
            UpdateActiveLinkDisplay(_connectionManager.ActiveLink);
            
            // Force indicator repaint
            _picLteIndicator?.Invalidate();
            _picRadioMasterIndicator?.Invalidate();
        }

        private void UpdateLinkDisplay(LinkType linkType, LinkStatistics stats, bool isActive)
        {
            Label lblStatus, lblLatency, lblPacketLoss;
            ProgressBar prgHealth;
            Button btnSwitch;

            if (linkType == LinkType.LTE)
            {
                lblStatus = _lblLteStatus;
                lblLatency = _lblLteLatency;
                lblPacketLoss = _lblLtePacketLoss;
                prgHealth = _prgLteHealth;
                btnSwitch = _btnSwitchToLte;
            }
            else
            {
                lblStatus = _lblRadioMasterStatus;
                lblLatency = _lblRadioMasterLatency;
                lblPacketLoss = _lblRadioMasterPacketLoss;
                prgHealth = _prgRadioMasterHealth;
                btnSwitch = _btnSwitchToRadioMaster;
            }

            // Update status
            lblStatus.Text = stats.StatusText;
            lblStatus.ForeColor = GetHealthColor(stats.Health);

            // Update metrics
            lblLatency.Text = stats.IsConnected ? $"Latency: {stats.LatencyMs:F0}ms" : "Latency: --";
            lblPacketLoss.Text = stats.IsConnected ? $"Loss: {stats.PacketLossPercent:F1}%" : "Loss: --";

            // Update health bar
            prgHealth.Value = stats.IsConnected ? HealthToPercent(stats.Health) : 0;

            // Update switch button
            btnSwitch.Enabled = stats.IsConnected && !isActive;
            btnSwitch.Text = isActive ? "ACTIVE" : "Switch";
            btnSwitch.BackColor = isActive ? Color.ForestGreen : _accentColor;
        }

        private void UpdateActiveLinkDisplay(LinkType activeLink)
        {
            _lblActiveLink.Text = $"Active: {(activeLink == LinkType.None ? "NONE" : activeLink.ToString())}";
            _lblActiveLink.ForeColor = activeLink switch
            {
                LinkType.LTE => Color.LimeGreen,
                LinkType.RadioMaster => Color.Cyan,
                _ => Color.Yellow
            };

            _lblOverallStatus.Text = _connectionManager.GetStatusSummary();
        }

        private Color GetIndicatorColor(LinkType linkType)
        {
            var stats = linkType == LinkType.LTE 
                ? _connectionManager.LteStatistics 
                : _connectionManager.RadioMasterStatistics;

            return GetHealthColor(stats.Health);
        }

        private Color GetHealthColor(LinkHealth health)
        {
            return health switch
            {
                LinkHealth.Excellent => Color.LimeGreen,
                LinkHealth.Good => Color.LightGreen,
                LinkHealth.Fair => Color.Gold,
                LinkHealth.Poor => Color.Orange,
                LinkHealth.Critical => Color.OrangeRed,
                LinkHealth.Disconnected => Color.Red,
                _ => Color.Gray
            };
        }

        private int HealthToPercent(LinkHealth health)
        {
            return health switch
            {
                LinkHealth.Excellent => 100,
                LinkHealth.Good => 80,
                LinkHealth.Fair => 60,
                LinkHealth.Poor => 40,
                LinkHealth.Critical => 20,
                _ => 0
            };
        }

        // ============================================================
        // Actions
        // ============================================================

        private void SwitchToLink(LinkType targetLink)
        {
            var result = _connectionManager.SwitchToLink(targetLink);
            
            if (!result)
            {
                MessageBox.Show(
                    $"Cannot switch to {targetLink}.\nThe link may not be connected or healthy.",
                    "Switch Failed",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Warning
                );
            }
            else
            {
                var logEntry = $"[{DateTime.Now:HH:mm:ss}] Manual switch to {targetLink}";
                _failoverLog.Add(logEntry);
                _lstFailoverLog.Items.Add(logEntry);
            }
        }

        // ============================================================
        // Cleanup
        // ============================================================

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _updateTimer?.Stop();
                _updateTimer?.Dispose();

                if (_connectionManager != null)
                {
                    _connectionManager.LinkStatusChanged -= OnLinkStatusChanged;
                    _connectionManager.FailoverOccurred -= OnFailoverOccurred;
                    _connectionManager.ActiveLinkChanged -= OnActiveLinkChanged;
                }
            }

            base.Dispose(disposing);
        }
    }
}
