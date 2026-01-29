// ============================================================
// EKF Source Control Panel - EKF Source Switching UI
// ============================================================
// Provides UI controls for switching ArduPilot EKF sources:
// - Manual source selection (GPS, Vision, OptFlow)
// - Automatic source switching based on GPS fix
// - Status display showing current source and GPS health
// ============================================================

using System;
using System.Drawing;
using System.Timers;
using System.Windows.Forms;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Control panel for EKF source switching.
    /// Supports manual selection and automatic GPS-based switching.
    /// </summary>
    public class EkfSourceControlPanel : UserControl
    {
        // ============================================================
        // Fields
        // ============================================================

        private readonly DualLinkSender _sender;
        private System.Timers.Timer _autoSwitchTimer;
        private bool _autoSwitchEnabled = false;
        private DualLinkSender.EkfSource _currentSource = DualLinkSender.EkfSource.GPS;
        private DualLinkSender.EkfSource _lastAutoSource = DualLinkSender.EkfSource.GPS;
        private int _lastGpsFix = 0;
        private bool _forceVisionMode = false;

        // UI Controls
        private GroupBox _grpEkfSource;
        private ComboBox _cmbSource;
        private Button _btnApplySource;
        private CheckBox _chkAutoSwitch;
        private CheckBox _chkForceVision;
        private Label _lblCurrentSource;
        private Label _lblGpsStatus;
        private Label _lblVisionStatus;
        private Panel _statusIndicator;

        // Colors
        private static readonly Color BG_COLOR = Color.FromArgb(45, 45, 48);
        private static readonly Color PANEL_COLOR = Color.FromArgb(60, 60, 65);
        private static readonly Color ACCENT_COLOR = Color.FromArgb(0, 122, 204);
        private static readonly Color SUCCESS_COLOR = Color.FromArgb(92, 184, 92);
        private static readonly Color WARNING_COLOR = Color.FromArgb(240, 173, 78);
        private static readonly Color ERROR_COLOR = Color.FromArgb(217, 83, 79);

        // ============================================================
        // Constructor
        // ============================================================

        public EkfSourceControlPanel(DualLinkSender sender)
        {
            _sender = sender ?? throw new ArgumentNullException(nameof(sender));
            InitializeComponents();
            InitializeAutoSwitchTimer();
        }

        // ============================================================
        // Properties
        // ============================================================

        /// <summary>
        /// Gets or sets whether automatic source switching is enabled.
        /// When enabled, switches to Vision (SRC2) if GPS fix is lost.
        /// </summary>
        public bool AutoSwitchEnabled
        {
            get => _autoSwitchEnabled;
            set
            {
                _autoSwitchEnabled = value;
                if (_chkAutoSwitch != null && _chkAutoSwitch.Checked != value)
                {
                    _chkAutoSwitch.Checked = value;
                }
            }
        }

        /// <summary>
        /// Gets or sets whether to force Vision mode regardless of GPS.
        /// Useful for testing VIO outdoors.
        /// </summary>
        public bool ForceVisionMode
        {
            get => _forceVisionMode;
            set
            {
                _forceVisionMode = value;
                if (_chkForceVision != null && _chkForceVision.Checked != value)
                {
                    _chkForceVision.Checked = value;
                }
            }
        }

        /// <summary>
        /// Gets the current EKF source.
        /// </summary>
        public DualLinkSender.EkfSource CurrentSource => _currentSource;

        // ============================================================
        // Events
        // ============================================================

        /// <summary>
        /// Raised when the EKF source is changed (manually or automatically).
        /// </summary>
        public event EventHandler<EkfSourceChangedEventArgs> SourceChanged;

        // ============================================================
        // Initialization
        // ============================================================

        private void InitializeComponents()
        {
            this.BackColor = BG_COLOR;
            this.Size = new Size(340, 180);

            _grpEkfSource = new GroupBox
            {
                Text = "EKF Source Control",
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, 5),
                Size = new Size(320, 170),
                BackColor = PANEL_COLOR
            };

            int y = 22;

            // Status indicator and current source
            _statusIndicator = new Panel
            {
                Location = new Point(15, y),
                Size = new Size(12, 12),
                BackColor = WARNING_COLOR
            };
            _grpEkfSource.Controls.Add(_statusIndicator);

            _lblCurrentSource = new Label
            {
                Text = "Current: GPS (SRC1)",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.White,
                Location = new Point(32, y - 2),
                AutoSize = true
            };
            _grpEkfSource.Controls.Add(_lblCurrentSource);

            y += 22;

            // GPS and Vision status
            _lblGpsStatus = new Label
            {
                Text = "GPS: No Fix",
                Font = new Font("Segoe UI", 8),
                ForeColor = ERROR_COLOR,
                Location = new Point(15, y),
                AutoSize = true
            };
            _grpEkfSource.Controls.Add(_lblGpsStatus);

            _lblVisionStatus = new Label
            {
                Text = "| Vision: Unknown",
                Font = new Font("Segoe UI", 8),
                ForeColor = WARNING_COLOR,
                Location = new Point(120, y),
                AutoSize = true
            };
            _grpEkfSource.Controls.Add(_lblVisionStatus);

            y += 25;

            // Source selection dropdown
            var lblSelect = new Label
            {
                Text = "Select Source:",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.White,
                Location = new Point(15, y + 3),
                AutoSize = true
            };
            _grpEkfSource.Controls.Add(lblSelect);

            _cmbSource = new ComboBox
            {
                Location = new Point(110, y),
                Size = new Size(130, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                Font = new Font("Segoe UI", 9),
                BackColor = Color.FromArgb(80, 80, 85),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
            };
            _cmbSource.Items.AddRange(new object[] 
            { 
                "SRC1: GPS",
                "SRC2: Vision/VIO",
                "SRC3: Optical Flow"
            });
            _cmbSource.SelectedIndex = 0;
            _grpEkfSource.Controls.Add(_cmbSource);

            _btnApplySource = new Button
            {
                Text = "Apply",
                Location = new Point(248, y - 1),
                Size = new Size(60, 26),
                FlatStyle = FlatStyle.Flat,
                BackColor = ACCENT_COLOR,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
                Cursor = Cursors.Hand
            };
            _btnApplySource.FlatAppearance.BorderSize = 0;
            _btnApplySource.Click += BtnApplySource_Click;
            _grpEkfSource.Controls.Add(_btnApplySource);

            y += 32;

            // Auto-switch checkbox
            _chkAutoSwitch = new CheckBox
            {
                Text = "Auto-switch: GPS=SRC1, No GPS=SRC2",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.White,
                Location = new Point(15, y),
                AutoSize = true,
                Checked = false
            };
            _chkAutoSwitch.CheckedChanged += ChkAutoSwitch_CheckedChanged;
            _grpEkfSource.Controls.Add(_chkAutoSwitch);

            y += 25;

            // Force Vision checkbox (for outdoor VIO testing)
            _chkForceVision = new CheckBox
            {
                Text = "Force Vision (SRC2) - outdoor VIO test",
                Font = new Font("Segoe UI", 9),
                ForeColor = WARNING_COLOR,
                Location = new Point(15, y),
                AutoSize = true,
                Checked = false
            };
            _chkForceVision.CheckedChanged += ChkForceVision_CheckedChanged;
            _grpEkfSource.Controls.Add(_chkForceVision);

            this.Controls.Add(_grpEkfSource);
        }

        private void InitializeAutoSwitchTimer()
        {
            _autoSwitchTimer = new System.Timers.Timer(1000); // Check every second
            _autoSwitchTimer.Elapsed += AutoSwitchTimer_Elapsed;
            _autoSwitchTimer.AutoReset = true;
            _autoSwitchTimer.Start();
        }

        // ============================================================
        // Event Handlers
        // ============================================================

        private async void BtnApplySource_Click(object sender, EventArgs e)
        {
            var source = (DualLinkSender.EkfSource)(_cmbSource.SelectedIndex + 1);
            
            _btnApplySource.Enabled = false;
            _btnApplySource.Text = "...";

            try
            {
                var result = await _sender.SetEkfSource(source);
                
                if (result.Success)
                {
                    _currentSource = source;
                    UpdateSourceDisplay();
                    SourceChanged?.Invoke(this, new EkfSourceChangedEventArgs(source, false));
                }
                else
                {
                    MessageBox.Show(result.Message, "EKF Source Switch Failed", 
                        MessageBoxButtons.OK, MessageBoxIcon.Warning);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error switching EKF source: {ex.Message}", "Error",
                    MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                _btnApplySource.Enabled = true;
                _btnApplySource.Text = "Apply";
            }
        }

        private void ChkAutoSwitch_CheckedChanged(object sender, EventArgs e)
        {
            _autoSwitchEnabled = _chkAutoSwitch.Checked;
            
            if (_autoSwitchEnabled)
            {
                // Disable force vision when auto-switch is enabled
                _chkForceVision.Checked = false;
                _chkForceVision.Enabled = false;
            }
            else
            {
                _chkForceVision.Enabled = true;
            }
        }

        private void ChkForceVision_CheckedChanged(object sender, EventArgs e)
        {
            _forceVisionMode = _chkForceVision.Checked;
            
            if (_forceVisionMode)
            {
                // Immediately switch to Vision
                _ = SwitchSourceAsync(DualLinkSender.EkfSource.ExternalNav, true);
            }
        }

        private void AutoSwitchTimer_Elapsed(object sender, ElapsedEventArgs e)
        {
            try
            {
                // Get current GPS status from Mission Planner
                var cs = MainV2.comPort?.MAV?.cs;
                if (cs == null) return;

                int gpsFix = (int)cs.gpsstatus;
                bool hasGpsFix = gpsFix >= 3; // 3D fix or better
                
                // Update UI (thread-safe)
                this.BeginInvoke(new Action(() =>
                {
                    UpdateGpsStatus(gpsFix);
                    UpdateVisionStatus();
                }));

                // Handle force vision mode
                if (_forceVisionMode)
                {
                    if (_currentSource != DualLinkSender.EkfSource.ExternalNav)
                    {
                        _ = SwitchSourceAsync(DualLinkSender.EkfSource.ExternalNav, true);
                    }
                    return;
                }

                // Handle auto-switch
                if (!_autoSwitchEnabled) return;

                DualLinkSender.EkfSource targetSource;
                
                if (hasGpsFix)
                {
                    targetSource = DualLinkSender.EkfSource.GPS;
                }
                else
                {
                    targetSource = DualLinkSender.EkfSource.ExternalNav;
                }

                // Only switch if source changed
                if (targetSource != _lastAutoSource)
                {
                    _lastAutoSource = targetSource;
                    _ = SwitchSourceAsync(targetSource, true);
                }

                _lastGpsFix = gpsFix;
            }
            catch
            {
                // Ignore errors during auto-switch check
            }
        }

        // ============================================================
        // Helper Methods
        // ============================================================

        private async System.Threading.Tasks.Task SwitchSourceAsync(
            DualLinkSender.EkfSource source, bool isAutomatic)
        {
            try
            {
                var result = await _sender.SetEkfSource(source);
                
                if (result.Success)
                {
                    _currentSource = source;
                    
                    this.BeginInvoke(new Action(() =>
                    {
                        UpdateSourceDisplay();
                        _cmbSource.SelectedIndex = (int)source - 1;
                    }));
                    
                    SourceChanged?.Invoke(this, new EkfSourceChangedEventArgs(source, isAutomatic));
                }
            }
            catch
            {
                // Ignore switch errors
            }
        }

        private void UpdateSourceDisplay()
        {
            string sourceName = _currentSource switch
            {
                DualLinkSender.EkfSource.GPS => "GPS (SRC1)",
                DualLinkSender.EkfSource.ExternalNav => "Vision/VIO (SRC2)",
                DualLinkSender.EkfSource.OpticalFlow => "OptFlow (SRC3)",
                _ => "Unknown"
            };

            _lblCurrentSource.Text = $"Current: {sourceName}";

            // Update indicator color
            _statusIndicator.BackColor = _currentSource switch
            {
                DualLinkSender.EkfSource.GPS => SUCCESS_COLOR,
                DualLinkSender.EkfSource.ExternalNav => ACCENT_COLOR,
                DualLinkSender.EkfSource.OpticalFlow => WARNING_COLOR,
                _ => ERROR_COLOR
            };
        }

        private void UpdateGpsStatus(int gpsFix)
        {
            string status = gpsFix switch
            {
                0 => "No GPS",
                1 => "No Fix",
                2 => "2D Fix",
                3 => "3D Fix",
                4 => "DGPS",
                5 => "RTK Float",
                6 => "RTK Fixed",
                _ => $"Fix: {gpsFix}"
            };

            _lblGpsStatus.Text = $"GPS: {status}";
            _lblGpsStatus.ForeColor = gpsFix >= 3 ? SUCCESS_COLOR : ERROR_COLOR;
        }

        private void UpdateVisionStatus()
        {
            // Check if Jetson is connected and VIO is running
            bool vioActive = _sender?.IsJetsonConnected ?? false;
            
            _lblVisionStatus.Text = vioActive ? "| Vision: Active" : "| Vision: Offline";
            _lblVisionStatus.ForeColor = vioActive ? SUCCESS_COLOR : WARNING_COLOR;
        }

        /// <summary>
        /// Manually set the current source (call after reading from ArduPilot params).
        /// </summary>
        public void SetCurrentSource(DualLinkSender.EkfSource source)
        {
            _currentSource = source;
            _lastAutoSource = source;
            
            if (_cmbSource != null)
            {
                _cmbSource.SelectedIndex = (int)source - 1;
            }
            UpdateSourceDisplay();
        }

        // ============================================================
        // Cleanup
        // ============================================================

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _autoSwitchTimer?.Stop();
                _autoSwitchTimer?.Dispose();
            }
            base.Dispose(disposing);
        }
    }

    /// <summary>
    /// Event args for EKF source change events.
    /// </summary>
    public class EkfSourceChangedEventArgs : EventArgs
    {
        public DualLinkSender.EkfSource NewSource { get; }
        public bool IsAutomatic { get; }

        public EkfSourceChangedEventArgs(DualLinkSender.EkfSource newSource, bool isAutomatic)
        {
            NewSource = newSource;
            IsAutomatic = isAutomatic;
        }
    }
}
