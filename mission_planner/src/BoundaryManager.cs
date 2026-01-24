// ============================================================
// NOMAD Flight Boundary Manager
// ============================================================
// Manages flight boundaries for AEAC 2026 competition.
// Supports soft (warning) and hard (kill required) boundaries.
// Provides real-time boundary checking and warnings.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Media;
using System.Timers;
using System.Windows.Forms;
using MissionPlanner.Utilities;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Boundary violation event arguments.
    /// </summary>
    public class BoundaryViolationEventArgs : EventArgs
    {
        public string BoundaryType { get; set; } // "soft" or "hard"
        public string BoundaryName { get; set; }
        public GpsPoint DronePosition { get; set; }
        public double? AltitudeAgl { get; set; }
        public string RequiredAction { get; set; }
        public DateTime Timestamp { get; set; }
    }

    /// <summary>
    /// Boundary status changed event arguments.
    /// </summary>
    public class BoundaryStatusEventArgs : EventArgs
    {
        public string Status { get; set; } // "inside", "soft_violation", "hard_violation"
        public double? DistanceToBoundaryMeters { get; set; }
        public string NearestBoundaryName { get; set; }
    }

    /// <summary>
    /// Manages flight boundaries and provides violation monitoring.
    /// </summary>
    public class BoundaryMonitor : IDisposable
    {
        private readonly MissionConfig _missionConfig;
        private readonly NOMADConfig _config;
        private System.Timers.Timer _monitorTimer;
        private string _lastStatus = "inside";
        private DateTime? _hardViolationStart;
        private bool _isDisposed;
        
        /// <summary>
        /// Fired when a boundary violation is detected.
        /// </summary>
        public event EventHandler<BoundaryViolationEventArgs> BoundaryViolation;
        
        /// <summary>
        /// Fired when boundary status changes.
        /// </summary>
        public event EventHandler<BoundaryStatusEventArgs> BoundaryStatusChanged;

        /// <summary>
        /// Current boundary status.
        /// </summary>
        public string CurrentStatus { get; private set; } = "inside";

        /// <summary>
        /// Time remaining before auto-kill (seconds), null if not in hard violation.
        /// </summary>
        public int? KillCountdown { get; private set; }

        /// <summary>
        /// Is boundary monitoring active.
        /// </summary>
        public bool IsMonitoring { get; private set; }

        public BoundaryMonitor(MissionConfig missionConfig, NOMADConfig config)
        {
            _missionConfig = missionConfig;
            _config = config;
        }

        /// <summary>
        /// Start boundary monitoring.
        /// </summary>
        public void StartMonitoring(int intervalMs = 500)
        {
            if (IsMonitoring) return;

            _monitorTimer = new System.Timers.Timer(intervalMs);
            _monitorTimer.Elapsed += MonitorTimer_Elapsed;
            _monitorTimer.AutoReset = true;
            _monitorTimer.Start();
            IsMonitoring = true;

            Console.WriteLine("NOMAD: Boundary monitoring started");
        }

        /// <summary>
        /// Stop boundary monitoring.
        /// </summary>
        public void StopMonitoring()
        {
            if (!IsMonitoring) return;

            _monitorTimer?.Stop();
            _monitorTimer?.Dispose();
            _monitorTimer = null;
            IsMonitoring = false;
            _hardViolationStart = null;
            KillCountdown = null;

            Console.WriteLine("NOMAD: Boundary monitoring stopped");
        }

        private void MonitorTimer_Elapsed(object sender, ElapsedEventArgs e)
        {
            try
            {
                CheckBoundaries();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Boundary check error - {ex.Message}");
            }
        }

        /// <summary>
        /// Check boundaries for current drone position.
        /// </summary>
        public void CheckBoundaries()
        {
            // Get current position from Mission Planner
            var lat = MissionPlanner.MainV2.comPort?.MAV?.cs?.lat ?? 0;
            var lon = MissionPlanner.MainV2.comPort?.MAV?.cs?.lng ?? 0;
            var altAgl = MissionPlanner.MainV2.comPort?.MAV?.cs?.alt ?? 0; // Alt above home

            if (lat == 0 && lon == 0) return; // No position

            var position = new GpsPoint(lat, lon);
            var status = _missionConfig.CheckBoundaryStatus(position, altAgl);

            // Handle status change
            if (status != _lastStatus)
            {
                HandleStatusChange(status, position, altAgl);
                _lastStatus = status;
            }

            // Update kill countdown for hard violations
            if (status == "hard_violation")
            {
                if (_hardViolationStart.HasValue)
                {
                    var elapsed = (DateTime.Now - _hardViolationStart.Value).TotalSeconds;
                    var remaining = _missionConfig.Failsafe.HardBoundaryKillDelaySec - (int)elapsed;
                    KillCountdown = Math.Max(0, remaining);
                }
            }
            else
            {
                _hardViolationStart = null;
                KillCountdown = null;
            }

            CurrentStatus = status;
        }

        private void HandleStatusChange(string newStatus, GpsPoint position, double altAgl)
        {
            CurrentStatus = newStatus;

            // Fire status changed event
            BoundaryStatusChanged?.Invoke(this, new BoundaryStatusEventArgs
            {
                Status = newStatus,
                NearestBoundaryName = newStatus == "soft_violation" 
                    ? _missionConfig.SoftBoundary.Name 
                    : _missionConfig.HardBoundary.Name,
            });

            if (newStatus == "soft_violation")
            {
                HandleSoftViolation(position, altAgl);
            }
            else if (newStatus == "hard_violation")
            {
                HandleHardViolation(position, altAgl);
            }
            else
            {
                // Back inside boundaries
                _hardViolationStart = null;
                KillCountdown = null;
            }
        }

        private void HandleSoftViolation(GpsPoint position, double altAgl)
        {
            var args = new BoundaryViolationEventArgs
            {
                BoundaryType = "soft",
                BoundaryName = _missionConfig.SoftBoundary.Name,
                DronePosition = position,
                AltitudeAgl = altAgl,
                RequiredAction = "Turn around - approaching boundary",
                Timestamp = DateTime.Now,
            };

            // Log violation
            _missionConfig.BoundaryViolations.Add(new BoundaryViolation
            {
                Timestamp = args.Timestamp,
                BoundaryName = args.BoundaryName,
                BoundaryType = args.BoundaryType,
                DronePosition = position,
                Action = args.RequiredAction,
            });
            _missionConfig.Save();

            // Fire event
            BoundaryViolation?.Invoke(this, args);

            // Audio warning
            if (_missionConfig.Failsafe.EnableAudioWarnings)
            {
                PlayWarningSound(false);
            }
        }

        private void HandleHardViolation(GpsPoint position, double altAgl)
        {
            if (!_hardViolationStart.HasValue)
            {
                _hardViolationStart = DateTime.Now;
            }

            var args = new BoundaryViolationEventArgs
            {
                BoundaryType = "hard",
                BoundaryName = _missionConfig.HardBoundary.Name,
                DronePosition = position,
                AltitudeAgl = altAgl,
                RequiredAction = $"KILL REQUIRED within {_missionConfig.Failsafe.HardBoundaryKillDelaySec} seconds!",
                Timestamp = DateTime.Now,
            };

            // Log violation
            _missionConfig.BoundaryViolations.Add(new BoundaryViolation
            {
                Timestamp = args.Timestamp,
                BoundaryName = args.BoundaryName,
                BoundaryType = args.BoundaryType,
                DronePosition = position,
                Action = args.RequiredAction,
            });
            _missionConfig.Save();

            // Fire event
            BoundaryViolation?.Invoke(this, args);

            // Urgent audio warning
            if (_missionConfig.Failsafe.EnableAudioWarnings)
            {
                PlayWarningSound(true);
            }
        }

        private void PlayWarningSound(bool urgent)
        {
            try
            {
                if (urgent)
                {
                    SystemSounds.Hand.Play();
                }
                else
                {
                    SystemSounds.Exclamation.Play();
                }
            }
            catch { /* Ignore sound errors */ }
        }

        public void Dispose()
        {
            if (_isDisposed) return;
            _isDisposed = true;
            StopMonitoring();
        }
    }

    /// <summary>
    /// UI Panel for configuring and monitoring flight boundaries.
    /// </summary>
    public class BoundaryConfigPanel : UserControl
    {
        private readonly MissionConfig _missionConfig;
        private readonly BoundaryMonitor _monitor;
        
        private DataGridView _dgvSoftBoundary;
        private DataGridView _dgvHardBoundary;
        private Label _lblStatus;
        private Label _lblCountdown;
        private Panel _statusPanel;
        private Button _btnImportKml;
        private Button _btnImportGoogleMaps;
        private Button _btnClearSoft;
        private Button _btnClearHard;
        private NumericUpDown _nudMaxAlt;
        private CheckBox _chkEnableMonitoring;
        private ComboBox _cmbSoftAction;
        private ComboBox _cmbHardAction;
        private NumericUpDown _nudKillDelay;

        public BoundaryConfigPanel(MissionConfig missionConfig, BoundaryMonitor monitor)
        {
            _missionConfig = missionConfig;
            _monitor = monitor;

            BackColor = Color.FromArgb(30, 30, 30);
            Dock = DockStyle.Fill;

            InitializeComponents();
            LoadConfiguration();

            // Subscribe to monitor events
            _monitor.BoundaryStatusChanged += Monitor_BoundaryStatusChanged;
            _monitor.BoundaryViolation += Monitor_BoundaryViolation;
        }

        private void InitializeComponents()
        {
            var mainPanel = new Panel
            {
                Dock = DockStyle.Fill,
                AutoScroll = true,
                Padding = new Padding(10),
            };

            int yOffset = 10;

            // ============================================================
            // Status Panel
            // ============================================================
            _statusPanel = new Panel
            {
                Location = new Point(10, yOffset),
                Size = new Size(550, 60),
                BackColor = Color.FromArgb(40, 100, 40), // Green = inside
            };

            _lblStatus = new Label
            {
                Text = "[OK] Inside Boundaries",
                Font = new Font("Segoe UI", 14, FontStyle.Bold),
                ForeColor = Color.White,
                Location = new Point(15, 8),
                AutoSize = true,
            };
            _statusPanel.Controls.Add(_lblStatus);

            _lblCountdown = new Label
            {
                Text = "",
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = Color.Yellow,
                Location = new Point(15, 35),
                AutoSize = true,
                Visible = false,
            };
            _statusPanel.Controls.Add(_lblCountdown);

            mainPanel.Controls.Add(_statusPanel);
            yOffset += 70;

            // ============================================================
            // Monitoring Controls
            // ============================================================
            var monitorGroup = new GroupBox
            {
                Text = "Monitoring Settings",
                ForeColor = Color.FromArgb(0, 150, 200),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(550, 100),
                BackColor = Color.FromArgb(45, 45, 48),
            };

            _chkEnableMonitoring = new CheckBox
            {
                Text = "Enable Real-time Boundary Monitoring",
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
                Location = new Point(15, 25),
                AutoSize = true,
                Checked = true,
            };
            _chkEnableMonitoring.CheckedChanged += ChkEnableMonitoring_Changed;
            monitorGroup.Controls.Add(_chkEnableMonitoring);

            var lblMaxAlt = new Label
            {
                Text = "Max Altitude AGL:",
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
                Location = new Point(15, 55),
                AutoSize = true,
            };
            monitorGroup.Controls.Add(lblMaxAlt);

            _nudMaxAlt = new NumericUpDown
            {
                Location = new Point(130, 53),
                Size = new Size(80, 25),
                Minimum = 10,
                Maximum = 400,
                Value = (decimal)_missionConfig.MaxAltitudeAglMeters,
                DecimalPlaces = 0,
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
            };
            _nudMaxAlt.ValueChanged += (s, e) => 
            {
                _missionConfig.MaxAltitudeAglMeters = (double)_nudMaxAlt.Value;
                _missionConfig.Save();
            };
            monitorGroup.Controls.Add(_nudMaxAlt);

            var lblMeters = new Label
            {
                Text = "m (122m = 400ft)",
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
                Location = new Point(215, 57),
                AutoSize = true,
            };
            monitorGroup.Controls.Add(lblMeters);

            mainPanel.Controls.Add(monitorGroup);
            yOffset += 110;

            // ============================================================
            // Soft Boundary Configuration
            // ============================================================
            var softGroup = new GroupBox
            {
                Text = "Soft Boundary (Yellow - Warning)",
                ForeColor = Color.Yellow,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(550, 200),
                BackColor = Color.FromArgb(45, 45, 48),
            };

            _dgvSoftBoundary = CreateBoundaryGrid();
            _dgvSoftBoundary.Location = new Point(15, 25);
            _dgvSoftBoundary.Size = new Size(400, 130);
            softGroup.Controls.Add(_dgvSoftBoundary);

            var btnAddSoft = new Button
            {
                Text = "+",
                Location = new Point(420, 25),
                Size = new Size(30, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 120, 60),
                ForeColor = Color.White,
            };
            btnAddSoft.Click += (s, e) => AddBoundaryPoint(_dgvSoftBoundary, _missionConfig.SoftBoundary);
            softGroup.Controls.Add(btnAddSoft);

            _btnClearSoft = new Button
            {
                Text = "Clear",
                Location = new Point(455, 25),
                Size = new Size(50, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(120, 60, 60),
                ForeColor = Color.White,
            };
            _btnClearSoft.Click += (s, e) => ClearBoundary(_dgvSoftBoundary, _missionConfig.SoftBoundary);
            softGroup.Controls.Add(_btnClearSoft);

            var lblSoftAction = new Label
            {
                Text = "On Violation:",
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
                Location = new Point(15, 165),
                AutoSize = true,
            };
            softGroup.Controls.Add(lblSoftAction);

            _cmbSoftAction = new ComboBox
            {
                Location = new Point(100, 162),
                Size = new Size(200, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
            };
            _cmbSoftAction.Items.AddRange(new object[] { "Warn (Audio)", "Warn (Visual)", "Warn (Both)", "Return to Boundary" });
            _cmbSoftAction.SelectedIndex = 2;
            _cmbSoftAction.SelectedIndexChanged += (s, e) => 
            {
                _missionConfig.Failsafe.SoftBoundaryAction = _cmbSoftAction.SelectedItem.ToString().ToLower().Replace(" ", "_").Replace("(", "").Replace(")", "");
                _missionConfig.Save();
            };
            softGroup.Controls.Add(_cmbSoftAction);

            mainPanel.Controls.Add(softGroup);
            yOffset += 210;

            // ============================================================
            // Hard Boundary Configuration
            // ============================================================
            var hardGroup = new GroupBox
            {
                Text = "Hard Boundary (Red - Kill Required)",
                ForeColor = Color.Red,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(550, 220),
                BackColor = Color.FromArgb(45, 45, 48),
            };

            _dgvHardBoundary = CreateBoundaryGrid();
            _dgvHardBoundary.Location = new Point(15, 25);
            _dgvHardBoundary.Size = new Size(400, 130);
            hardGroup.Controls.Add(_dgvHardBoundary);

            var btnAddHard = new Button
            {
                Text = "+",
                Location = new Point(420, 25),
                Size = new Size(30, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 120, 60),
                ForeColor = Color.White,
            };
            btnAddHard.Click += (s, e) => AddBoundaryPoint(_dgvHardBoundary, _missionConfig.HardBoundary);
            hardGroup.Controls.Add(btnAddHard);

            _btnClearHard = new Button
            {
                Text = "Clear",
                Location = new Point(455, 25),
                Size = new Size(50, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(120, 60, 60),
                ForeColor = Color.White,
            };
            _btnClearHard.Click += (s, e) => ClearBoundary(_dgvHardBoundary, _missionConfig.HardBoundary);
            hardGroup.Controls.Add(_btnClearHard);

            var lblHardAction = new Label
            {
                Text = "On Violation:",
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
                Location = new Point(15, 165),
                AutoSize = true,
            };
            hardGroup.Controls.Add(lblHardAction);

            _cmbHardAction = new ComboBox
            {
                Location = new Point(100, 162),
                Size = new Size(200, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
            };
            _cmbHardAction.Items.AddRange(new object[] { "Warn and Kill", "Auto Kill", "Warn Only" });
            _cmbHardAction.SelectedIndex = 0;
            hardGroup.Controls.Add(_cmbHardAction);

            var lblKillDelay = new Label
            {
                Text = "Kill Delay:",
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
                Location = new Point(320, 165),
                AutoSize = true,
            };
            hardGroup.Controls.Add(lblKillDelay);

            _nudKillDelay = new NumericUpDown
            {
                Location = new Point(390, 162),
                Size = new Size(60, 25),
                Minimum = 1,
                Maximum = 30,
                Value = _missionConfig.Failsafe.HardBoundaryKillDelaySec,
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
            };
            _nudKillDelay.ValueChanged += (s, e) => 
            {
                _missionConfig.Failsafe.HardBoundaryKillDelaySec = (int)_nudKillDelay.Value;
                _missionConfig.Save();
            };
            hardGroup.Controls.Add(_nudKillDelay);

            var lblSec = new Label
            {
                Text = "sec",
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
                Location = new Point(455, 167),
                AutoSize = true,
            };
            hardGroup.Controls.Add(lblSec);

            mainPanel.Controls.Add(hardGroup);
            yOffset += 230;

            // ============================================================
            // Import Buttons
            // ============================================================
            var importGroup = new GroupBox
            {
                Text = "Import Boundaries",
                ForeColor = Color.FromArgb(0, 150, 200),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(550, 70),
                BackColor = Color.FromArgb(45, 45, 48),
            };

            _btnImportKml = new Button
            {
                Text = "Import from KML/KMZ",
                Location = new Point(15, 25),
                Size = new Size(170, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 60, 63),
                ForeColor = Color.White,
            };
            _btnImportKml.Click += BtnImportKml_Click;
            importGroup.Controls.Add(_btnImportKml);

            _btnImportGoogleMaps = new Button
            {
                Text = "Paste Google Maps Link",
                Location = new Point(195, 25),
                Size = new Size(170, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 60, 63),
                ForeColor = Color.White,
            };
            _btnImportGoogleMaps.Click += BtnImportGoogleMaps_Click;
            importGroup.Controls.Add(_btnImportGoogleMaps);

            var btnGetFromMP = new Button
            {
                Text = "Get from MP Fence",
                Location = new Point(375, 25),
                Size = new Size(160, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 60, 63),
                ForeColor = Color.White,
            };
            btnGetFromMP.Click += BtnGetFromMP_Click;
            importGroup.Controls.Add(btnGetFromMP);

            mainPanel.Controls.Add(importGroup);

            Controls.Add(mainPanel);
        }

        private DataGridView CreateBoundaryGrid()
        {
            var dgv = new DataGridView
            {
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
                    ForeColor = Color.White,
                    SelectionBackColor = Color.FromArgb(0, 122, 204),
                    SelectionForeColor = Color.White,
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
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
            };

            return dgv;
        }

        private void LoadConfiguration()
        {
            // Load soft boundary
            _dgvSoftBoundary.Rows.Clear();
            foreach (var point in _missionConfig.SoftBoundary.Vertices)
            {
                _dgvSoftBoundary.Rows.Add(point.Lat.ToString("F6"), point.Lon.ToString("F6"));
            }

            // Load hard boundary
            _dgvHardBoundary.Rows.Clear();
            foreach (var point in _missionConfig.HardBoundary.Vertices)
            {
                _dgvHardBoundary.Rows.Add(point.Lat.ToString("F6"), point.Lon.ToString("F6"));
            }
        }

        private void AddBoundaryPoint(DataGridView dgv, FlightBoundary boundary)
        {
            // Get current position or use default
            var lat = MissionPlanner.MainV2.comPort?.MAV?.cs?.lat ?? 45.0;
            var lon = MissionPlanner.MainV2.comPort?.MAV?.cs?.lng ?? -73.0;

            var point = new GpsPoint(lat, lon);
            boundary.Vertices.Add(point);
            dgv.Rows.Add(lat.ToString("F6"), lon.ToString("F6"));
            
            _missionConfig.Save();
        }

        private void ClearBoundary(DataGridView dgv, FlightBoundary boundary)
        {
            if (CustomMessageBox.Show("Clear all boundary points?", "Confirm", 
                CustomMessageBox.MessageBoxButtons.YesNo) == CustomMessageBox.DialogResult.Yes)
            {
                boundary.Vertices.Clear();
                dgv.Rows.Clear();
                _missionConfig.Save();
            }
        }

        private void ChkEnableMonitoring_Changed(object sender, EventArgs e)
        {
            if (_chkEnableMonitoring.Checked)
            {
                _monitor.StartMonitoring();
            }
            else
            {
                _monitor.StopMonitoring();
            }
        }

        private void Monitor_BoundaryStatusChanged(object sender, BoundaryStatusEventArgs e)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => Monitor_BoundaryStatusChanged(sender, e)));
                return;
            }

            switch (e.Status)
            {
                case "inside":
                    _statusPanel.BackColor = Color.FromArgb(40, 100, 40);
                    _lblStatus.Text = "[OK] Inside Boundaries";
                    _lblCountdown.Visible = false;
                    break;

                case "soft_violation":
                    _statusPanel.BackColor = Color.FromArgb(180, 150, 0);
                    _lblStatus.Text = "[!] SOFT BOUNDARY - Turn Around!";
                    _lblCountdown.Visible = false;
                    break;

                case "hard_violation":
                    _statusPanel.BackColor = Color.FromArgb(180, 40, 40);
                    _lblStatus.Text = "[!!] HARD BOUNDARY VIOLATION!";
                    _lblCountdown.Visible = true;
                    break;
            }
        }

        private void Monitor_BoundaryViolation(object sender, BoundaryViolationEventArgs e)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => Monitor_BoundaryViolation(sender, e)));
                return;
            }

            if (e.BoundaryType == "hard" && _monitor.KillCountdown.HasValue)
            {
                _lblCountdown.Text = $"KILL IN {_monitor.KillCountdown} SECONDS!";
            }
        }

        private void BtnImportKml_Click(object sender, EventArgs e)
        {
            using (var ofd = new OpenFileDialog
            {
                Filter = "KML/KMZ Files|*.kml;*.kmz|All Files|*.*",
                Title = "Import Boundary from KML"
            })
            {
                if (ofd.ShowDialog() == DialogResult.OK)
                {
                    try
                    {
                        // Parse KML file for polygon coordinates
                        // This is a simplified parser - you may need to use a proper KML library
                        var content = File.ReadAllText(ofd.FileName);
                        var points = ParseKmlCoordinates(content);
                        
                        if (points.Count > 0)
                        {
                            var result = CustomMessageBox.Show(
                                "Import as Soft or Hard boundary?",
                                "Select Boundary Type",
                                CustomMessageBox.MessageBoxButtons.YesNo);

                            if (result == CustomMessageBox.DialogResult.Yes)
                            {
                                // Soft boundary
                                _missionConfig.SoftBoundary.Vertices = points;
                                LoadConfiguration();
                            }
                            else
                            {
                                // Hard boundary
                                _missionConfig.HardBoundary.Vertices = points;
                                LoadConfiguration();
                            }
                            _missionConfig.Save();
                            CustomMessageBox.Show($"Imported {points.Count} boundary points", "Success");
                        }
                    }
                    catch (Exception ex)
                    {
                        CustomMessageBox.Show($"Error importing KML: {ex.Message}", "Error");
                    }
                }
            }
        }

        private List<GpsPoint> ParseKmlCoordinates(string kmlContent)
        {
            var points = new List<GpsPoint>();
            
            // Simple regex-based parsing (for production, use a proper KML library)
            var coordsMatch = System.Text.RegularExpressions.Regex.Match(
                kmlContent, @"<coordinates>\s*(.*?)\s*</coordinates>", 
                System.Text.RegularExpressions.RegexOptions.Singleline);

            if (coordsMatch.Success)
            {
                var coordString = coordsMatch.Groups[1].Value;
                var coordPairs = coordString.Split(new[] { ' ', '\n', '\r', '\t' }, 
                    StringSplitOptions.RemoveEmptyEntries);

                foreach (var pair in coordPairs)
                {
                    var parts = pair.Split(',');
                    if (parts.Length >= 2)
                    {
                        if (double.TryParse(parts[0], out double lon) &&
                            double.TryParse(parts[1], out double lat))
                        {
                            points.Add(new GpsPoint(lat, lon));
                        }
                    }
                }
            }

            return points;
        }

        private void BtnImportGoogleMaps_Click(object sender, EventArgs e)
        {
            var input = Microsoft.VisualBasic.Interaction.InputBox(
                "Paste Google Maps coordinates (one per line, format: lat,lon or lon,lat):",
                "Import from Google Maps",
                "");

            if (string.IsNullOrWhiteSpace(input)) return;

            try
            {
                var points = new List<GpsPoint>();
                var lines = input.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);

                foreach (var line in lines)
                {
                    var parts = line.Split(new[] { ',', '\t', ' ' }, StringSplitOptions.RemoveEmptyEntries);
                    if (parts.Length >= 2)
                    {
                        if (double.TryParse(parts[0], out double val1) &&
                            double.TryParse(parts[1], out double val2))
                        {
                            // Determine if format is lat,lon or lon,lat
                            // Latitude is typically smaller in absolute value for most locations
                            double lat, lon;
                            if (Math.Abs(val1) <= 90 && Math.Abs(val2) > 90)
                            {
                                lat = val1;
                                lon = val2;
                            }
                            else if (Math.Abs(val2) <= 90 && Math.Abs(val1) > 90)
                            {
                                lat = val2;
                                lon = val1;
                            }
                            else
                            {
                                // Default to lat,lon
                                lat = val1;
                                lon = val2;
                            }
                            points.Add(new GpsPoint(lat, lon));
                        }
                    }
                }

                if (points.Count > 0)
                {
                    var result = CustomMessageBox.Show(
                        $"Import {points.Count} points as Soft (Yes) or Hard (No) boundary?",
                        "Select Boundary Type",
                        CustomMessageBox.MessageBoxButtons.YesNo);

                    if (result == CustomMessageBox.DialogResult.Yes)
                    {
                        _missionConfig.SoftBoundary.Vertices = points;
                    }
                    else
                    {
                        _missionConfig.HardBoundary.Vertices = points;
                    }
                    _missionConfig.Save();
                    LoadConfiguration();
                    CustomMessageBox.Show($"Imported {points.Count} boundary points", "Success");
                }
                else
                {
                    CustomMessageBox.Show("No valid coordinates found", "Warning");
                }
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error parsing coordinates: {ex.Message}", "Error");
            }
        }

        private void BtnGetFromMP_Click(object sender, EventArgs e)
        {
            try
            {
                // Try to get fence from Mission Planner
                var fence = MissionPlanner.MainV2.comPort?.MAV?.fencepoints;
                if (fence != null && fence.Count > 0)
                {
                    var points = fence.Select(p => new GpsPoint(p.lat, p.lng)).ToList();

                    var result = CustomMessageBox.Show(
                        $"Import {points.Count} fence points as Soft (Yes) or Hard (No) boundary?",
                        "Select Boundary Type",
                        CustomMessageBox.MessageBoxButtons.YesNo);

                    if (result == CustomMessageBox.DialogResult.Yes)
                    {
                        _missionConfig.SoftBoundary.Vertices = points;
                    }
                    else
                    {
                        _missionConfig.HardBoundary.Vertices = points;
                    }
                    _missionConfig.Save();
                    LoadConfiguration();
                    CustomMessageBox.Show($"Imported {points.Count} fence points", "Success");
                }
                else
                {
                    CustomMessageBox.Show("No fence points found in Mission Planner", "Warning");
                }
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error getting fence: {ex.Message}", "Error");
            }
        }
    }
}
