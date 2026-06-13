// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Boundary View - Flight Boundary Configuration & Monitoring
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Windows.Forms;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Boundary preset for save/load functionality
    /// </summary>
    public class BoundaryPreset
    {
        public string Name { get; set; }
        public string Description { get; set; }
        public DateTime CreatedAt { get; set; }
        public List<GpsPoint> SoftBoundary { get; set; } = new List<GpsPoint>();
        public List<GpsPoint> HardBoundary { get; set; } = new List<GpsPoint>();
        public double MaxAltitudeMeters { get; set; } = 122.0; // 400ft
    }

    public partial class NOMADBoundaryView : NOMADViewBase, IUpdatableView
    {
        private readonly GeofenceConfig _missionConfig;
        private readonly NOMADConfig _config;
        private readonly BoundaryMonitor _monitor;

        // Status display
        private Panel _statusPanel;
        private Label _lblStatus;
        private Label _lblCountdown;
        private Label _lblAltitude;
        private Label _lblPosition;

        // Boundary grids
        private DataGridView _dgvSoftBoundary;
        private DataGridView _dgvHardBoundary;

        private CheckBox _chkEnableMonitoring;
        private NumericUpDown _nudMaxAlt;

        // Building location

        // Violation action controls
        private ComboBox _cmbSoftAction;
        private ComboBox _cmbHardAction;
        private NumericUpDown _nudKillDelay;

        // Preset management
        private ComboBox _cmbPresets;
        private List<BoundaryPreset> _presets = new List<BoundaryPreset>();
        private static readonly string PresetsDir = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "Mission Planner", "plugins", "NOMAD", "boundary_presets");

        public NOMADBoundaryView(GeofenceConfig missionConfig, NOMADConfig config, BoundaryMonitor monitor)
        {
            _missionConfig = missionConfig ?? GeofenceConfig.Load();
            _config = config;
            _monitor = monitor;

            LoadPresets();
            InitializeUI();
            LoadBoundaries();

            // Subscribe to monitor events
            if (_monitor != null)
            {
                _monitor.BoundaryStatusChanged += Monitor_BoundaryStatusChanged;
                _monitor.BoundaryViolation += Monitor_BoundaryViolation;
            }
        }

        // Return location fields
        private TextBox _txtReturnLat;
        private TextBox _txtReturnLon;

        private void InitializeUI()
        {
            // ============================================================
            // Two-column layout: Left = boundaries, Right = settings
            // ============================================================
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 2,
                Padding = new Padding(0),
            };
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            // Row 0: status (fixed height), Row 1: content (fill)
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 75));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            // ============================================================
            // Status Panel (spans both columns)
            // ============================================================
            _statusPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(80, 80, 90),
                Margin = new Padding(3),
            };

            _lblStatus = new Label
            {
                Text = "[?] Waiting for GPS Position",
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

            _lblPosition = new Label
            {
                Text = "Position: --",
                Font = new Font("Consolas", 8),
                ForeColor = Color.White,
                Location = new Point(15, 50),
                AutoSize = true,
            };
            _statusPanel.Controls.Add(_lblPosition);

            _lblAltitude = new Label
            {
                Text = "Alt: -- / 122m",
                Font = new Font("Consolas", 8),
                ForeColor = Color.White,
                Location = new Point(280, 50),
                AutoSize = true,
            };
            _statusPanel.Controls.Add(_lblAltitude);

            mainLayout.Controls.Add(_statusPanel, 0, 0);
            mainLayout.SetColumnSpan(_statusPanel, 2);

            // ============================================================
            // LEFT COLUMN: Boundary point grids + Import/Export
            // ============================================================
            var leftScroll = new Panel { Dock = DockStyle.Fill, AutoScroll = true, Padding = new Padding(3) };

            // Dock.Top stacking: add in REVERSE order (last added = top)
            // 3) Import/Export card (bottom)
            var importCard = CreateCard("IMPORT / EXPORT");
            importCard.Dock = DockStyle.Top;
            importCard.Height = 130;
            importCard.Margin = new Padding(0, 3, 0, 0);

            // Auto-draw toggle: when checked, AutoDrawBoundariesIfEnabled()
            // pushes imported/edited polygons to the map without a manual
            // refresh. Named so AutoDrawBoundariesIfEnabled can Find it.
            var chkAutoDraw = new CheckBox
            {
                Name = "chkAutoDraw",
                Text = "Auto-draw on map",
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
                Location = new Point(10, 92),
                AutoSize = true,
                Checked = true,
            };
            importCard.Controls.Add(chkAutoDraw);

            var lblImport = new Label { Text = "Import:", Font = new Font("Segoe UI", 7.5f, FontStyle.Bold), ForeColor = TEXT_SECONDARY, Location = new Point(10, 42), AutoSize = true };
            importCard.Controls.Add(lblImport);
            var btnImportKml = CreateButton("KML", ACCENT_COLOR, 48, 22); btnImportKml.Font = new Font("Segoe UI", 7, FontStyle.Bold); btnImportKml.Location = new Point(60, 39); btnImportKml.Click += BtnImportKml_Click; importCard.Controls.Add(btnImportKml);
            var btnImportCSV = CreateButton("Coords", ACCENT_COLOR, 55, 22); btnImportCSV.Font = new Font("Segoe UI", 7, FontStyle.Bold); btnImportCSV.Location = new Point(112, 39); btnImportCSV.Click += BtnImportGoogleMaps_Click; importCard.Controls.Add(btnImportCSV);
            var btnGetFromMP = CreateButton("MP Fence", ACCENT_COLOR, 65, 22); btnGetFromMP.Font = new Font("Segoe UI", 7, FontStyle.Bold); btnGetFromMP.Location = new Point(171, 39); btnGetFromMP.Click += BtnGetFromMP_Click; importCard.Controls.Add(btnGetFromMP);
            var lblExport = new Label { Text = "Send Fence:", Font = new Font("Segoe UI", 7.5f, FontStyle.Bold), ForeColor = TEXT_SECONDARY, Location = new Point(10, 68), AutoSize = true };
            importCard.Controls.Add(lblExport);
            var btnExportToMPFence = CreateButton("Push to MP + Drone", SUCCESS_COLOR, 130, 22); btnExportToMPFence.Font = new Font("Segoe UI", 7, FontStyle.Bold); btnExportToMPFence.Location = new Point(75, 65); btnExportToMPFence.Click += BtnExportToMPFence_Click; importCard.Controls.Add(btnExportToMPFence);
            var btnClearVehicleFence = CreateButton("Clear on Veh", ERROR_COLOR, 75, 22); btnClearVehicleFence.Font = new Font("Segoe UI", 7, FontStyle.Bold); btnClearVehicleFence.Location = new Point(208, 65); btnClearVehicleFence.Click += BtnClearVehicleFence_Click; importCard.Controls.Add(btnClearVehicleFence);
            leftScroll.Controls.Add(importCard);

            // 2) Hard Boundary (middle)
            var hardCard = CreateCard("HARD BOUNDARY (Descend)");
            hardCard.Dock = DockStyle.Top;
            hardCard.Height = 210;
            hardCard.Margin = new Padding(0, 3, 0, 0);

            _dgvHardBoundary = CreateBoundaryGrid();
            _dgvHardBoundary.Location = new Point(10, 40);
            _dgvHardBoundary.Size = new Size(200, 110);
            _dgvHardBoundary.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;
            _dgvHardBoundary.CellValueChanged += (s, e) => SaveBoundaryFromGrid(_dgvHardBoundary, _missionConfig.HardBoundary);
            hardCard.Controls.Add(_dgvHardBoundary);
            var btnPasteHard = CreateButton("Paste", ACCENT_COLOR, 55, 24); btnPasteHard.Font = new Font("Segoe UI", 7.5f, FontStyle.Bold); btnPasteHard.Location = new Point(10, 158); btnPasteHard.Click += (s, e) => PasteCoordinates(_dgvHardBoundary, _missionConfig.HardBoundary, "hard"); hardCard.Controls.Add(btnPasteHard);
            var btnAddHard = CreateButton("+ Add", SUCCESS_COLOR, 45, 24); btnAddHard.Font = new Font("Segoe UI", 7.5f, FontStyle.Bold); btnAddHard.Location = new Point(70, 158); btnAddHard.Click += (s, e) => AddManualPoint(_dgvHardBoundary, _missionConfig.HardBoundary); hardCard.Controls.Add(btnAddHard);
            var btnDelHard = CreateButton("Del Pt", ACCENT_COLOR, 50, 24); btnDelHard.Font = new Font("Segoe UI", 7.5f, FontStyle.Bold); btnDelHard.Location = new Point(120, 158); btnDelHard.Click += (s, e) => DeleteSelectedPoint(_dgvHardBoundary, _missionConfig.HardBoundary); hardCard.Controls.Add(btnDelHard);
            var btnClearHard = CreateButton("Clear", ERROR_COLOR, 45, 24); btnClearHard.Font = new Font("Segoe UI", 7.5f, FontStyle.Bold); btnClearHard.Location = new Point(175, 158); btnClearHard.Click += (s, e) => ClearBoundary(_dgvHardBoundary, _missionConfig.HardBoundary); hardCard.Controls.Add(btnClearHard);
            var lblHardCount = new Label { Name = "lblHardCount", Text = $"{_missionConfig.HardBoundary.Vertices.Count} pts", Font = new Font("Segoe UI", 8, FontStyle.Bold), ForeColor = Color.Red, Location = new Point(228, 162), AutoSize = true };
            hardCard.Controls.Add(lblHardCount);
            var lblHardSaved = new Label { Name = "lblHardSaved", Text = "", Font = new Font("Segoe UI", 7), ForeColor = TEXT_SECONDARY, Location = new Point(10, 184), AutoSize = true };
            hardCard.Controls.Add(lblHardSaved);
            leftScroll.Controls.Add(hardCard);

            // 1) Soft Boundary (top - added last so it docks on top)
            var softCard = CreateCard("SOFT BOUNDARY (Warning)");
            softCard.Dock = DockStyle.Top;
            softCard.Height = 235;
            softCard.Margin = new Padding(0, 0, 0, 0);

            _dgvSoftBoundary = CreateBoundaryGrid();
            _dgvSoftBoundary.Location = new Point(10, 40);
            _dgvSoftBoundary.Size = new Size(200, 110);
            _dgvSoftBoundary.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;
            _dgvSoftBoundary.CellValueChanged += (s, e) => SaveBoundaryFromGrid(_dgvSoftBoundary, _missionConfig.SoftBoundary);
            softCard.Controls.Add(_dgvSoftBoundary);
            var btnPasteSoft = CreateButton("Paste", ACCENT_COLOR, 55, 24); btnPasteSoft.Font = new Font("Segoe UI", 7.5f, FontStyle.Bold); btnPasteSoft.Location = new Point(10, 158); btnPasteSoft.Click += (s, e) => PasteCoordinates(_dgvSoftBoundary, _missionConfig.SoftBoundary, "soft"); softCard.Controls.Add(btnPasteSoft);
            var btnAddSoft = CreateButton("+ Add", SUCCESS_COLOR, 45, 24); btnAddSoft.Font = new Font("Segoe UI", 7.5f, FontStyle.Bold); btnAddSoft.Location = new Point(70, 158); btnAddSoft.Click += (s, e) => AddManualPoint(_dgvSoftBoundary, _missionConfig.SoftBoundary); softCard.Controls.Add(btnAddSoft);
            var btnDelSoft = CreateButton("Del Pt", ACCENT_COLOR, 50, 24); btnDelSoft.Font = new Font("Segoe UI", 7.5f, FontStyle.Bold); btnDelSoft.Location = new Point(120, 158); btnDelSoft.Click += (s, e) => DeleteSelectedPoint(_dgvSoftBoundary, _missionConfig.SoftBoundary); softCard.Controls.Add(btnDelSoft);
            var btnClearSoft = CreateButton("Clear", ERROR_COLOR, 45, 24); btnClearSoft.Font = new Font("Segoe UI", 7.5f, FontStyle.Bold); btnClearSoft.Location = new Point(175, 158); btnClearSoft.Click += (s, e) => ClearBoundary(_dgvSoftBoundary, _missionConfig.SoftBoundary); softCard.Controls.Add(btnClearSoft);
            var lblSoftCount = new Label { Name = "lblSoftCount", Text = $"{_missionConfig.SoftBoundary.Vertices.Count} pts", Font = new Font("Segoe UI", 8, FontStyle.Bold), ForeColor = Color.Yellow, Location = new Point(228, 162), AutoSize = true };
            softCard.Controls.Add(lblSoftCount);

            // Derived mode: soft boundary auto-generated as hard boundary inset
            // inward by a configurable distance. Locks manual soft editing.
            var chkAutoSoft = new CheckBox { Text = "Auto: hard boundary −", ForeColor = Color.White, Font = new Font("Segoe UI", 8), Location = new Point(10, 184), AutoSize = true, Checked = _missionConfig.SoftBoundaryFromHard };
            softCard.Controls.Add(chkAutoSoft);
            var nudSoftInset = new NumericUpDown { Location = new Point(150, 182), Size = new Size(48, 22), Minimum = 1, Maximum = 100, Value = (decimal)Math.Max(1, Math.Min(100, _missionConfig.SoftBoundaryInsetMeters)), BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White };
            softCard.Controls.Add(nudSoftInset);
            var lblInsetM = new Label { Text = "m inward", Font = new Font("Segoe UI", 7), ForeColor = TEXT_SECONDARY, Location = new Point(201, 186), AutoSize = true };
            softCard.Controls.Add(lblInsetM);
            var lblSoftSaved = new Label { Name = "lblSoftSaved", Text = "", Font = new Font("Segoe UI", 7), ForeColor = TEXT_SECONDARY, Location = new Point(10, 210), AutoSize = true };
            softCard.Controls.Add(lblSoftSaved);

            Action applySoftAutoState = () =>
            {
                bool auto = chkAutoSoft.Checked;
                _dgvSoftBoundary.ReadOnly = auto;
                _dgvSoftBoundary.AllowUserToDeleteRows = !auto;
                btnPasteSoft.Enabled = !auto;
                btnAddSoft.Enabled = !auto;
                btnDelSoft.Enabled = !auto;
                btnClearSoft.Enabled = !auto;
            };
            applySoftAutoState();
            chkAutoSoft.CheckedChanged += (s, e) =>
            {
                _missionConfig.SoftBoundaryFromHard = chkAutoSoft.Checked;
                _missionConfig.Save();
                applySoftAutoState();
                UpdatePointCounts();          // triggers SyncSoftFromHard when enabled
                AutoDrawBoundariesIfEnabled();
            };
            nudSoftInset.ValueChanged += (s, e) =>
            {
                _missionConfig.SoftBoundaryInsetMeters = (double)nudSoftInset.Value;
                _missionConfig.Save();
                UpdatePointCounts();
                AutoDrawBoundariesIfEnabled();
            };

            leftScroll.Controls.Add(softCard);

            mainLayout.Controls.Add(leftScroll, 0, 1);

            // ============================================================
            // RIGHT COLUMN: Settings, return point, actions, presets
            // ============================================================
            var rightScroll = new Panel { Dock = DockStyle.Fill, AutoScroll = true, Padding = new Padding(3) };

            // Saved Boundaries card is added LAST (below) so it docks at TOP.
            // Keep variable here for later assembly.
            var presetCard = CreateCard("SAVED BOUNDARIES (Plugin Storage)");
            presetCard.Dock = DockStyle.Top;
            presetCard.Height = 110;
            presetCard.Margin = new Padding(0, 0, 0, 0);

            var lblPresetHint = new Label { Text = "Current points auto-save. Use this to save named copies you can reload later.", Font = new Font("Segoe UI", 7), ForeColor = TEXT_SECONDARY, Location = new Point(10, 22), AutoSize = false, Size = new Size(295, 28) };
            presetCard.Controls.Add(lblPresetHint);

            _cmbPresets = new ComboBox { Location = new Point(10, 55), Size = new Size(150, 22), DropDownStyle = ComboBoxStyle.DropDownList, BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White, Font = new Font("Segoe UI", 8) };
            RefreshPresetCombo();
            presetCard.Controls.Add(_cmbPresets);
            var btnLoadPreset = CreateButton("Load", SUCCESS_COLOR, 45, 22); btnLoadPreset.Font = new Font("Segoe UI", 7, FontStyle.Bold); btnLoadPreset.Location = new Point(165, 55); btnLoadPreset.Click += LoadSelectedPreset; presetCard.Controls.Add(btnLoadPreset);
            var btnSavePreset = CreateButton("Save As...", ACCENT_COLOR, 65, 22); btnSavePreset.Font = new Font("Segoe UI", 7, FontStyle.Bold); btnSavePreset.Location = new Point(215, 55); btnSavePreset.Click += SaveCurrentAsPreset; presetCard.Controls.Add(btnSavePreset);
            var btnDeletePreset = CreateButton("Del", ERROR_COLOR, 35, 22); btnDeletePreset.Font = new Font("Segoe UI", 7, FontStyle.Bold); btnDeletePreset.Location = new Point(282, 55); btnDeletePreset.Click += DeleteSelectedPreset; presetCard.Controls.Add(btnDeletePreset);
            var lblAutoSave = new Label { Name = "lblAutoSaveStatus", Text = "Auto-saved to plugin config", Font = new Font("Segoe UI", 7, FontStyle.Italic), ForeColor = SUCCESS_COLOR, Location = new Point(10, 85), AutoSize = true };
            presetCard.Controls.Add(lblAutoSave);
            // Added later in stacking order to put it on top.

            // 3) Violation Actions
            var actionCard = CreateCard("VIOLATION ACTIONS");
            actionCard.Dock = DockStyle.Top;
            actionCard.Height = 130;
            actionCard.Margin = new Padding(0, 3, 0, 0);

            var lblSoftAction = new Label { Text = "Soft:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(10, 40), AutoSize = true };
            actionCard.Controls.Add(lblSoftAction);
            _cmbSoftAction = new ComboBox { Location = new Point(42, 37), Size = new Size(145, 22), DropDownStyle = ComboBoxStyle.DropDownList, BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White, Font = new Font("Segoe UI", 8) };
            _cmbSoftAction.Items.AddRange(new object[] { "Warn (Audio)", "Warn (Visual)", "Warn (Both)", "Return to Boundary" });
            // Display labels are decoupled from the persisted action strings —
            // map by index so the UI wording can change without breaking configs.
            var softActions = new[] { "warn_audio", "warn_visual", "warn_both", "return_to_boundary" };
            _cmbSoftAction.SelectedIndex = Math.Max(0, Array.IndexOf(softActions, _missionConfig.Failsafe.SoftBoundaryAction ?? "warn_both"));
            _cmbSoftAction.SelectedIndexChanged += (s, e) => { _missionConfig.Failsafe.SoftBoundaryAction = softActions[_cmbSoftAction.SelectedIndex]; _missionConfig.Save(); };
            actionCard.Controls.Add(_cmbSoftAction);
            var lblHardAction = new Label { Text = "Hard:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(10, 65), AutoSize = true };
            actionCard.Controls.Add(lblHardAction);
            _cmbHardAction = new ComboBox { Location = new Point(42, 62), Size = new Size(145, 22), DropDownStyle = ComboBoxStyle.DropDownList, BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White, Font = new Font("Segoe UI", 8) };
            _cmbHardAction.Items.AddRange(new object[] { "Warn + Descend", "Auto Descend", "Warn Only" });
            var hardActions = new[] { "warn_and_kill", "auto_kill", "warn_only" };
            _cmbHardAction.SelectedIndex = Math.Max(0, Array.IndexOf(hardActions, _missionConfig.Failsafe.HardBoundaryAction ?? "warn_and_kill"));
            _cmbHardAction.SelectedIndexChanged += (s, e) => { _missionConfig.Failsafe.HardBoundaryAction = hardActions[_cmbHardAction.SelectedIndex]; _missionConfig.Save(); };
            actionCard.Controls.Add(_cmbHardAction);
            var lblKillDelay = new Label { Text = "Delay:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(195, 65), AutoSize = true };
            actionCard.Controls.Add(lblKillDelay);
            _nudKillDelay = new NumericUpDown { Location = new Point(232, 62), Size = new Size(45, 22), Minimum = 1, Maximum = 30, Value = _missionConfig.Failsafe.HardBoundaryKillDelaySec, BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White };
            _nudKillDelay.ValueChanged += (s, e) => { _missionConfig.Failsafe.HardBoundaryKillDelaySec = (int)_nudKillDelay.Value; _missionConfig.Save(); };
            actionCard.Controls.Add(_nudKillDelay);
            var lblSec = new Label { Text = "s", Font = new Font("Segoe UI", 7), ForeColor = TEXT_SECONDARY, Location = new Point(280, 67), AutoSize = true };
            actionCard.Controls.Add(lblSec);
            var lblDescent = new Label { Text = "Descent rate:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(10, 92), AutoSize = true };
            actionCard.Controls.Add(lblDescent);
            var nudDescentRate = new NumericUpDown { Location = new Point(90, 89), Size = new Size(55, 22), Minimum = 0.5m, Maximum = 10, Increment = 0.5m, DecimalPlaces = 1, Value = (decimal)_missionConfig.TerminationDescentRateMps, BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White };
            nudDescentRate.ValueChanged += (s, e) => { _missionConfig.TerminationDescentRateMps = (double)nudDescentRate.Value; _missionConfig.Save(); };
            actionCard.Controls.Add(nudDescentRate);
            var lblMps = new Label { Text = "m/s (LAND_SPEED on push)", Font = new Font("Segoe UI", 7), ForeColor = TEXT_SECONDARY, Location = new Point(150, 93), AutoSize = true };
            actionCard.Controls.Add(lblMps);
            rightScroll.Controls.Add(actionCard);

            // 2) Return Location
            var returnCard = CreateCard("RETURN LOCATION");
            returnCard.Dock = DockStyle.Top;
            returnCard.Height = 100;
            returnCard.Margin = new Padding(0, 3, 0, 0);

            var lblRetLat = new Label { Text = "Lat:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(10, 40), AutoSize = true };
            returnCard.Controls.Add(lblRetLat);
            _txtReturnLat = new TextBox { Location = new Point(35, 37), Size = new Size(95, 22), BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White, Font = new Font("Segoe UI", 8) };
            returnCard.Controls.Add(_txtReturnLat);
            var lblRetLon = new Label { Text = "Lon:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(135, 40), AutoSize = true };
            returnCard.Controls.Add(lblRetLon);
            _txtReturnLon = new TextBox { Location = new Point(162, 37), Size = new Size(95, 22), BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White, Font = new Font("Segoe UI", 8) };
            returnCard.Controls.Add(_txtReturnLon);
            if (_missionConfig.ReturnPoint != null)
            {
                _txtReturnLat.Text = _missionConfig.ReturnPoint.Lat.ToString("F7");
                _txtReturnLon.Text = _missionConfig.ReturnPoint.Lon.ToString("F7");
            }

            var btnReturnCurrent = CreateButton("Use Current", ACCENT_COLOR, 85, 22); btnReturnCurrent.Font = new Font("Segoe UI", 7, FontStyle.Bold); btnReturnCurrent.Location = new Point(10, 68);
            btnReturnCurrent.Click += (s, e) => { double lat = MainV2.comPort?.MAV?.cs?.lat ?? 0; double lon = MainV2.comPort?.MAV?.cs?.lng ?? 0; if (lat != 0 || lon != 0) { _txtReturnLat.Text = lat.ToString("F7"); _txtReturnLon.Text = lon.ToString("F7"); SaveReturnPoint(); } else CustomMessageBox.Show("No GPS position available.", "Warning"); };
            returnCard.Controls.Add(btnReturnCurrent);
            var btnReturnSave = CreateButton("Save", SUCCESS_COLOR, 50, 22); btnReturnSave.Font = new Font("Segoe UI", 7, FontStyle.Bold); btnReturnSave.Location = new Point(100, 68); btnReturnSave.Click += (s, e) => SaveReturnPoint(); returnCard.Controls.Add(btnReturnSave);
            var btnReturnCentroid = CreateButton("Use Centroid", INFO_COLOR, 85, 22); btnReturnCentroid.Font = new Font("Segoe UI", 7, FontStyle.Bold); btnReturnCentroid.Location = new Point(155, 68);
            btnReturnCentroid.Click += (s, e) => { var boundary = _missionConfig.HardBoundary?.Vertices?.Count > 0 ? _missionConfig.HardBoundary : _missionConfig.SoftBoundary; if (boundary?.Vertices?.Count >= 3) { double cLat = 0, cLon = 0; foreach (var v in boundary.Vertices) { cLat += v.Lat; cLon += v.Lon; } cLat /= boundary.Vertices.Count; cLon /= boundary.Vertices.Count; _txtReturnLat.Text = cLat.ToString("F7"); _txtReturnLon.Text = cLon.ToString("F7"); SaveReturnPoint(); } else CustomMessageBox.Show("No boundary defined.", "Warning"); };
            returnCard.Controls.Add(btnReturnCentroid);
            rightScroll.Controls.Add(returnCard);

            // 1) Monitoring Settings (top - added last)
            var settingsCard = CreateCard("MONITORING");
            settingsCard.Dock = DockStyle.Top;
            settingsCard.Height = 100;
            settingsCard.Margin = new Padding(0, 0, 0, 0);

            _chkEnableMonitoring = new CheckBox { Text = "Real-time Monitor", ForeColor = Color.White, Font = new Font("Segoe UI", 8), Location = new Point(10, 40), AutoSize = true, Checked = _monitor?.IsMonitoring ?? _missionConfig.MonitoringEnabled };
            _chkEnableMonitoring.CheckedChanged += (s, e) =>
            {
                // Persist so monitoring survives page switches and MP restarts
                // (the monitor itself is plugin-owned and keeps running).
                _missionConfig.MonitoringEnabled = _chkEnableMonitoring.Checked;
                _missionConfig.Save();
                if (_chkEnableMonitoring.Checked) _monitor?.StartMonitoring(); else _monitor?.StopMonitoring();
            };
            settingsCard.Controls.Add(_chkEnableMonitoring);
            var lblMaxAlt = new Label { Text = "Max Alt:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(10, 68), AutoSize = true };
            settingsCard.Controls.Add(lblMaxAlt);
            _nudMaxAlt = new NumericUpDown { Location = new Point(65, 65), Size = new Size(60, 22), Minimum = 10, Maximum = 150, Value = (decimal)_missionConfig.MaxAltitudeAglMeters, BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White };
            _nudMaxAlt.ValueChanged += (s, e) => { _missionConfig.MaxAltitudeAglMeters = (double)_nudMaxAlt.Value; _missionConfig.Save(); };
            settingsCard.Controls.Add(_nudMaxAlt);
            var lblMeters = new Label { Text = "m AGL", Font = new Font("Segoe UI", 7), ForeColor = TEXT_SECONDARY, Location = new Point(128, 69), AutoSize = true };
            settingsCard.Controls.Add(lblMeters);

            // Test audio: pick a pattern from a dropdown and play it. Lets the user
            // confirm the alert sound + know what each in-flight beep means.
            var cmbTestAudio = new ComboBox
            {
                Location = new Point(170, 60),
                Size = new Size(115, 22),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 7.5f),
            };
            cmbTestAudio.Items.AddRange(new object[] {
                "Soft boundary",
                "Hard boundary",
                "Battery warning",
                "Battery critical",
            });
            cmbTestAudio.SelectedIndex = 0;
            settingsCard.Controls.Add(cmbTestAudio);
            var btnTestAudio = CreateButton("Test", ACCENT_COLOR, 50, 22);
            btnTestAudio.Font = new Font("Segoe UI", 7, FontStyle.Bold);
            btnTestAudio.Location = new Point(170, 85);
            btnTestAudio.Click += (s, e) =>
            {
                var kind = (AlertKind)cmbTestAudio.SelectedIndex;
                AudioAlerts.Play(kind, ignoreRateLimit: true);
                string spoken;
                switch (kind)
                {
                    case AlertKind.BoundarySoft:    spoken = "Soft boundary warning. Turn around."; break;
                    case AlertKind.BoundaryHard:    spoken = "Hard boundary violation. Descend immediately."; break;
                    case AlertKind.BatteryWarning:  spoken = "Battery low. Test alert."; break;
                    case AlertKind.BatteryCritical: spoken = "Battery critical. Land now. Test alert."; break;
                    default: spoken = "Test alert."; break;
                }
                AudioAlerts.Speak(spoken, ignoreRateLimit: true);
                CustomMessageBox.Show(AudioAlerts.DescribePattern(kind) + "\n\nSpoken: " + spoken, "Playing test alert");
            };
            settingsCard.Controls.Add(btnTestAudio);

            rightScroll.Controls.Add(settingsCard);

            // Added LAST so it docks at the very top — promotes Saved Boundaries.
            rightScroll.Controls.Add(presetCard);

            mainLayout.Controls.Add(rightScroll, 1, 1);

            this.Controls.Add(mainLayout);
        }

        private void SaveReturnPoint()
        {
            if (double.TryParse(_txtReturnLat.Text, out double lat) &&
                double.TryParse(_txtReturnLon.Text, out double lon))
            {
                _missionConfig.ReturnPoint = new GpsPoint(lat, lon);
                _missionConfig.Save();
                CustomMessageBox.Show($"Return point saved: {lat:F7}, {lon:F7}", "Saved");
            }
            else
            {
                CustomMessageBox.Show("Enter valid latitude and longitude.", "Warning");
            }
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
                    SelectionBackColor = NOMADTheme.ACCENT,
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
    }
}
