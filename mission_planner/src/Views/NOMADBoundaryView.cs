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

        // ============================================================
        // Small layout helpers — everything reflows (docked cards of AutoSize
        // rows, wrapping control clusters) so the view fits any aspect ratio
        // without overlap or clipping.
        // ============================================================

        private static Label Lbl(string text, Color color, float size = NOMADTheme.SIZE_SMALL, FontStyle style = FontStyle.Regular)
            => new Label
            {
                Text = text,
                ForeColor = color,
                Font = NOMADTheme.Font(size, style),
                AutoSize = true,
                Margin = new Padding(0, 4, NOMADTheme.GAP, 0),
            };

        private Button Btn(string text, Color color, EventHandler onClick)
        {
            var b = new Button
            {
                Text = text,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                Padding = new Padding(7, 2, 7, 2),
                Margin = new Padding(0, 0, NOMADTheme.GAP, NOMADTheme.GAP),
                FlatStyle = FlatStyle.Flat,
                BackColor = color,
                ForeColor = Color.White,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL, FontStyle.Bold),
                Cursor = Cursors.Hand,
            };
            b.FlatAppearance.BorderSize = 0;
            b.Click += onClick;
            return b;
        }

        private static ComboBox Combo(int width, params string[] items)
        {
            var cb = new ComboBox
            {
                Width = width,
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = NOMADTheme.CONTROL_BG,
                ForeColor = Color.White,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL),
                Margin = new Padding(0, 1, NOMADTheme.GAP, 0),
            };
            cb.Items.AddRange(items);
            return cb;
        }

        // A themed card: AutoSize, docks to the top of its scroll column. Add full-
        // width rows to `body` via AddRow.
        private static TableLayoutPanel Card(string title, out TableLayoutPanel body)
        {
            var card = new TableLayoutPanel
            {
                ColumnCount = 1,
                RowCount = 2,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                Dock = DockStyle.Top,
                BackColor = NOMADTheme.CARD_BG,
                // Tight inner padding + a minimal inter-card margin so the sections
                // pack closely and the column rarely needs scrolling.
                Padding = new Padding(NOMADTheme.PAD, 4, NOMADTheme.PAD, 4),
                Margin = new Padding(0, 0, 0, 2),
            };
            card.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            card.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            card.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            var sectionTitle = ControlFactory.SectionTitle(title);
            sectionTitle.Margin = new Padding(0, 0, 0, 3); // tighter than the default GAP
            card.Controls.Add(sectionTitle, 0, 0);

            body = new TableLayoutPanel
            {
                ColumnCount = 1,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                Dock = DockStyle.Top,
                BackColor = Color.Transparent,
                Margin = new Padding(0),
            };
            body.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            card.Controls.Add(body, 0, 1);
            return card;
        }

        // Append a full-width row to a card body with an explicit height policy
        // (AutoSize for normal rows; Absolute for fixed-height controls like grids).
        private static void AddRow(TableLayoutPanel body, Control row, SizeType sizeType = SizeType.AutoSize, float height = 0f)
        {
            int r = body.RowCount;
            body.RowCount = r + 1;
            body.RowStyles.Add(new RowStyle(sizeType, height));
            row.Dock = DockStyle.Fill;
            row.Margin = new Padding(0, 0, 0, 2);
            body.Controls.Add(row, 0, r);
        }

        // A left-aligned cluster of controls that wraps when the card is narrow.
        private static FlowLayoutPanel Row(params Control[] children)
        {
            var flow = new FlowLayoutPanel
            {
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                WrapContents = true,
                FlowDirection = FlowDirection.LeftToRight,
                BackColor = Color.Transparent,
                Margin = new Padding(0),
                Padding = new Padding(0),
            };
            foreach (var c in children)
                flow.Controls.Add(c);
            return flow;
        }

        private void InitializeUI()
        {
            // Two-column layout: Left = boundaries, Right = settings. Columns are
            // percentage-based and each column scrolls, so content never clips.
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 2,
                Padding = new Padding(0),
            };
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 82)); // status banner
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100)); // content

            mainLayout.Controls.Add(BuildStatusPanel(), 0, 0);
            mainLayout.SetColumnSpan(_statusPanel, 2);

            var leftScroll = new Panel { Dock = DockStyle.Fill, AutoScroll = true, Padding = new Padding(3) };
            BuildLeftColumn(leftScroll);
            mainLayout.Controls.Add(leftScroll, 0, 1);

            var rightScroll = new Panel { Dock = DockStyle.Fill, AutoScroll = true, Padding = new Padding(3) };
            BuildRightColumn(rightScroll);
            mainLayout.Controls.Add(rightScroll, 1, 1);

            this.Controls.Add(mainLayout);
        }

        // ============================================================
        // Status banner (spans both columns)
        // ============================================================
        private Panel BuildStatusPanel()
        {
            _statusPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(80, 80, 90),
                Margin = new Padding(3),
                Padding = new Padding(NOMADTheme.PAD, NOMADTheme.GAP, NOMADTheme.PAD, NOMADTheme.GAP),
            };

            var info = new FlowLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                WrapContents = true,
                BackColor = Color.Transparent,
                Margin = new Padding(0),
                Padding = new Padding(0),
            };
            _lblCountdown = new Label { Text = "", Font = NOMADTheme.Font(NOMADTheme.SIZE_LARGE, FontStyle.Bold), ForeColor = Color.Yellow, AutoSize = true, Visible = false, Margin = new Padding(0, 2, NOMADTheme.PAD, 0) };
            _lblPosition = new Label { Text = "Position: --", Font = NOMADTheme.Mono(NOMADTheme.SIZE_SMALL), ForeColor = Color.White, AutoSize = true, Margin = new Padding(0, 4, NOMADTheme.PAD, 0) };
            _lblAltitude = new Label { Text = "Alt: -- / 122m", Font = NOMADTheme.Mono(NOMADTheme.SIZE_SMALL), ForeColor = Color.White, AutoSize = true, Margin = new Padding(0, 4, 0, 0) };
            info.Controls.Add(_lblCountdown);
            info.Controls.Add(_lblPosition);
            info.Controls.Add(_lblAltitude);

            _lblStatus = new Label
            {
                Text = "[?] Waiting for GPS Position",
                Font = NOMADTheme.Font(NOMADTheme.SIZE_TITLE, FontStyle.Bold),
                ForeColor = Color.White,
                AutoSize = true,
                Dock = DockStyle.Top,
                Margin = new Padding(0),
            };

            // Dock=Top: add the info row first, then the title so the title sits on top.
            _statusPanel.Controls.Add(info);
            _statusPanel.Controls.Add(_lblStatus);
            return _statusPanel;
        }

        // ============================================================
        // LEFT COLUMN: Boundary point grids + Import/Export
        // (added in reverse so the first card docks at the top)
        // ============================================================
        private void BuildLeftColumn(Panel host)
        {
            host.Controls.Add(BuildImportCard());
            host.Controls.Add(BuildHardBoundaryCard());
            host.Controls.Add(BuildSoftBoundaryCard());
        }

        private TableLayoutPanel BuildSoftBoundaryCard()
        {
            var card = Card("SOFT BOUNDARY (Warning)", out var body);

            _dgvSoftBoundary = ControlFactory.BoundaryGrid();
            _dgvSoftBoundary.CellValueChanged += (s, e) => SaveBoundaryFromGrid(_dgvSoftBoundary, _missionConfig.SoftBoundary);
            AddRow(body, _dgvSoftBoundary, SizeType.Absolute, 120);

            var btnPasteSoft = Btn("Paste", ACCENT_COLOR, (s, e) => PasteCoordinates(_dgvSoftBoundary, _missionConfig.SoftBoundary, "soft"));
            var btnAddSoft = Btn("+ Add", SUCCESS_COLOR, (s, e) => AddManualPoint(_dgvSoftBoundary, _missionConfig.SoftBoundary));
            var btnDelSoft = Btn("Del Pt", ACCENT_COLOR, (s, e) => DeleteSelectedPoint(_dgvSoftBoundary, _missionConfig.SoftBoundary));
            var btnClearSoft = Btn("Clear", ERROR_COLOR, (s, e) => ClearBoundary(_dgvSoftBoundary, _missionConfig.SoftBoundary));
            var lblSoftCount = new Label { Name = "lblSoftCount", Text = $"{_missionConfig.SoftBoundary.Vertices.Count} pts", Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL, FontStyle.Bold), ForeColor = Color.Yellow, AutoSize = true, Margin = new Padding(0, 4, 0, 0) };
            AddRow(body, Row(btnPasteSoft, btnAddSoft, btnDelSoft, btnClearSoft, lblSoftCount));

            // Derived mode: soft boundary auto-generated as hard boundary inset
            // inward by a configurable distance. Locks manual soft editing.
            var chkAutoSoft = new CheckBox { Text = "Auto: hard boundary −", ForeColor = Color.White, Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL), AutoSize = true, Checked = _missionConfig.SoftBoundaryFromHard, Margin = new Padding(0, 2, NOMADTheme.GAP, 0) };
            var nudSoftInset = ControlFactory.Numeric(1, 100, (decimal)Math.Max(1, Math.Min(100, _missionConfig.SoftBoundaryInsetMeters)), width: 52);
            AddRow(body, Row(chkAutoSoft, nudSoftInset, Lbl("m inward", TEXT_SECONDARY)));

            var lblSoftSaved = new Label { Name = "lblSoftSaved", Text = "", Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL), ForeColor = TEXT_SECONDARY, AutoSize = true };
            AddRow(body, lblSoftSaved);

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

            return card;
        }

        private TableLayoutPanel BuildHardBoundaryCard()
        {
            var card = Card("HARD BOUNDARY (Descend)", out var body);

            _dgvHardBoundary = ControlFactory.BoundaryGrid();
            _dgvHardBoundary.CellValueChanged += (s, e) => SaveBoundaryFromGrid(_dgvHardBoundary, _missionConfig.HardBoundary);
            AddRow(body, _dgvHardBoundary, SizeType.Absolute, 120);

            var btnPasteHard = Btn("Paste", ACCENT_COLOR, (s, e) => PasteCoordinates(_dgvHardBoundary, _missionConfig.HardBoundary, "hard"));
            var btnAddHard = Btn("+ Add", SUCCESS_COLOR, (s, e) => AddManualPoint(_dgvHardBoundary, _missionConfig.HardBoundary));
            var btnDelHard = Btn("Del Pt", ACCENT_COLOR, (s, e) => DeleteSelectedPoint(_dgvHardBoundary, _missionConfig.HardBoundary));
            var btnClearHard = Btn("Clear", ERROR_COLOR, (s, e) => ClearBoundary(_dgvHardBoundary, _missionConfig.HardBoundary));
            var lblHardCount = new Label { Name = "lblHardCount", Text = $"{_missionConfig.HardBoundary.Vertices.Count} pts", Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL, FontStyle.Bold), ForeColor = Color.Red, AutoSize = true, Margin = new Padding(0, 4, 0, 0) };
            AddRow(body, Row(btnPasteHard, btnAddHard, btnDelHard, btnClearHard, lblHardCount));

            var lblHardSaved = new Label { Name = "lblHardSaved", Text = "", Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL), ForeColor = TEXT_SECONDARY, AutoSize = true };
            AddRow(body, lblHardSaved);

            return card;
        }

        private TableLayoutPanel BuildImportCard()
        {
            var card = Card("IMPORT / EXPORT", out var body);

            var btnImportKml = Btn("KML", ACCENT_COLOR, BtnImportKml_Click);
            var btnImportCSV = Btn("Coords", ACCENT_COLOR, BtnImportGoogleMaps_Click);
            var btnGetFromMP = Btn("MP Fence", ACCENT_COLOR, BtnGetFromMP_Click);
            AddRow(body, Row(Lbl("Import:", TEXT_SECONDARY, NOMADTheme.SIZE_SMALL, FontStyle.Bold), btnImportKml, btnImportCSV, btnGetFromMP));

            var btnExportToMPFence = Btn("Push to MP + Drone", SUCCESS_COLOR, BtnExportToMPFence_Click);
            var btnClearVehicleFence = Btn("Clear on Veh", ERROR_COLOR, BtnClearVehicleFence_Click);
            AddRow(body, Row(Lbl("Send Fence:", TEXT_SECONDARY, NOMADTheme.SIZE_SMALL, FontStyle.Bold), btnExportToMPFence, btnClearVehicleFence));

            // Auto-draw toggle: when checked, AutoDrawBoundariesIfEnabled() pushes
            // imported/edited polygons to the map without a manual refresh. Named so
            // AutoDrawBoundariesIfEnabled can Find it.
            var chkAutoDraw = new CheckBox { Name = "chkAutoDraw", Text = "Auto-draw on map", ForeColor = Color.White, Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL), AutoSize = true, Checked = true };
            AddRow(body, chkAutoDraw);

            return card;
        }

        // ============================================================
        // RIGHT COLUMN: Settings, return point, actions, presets
        // ============================================================
        private void BuildRightColumn(Panel host)
        {
            host.Controls.Add(BuildActionCard());
            host.Controls.Add(BuildReturnCard());
            host.Controls.Add(BuildMonitoringCard());
            host.Controls.Add(BuildPresetCard());
        }

        private TableLayoutPanel BuildMonitoringCard()
        {
            var card = Card("MONITORING", out var body);

            _chkEnableMonitoring = new CheckBox { Text = "Real-time Monitor", ForeColor = Color.White, Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL), AutoSize = true, Checked = _monitor?.IsMonitoring ?? _missionConfig.MonitoringEnabled };
            _chkEnableMonitoring.CheckedChanged += (s, e) =>
            {
                // Persist so monitoring survives page switches and MP restarts
                // (the monitor itself is plugin-owned and keeps running).
                _missionConfig.MonitoringEnabled = _chkEnableMonitoring.Checked;
                _missionConfig.Save();
                if (_chkEnableMonitoring.Checked) _monitor?.StartMonitoring(); else _monitor?.StopMonitoring();
            };
            AddRow(body, _chkEnableMonitoring);

            _nudMaxAlt = ControlFactory.Numeric(10, 150, (decimal)_missionConfig.MaxAltitudeAglMeters, width: 60);
            _nudMaxAlt.ValueChanged += (s, e) => { _missionConfig.MaxAltitudeAglMeters = (double)_nudMaxAlt.Value; _missionConfig.Save(); };
            AddRow(body, Row(Lbl("Max Alt:", TEXT_PRIMARY), _nudMaxAlt, Lbl("m AGL", TEXT_SECONDARY)));

            // Test audio: pick a pattern from a dropdown and play it. Lets the user
            // confirm the alert sound + know what each in-flight beep means.
            var cmbTestAudio = Combo(130, "Soft boundary", "Hard boundary", "Battery warning", "Battery critical");
            cmbTestAudio.SelectedIndex = 0;
            var btnTestAudio = Btn("Test", ACCENT_COLOR, (s, e) =>
            {
                var kind = (AlertKind)cmbTestAudio.SelectedIndex;
                AudioAlerts.Play(kind, ignoreRateLimit: true);
                string spoken;
                switch (kind)
                {
                    case AlertKind.BoundarySoft: spoken = "Soft boundary warning. Turn around."; break;
                    case AlertKind.BoundaryHard: spoken = "Hard boundary violation. Descend immediately."; break;
                    case AlertKind.BatteryWarning: spoken = "Battery low. Test alert."; break;
                    case AlertKind.BatteryCritical: spoken = "Battery critical. Land now. Test alert."; break;
                    default: spoken = "Test alert."; break;
                }
                AudioAlerts.Speak(spoken, ignoreRateLimit: true);
                CustomMessageBox.Show(AudioAlerts.DescribePattern(kind) + "\n\nSpoken: " + spoken, "Playing test alert");
            });
            AddRow(body, Row(Lbl("Test audio:", TEXT_PRIMARY), cmbTestAudio, btnTestAudio));

            return card;
        }

        private TableLayoutPanel BuildReturnCard()
        {
            var card = Card("RETURN LOCATION", out var body);

            _txtReturnLat = new TextBox { Width = 110, BackColor = NOMADTheme.CONTROL_BG, ForeColor = Color.White, Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL), Margin = new Padding(0, 1, NOMADTheme.GAP, 0) };
            _txtReturnLon = new TextBox { Width = 110, BackColor = NOMADTheme.CONTROL_BG, ForeColor = Color.White, Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL), Margin = new Padding(0, 1, NOMADTheme.GAP, 0) };
            if (_missionConfig.ReturnPoint != null)
            {
                _txtReturnLat.Text = _missionConfig.ReturnPoint.Lat.ToString("F7");
                _txtReturnLon.Text = _missionConfig.ReturnPoint.Lon.ToString("F7");
            }
            AddRow(body, Row(Lbl("Lat:", TEXT_PRIMARY), _txtReturnLat, Lbl("Lon:", TEXT_PRIMARY), _txtReturnLon));

            var btnReturnCurrent = Btn("Use Current", ACCENT_COLOR, (s, e) =>
            {
                double lat = MainV2.comPort?.MAV?.cs?.lat ?? 0; double lon = MainV2.comPort?.MAV?.cs?.lng ?? 0;
                if (lat != 0 || lon != 0) { _txtReturnLat.Text = lat.ToString("F7"); _txtReturnLon.Text = lon.ToString("F7"); SaveReturnPoint(); }
                else CustomMessageBox.Show("No GPS position available.", "Warning");
            });
            var btnReturnSave = Btn("Save", SUCCESS_COLOR, (s, e) => SaveReturnPoint());
            var btnReturnCentroid = Btn("Use Centroid", INFO_COLOR, (s, e) =>
            {
                var boundary = _missionConfig.HardBoundary?.Vertices?.Count > 0 ? _missionConfig.HardBoundary : _missionConfig.SoftBoundary;
                if (boundary?.Vertices?.Count >= 3)
                {
                    double cLat = 0, cLon = 0;
                    foreach (var v in boundary.Vertices) { cLat += v.Lat; cLon += v.Lon; }
                    cLat /= boundary.Vertices.Count; cLon /= boundary.Vertices.Count;
                    _txtReturnLat.Text = cLat.ToString("F7"); _txtReturnLon.Text = cLon.ToString("F7"); SaveReturnPoint();
                }
                else CustomMessageBox.Show("No boundary defined.", "Warning");
            });
            AddRow(body, Row(btnReturnCurrent, btnReturnSave, btnReturnCentroid));

            return card;
        }

        private TableLayoutPanel BuildActionCard()
        {
            var card = Card("VIOLATION ACTIONS", out var body);

            _cmbSoftAction = Combo(150, "Warn (Audio)", "Warn (Visual)", "Warn (Both)", "Return to Boundary");
            // Display labels are decoupled from the persisted action strings — map
            // by index so the UI wording can change without breaking configs.
            var softActions = new[] { "warn_audio", "warn_visual", "warn_both", "return_to_boundary" };
            _cmbSoftAction.SelectedIndex = Math.Max(0, Array.IndexOf(softActions, _missionConfig.Failsafe.SoftBoundaryAction ?? "warn_both"));
            _cmbSoftAction.SelectedIndexChanged += (s, e) => { _missionConfig.Failsafe.SoftBoundaryAction = softActions[_cmbSoftAction.SelectedIndex]; _missionConfig.Save(); };
            AddRow(body, Row(Lbl("Soft:", TEXT_PRIMARY), _cmbSoftAction));

            _cmbHardAction = Combo(150, "Warn + Descend", "Auto Descend", "Warn Only");
            var hardActions = new[] { "warn_and_kill", "auto_kill", "warn_only" };
            _cmbHardAction.SelectedIndex = Math.Max(0, Array.IndexOf(hardActions, _missionConfig.Failsafe.HardBoundaryAction ?? "warn_and_kill"));
            _cmbHardAction.SelectedIndexChanged += (s, e) => { _missionConfig.Failsafe.HardBoundaryAction = hardActions[_cmbHardAction.SelectedIndex]; _missionConfig.Save(); };
            _nudKillDelay = ControlFactory.Numeric(1, 30, _missionConfig.Failsafe.HardBoundaryKillDelaySec, width: 48);
            _nudKillDelay.ValueChanged += (s, e) => { _missionConfig.Failsafe.HardBoundaryKillDelaySec = (int)_nudKillDelay.Value; _missionConfig.Save(); };
            AddRow(body, Row(Lbl("Hard:", TEXT_PRIMARY), _cmbHardAction, Lbl("Delay:", TEXT_PRIMARY), _nudKillDelay, Lbl("s", TEXT_SECONDARY)));

            var nudDescentRate = ControlFactory.Numeric(0.5m, 10m, (decimal)_missionConfig.TerminationDescentRateMps, increment: 0.5m, decimals: 1, width: 56);
            nudDescentRate.ValueChanged += (s, e) => { _missionConfig.TerminationDescentRateMps = (double)nudDescentRate.Value; _missionConfig.Save(); };
            AddRow(body, Row(Lbl("Descent rate:", TEXT_PRIMARY), nudDescentRate, Lbl("m/s (LAND_SPEED on push)", TEXT_SECONDARY)));

            return card;
        }

        private TableLayoutPanel BuildPresetCard()
        {
            var card = Card("SAVED BOUNDARIES (Plugin Storage)", out var body);

            AddRow(body, new Label { Text = "Current points auto-save. Use this to save named copies you can reload later.", Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL), ForeColor = TEXT_SECONDARY, AutoSize = true, MaximumSize = new Size(320, 0) });

            _cmbPresets = Combo(150);
            RefreshPresetCombo();
            var btnLoadPreset = Btn("Load", SUCCESS_COLOR, LoadSelectedPreset);
            var btnSavePreset = Btn("Save As...", ACCENT_COLOR, SaveCurrentAsPreset);
            var btnDeletePreset = Btn("Del", ERROR_COLOR, DeleteSelectedPreset);
            AddRow(body, Row(_cmbPresets, btnLoadPreset, btnSavePreset, btnDeletePreset));

            AddRow(body, new Label { Name = "lblAutoSaveStatus", Text = "Auto-saved to plugin config", Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL, FontStyle.Italic), ForeColor = SUCCESS_COLOR, AutoSize = true });

            return card;
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
    }
}
