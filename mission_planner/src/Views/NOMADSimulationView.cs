// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    [global::MissionPlanner.Utilities.PreventTheming]
    internal sealed class NOMADSimulationView : NOMADViewBase, IUpdatableView
    {
        private readonly SimulationSession _session;
        private readonly CancellationToken _shutdown;
        private readonly bool _detectOnOpen;
        private readonly TextBox _tools = Input("SimulationTools"),
            _workspace = Input("SimulationWorkspace"), _world = Input("SimulationWorld"), _python = Input("SimulationPython");
        private readonly ComboBox _distribution = new ComboBox { Name = "SimulationDistribution", Dock = DockStyle.Fill,
            DropDownStyle = ComboBoxStyle.DropDown, BackColor = NOMADTheme.INPUT_BG, ForeColor = NOMADTheme.TEXT_PRIMARY };
        private readonly ComboBox _scenarios = new ComboBox { Name = "SimulationScenario", Dock = DockStyle.Fill,
            DropDownStyle = ComboBoxStyle.DropDownList, BackColor = NOMADTheme.INPUT_BG, ForeColor = NOMADTheme.TEXT_PRIMARY };
        private readonly CheckBox _visible = new CheckBox { Text = "Show Gazebo", AutoSize = true },
            _autoConnect = new CheckBox { Text = "Connect automatically", AutoSize = true };
        private readonly NumericUpDown _minutes = new NumericUpDown { Minimum = 1, Maximum = 1440, Width = 70 };
        private readonly Label _status = new Label { Dock = DockStyle.Fill, ForeColor = NOMADTheme.TEXT_SECONDARY };
        private readonly TextBox _url = new TextBox { Dock = DockStyle.Fill, ReadOnly = true };
        private readonly Panel _previewPanel = new Panel { Dock = DockStyle.Fill, BackColor = Color.Black };
        private Button _start, _connect, _stop, _logs, _browseTools, _loadScenario, _loadWorld, _detect;
        private EmbeddedVideoPlayer _preview;
        private string _previewUrl, _actionError;
        private bool _detecting, _detectionStarted;

        internal NOMADSimulationView(SimulationSession session, CancellationToken shutdown, bool detectOnOpen = true)
        {
            _session = session; _shutdown = shutdown; _detectOnOpen = detectOnOpen;
            Font = NOMADTheme.Font(); ForeColor = TEXT_PRIMARY;
            var preferences = SimulationPreferences.Load();
            if (string.IsNullOrEmpty(preferences.ToolsDirectory))
            {
                var directory = new DirectoryInfo(Path.GetDirectoryName(typeof(NOMADSimulationView).Assembly.Location));
                for (int i = 0; directory != null && i < 8; i++, directory = directory.Parent)
                {
                    string candidate = Path.Combine(directory.FullName, "scripts", "simulation");
                    if (File.Exists(Path.Combine(candidate, "interactive.py"))) { preferences.ToolsDirectory = candidate; break; }
                }
            }
            _tools.Text = preferences.ToolsDirectory; _distribution.Text = preferences.Distribution;
            _workspace.Text = preferences.Workspace; _world.Text = preferences.WorldPath;
            _python.Text = preferences.PythonExecutable;
            _visible.Checked = preferences.Visible; _autoConnect.Checked = preferences.AutoConnect;
            _minutes.Value = Math.Max(1, Math.Min(1440, preferences.DurationMinutes));
            _minutes.BackColor = NOMADTheme.INPUT_BG; _minutes.ForeColor = TEXT_PRIMARY;
            _url.BackColor = NOMADTheme.INPUT_BG; _url.ForeColor = TEXT_PRIMARY;
            var layout = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 3, RowCount = 11 };
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 180));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 112));
            for (int i = 0; i < 9; i++) { layout.RowStyles.Add(new RowStyle(SizeType.Absolute, i == 8 ? 68 : 38)); }
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 36));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            _browseTools = Button("Browse…", BrowseTools);
            _loadScenario = Button("Load scenario…", LoadScenario);
            _loadWorld = Button("Load world…", LoadWorld);
            Row(layout, 0, "Simulation tools folder", _tools, _browseTools);
            _detect = AsyncButton("Detect setup", DetectSetup);
            Row(layout, 1, "WSL distribution", _distribution, _detect);
            Row(layout, 2, "Python in WSL", _python, null);
            Row(layout, 3, "Simulator folder in WSL", _workspace, null);
            Row(layout, 4, "Scenario", _scenarios, _loadScenario);
            Row(layout, 5, "World override (optional)", _world, _loadWorld);
            var options = new FlowLayoutPanel { Dock = DockStyle.Fill, WrapContents = false };
            options.Controls.AddRange(new Control[] { _visible, _autoConnect, new Label { Text = "Minutes:", AutoSize = true }, _minutes });
            layout.Controls.Add(options, 0, 6); layout.SetColumnSpan(options, 3);
            var actions = new FlowLayoutPanel { Dock = DockStyle.Fill, WrapContents = false };
            _start = AsyncButton("Start", Start);
            _connect = AsyncButton("Connect", () => _session.ConnectAsync());
            _stop = AsyncButton("Stop simulation", () => _session.StopAsync());
            _logs = Button("Open logs", () => { var path = _session?.Status.Directory;
                if (Directory.Exists(path)) { Process.Start(new ProcessStartInfo(path) { UseShellExecute = true }); } });
            actions.Controls.AddRange(new Control[] { _start, _connect, _stop, _logs });
            layout.Controls.Add(actions, 0, 7); layout.SetColumnSpan(actions, 3);
            layout.Controls.Add(_status, 0, 8); layout.SetColumnSpan(_status, 3);
            layout.Controls.Add(_url, 0, 9); layout.SetColumnSpan(_url, 2);
            layout.Controls.Add(Button("Copy video URL", () => { if (_url.Text.Length > 0) { Clipboard.SetText(_url.Text); } }), 2, 9);
            layout.Controls.Add(_previewPanel, 0, 10); layout.SetColumnSpan(_previewPanel, 3);
            Controls.Add(layout);
            _tools.Validated += (sender, args) => ReadScenarios(null);
            ReadScenarios(preferences.ScenarioPath);
            if (_session != null) { _session.Changed += OnChanged; }
            UpdateData();
        }

        protected override void OnHandleCreated(EventArgs e)
        {
            base.OnHandleCreated(e);
            if (!_detectOnOpen || _detectionStarted || _session == null) { return; }
            _detectionStarted = true;
            BeginInvoke((Action)(async () => await DetectSetup()));
        }

        private async Task DetectSetup()
        {
            if (_detecting || IsDisposed || _session?.Status.Active == true || _shutdown.IsCancellationRequested) { return; }
            _detecting = true; _actionError = null; UpdateData();
            try
            {
                var found = await SimulationSetup.DetectAsync(_distribution.Text.Trim(), _workspace.Text.Trim());
                if (IsDisposed || _shutdown.IsCancellationRequested) { return; }
                _distribution.Items.Clear(); _distribution.Items.AddRange(found.Distributions);
                _distribution.Text = found.Distribution; _workspace.Text = found.Workspace;
                if (string.IsNullOrWhiteSpace(_python.Text) || _python.Text == "python3")
                { _python.Text = string.IsNullOrEmpty(found.Python) ? "python3" : found.Python; }
                _actionError = "Using " + found.Distribution + ". " + (found.Prepared ? "Simulator folder found." :
                    "Default folder selected; prepare the simulator workspace before Start.");
            }
            catch (Exception error) { if (!IsDisposed) { _actionError = "Setup detection: " + error.Message; } }
            finally { _detecting = false; if (!IsDisposed) { UpdateData(); } }
        }

        private static TextBox Input(string name) => new TextBox { Name = name, Dock = DockStyle.Fill,
            BackColor = NOMADTheme.INPUT_BG, ForeColor = NOMADTheme.TEXT_PRIMARY, BorderStyle = BorderStyle.FixedSingle };
        private static void Row(TableLayoutPanel table, int row, string text, Control field, Control action)
        {
            table.Controls.Add(new Label { Text = text, Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleLeft }, 0, row);
            table.Controls.Add(field, 1, row);
            if (action != null) { table.Controls.Add(action, 2, row); }
        }
        private static Button Button(string text, Action click)
        {
            var button = new ControlFactory.ThemedButton { Text = text, UseMnemonic = false, AutoSize = true, MinimumSize = new Size(90, 29),
                FlatStyle = FlatStyle.Flat, BackColor = NOMADTheme.BUTTON_BG, ForeColor = NOMADTheme.TEXT_PRIMARY };
            button.Click += (sender, args) => click();
            return button;
        }
        private Button AsyncButton(string text, Func<Task> action)
        {
            var button = Button(text, () => { });
            button.Click += async (sender, args) =>
            {
                _actionError = null;
                try { await action(); }
                catch (Exception error) { _actionError = error.Message; }
                if (!IsDisposed) { UpdateData(); }
            };
            return button;
        }
        private async Task Start()
        {
            var scenario = _scenarios.SelectedItem as SimulationScenario;
            if (scenario == null) { throw new InvalidOperationException("Choose or load a scenario configuration."); }
            var options = new SimulationPreferences { ToolsDirectory = _tools.Text.Trim(), Distribution = _distribution.Text.Trim(),
                Workspace = _workspace.Text.Trim(), PythonExecutable = _python.Text.Trim(), ScenarioPath = scenario.Path, WorldPath = _world.Text.Trim(),
                Visible = _visible.Checked, AutoConnect = _autoConnect.Checked, DurationMinutes = (int)_minutes.Value };
            options.Validate(); options.Save();
            await _session.StartAsync(options, scenario);
        }
        private void BrowseTools()
        {
            using (var dialog = new FolderBrowserDialog { Description = "Choose scripts/simulation from a NOMAD source checkout",
                ShowNewFolderButton = false })
            { if (dialog.ShowDialog(FindForm()) == DialogResult.OK) { _tools.Text = dialog.SelectedPath; ReadScenarios(null); } }
        }
        private void ReadScenarios(string selected)
        {
            string previous = selected ?? (_scenarios.SelectedItem as SimulationScenario)?.Path;
            _scenarios.Items.Clear();
            string directory = Path.Combine(_tools.Text, "scenarios");
            if (Directory.Exists(directory))
            {
                foreach (string path in Directory.GetFiles(directory, "*.json").OrderBy(value => value))
                { try { _scenarios.Items.Add(SimulationScenario.Load(path)); }
                    catch (Exception error) when (error is IOException || error is ArgumentException || error is Newtonsoft.Json.JsonException)
                    { _actionError = "Scenario unavailable: " + Path.GetFileName(path); } }
            }
            if (File.Exists(previous)) { AddScenario(previous); }
            if (_scenarios.SelectedIndex < 0 && _scenarios.Items.Count > 0) { _scenarios.SelectedIndex = 0; }
        }
        private void AddScenario(string path)
        {
            var existing = _scenarios.Items.Cast<SimulationScenario>().FirstOrDefault(item =>
                string.Equals(item.Path, Path.GetFullPath(path), StringComparison.OrdinalIgnoreCase));
            var loaded = SimulationScenario.Load(path);
            if (existing == null) { _scenarios.Items.Add(loaded); }
            else { _scenarios.Items[_scenarios.Items.IndexOf(existing)] = loaded; }
            _scenarios.SelectedItem = loaded;
        }
        private void LoadScenario()
        {
            using (var dialog = new OpenFileDialog { Filter = "Simulation scenario (*.json)|*.json", CheckFileExists = true })
            { if (dialog.ShowDialog(FindForm()) == DialogResult.OK)
                { try { AddScenario(dialog.FileName); } catch (Exception error) { _actionError = error.Message; UpdateData(); } } }
        }
        private void LoadWorld()
        {
            using (var dialog = new OpenFileDialog { Filter = "Gazebo world (*.sdf;*.world)|*.sdf;*.world", CheckFileExists = true })
            { if (dialog.ShowDialog(FindForm()) == DialogResult.OK) { _world.Text = dialog.FileName; } }
        }
        private void OnChanged()
        {
            if (IsDisposed || Disposing || !IsHandleCreated) { return; }
            if (InvokeRequired)
            { try { BeginInvoke((Action)UpdateData); } catch (InvalidOperationException) { } }
            else { UpdateData(); }
        }
        public void UpdateData()
        {
            if (IsDisposed || Disposing) { return; }
            var status = _session?.Status;
            _start.Enabled = status != null && !status.Active && !_detecting && status.Phase != "Cleanup failed" && !_shutdown.IsCancellationRequested;
            _connect.Enabled = status?.CanConnect == true;
            _stop.Enabled = status?.Active == true;
            _logs.Enabled = Directory.Exists(status?.Directory);
            _status.Text = _detecting ? "Finding WSL and the simulator folder…" :
                _actionError ?? (status == null ? "Simulation owner unavailable." : status.Phase + " — " + status.Message);
            _url.Text = status?.VideoUrl ?? "";
            foreach (Control control in new Control[] { _tools, _distribution, _workspace, _python, _scenarios, _world,
                _visible, _autoConnect, _minutes, _browseTools, _loadScenario, _loadWorld, _detect })
            { control.Enabled = status?.Active != true && !_detecting; }
            if (_shutdown.IsCancellationRequested || !Visible || string.IsNullOrEmpty(status?.VideoUrl)) { PausePreview(); return; }
            if (_previewUrl == status.VideoUrl) { return; }
            PausePreview(); _previewUrl = status.VideoUrl;
            var input = new VideoInputProfile { Name = "Simulation preview", Url = status.VideoUrl,
                HorizontalFovDeg = status.CameraFov, TrackingCalibrated = false };
            // This borrowed view offers pixel detection/selection. Normal profile-bound gimbal control stays on Video.
            _preview = new EmbeddedVideoPlayer(input, true, _shutdown, MediaWorkerPipeline.Create);
            _previewPanel.Controls.Add(_preview); _preview.Dock = DockStyle.Fill; _preview.StartStream();
        }
        internal void PausePreview()
        { _preview?.Dispose(); _preview = null; _previewUrl = null; }
        protected override void Dispose(bool disposing)
        {
            if (disposing) { if (_session != null) { _session.Changed -= OnChanged; } PausePreview(); }
            base.Dispose(disposing);
        }
    }
}
