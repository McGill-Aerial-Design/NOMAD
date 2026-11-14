// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using TextBox = System.Windows.Controls.TextBox;
using ComboBox = System.Windows.Controls.ComboBox;
using CheckBox = System.Windows.Controls.CheckBox;
using Label = System.Windows.Controls.TextBlock;
using Button = System.Windows.Controls.Button;

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
        private readonly ComboBox _distribution = new ComboBox { Name = "SimulationDistribution", IsEditable = true };
        private readonly ComboBox _scenarios = new ComboBox { Name = "SimulationScenario" };
        private readonly CheckBox _visible = new CheckBox { Content = "Show Gazebo" },
            _autoConnect = new CheckBox { Content = "Connect automatically" };
        private readonly TextBox _minutes = new TextBox { Width = 70 };
        private readonly Label _status = WpfUi.Text("");
        private readonly TextBox _url = new TextBox { IsReadOnly = true };
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
            _visible.IsChecked = preferences.Visible; _autoConnect.IsChecked = preferences.AutoConnect;
            _minutes.Text = Math.Max(1, Math.Min(1440, preferences.DurationMinutes)).ToString();
            var layout = new System.Windows.Controls.StackPanel { Margin = new System.Windows.Thickness(0, 0, 0, 8) };
            _browseTools = Button("Browse…", BrowseTools);
            _loadScenario = Button("Load scenario…", LoadScenario);
            _loadWorld = Button("Load world…", LoadWorld);
            layout.Children.Add(Row("Simulation tools folder", _tools, _browseTools));
            _detect = AsyncButton("Detect setup", DetectSetup);
            layout.Children.Add(Row("WSL distribution", _distribution, _detect));
            layout.Children.Add(Row("Python in WSL", _python, null));
            layout.Children.Add(Row("Simulator folder in WSL", _workspace, null));
            layout.Children.Add(Row("Scenario", _scenarios, _loadScenario));
            layout.Children.Add(Row("World override (optional)", _world, _loadWorld));
            var options = new System.Windows.Controls.WrapPanel();
            foreach (var element in new System.Windows.UIElement[] { _visible, _autoConnect, WpfUi.Text("Minutes:"), _minutes })
            { options.Children.Add(element); }
            layout.Children.Add(options);
            var actions = new System.Windows.Controls.WrapPanel();
            _start = AsyncButton("Start", Start);
            _connect = AsyncButton("Connect", () => _session.ConnectAsync());
            _stop = AsyncButton("Stop simulation", () => _session.StopAsync());
            _logs = Button("Open logs", () => { var path = _session?.Status.Directory;
                if (Directory.Exists(path)) { Process.Start(new ProcessStartInfo(path) { UseShellExecute = true }); } });
            foreach (var action in new[] { _start, _connect, _stop, _logs }) { actions.Children.Add(action); }
            layout.Children.Add(actions);
            layout.Children.Add(_status);
            layout.Children.Add(Row("Video URL", _url, Button("Copy video URL", () =>
            { if (_url.Text.Length > 0) { Clipboard.SetText(_url.Text); } })));
            Controls.Add(_previewPanel);
            var header = WpfUi.Host(new System.Windows.Controls.ScrollViewer
            { Content = layout, MaxHeight = 500, VerticalScrollBarVisibility = System.Windows.Controls.ScrollBarVisibility.Auto });
            header.Dock = DockStyle.Top; header.AutoSize = true;
            Controls.Add(header);
            _tools.LostKeyboardFocus += (sender, args) => ReadScenarios(null);
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
                _distribution.Items.Clear();
                foreach (var distribution in found.Distributions) { _distribution.Items.Add(distribution); }
                _distribution.Text = found.Distribution; _workspace.Text = found.Workspace;
                if (string.IsNullOrWhiteSpace(_python.Text) || _python.Text == "python3")
                { _python.Text = string.IsNullOrEmpty(found.Python) ? "python3" : found.Python; }
                _actionError = "Using " + found.Distribution + ". " + (found.Prepared ? "Simulator folder found." :
                    "Default folder selected; prepare the simulator workspace before Start.");
            }
            catch (Exception error) { if (!IsDisposed) { _actionError = "Setup detection: " + error.Message; } }
            finally { _detecting = false; if (!IsDisposed) { UpdateData(); } }
        }

        private static TextBox Input(string name) => new TextBox { Name = name };
        private static System.Windows.Controls.Grid Row(string text, System.Windows.FrameworkElement field,
            System.Windows.FrameworkElement action)
        {
            var row = new System.Windows.Controls.Grid { Margin = new System.Windows.Thickness(0, 0, 0, 6) };
            row.ColumnDefinitions.Add(new System.Windows.Controls.ColumnDefinition { Width = new System.Windows.GridLength(180) });
            row.ColumnDefinitions.Add(new System.Windows.Controls.ColumnDefinition());
            row.ColumnDefinitions.Add(new System.Windows.Controls.ColumnDefinition { Width = System.Windows.GridLength.Auto });
            var label = WpfUi.Text(text); label.VerticalAlignment = System.Windows.VerticalAlignment.Center;
            row.Children.Add(label);
            System.Windows.Controls.Grid.SetColumn(field, 1); row.Children.Add(field);
            if (action != null) { System.Windows.Controls.Grid.SetColumn(action, 2); row.Children.Add(action); }
            return row;
        }
        private static Button Button(string text, Action click)
        {
            return WpfUi.Action(text, click);
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
            if (!int.TryParse(_minutes.Text, out int minutes) || minutes < 1 || minutes > 1440)
            { throw new ArgumentException("Duration must be a whole number between 1 and 1440 minutes."); }
            var options = new SimulationPreferences { ToolsDirectory = _tools.Text.Trim(), Distribution = _distribution.Text.Trim(),
                Workspace = _workspace.Text.Trim(), PythonExecutable = _python.Text.Trim(), ScenarioPath = scenario.Path, WorldPath = _world.Text.Trim(),
                Visible = _visible.IsChecked == true, AutoConnect = _autoConnect.IsChecked == true, DurationMinutes = minutes };
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
            _start.IsEnabled = status != null && !status.Active && !_detecting && status.Phase != "Cleanup failed" && !_shutdown.IsCancellationRequested;
            _connect.IsEnabled = status?.CanConnect == true;
            _stop.IsEnabled = status?.Active == true;
            _logs.IsEnabled = Directory.Exists(status?.Directory);
            _status.Text = _detecting ? "Finding WSL and the simulator folder…" :
                _actionError ?? (status == null ? "Simulation owner unavailable." : status.Phase + " — " + status.Message);
            _url.Text = status?.VideoUrl ?? "";
            foreach (var control in new System.Windows.UIElement[] { _tools, _distribution, _workspace, _python, _scenarios, _world,
                _visible, _autoConnect, _minutes, _browseTools, _loadScenario, _loadWorld, _detect })
            { control.IsEnabled = status?.Active != true && !_detecting; }
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
