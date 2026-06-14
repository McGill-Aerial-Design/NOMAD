// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Log;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADLogView
    {
        private Panel _postFlightContent;
        private Label _loadStatus;
        private NomadMarqueeBar _loadProgress;
        private Button _openInMpButton;
        private Button _exportMarkdownButton;
        private Button _exportPngButton;

        private void InitializePostFlightUI()
        {
            _postFlightLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 3,
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(10),
            };
            _postFlightLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            _postFlightLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 43));
            _postFlightLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 57));

            _postFlightLayout.Controls.Add(CreateImportToolbar(), 0, 0);
            _postFlightLayout.Controls.Add(CreateSummaryArea(), 0, 1);
            _postFlightLayout.Controls.Add(CreatePlotArea(), 0, 2);
            _postFlightContent = _postFlightLayout;
            _postFlightTab.Controls.Add(_postFlightLayout);
        }

        private Control CreateImportToolbar()
        {
            _importToolbar = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                FlowDirection = FlowDirection.LeftToRight,
                WrapContents = true,
                BackColor = NOMADTheme.BG_DARK,
                Margin = new Padding(0, 0, 0, NOMADTheme.GAP),
            };

            var loadButton = SmallButton("Open Log...", NOMADTheme.ACCENT);
            loadButton.Click += async (s, e) => await ChooseAndLoadLogAsync();
            var sampleButton = SmallButton("Load Sample", NOMADTheme.BUTTON_BG);
            sampleButton.Click += (s, e) => LoadSample();
            var autopilotButton = SmallButton("Latest FC Log", NOMADTheme.BUTTON_BG);
            autopilotButton.Click += async (s, e) => await DownloadLatestFromAutopilotAsync();
            var browseAutopilotButton = SmallButton("Browse FC Logs", NOMADTheme.BUTTON_BG);
            browseAutopilotButton.Click += async (s, e) => await BrowseAutopilotLogsAsync();
            var jetsonButton = SmallButton("Jetson Latest", NOMADTheme.BUTTON_BG);
            jetsonButton.Enabled = _sender != null;
            jetsonButton.Click += async (s, e) => await DownloadLatestFromJetsonAsync();
            _openInMpButton = SmallButton("Open in MP", NOMADTheme.BUTTON_BG);
            _openInMpButton.Enabled = false;
            _openInMpButton.Click += (s, e) => OpenInMissionPlanner();
            _exportMarkdownButton = SmallButton("Export Summary", NOMADTheme.BUTTON_BG);
            _exportMarkdownButton.Enabled = false;
            _exportMarkdownButton.Click += (s, e) => ExportMarkdown();
            _exportPngButton = SmallButton("Export PNG", NOMADTheme.BUTTON_BG);
            _exportPngButton.Enabled = false;
            _exportPngButton.Click += (s, e) => ExportPng();

            _loadProgress = new NomadMarqueeBar
            {
                Width = 110,
                Height = 20,
                Margin = new Padding(12, 10, 4, 0),
                Visible = false,
            };
            _loadStatus = new Label
            {
                Text = "Open a DataFlash .bin/.log or MAVLink .tlog file.",
                AutoSize = true,
                ForeColor = TEXT_SECONDARY,
                Font = NOMADTheme.Font(),
                Margin = new Padding(8, 13, 0, 0),
            };

            _importToolbar.Controls.Add(loadButton);
            _importToolbar.Controls.Add(sampleButton);
            _importToolbar.Controls.Add(autopilotButton);
            _importToolbar.Controls.Add(browseAutopilotButton);
            _importToolbar.Controls.Add(jetsonButton);
            _importToolbar.Controls.Add(_openInMpButton);
            _importToolbar.Controls.Add(_exportMarkdownButton);
            _importToolbar.Controls.Add(_exportPngButton);
            _importToolbar.Controls.Add(_loadProgress);
            _importToolbar.Controls.Add(_loadStatus);
            return _importToolbar;
        }

        private Button SmallButton(string text, Color color)
        {
            Button button = CreateButton(text, color, 116, 34);
            button.Font = NOMADTheme.Font(NOMADTheme.SIZE_BODY, FontStyle.Bold);
            button.FlatAppearance.BorderSize = color == NOMADTheme.ACCENT ? 0 : 1;
            button.FlatAppearance.BorderColor = NOMADTheme.ACCENT;
            return button;
        }

        private async Task ChooseAndLoadLogAsync()
        {
            using (var dialog = new OpenFileDialog
            {
                Title = "Open flight log",
                Filter = "Flight logs (*.bin;*.log;*.tlog)|*.bin;*.log;*.tlog|" +
                    "DataFlash logs (*.bin;*.log)|*.bin;*.log|" +
                    "MAVLink telemetry logs (*.tlog)|*.tlog|All files (*.*)|*.*",
                CheckFileExists = true,
                Multiselect = false,
                InitialDirectory = ExistingDirectory(_config.DefaultLogDirectory),
            })
            {
                if (dialog.ShowDialog(FindForm()) == DialogResult.OK)
                    await LoadFileAsync(dialog.FileName);
            }
        }

        private async Task LoadFileAsync(string path)
        {
            int generation = Interlocked.Increment(ref _loadGeneration);
            _loadCancellation?.Cancel();
            _loadCancellation?.Dispose();
            _loadCancellation = new CancellationTokenSource();
            CancellationToken token = _loadCancellation.Token;
            await RunOnUiThreadAsync(() => SetLoading(true, $"Loading {Path.GetFileName(path)}..."));

            try
            {
                var loaded = await Task.Run(() =>
                {
                    token.ThrowIfCancellationRequested();
                    bool telemetryLog = string.Equals(
                        Path.GetExtension(path),
                        ".tlog",
                        StringComparison.OrdinalIgnoreCase);
                    IFlightLogData data = telemetryLog
                        ? (IFlightLogData)new MavlinkTelemetryLogModel(path)
                        : new DFLogModel(path);
                    try
                    {
                        token.ThrowIfCancellationRequested();
                        LogSummary summary = LogAnalysis.Analyze(data, CreateThresholds());
                        return (Data: data, Summary: summary);
                    }
                    catch
                    {
                        data.Dispose();
                        throw;
                    }
                }, token).ConfigureAwait(false);

                if (generation != _loadGeneration || token.IsCancellationRequested)
                {
                    loaded.Data.Dispose();
                    return;
                }

                bool accepted = false;
                await RunOnUiThreadAsync(() =>
                {
                    if (generation != _loadGeneration || token.IsCancellationRequested)
                        return;

                    ReplaceLog(loaded.Data, loaded.Summary);
                    SetLoading(
                        false,
                        $"{Path.GetFileName(path)} loaded ({loaded.Data.MessageTypes.Count} message types).");
                    accepted = true;
                }).ConfigureAwait(false);
                if (!accepted)
                    loaded.Data.Dispose();
            }
            catch (OperationCanceledException)
            {
                await RunOnUiThreadAsync(() =>
                {
                    if (generation == _loadGeneration)
                        SetLoading(false, "Load cancelled.");
                }).ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                await RunOnUiThreadAsync(() =>
                {
                    SetLoading(false, $"Could not load log: {ex.Message}");
                    MessageBox.Show(FindForm(), ex.Message, "Flight Log Load Failed",
                        MessageBoxButtons.OK, MessageBoxIcon.Error);
                }).ConfigureAwait(false);
            }
        }

        private void LoadSample()
        {
            var sample = new SampleFlightLogData();
            ReplaceLog(sample, LogAnalysis.Analyze(sample, CreateThresholds()));
            SetLoading(false, "Loaded built-in sample flight.");
        }

        private void ReplaceLog(IFlightLogData data, LogSummary summary)
        {
            IFlightLogData previous = _logData;
            _logData = data;
            _summary = summary;
            RenderSummary(summary);
            PopulateFieldTree(data);
            _offlinePlot.ClearSeries();
            _offlinePlot.SetModes(summary.Modes);
            _openInMpButton.Enabled = !string.IsNullOrWhiteSpace(data.SourcePath)
                && !string.Equals(Path.GetExtension(data.SourcePath), ".tlog", StringComparison.OrdinalIgnoreCase);
            _exportMarkdownButton.Enabled = true;
            _exportPngButton.Enabled = true;
            previous?.Dispose();
        }

        private async Task DownloadLatestFromAutopilotAsync()
        {
            if (MainV2.comPort?.BaseStream?.IsOpen != true)
            {
                MessageBox.Show(FindForm(), "Connect Mission Planner to the flight controller first.",
                    "Autopilot Logs", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            var port = MainV2.comPort;
            int downloadActive = 1;
            ProgressEventHandler progress = (percent, message) =>
            {
                RunOnUiThreadAsync(() =>
                {
                    if (Volatile.Read(ref downloadActive) == 0)
                        return;
                    string detail = string.IsNullOrWhiteSpace(message) ? "Downloading" : message.Trim();
                    SetLoading(true, $"{detail} ({Math.Max(0, Math.Min(100, percent))}%)");
                });
            };
            port.Progress += progress;
            try
            {
                await RunOnUiThreadAsync(() => SetLoading(true, "Reading flight-controller log list..."));
#pragma warning disable CS0612 // Mission Planner exposes no non-obsolete public log-list API.
                var entries = await Task.Run(() => port.GetLogList()).ConfigureAwait(false);
#pragma warning restore CS0612
                if (entries == null || entries.Count == 0)
                {
                    await RunOnUiThreadAsync(() => SetLoading(false, "No flight-controller logs were found."));
                    return;
                }

                var latest = entries
                    .OrderByDescending(entry => entry.time_utc)
                    .ThenByDescending(entry => entry.id)
                    .First();
                string size = FormatBytes(latest.size);
                await RunOnUiThreadAsync(() => SetLoading(
                    true,
                    $"Downloading FC log #{latest.id} ({size}) over MAVLink. Radio links can take several minutes."));

                string path = await port.GetLog(
                    (byte)port.sysidcurrent,
                    (byte)port.compidcurrent,
                    latest.id).ConfigureAwait(false);
                if (string.IsNullOrWhiteSpace(path) || !File.Exists(path))
                    throw new IOException("Mission Planner did not return a downloaded log file.");
                await LoadFileAsync(path).ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                await RunOnUiThreadAsync(() =>
                {
                    SetLoading(false, $"Flight-controller log download failed: {ex.Message}");
                    MessageBox.Show(
                        FindForm(),
                        ex.Message,
                        "Autopilot Log Download",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Error);
                }).ConfigureAwait(false);
            }
            finally
            {
                Interlocked.Exchange(ref downloadActive, 0);
                port.Progress -= progress;
            }
        }

        private async Task BrowseAutopilotLogsAsync()
        {
            if (MainV2.comPort?.BaseStream?.IsOpen != true)
            {
                MessageBox.Show(FindForm(), "Connect Mission Planner to the flight controller first.",
                    "Autopilot Logs", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            string directory = GetLogDirectory();
            DateTime started = DateTime.UtcNow;
            using (var downloader = new LogDownloadMavLink())
                downloader.ShowDialog(FindForm());

            string latest = FindNewestLog(directory, started.AddSeconds(-2));
            if (latest != null)
                await LoadFileAsync(latest);
            else
                SetLoading(false, "Autopilot downloader closed. Open the downloaded file when ready.");
        }

        private async Task DownloadLatestFromJetsonAsync()
        {
            if (_sender == null) return;
            string remoteDirectory = string.IsNullOrWhiteSpace(_config.JetsonLogDirectory)
                ? "~/NOMAD/logs"
                : _config.JetsonLogDirectory.Trim();
            if (remoteDirectory.IndexOfAny(new[] { '\r', '\n', '\0' }) >= 0)
            {
                MessageBox.Show(FindForm(), "The configured Jetson log directory is invalid.",
                    "Jetson Logs", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            await RunOnUiThreadAsync(() => SetLoading(true, "Finding latest Jetson flight log..."));
            string remoteDirectoryExpression = RemoteDirectoryExpression(remoteDirectory);
            string command = $"find {remoteDirectoryExpression} -maxdepth 3 -type f " +
                "\\( -iname '*.bin' -o -iname '*.log' \\) " +
                "-printf '%T@ %p\\n' | sort -nr | head -1 | cut -d' ' -f2-";
            var result = await _sender.ExecuteSSHCommandAsync(command, 20);
            string remotePath = result.Success ? (result.Data ?? "").Trim() : "";
            if (string.IsNullOrWhiteSpace(remotePath))
            {
                await RunOnUiThreadAsync(() =>
                    SetLoading(false, result.Success ? "No Jetson flight logs were found." : result.Message));
                return;
            }

            string directory = GetLogDirectory();
            Directory.CreateDirectory(directory);
            string localPath = UniquePath(directory, Path.GetFileName(remotePath));
            var download = await _sender.DownloadFileViaScpAsync(remotePath, localPath, 120);
            if (!download.Success)
            {
                await RunOnUiThreadAsync(() => SetLoading(false, download.Message));
                return;
            }
            await LoadFileAsync(localPath);
        }

        private void OpenInMissionPlanner()
        {
            if (_logData == null || string.IsNullOrWhiteSpace(_logData.SourcePath)) return;
            try
            {
                var browser = new LogBrowse();
                browser.LoadLog(_logData.SourcePath);
                browser.Show(FindForm());
            }
            catch (Exception ex)
            {
                MessageBox.Show(FindForm(), ex.Message, "Mission Planner Log Browse",
                    MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void ExportMarkdown()
        {
            if (_summary == null) return;
            using (var dialog = new SaveFileDialog
            {
                Title = "Export flight log summary",
                Filter = "Markdown (*.md)|*.md",
                FileName = Path.GetFileNameWithoutExtension(_summary.SourceName) + "-summary.md",
            })
            {
                if (dialog.ShowDialog(FindForm()) != DialogResult.OK) return;
                File.WriteAllText(dialog.FileName, BuildMarkdown(_summary), Encoding.UTF8);
                SetLoading(false, $"Summary exported to {dialog.FileName}");
            }
        }

        private void ExportPng()
        {
            if (_summary == null || _postFlightContent.Width <= 0 || _postFlightContent.Height <= 0) return;
            using (var dialog = new SaveFileDialog
            {
                Title = "Export flight log view",
                Filter = "PNG image (*.png)|*.png",
                FileName = Path.GetFileNameWithoutExtension(_summary.SourceName) + "-analysis.png",
            })
            {
                if (dialog.ShowDialog(FindForm()) != DialogResult.OK) return;
                using (var bitmap = new Bitmap(_postFlightContent.Width, _postFlightContent.Height))
                {
                    _postFlightContent.DrawToBitmap(bitmap, _postFlightContent.ClientRectangle);
                    bitmap.Save(dialog.FileName, System.Drawing.Imaging.ImageFormat.Png);
                }
                SetLoading(false, $"Image exported to {dialog.FileName}");
            }
        }

        private static string BuildMarkdown(LogSummary summary)
        {
            var text = new StringBuilder();
            text.AppendLine($"# Flight Log Summary - {summary.SourceName}");
            text.AppendLine();
            text.AppendLine($"**Verdict:** {summary.OverallVerdict} - {summary.OverallText}");
            text.AppendLine();
            text.AppendLine("| Metric | Value | Detail |");
            text.AppendLine("|---|---:|---|");
            foreach (LogMetric metric in summary.Metrics)
                text.AppendLine($"| {metric.Label} | {metric.Value} {metric.Unit} | {metric.Detail} |");
            text.AppendLine();
            text.AppendLine("## Issues");
            if (summary.Anomalies.Count == 0) text.AppendLine("- None");
            foreach (LogAnomaly issue in summary.Anomalies)
            {
                string issueTime = TimeSpan.FromSeconds(issue.TimeSeconds).ToString(@"mm\:ss");
                text.AppendLine(
                    $"- **{issue.Verdict}: {issue.Title}** at {issueTime} - {issue.Detail}");
            }
            text.AppendLine();
            text.AppendLine("## Flight Modes");
            foreach (ModeSpan mode in summary.Modes)
                text.AppendLine($"- {mode.Mode}: {TimeSpan.FromSeconds(mode.DurationSeconds):mm\\:ss}");
            return text.ToString();
        }

        private void SetLoading(bool loading, string status)
        {
            _loadProgress.Visible = loading;
            _loadStatus.Text = status ?? "";
        }

        private LogAnalysisThresholds CreateThresholds()
        {
            return new LogAnalysisThresholds
            {
                VibrationWarning = _config.LogVibrationWarning,
                VibrationCritical = Math.Max(_config.LogVibrationWarning, _config.LogVibrationCritical),
                HdopWarning = _config.LogHdopWarning,
                HdopCritical = Math.Max(_config.LogHdopWarning, _config.LogHdopCritical),
                MinimumSatellites = _config.LogMinimumSatellites,
                AttitudeRmsWarning = _config.LogTuneRmsWarning,
                AttitudeRmsCritical = Math.Max(_config.LogTuneRmsWarning, _config.LogTuneRmsCritical),
                EkfVarianceWarning = _config.LogEkfVarianceWarning,
                EkfVarianceCritical = Math.Max(
                    _config.LogEkfVarianceWarning,
                    _config.LogEkfVarianceCritical),
            };
        }

        private string GetLogDirectory()
        {
            string configured = ExistingDirectory(_config.DefaultLogDirectory);
            if (!string.IsNullOrWhiteSpace(configured)) return configured;
            string documents = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
            return Path.Combine(documents, "Mission Planner", "logs");
        }

        private static string ExistingDirectory(string path)
            => !string.IsNullOrWhiteSpace(path) && Directory.Exists(path) ? path : null;

        private static string FindNewestLog(string directory, DateTime notBeforeUtc)
        {
            if (!Directory.Exists(directory)) return null;
            return Directory.EnumerateFiles(directory, "*.*", SearchOption.AllDirectories)
                .Where(path => path.EndsWith(".bin", StringComparison.OrdinalIgnoreCase)
                    || path.EndsWith(".log", StringComparison.OrdinalIgnoreCase))
                .Select(path => new FileInfo(path))
                .Where(info => info.LastWriteTimeUtc >= notBeforeUtc)
                .OrderByDescending(info => info.LastWriteTimeUtc)
                .Select(info => info.FullName)
                .FirstOrDefault();
        }

        private static string UniquePath(string directory, string fileName)
        {
            string safeName = string.IsNullOrWhiteSpace(fileName) ? "flight-log.bin" : fileName;
            string candidate = Path.Combine(directory, safeName);
            if (!File.Exists(candidate)) return candidate;
            string timestampedName =
                $"{Path.GetFileNameWithoutExtension(safeName)}-{DateTime.Now:yyyyMMdd-HHmmss}" +
                Path.GetExtension(safeName);
            return Path.Combine(directory, timestampedName);
        }

        private static string FormatBytes(uint bytes)
        {
            if (bytes < 1024) return $"{bytes} B";
            if (bytes < 1024 * 1024) return $"{bytes / 1024d:F1} KB";
            return $"{bytes / (1024d * 1024d):F1} MB";
        }

        private static string ShellQuote(string value) => "'" + value.Replace("'", "'\"'\"'") + "'";

        private static string RemoteDirectoryExpression(string path)
        {
            if (path.StartsWith("~/", StringComparison.Ordinal))
                return "$HOME/" + ShellQuote(path.Substring(2));
            return ShellQuote(path);
        }
    }
}
