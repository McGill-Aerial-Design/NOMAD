// ============================================================
// NOMAD Task 2 Upload Panel
// ============================================================
// Submits Task 2 spray artifacts (before image, after image,
// optional video of the spray) to Google Drive.
//
// Two flows:
//   1. AUTO  — polls /api/spray/status; when state transitions
//              to "complete", fetches /api/task/2/spray/last_artifacts
//              and uploads to the Task 2 Drive folder.
//   2. MANUAL — Start button captures the "before" image and
//               begins recording; Stop captures the "after"
//               image, ends recording, and uploads.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Net.Http;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public class Task2UploadPanel : UserControl
    {
        private static readonly Color CARD_BG        = NOMADTheme.CARD_BG;
        private static readonly Color TEXT_PRIMARY   = NOMADTheme.TEXT_PRIMARY;
        private static readonly Color TEXT_SECONDARY = NOMADTheme.TEXT_SECONDARY;
        private static readonly Color ACCENT_COLOR   = NOMADTheme.ACCENT;
        private static readonly Color SUCCESS_COLOR  = NOMADTheme.SUCCESS;
        private static readonly Color WARNING_COLOR  = NOMADTheme.WARNING;
        private static readonly Color ERROR_COLOR    = NOMADTheme.ERROR;

        private readonly NOMADConfig _config;

        private Button   _btnStart;
        private Button   _btnStop;
        private Button   _btnUploadNow;
        private CheckBox _chkAutoUpload;
        private PictureBox _picBefore;
        private PictureBox _picAfter;
        private Label _lblBefore;
        private Label _lblAfter;
        private Label _lblVideo;
        private Label _lblStatus;
        private TextBox _txtLog;
        private ProgressBar _progress;

        private System.Threading.Timer _statusPollTimer;
        private string _lastSprayState = "idle";
        private string _lastUploadedSessionKey = "";   // dedupe auto-uploads
        private volatile bool _uploadInFlight;

        public Task2UploadPanel(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            InitializeUI();

            _statusPollTimer = new System.Threading.Timer(
                _ => PollSprayStatus(), null,
                TimeSpan.FromSeconds(2), TimeSpan.FromSeconds(2));

            this.Disposed += (s, e) =>
            {
                _statusPollTimer?.Dispose();
                _picBefore?.Image?.Dispose();
                _picAfter?.Image?.Dispose();
            };
        }

        // ============================================================
        // UI
        // ============================================================
        private void InitializeUI()
        {
            BackColor = CARD_BG;
            Dock = DockStyle.Fill;
            Padding = new Padding(8);
            AutoScroll = true;

            int y = 8;

            // ---- Manual flow ----
            var lblManual = new Label
            {
                Text = "MANUAL SPRAY SUBMISSION",
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(8, y),
                AutoSize = true,
            };
            Controls.Add(lblManual);
            y += 22;

            _btnStart = MakeButton("Start (capture before + record)", Color.FromArgb(40, 130, 60), 240, 30);
            _btnStart.Location = new Point(8, y);
            _btnStart.Click += async (s, e) => await ManualStart();
            Controls.Add(_btnStart);

            _btnStop = MakeButton("Stop (capture after + upload)", Color.FromArgb(180, 80, 30), 240, 30);
            _btnStop.Location = new Point(254, y);
            _btnStop.Enabled = false;
            _btnStop.Click += async (s, e) => await ManualStop();
            Controls.Add(_btnStop);
            y += 38;

            // ---- Auto flow ----
            _chkAutoUpload = new CheckBox
            {
                Text = "Auto-upload after autonomous spray completes",
                Checked = true,
                ForeColor = TEXT_PRIMARY,
                Font = new Font("Segoe UI", 9),
                Location = new Point(8, y),
                AutoSize = true,
            };
            Controls.Add(_chkAutoUpload);
            y += 24;

            _btnUploadNow = MakeButton("Upload last spray now", Color.FromArgb(0, 122, 204), 200, 28);
            _btnUploadNow.Location = new Point(8, y);
            _btnUploadNow.Click += async (s, e) => await UploadLastArtifacts(manualTrigger: true);
            Controls.Add(_btnUploadNow);
            y += 36;

            _progress = new ProgressBar
            {
                Location = new Point(8, y),
                Size = new Size(490, 14),
                Style = ProgressBarStyle.Continuous,
            };
            Controls.Add(_progress);
            y += 22;

            _lblStatus = new Label
            {
                Text = "Idle",
                Font = new Font("Consolas", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(8, y),
                AutoSize = true,
            };
            Controls.Add(_lblStatus);
            y += 22;

            // ---- Image previews ----
            _lblBefore = new Label
            {
                Text = "BEFORE",
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(8, y),
                AutoSize = true,
            };
            Controls.Add(_lblBefore);
            _lblAfter = new Label
            {
                Text = "AFTER",
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(254, y),
                AutoSize = true,
            };
            Controls.Add(_lblAfter);
            y += 18;

            _picBefore = new PictureBox
            {
                Location = new Point(8, y),
                Size = new Size(240, 180),
                BackColor = Color.FromArgb(20, 20, 22),
                BorderStyle = BorderStyle.FixedSingle,
                SizeMode = PictureBoxSizeMode.Zoom,
            };
            Controls.Add(_picBefore);
            _picAfter = new PictureBox
            {
                Location = new Point(254, y),
                Size = new Size(240, 180),
                BackColor = Color.FromArgb(20, 20, 22),
                BorderStyle = BorderStyle.FixedSingle,
                SizeMode = PictureBoxSizeMode.Zoom,
            };
            Controls.Add(_picAfter);
            y += 188;

            _lblVideo = new Label
            {
                Text = "Video: --",
                Font = new Font("Consolas", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(8, y),
                AutoSize = true,
            };
            Controls.Add(_lblVideo);
            y += 22;

            _txtLog = new TextBox
            {
                Location = new Point(8, y),
                Size = new Size(490, 130),
                Multiline = true,
                ReadOnly = true,
                ScrollBars = ScrollBars.Vertical,
                BackColor = Color.FromArgb(20, 20, 22),
                ForeColor = SUCCESS_COLOR,
                Font = new Font("Consolas", 8),
                BorderStyle = BorderStyle.FixedSingle,
            };
            Controls.Add(_txtLog);
        }

        // ============================================================
        // Manual flow
        // ============================================================
        private async Task ManualStart()
        {
            try
            {
                _btnStart.Enabled = false;
                SetStatus("Starting manual session: capturing before + recording video…", ACCENT_COLOR);
                Log("manual.start → POST /api/task/2/spray/manual/start");

                var resp = await JetsonApiService.PostAsync("/api/task/2/spray/manual/start");
                if (!resp.IsSuccessStatusCode)
                {
                    var body = await resp.Content.ReadAsStringAsync();
                    SetStatus($"Start failed: HTTP {(int)resp.StatusCode}", ERROR_COLOR);
                    Log("  ← " + body);
                    _btnStart.Enabled = true;
                    return;
                }

                var json = JObject.Parse(await resp.Content.ReadAsStringAsync());
                var beforePath = json["before_image_path"]?.ToString();
                Log("  ← " + json.ToString(Newtonsoft.Json.Formatting.None));
                if (!string.IsNullOrEmpty(beforePath))
                    LoadPreview(_picBefore, beforePath);

                _btnStop.Enabled = true;
                SetStatus("Manual session active — spray now, then click Stop", SUCCESS_COLOR);
            }
            catch (Exception ex)
            {
                SetStatus($"Start error: {ex.Message}", ERROR_COLOR);
                Log("ERROR " + ex);
                _btnStart.Enabled = true;
            }
        }

        private async Task ManualStop()
        {
            try
            {
                _btnStop.Enabled = false;
                SetStatus("Capturing after image, finalising recording…", ACCENT_COLOR);
                Log("manual.stop → POST /api/task/2/spray/manual/stop");

                var resp = await JetsonApiService.PostAsync("/api/task/2/spray/manual/stop");
                if (!resp.IsSuccessStatusCode)
                {
                    var body = await resp.Content.ReadAsStringAsync();
                    SetStatus($"Stop failed: HTTP {(int)resp.StatusCode}", ERROR_COLOR);
                    Log("  ← " + body);
                    _btnStop.Enabled = true;
                    return;
                }

                var json = JObject.Parse(await resp.Content.ReadAsStringAsync());
                Log("  ← " + json.ToString(Newtonsoft.Json.Formatting.None));
                _btnStart.Enabled = true;

                var afterPath = json["after_image_path"]?.ToString();
                if (!string.IsNullOrEmpty(afterPath))
                    LoadPreview(_picAfter, afterPath);

                var sessionKey = json["session_id"]?.ToString() ?? Guid.NewGuid().ToString();
                await UploadArtifactsFromJson(json, sessionKey);
            }
            catch (Exception ex)
            {
                SetStatus($"Stop error: {ex.Message}", ERROR_COLOR);
                Log("ERROR " + ex);
                _btnStop.Enabled = true;
            }
        }

        // ============================================================
        // Auto flow — poll spray status
        // ============================================================
        private async void PollSprayStatus()
        {
            if (IsDisposed) return;
            try
            {
                var resp = await JetsonApiService.GetAsync("/api/spray/status");
                if (!resp.IsSuccessStatusCode) return;
                var json = JObject.Parse(await resp.Content.ReadAsStringAsync());
                var state = json["state"]?.ToString() ?? "idle";

                bool justCompleted = _lastSprayState != "complete" && state == "complete";
                _lastSprayState = state;

                if (justCompleted && _chkAutoUpload != null && _chkAutoUpload.Checked && !_uploadInFlight)
                {
                    BeginInvoke(new Action(async () =>
                    {
                        Log($"Auto-upload: spray state→complete, fetching artifacts…");
                        await UploadLastArtifacts(manualTrigger: false);
                    }));
                }
            }
            catch { }
        }

        // ============================================================
        // Upload helpers
        // ============================================================
        private async Task UploadLastArtifacts(bool manualTrigger)
        {
            if (_uploadInFlight) return;
            try
            {
                var resp = await JetsonApiService.GetAsync("/api/task/2/spray/last_artifacts");
                if (!resp.IsSuccessStatusCode)
                {
                    if (manualTrigger)
                        SetStatus($"No artifacts available (HTTP {(int)resp.StatusCode})", WARNING_COLOR);
                    return;
                }
                var json = JObject.Parse(await resp.Content.ReadAsStringAsync());
                var sessionKey = json["session_id"]?.ToString() ?? "";
                if (!manualTrigger && sessionKey == _lastUploadedSessionKey)
                {
                    Log("Skipping auto-upload (session already uploaded)");
                    return;
                }

                var beforePath = json["before_image_path"]?.ToString();
                var afterPath  = json["after_image_path"]?.ToString();
                if (!string.IsNullOrEmpty(beforePath)) LoadPreview(_picBefore, beforePath);
                if (!string.IsNullOrEmpty(afterPath))  LoadPreview(_picAfter,  afterPath);

                await UploadArtifactsFromJson(json, sessionKey);
            }
            catch (Exception ex)
            {
                SetStatus($"Upload error: {ex.Message}", ERROR_COLOR);
                Log("ERROR " + ex);
            }
        }

        private async Task UploadArtifactsFromJson(JObject json, string sessionKey)
        {
            _uploadInFlight = true;
            try
            {
                _progress.Value = 0;
                _progress.Style = ProgressBarStyle.Marquee;
                SetStatus("Uploading artifacts to Google Drive…", ACCENT_COLOR);

                var beforePath = json["before_image_path"]?.ToString();
                var afterPath  = json["after_image_path"]?.ToString();
                var videoPath  = json["video_path"]?.ToString();

                var gdrive = new GoogleDriveUploadService();
                if (!gdrive.HasToken())
                {
                    SetStatus("Google Drive token missing — see Settings", ERROR_COLOR);
                    return;
                }
                string folderId = gdrive.GetTask2FolderId();
                string ts = DateTime.Now.ToString("yyyyMMdd_HHmmss");

                int ok = 0, fail = 0;
                async Task UploadOne(string remotePath, string namePrefix)
                {
                    if (string.IsNullOrEmpty(remotePath)) return;
                    try
                    {
                        var local = await DownloadArtifact(remotePath);
                        if (local == null) { fail++; return; }
                        var fname = $"{namePrefix}_{ts}{Path.GetExtension(local)}";
                        var fileId = await gdrive.UploadFileAsync(local, fname, folderId);
                        Log($"  uploaded {fname} → {fileId}");
                        ok++;
                    }
                    catch (Exception ex) { Log($"  upload failed for {remotePath}: {ex.Message}"); fail++; }
                }

                await UploadOne(beforePath, "task2_before");
                await UploadOne(afterPath,  "task2_after");
                await UploadOne(videoPath,  "task2_spray");

                _progress.Style = ProgressBarStyle.Continuous;
                _progress.Value = 100;
                _lastUploadedSessionKey = sessionKey;
                SetStatus($"Upload done — {ok} ok, {fail} failed", fail == 0 ? SUCCESS_COLOR : WARNING_COLOR);
            }
            finally
            {
                _uploadInFlight = false;
            }
        }

        /// <summary>
        /// Pull a server-side artifact down to a temp file. The server is expected
        /// to expose artifacts under /api/task/2/spray/artifact?path=...
        /// </summary>
        private async Task<string> DownloadArtifact(string remotePath)
        {
            try
            {
                var url = $"/api/task/2/spray/artifact?path={Uri.EscapeDataString(remotePath)}";
                var resp = await JetsonApiService.GetAsync(url);
                if (!resp.IsSuccessStatusCode) return null;
                var bytes = await resp.Content.ReadAsByteArrayAsync();
                var ext = Path.GetExtension(remotePath);
                if (string.IsNullOrEmpty(ext)) ext = ".bin";
                var local = Path.Combine(Path.GetTempPath(),
                    $"nomad_t2_{Guid.NewGuid():N}{ext}");
                File.WriteAllBytes(local, bytes);
                return local;
            }
            catch { return null; }
        }

        private async void LoadPreview(PictureBox box, string remotePath)
        {
            try
            {
                var local = await DownloadArtifact(remotePath);
                if (local == null) return;
                if (box.IsDisposed) return;
                var img = Image.FromFile(local);
                BeginInvoke(new Action(() =>
                {
                    box.Image?.Dispose();
                    box.Image = img;
                }));
            }
            catch (Exception ex) { Log("preview err: " + ex.Message); }
        }

        // ============================================================
        private Button MakeButton(string text, Color back, int w, int h)
        {
            var b = new Button
            {
                Text = text,
                Size = new Size(w, h),
                FlatStyle = FlatStyle.Flat,
                BackColor = back,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Cursor = Cursors.Hand,
            };
            b.FlatAppearance.BorderColor = Color.FromArgb(80, 80, 85);
            return b;
        }

        private void SetStatus(string text, Color color)
        {
            if (InvokeRequired) { BeginInvoke(new Action(() => SetStatus(text, color))); return; }
            if (_lblStatus == null) return;
            _lblStatus.Text = text;
            _lblStatus.ForeColor = color;
        }

        private void Log(string line)
        {
            if (InvokeRequired) { BeginInvoke(new Action(() => Log(line))); return; }
            if (_txtLog == null) return;
            _txtLog.AppendText($"[{DateTime.Now:HH:mm:ss}] {line}\r\n");
        }
    }
}
