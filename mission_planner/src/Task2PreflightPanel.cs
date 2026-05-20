// ============================================================
// Task 2 Preflight Panel
// ============================================================
// Backend-backed readiness checklist for the one-target autonomy demo.
// It combines Jetson checks (/api/task/2/preflight) with the operator-set
// Phase 2 target color (/api/task/2/target_color).
// ============================================================

using System;
using System.Drawing;
using System.Net.Http;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public class Task2PreflightPanel : Panel
    {
        private readonly ComboBox _cmbColor;
        private readonly Button _btnSetColor;
        private readonly Button _btnRefresh;
        private readonly Label _lblReady;
        private readonly Label _lblColor;
        private readonly FlowLayoutPanel _checks;
        private readonly System.Windows.Forms.Timer _timer;
        private bool _refreshing;
        private bool _initialRefreshDone;

        private static readonly string[] TargetColors =
        {
            "purple", "blue", "red", "orange", "yellow",
            "green", "black", "white", "magenta", "cyan"
        };

        public Task2PreflightPanel()
        {
            BackColor = NOMADTheme.BG_DARK;
            Padding = new Padding(12);

            var layout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.BG_DARK,
                ColumnCount = 1,
                RowCount = 4,
            };
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 34));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 78));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 42));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            Controls.Add(layout);

            var header = new Label
            {
                Text = "Task 2 Autonomy Preflight",
                Dock = DockStyle.Fill,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = new Font("Segoe UI", 14, FontStyle.Bold),
                TextAlign = ContentAlignment.MiddleLeft,
            };
            layout.Controls.Add(header, 0, 0);

            var colorPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(10),
                Margin = new Padding(0, 0, 0, 8),
            };

            _lblColor = new Label
            {
                Text = "Target color: loading",
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Font = new Font("Consolas", 10, FontStyle.Bold),
                Location = new Point(10, 8),
                AutoSize = true,
            };
            colorPanel.Controls.Add(_lblColor);

            _cmbColor = new ComboBox
            {
                DropDownStyle = ComboBoxStyle.DropDownList,
                Location = new Point(10, 36),
                Width = 130,
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
            };
            _cmbColor.Items.AddRange(TargetColors);
            _cmbColor.SelectedIndex = 0;
            colorPanel.Controls.Add(_cmbColor);

            _btnSetColor = MakeButton("Set color", NOMADTheme.ACCENT, 100, 28);
            _btnSetColor.Location = new Point(150, 35);
            _btnSetColor.Click += async (s, e) => await SetColor();
            colorPanel.Controls.Add(_btnSetColor);

            _btnRefresh = MakeButton("Refresh checks", NOMADTheme.INFO, 130, 28);
            _btnRefresh.Location = new Point(260, 35);
            _btnRefresh.Click += async (s, e) => await RefreshChecks();
            colorPanel.Controls.Add(_btnRefresh);

            layout.Controls.Add(colorPanel, 0, 1);

            _lblReady = new Label
            {
                Text = "Readiness: not checked",
                Dock = DockStyle.Fill,
                ForeColor = NOMADTheme.WARNING,
                Font = new Font("Consolas", 12, FontStyle.Bold),
                TextAlign = ContentAlignment.MiddleLeft,
                Padding = new Padding(4, 6, 0, 0),
            };
            layout.Controls.Add(_lblReady, 0, 2);

            _checks = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoScroll = true,
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(0, 8, 0, 0),
            };
            layout.Controls.Add(_checks, 0, 3);

            _timer = new System.Windows.Forms.Timer { Interval = 5000 };
            _timer.Tick += async (s, e) => await RefreshChecks();
            _timer.Start();

            HandleCreated += async (s, e) =>
            {
                if (_initialRefreshDone) return;
                _initialRefreshDone = true;
                await LoadColor();
                await RefreshChecks();
            };
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _timer?.Stop();
                _timer?.Dispose();
            }
            base.Dispose(disposing);
        }

        private Button MakeButton(string text, Color bg, int width, int height)
        {
            var b = new Button
            {
                Text = text,
                BackColor = bg,
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Width = width,
                Height = height,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
            };
            b.FlatAppearance.BorderSize = 0;
            return b;
        }

        private async Task LoadColor()
        {
            try
            {
                var resp = await JetsonApiService.GetAsync("/api/task/2/target_color");
                if (!resp.IsSuccessStatusCode)
                {
                    _lblColor.Text = $"Target color: unavailable (HTTP {(int)resp.StatusCode})";
                    _lblColor.ForeColor = NOMADTheme.WARNING;
                    return;
                }

                var json = JObject.Parse(await resp.Content.ReadAsStringAsync());
                var color = json["color"]?.ToString() ?? "purple";
                var source = json["source"]?.ToString() ?? "default";
                SelectColor(color);
                _lblColor.Text = $"Target color: {color} ({source})";
                _lblColor.ForeColor = source == "file" ? NOMADTheme.SUCCESS : NOMADTheme.WARNING;
            }
            catch (Exception ex)
            {
                _lblColor.Text = $"Target color: error - {ex.Message}";
                _lblColor.ForeColor = NOMADTheme.ERROR;
            }
        }

        private void SelectColor(string color)
        {
            for (int i = 0; i < _cmbColor.Items.Count; i++)
            {
                if (string.Equals(_cmbColor.Items[i]?.ToString(), color, StringComparison.OrdinalIgnoreCase))
                {
                    _cmbColor.SelectedIndex = i;
                    return;
                }
            }
        }

        private async Task SetColor()
        {
            try
            {
                _btnSetColor.Enabled = false;
                var color = _cmbColor.SelectedItem?.ToString() ?? "purple";
                var body = new JObject { ["color"] = color };
                var content = new StringContent(body.ToString(), Encoding.UTF8, "application/json");
                var resp = await JetsonApiService.PostAsync("/api/task/2/target_color", content);
                if (!resp.IsSuccessStatusCode)
                {
                    _lblColor.Text = $"Set color failed: {await ExtractError(resp)}";
                    _lblColor.ForeColor = NOMADTheme.ERROR;
                    return;
                }

                _lblColor.Text = $"Target color: {color} (file)";
                _lblColor.ForeColor = NOMADTheme.SUCCESS;
                AudioAlerts.Speak($"Task 2 target color set to {color}.", ignoreRateLimit: true);
                await RefreshChecks();
            }
            catch (Exception ex)
            {
                _lblColor.Text = $"Set color error: {ex.Message}";
                _lblColor.ForeColor = NOMADTheme.ERROR;
            }
            finally
            {
                _btnSetColor.Enabled = true;
            }
        }

        private async Task RefreshChecks()
        {
            if (_refreshing) return;
            _refreshing = true;
            try
            {
                _btnRefresh.Enabled = false;
                var resp = await JetsonApiService.GetAsync("/api/task/2/preflight");
                if (!resp.IsSuccessStatusCode)
                {
                    _lblReady.Text = $"Readiness: unavailable - HTTP {(int)resp.StatusCode}";
                    _lblReady.ForeColor = NOMADTheme.ERROR;
                    return;
                }

                var json = JObject.Parse(await resp.Content.ReadAsStringAsync());
                var ready = json["ready"]?.Value<bool>() ?? false;
                _lblReady.Text = ready
                    ? "Readiness: READY for one-target autonomy demo"
                    : "Readiness: NOT READY - resolve red checks before claiming autonomy";
                _lblReady.ForeColor = ready ? NOMADTheme.SUCCESS : NOMADTheme.ERROR;

                _checks.SuspendLayout();
                _checks.Controls.Clear();
                var items = json["items"] as JArray;
                if (items != null)
                {
                    foreach (JObject item in items)
                    {
                        _checks.Controls.Add(MakeCheckRow(item));
                    }
                }
                _checks.ResumeLayout();
            }
            catch (Exception ex)
            {
                _lblReady.Text = $"Readiness: error - {ex.Message}";
                _lblReady.ForeColor = NOMADTheme.ERROR;
            }
            finally
            {
                _btnRefresh.Enabled = true;
                _refreshing = false;
            }
        }

        private Control MakeCheckRow(JObject item)
        {
            var ok = item["ok"]?.Value<bool>() ?? false;
            var label = item["label"]?.ToString() ?? item["key"]?.ToString() ?? "check";
            var detail = item["detail"]?.ToString() ?? "";

            var row = new Panel
            {
                Width = Math.Max(300, ClientSize.Width - 42),
                Height = 62,
                BackColor = NOMADTheme.CARD_BG,
                Margin = new Padding(0, 0, 0, 8),
                Padding = new Padding(8),
            };

            var status = new Label
            {
                Text = ok ? "OK" : "FAIL",
                ForeColor = Color.White,
                BackColor = ok ? NOMADTheme.SUCCESS : NOMADTheme.ERROR,
                Font = new Font("Consolas", 9, FontStyle.Bold),
                TextAlign = ContentAlignment.MiddleCenter,
                Location = new Point(8, 10),
                Size = new Size(44, 22),
            };
            row.Controls.Add(status);

            var title = new Label
            {
                Text = label,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Location = new Point(62, 6),
                AutoSize = false,
                Width = row.Width - 74,
                Height = 20,
            };
            row.Controls.Add(title);

            var detailLabel = new Label
            {
                Text = detail,
                ForeColor = ok ? NOMADTheme.TEXT_SECONDARY : NOMADTheme.WARNING,
                Font = new Font("Consolas", 8),
                Location = new Point(62, 28),
                AutoSize = false,
                Width = row.Width - 74,
                Height = 28,
            };
            row.Controls.Add(detailLabel);

            return row;
        }

        private static async Task<string> ExtractError(HttpResponseMessage resp)
        {
            try
            {
                var body = await resp.Content.ReadAsStringAsync();
                if (string.IsNullOrWhiteSpace(body)) return $"HTTP {(int)resp.StatusCode}";
                try { return JObject.Parse(body)["detail"]?.ToString() ?? body; }
                catch { return body; }
            }
            catch
            {
                return $"HTTP {(int)resp.StatusCode}";
            }
        }
    }
}
