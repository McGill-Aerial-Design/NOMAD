// ============================================================
// NOMAD Enhanced Health Dashboard
// ============================================================
// Provides comprehensive system health monitoring for the Jetson
// Orin Nano with real-time graphs and alerts.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Net.Http;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Enhanced health dashboard with graphs and detailed monitoring.
    /// </summary>
    public class EnhancedHealthDashboard : UserControl
    {
        // ============================================================
        // Fields
        // ============================================================
        
        private NOMADConfig _config;
        private readonly HttpClient _httpClient;
        private System.Threading.Timer _pollTimer;
        
        // Data history for graphs
        private readonly Queue<float> _cpuTempHistory = new Queue<float>();
        private readonly Queue<float> _gpuTempHistory = new Queue<float>();
        private readonly Queue<float> _cpuLoadHistory = new Queue<float>();
        private readonly Queue<float> _gpuLoadHistory = new Queue<float>();
        private readonly Queue<float> _memoryHistory = new Queue<float>();
        private const int HISTORY_LENGTH = 60; // 2 minutes at 2s interval
        
        // UI Controls
        private Panel _statusPanel;
        private Panel _metricsPanel;
        private Panel _graphPanel;
        private Panel _networkPanel;
        private Panel _alertsPanel;
        
        // Status indicators
        private Label _lblOverallStatus;
        private Label _lblLastUpdate;
        
        // Metric labels
        private Label _lblCpuTemp, _lblCpuLoad;
        private Label _lblGpuTemp, _lblGpuLoad;
        private Label _lblMemory, _lblDisk;
        private Label _lblPower, _lblFan;
        
        // Progress bars
        private ProgressBar _prgCpuTemp, _prgCpuLoad;
        private ProgressBar _prgGpuTemp, _prgGpuLoad;
        private ProgressBar _prgMemory, _prgDisk;
        
        // Network labels
        private Label _lblTailscaleStatus;
        private Label _lblTailscaleIP;
        private Label _lblLatency;
        private Label _lblVioStatus;
        
        // Graph panel for drawing
        private PictureBox _graphBox;
        private ComboBox _cmbGraphType;
        
        // Alerts
        private ListBox _lstAlerts;
        private readonly List<string> _alerts = new List<string>();
        
        // ============================================================
        // Constructor
        // ============================================================
        
        public EnhancedHealthDashboard(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            
            _httpClient = new HttpClient
            {
                Timeout = TimeSpan.FromSeconds(3)
            };
            
            InitializeUI();
            StartPolling();
        }
        
        // ============================================================
        // UI Initialization
        // ============================================================
        
        private void InitializeUI()
        {
            this.BackColor = Color.FromArgb(30, 30, 30);
            this.Dock = DockStyle.Fill;
            this.AutoScroll = true;
            
            var mainPanel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 3,
                BackColor = Color.Transparent,
                Padding = new Padding(10),
            };
            
            mainPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            mainPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, 80));
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, 280));
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            
            // Row 0: Overall Status
            _statusPanel = CreateStatusPanel();
            mainPanel.Controls.Add(_statusPanel, 0, 0);
            mainPanel.SetColumnSpan(_statusPanel, 2);
            
            // Row 1, Col 0: Metrics
            _metricsPanel = CreateMetricsPanel();
            mainPanel.Controls.Add(_metricsPanel, 0, 1);
            
            // Row 1, Col 1: Network/VIO
            _networkPanel = CreateNetworkPanel();
            mainPanel.Controls.Add(_networkPanel, 1, 1);
            
            // Row 2, Col 0: Graph
            _graphPanel = CreateGraphPanel();
            mainPanel.Controls.Add(_graphPanel, 0, 2);
            
            // Row 2, Col 1: Alerts
            _alertsPanel = CreateAlertsPanel();
            mainPanel.Controls.Add(_alertsPanel, 1, 2);
            
            this.Controls.Add(mainPanel);
        }
        
        private Panel CreateStatusPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
                Margin = new Padding(5),
                Padding = new Padding(15, 10, 15, 10),
            };
            
            _lblOverallStatus = new Label
            {
                Text = "● CONNECTING...",
                Font = new Font("Segoe UI", 18, FontStyle.Bold),
                ForeColor = Color.Yellow,
                Location = new Point(15, 15),
                AutoSize = true,
            };
            panel.Controls.Add(_lblOverallStatus);
            
            _lblLastUpdate = new Label
            {
                Text = "Last update: Never",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.Gray,
                Location = new Point(15, 50),
                AutoSize = true,
            };
            panel.Controls.Add(_lblLastUpdate);
            
            var btnRefresh = new Button
            {
                Text = "[REF] Refresh",
                Location = new Point(400, 20),
                Size = new Size(90, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
            };
            btnRefresh.Click += (s, e) => RefreshHealth();
            panel.Controls.Add(btnRefresh);
            
            return panel;
        }
        
        private Panel CreateMetricsPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
                Margin = new Padding(5),
                Padding = new Padding(10),
            };
            
            var title = new Label
            {
                Text = "System Metrics",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = Color.FromArgb(0, 150, 200),
                Location = new Point(10, 10),
                AutoSize = true,
            };
            panel.Controls.Add(title);
            
            int yOffset = 40;
            
            // CPU Temperature
            CreateMetricRow(panel, "CPU Temp:", ref _lblCpuTemp, ref _prgCpuTemp, ref yOffset, Color.Orange);
            
            // GPU Temperature
            CreateMetricRow(panel, "GPU Temp:", ref _lblGpuTemp, ref _prgGpuTemp, ref yOffset, Color.OrangeRed);
            
            // CPU Load
            CreateMetricRow(panel, "CPU Load:", ref _lblCpuLoad, ref _prgCpuLoad, ref yOffset, Color.DodgerBlue);
            
            // GPU Load
            CreateMetricRow(panel, "GPU Load:", ref _lblGpuLoad, ref _prgGpuLoad, ref yOffset, Color.LimeGreen);
            
            // Memory
            CreateMetricRow(panel, "Memory:", ref _lblMemory, ref _prgMemory, ref yOffset, Color.MediumPurple);
            
            // Disk
            CreateMetricRow(panel, "Disk:", ref _lblDisk, ref _prgDisk, ref yOffset, Color.Goldenrod);
            
            // Power and Fan
            _lblPower = new Label
            {
                Text = "Power: --W",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.LightGray,
                Location = new Point(10, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(_lblPower);
            
            _lblFan = new Label
            {
                Text = "Fan: --%",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.LightGray,
                Location = new Point(150, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(_lblFan);
            
            return panel;
        }
        
        private void CreateMetricRow(Panel panel, string label, ref Label valueLabel, ref ProgressBar progressBar, ref int yOffset, Color color)
        {
            var lblTitle = new Label
            {
                Text = label,
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.LightGray,
                Location = new Point(10, yOffset),
                Width = 70,
            };
            panel.Controls.Add(lblTitle);
            
            progressBar = new ProgressBar
            {
                Location = new Point(85, yOffset),
                Size = new Size(150, 18),
                Maximum = 100,
                Style = ProgressBarStyle.Continuous,
            };
            panel.Controls.Add(progressBar);
            
            valueLabel = new Label
            {
                Text = "--",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.White,
                Location = new Point(245, yOffset),
                Width = 80,
                TextAlign = ContentAlignment.MiddleRight,
            };
            panel.Controls.Add(valueLabel);
            
            yOffset += 30;
        }
        
        private Panel CreateNetworkPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
                Margin = new Padding(5),
                Padding = new Padding(10),
            };
            
            var title = new Label
            {
                Text = "Network & VIO",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = Color.FromArgb(100, 200, 100),
                Location = new Point(10, 10),
                AutoSize = true,
            };
            panel.Controls.Add(title);
            
            int yOffset = 40;
            
            // Tailscale Status
            var lblTsTitle = new Label
            {
                Text = "Tailscale:",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.LightGray,
                Location = new Point(10, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(lblTsTitle);
            
            _lblTailscaleStatus = new Label
            {
                Text = "Unknown",
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                ForeColor = Color.Yellow,
                Location = new Point(90, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(_lblTailscaleStatus);
            yOffset += 25;
            
            // Tailscale IP
            var lblIpTitle = new Label
            {
                Text = "Tailscale IP:",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.LightGray,
                Location = new Point(10, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(lblIpTitle);
            
            _lblTailscaleIP = new Label
            {
                Text = "--",
                Font = new Font("Consolas", 9),
                ForeColor = Color.White,
                Location = new Point(90, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(_lblTailscaleIP);
            yOffset += 35;
            
            // VIO Status
            var vioGroup = new GroupBox
            {
                Text = "VIO Pipeline",
                ForeColor = Color.FromArgb(255, 150, 50),
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(300, 100),
                BackColor = Color.FromArgb(40, 40, 43),
            };
            
            _lblVioStatus = new Label
            {
                Text = "Status: Unknown\nConfidence: --\nRate: -- Hz",
                Font = new Font("Consolas", 9),
                ForeColor = Color.White,
                Location = new Point(15, 25),
                Size = new Size(270, 60),
            };
            vioGroup.Controls.Add(_lblVioStatus);
            
            panel.Controls.Add(vioGroup);
            
            return panel;
        }
        
        private Panel CreateGraphPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
                Margin = new Padding(5),
                Padding = new Padding(10),
            };
            
            var title = new Label
            {
                Text = "History Graph",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = Color.FromArgb(200, 100, 200),
                Location = new Point(10, 10),
                AutoSize = true,
            };
            panel.Controls.Add(title);
            
            _cmbGraphType = new ComboBox
            {
                Location = new Point(120, 7),
                Size = new Size(130, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White,
            };
            _cmbGraphType.Items.AddRange(new object[] { "Temperature", "Load", "Memory" });
            _cmbGraphType.SelectedIndex = 0;
            _cmbGraphType.SelectedIndexChanged += (s, e) => DrawGraph();
            panel.Controls.Add(_cmbGraphType);
            
            _graphBox = new PictureBox
            {
                Location = new Point(10, 40),
                Size = new Size(320, 180),
                BackColor = Color.FromArgb(20, 20, 20),
                BorderStyle = BorderStyle.FixedSingle,
            };
            _graphBox.Paint += GraphBox_Paint;
            panel.Controls.Add(_graphBox);
            
            // Resize handling
            panel.Resize += (s, e) =>
            {
                _graphBox.Width = panel.Width - 25;
                _graphBox.Height = panel.Height - 55;
                DrawGraph();
            };
            
            return panel;
        }
        
        private Panel CreateAlertsPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
                Margin = new Padding(5),
                Padding = new Padding(10),
            };
            
            var title = new Label
            {
                Text = "[!] Alerts",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = Color.FromArgb(255, 100, 100),
                Location = new Point(10, 10),
                AutoSize = true,
            };
            panel.Controls.Add(title);
            
            var btnClear = new Button
            {
                Text = "Clear",
                Location = new Point(220, 7),
                Size = new Size(60, 25),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
            };
            btnClear.Click += (s, e) =>
            {
                _alerts.Clear();
                _lstAlerts.Items.Clear();
            };
            panel.Controls.Add(btnClear);
            
            _lstAlerts = new ListBox
            {
                Location = new Point(10, 40),
                Size = new Size(300, 180),
                BackColor = Color.FromArgb(20, 20, 20),
                ForeColor = Color.Orange,
                Font = new Font("Consolas", 8),
                BorderStyle = BorderStyle.FixedSingle,
            };
            panel.Controls.Add(_lstAlerts);
            
            // Resize handling
            panel.Resize += (s, e) =>
            {
                _lstAlerts.Width = panel.Width - 25;
                _lstAlerts.Height = panel.Height - 55;
                btnClear.Left = panel.Width - 80;
            };
            
            return panel;
        }
        
        // ============================================================
        // Data Polling
        // ============================================================
        
        private void StartPolling()
        {
            _pollTimer = new System.Threading.Timer(
                _ => PollHealth(),
                null,
                TimeSpan.FromMilliseconds(500),
                TimeSpan.FromMilliseconds(_config.HealthPollInterval)
            );
        }
        
        private async void PollHealth()
        {
            try
            {
                var response = await _httpClient.GetAsync($"{_config.EffectiveBaseUrl}/health/detailed");
                
                if (response.IsSuccessStatusCode)
                {
                    var json = await response.Content.ReadAsStringAsync();
                    var data = JObject.Parse(json);
                    
                    this.BeginInvoke((Action)(() => UpdateUI(data)));
                }
                else
                {
                    this.BeginInvoke((Action)(() => UpdateStatusError($"HTTP {response.StatusCode}")));
                }
            }
            catch (Exception ex)
            {
                this.BeginInvoke((Action)(() => UpdateStatusError(ex.Message)));
            }
        }
        
        public void RefreshHealth()
        {
            PollHealth();
        }
        
        // ============================================================
        // UI Updates
        // ============================================================
        
        private void UpdateUI(JObject data)
        {
            try
            {
                // Overall Status
                var status = data["status"]?.ToString() ?? "unknown";
                UpdateOverallStatus(status);
                
                // CPU
                var cpuTemp = data["cpu_temp"]?.Value<float>() ?? 0;
                var cpuLoad = data["cpu_load"]?.Value<float>() ?? 0;
                UpdateMetric(_lblCpuTemp, _prgCpuTemp, cpuTemp, "°C", 85, 95);
                UpdateMetric(_lblCpuLoad, _prgCpuLoad, cpuLoad, "%", 80, 95);
                
                // GPU
                var gpuTemp = data["gpu_temp"]?.Value<float>() ?? 0;
                var gpuLoad = data["gpu_load"]?.Value<float>() ?? 0;
                UpdateMetric(_lblGpuTemp, _prgGpuTemp, gpuTemp, "°C", 85, 95);
                UpdateMetric(_lblGpuLoad, _prgGpuLoad, gpuLoad, "%", 80, 95);
                
                // Memory
                var memUsed = data["memory_used_pct"]?.Value<float>() ?? 0;
                UpdateMetric(_lblMemory, _prgMemory, memUsed, "%", 80, 95);
                
                // Disk
                var diskUsed = data["disk_used_pct"]?.Value<float>() ?? 0;
                UpdateMetric(_lblDisk, _prgDisk, diskUsed, "%", 80, 95);
                
                // Power and Fan
                var power = data["power_draw_w"]?.Value<float>() ?? 0;
                var fan = data["fan_speed_pct"]?.Value<float>() ?? 0;
                _lblPower.Text = $"Power: {power:F1}W";
                _lblFan.Text = $"Fan: {fan:F0}%";
                
                // Tailscale
                var tsConnected = data["tailscale_connected"]?.Value<bool>() ?? false;
                var tsIp = data["tailscale_ip"]?.ToString() ?? "--";
                _lblTailscaleStatus.Text = tsConnected ? "Connected" : "Disconnected";
                _lblTailscaleStatus.ForeColor = tsConnected ? Color.LimeGreen : Color.Red;
                _lblTailscaleIP.Text = tsIp;
                
                // Update history
                UpdateHistory(cpuTemp, gpuTemp, cpuLoad, gpuLoad, memUsed);
                
                // Timestamp
                _lblLastUpdate.Text = $"Last update: {DateTime.Now:HH:mm:ss}";
                
                // Draw graph
                DrawGraph();
                
                // Check alerts
                CheckAlerts(cpuTemp, gpuTemp, memUsed, diskUsed);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Health UI error: {ex.Message}");
            }
        }
        
        private void UpdateOverallStatus(string status)
        {
            switch (status.ToLower())
            {
                case "ok":
                    _lblOverallStatus.Text = "● HEALTHY";
                    _lblOverallStatus.ForeColor = Color.LimeGreen;
                    break;
                case "warning":
                    _lblOverallStatus.Text = "● WARNING";
                    _lblOverallStatus.ForeColor = Color.Yellow;
                    break;
                case "critical":
                    _lblOverallStatus.Text = "● CRITICAL";
                    _lblOverallStatus.ForeColor = Color.Red;
                    break;
                default:
                    _lblOverallStatus.Text = "● UNKNOWN";
                    _lblOverallStatus.ForeColor = Color.Gray;
                    break;
            }
        }
        
        private void UpdateMetric(Label label, ProgressBar progress, float value, string unit, float warnThreshold, float critThreshold)
        {
            label.Text = $"{value:F1}{unit}";
            progress.Value = Math.Min(100, Math.Max(0, (int)value));
            
            if (value >= critThreshold)
            {
                label.ForeColor = Color.Red;
            }
            else if (value >= warnThreshold)
            {
                label.ForeColor = Color.Yellow;
            }
            else
            {
                label.ForeColor = Color.LimeGreen;
            }
        }
        
        private void UpdateStatusError(string error)
        {
            _lblOverallStatus.Text = "● OFFLINE";
            _lblOverallStatus.ForeColor = Color.Red;
            _lblLastUpdate.Text = $"Error: {error}";
        }
        
        private void UpdateHistory(float cpuTemp, float gpuTemp, float cpuLoad, float gpuLoad, float memory)
        {
            AddToHistory(_cpuTempHistory, cpuTemp);
            AddToHistory(_gpuTempHistory, gpuTemp);
            AddToHistory(_cpuLoadHistory, cpuLoad);
            AddToHistory(_gpuLoadHistory, gpuLoad);
            AddToHistory(_memoryHistory, memory);
        }
        
        private void AddToHistory(Queue<float> queue, float value)
        {
            queue.Enqueue(value);
            while (queue.Count > HISTORY_LENGTH)
            {
                queue.Dequeue();
            }
        }
        
        // ============================================================
        // Graph Drawing
        // ============================================================
        
        private void DrawGraph()
        {
            _graphBox?.Invalidate();
        }
        
        private void GraphBox_Paint(object sender, PaintEventArgs e)
        {
            var g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            
            var rect = _graphBox.ClientRectangle;
            
            // Background
            g.Clear(Color.FromArgb(20, 20, 20));
            
            // Grid lines
            using (var gridPen = new Pen(Color.FromArgb(40, 40, 40)))
            {
                for (int i = 0; i <= 4; i++)
                {
                    int y = rect.Height * i / 4;
                    g.DrawLine(gridPen, 0, y, rect.Width, y);
                }
            }
            
            // Get data based on selection
            Queue<float> data1, data2;
            Color color1, color2;
            string label1, label2;
            
            switch (_cmbGraphType?.SelectedIndex ?? 0)
            {
                case 0: // Temperature
                    data1 = _cpuTempHistory;
                    data2 = _gpuTempHistory;
                    color1 = Color.Orange;
                    color2 = Color.OrangeRed;
                    label1 = "CPU";
                    label2 = "GPU";
                    break;
                case 1: // Load
                    data1 = _cpuLoadHistory;
                    data2 = _gpuLoadHistory;
                    color1 = Color.DodgerBlue;
                    color2 = Color.LimeGreen;
                    label1 = "CPU";
                    label2 = "GPU";
                    break;
                default: // Memory
                    data1 = _memoryHistory;
                    data2 = _memoryHistory;
                    color1 = Color.MediumPurple;
                    color2 = Color.MediumPurple;
                    label1 = "Memory";
                    label2 = "";
                    break;
            }
            
            // Draw lines
            DrawGraphLine(g, rect, data1, color1, 100);
            if (_cmbGraphType?.SelectedIndex != 2)
            {
                DrawGraphLine(g, rect, data2, color2, 100);
            }
            
            // Legend
            using (var brush1 = new SolidBrush(color1))
            using (var brush2 = new SolidBrush(color2))
            using (var font = new Font("Segoe UI", 8))
            {
                g.FillRectangle(brush1, rect.Width - 80, 5, 10, 10);
                g.DrawString(label1, font, Brushes.White, rect.Width - 65, 3);
                
                if (!string.IsNullOrEmpty(label2))
                {
                    g.FillRectangle(brush2, rect.Width - 80, 20, 10, 10);
                    g.DrawString(label2, font, Brushes.White, rect.Width - 65, 18);
                }
            }
        }
        
        private void DrawGraphLine(Graphics g, Rectangle rect, Queue<float> data, Color color, float maxValue)
        {
            if (data.Count < 2) return;
            
            var values = data.ToArray();
            var points = new PointF[values.Length];
            
            for (int i = 0; i < values.Length; i++)
            {
                float x = rect.Width * i / (float)(HISTORY_LENGTH - 1);
                float y = rect.Height - (rect.Height * values[i] / maxValue);
                points[i] = new PointF(x, Math.Max(0, Math.Min(rect.Height, y)));
            }
            
            using (var pen = new Pen(color, 2))
            {
                g.DrawLines(pen, points);
            }
        }
        
        // ============================================================
        // Alerts
        // ============================================================
        
        private void CheckAlerts(float cpuTemp, float gpuTemp, float memory, float disk)
        {
            var timestamp = DateTime.Now.ToString("HH:mm:ss");
            
            if (cpuTemp > _config.TempCriticalC)
            {
                AddAlert($"[{timestamp}] CRITICAL: CPU Temp {cpuTemp:F1}°C");
            }
            else if (cpuTemp > _config.TempWarningC)
            {
                AddAlert($"[{timestamp}] WARNING: CPU Temp {cpuTemp:F1}°C");
            }
            
            if (gpuTemp > _config.TempCriticalC)
            {
                AddAlert($"[{timestamp}] CRITICAL: GPU Temp {gpuTemp:F1}°C");
            }
            else if (gpuTemp > _config.TempWarningC)
            {
                AddAlert($"[{timestamp}] WARNING: GPU Temp {gpuTemp:F1}°C");
            }
            
            if (memory > 95)
            {
                AddAlert($"[{timestamp}] CRITICAL: Memory at {memory:F0}%");
            }
            
            if (disk > 95)
            {
                AddAlert($"[{timestamp}] WARNING: Disk at {disk:F0}%");
            }
        }
        
        private void AddAlert(string alert)
        {
            // Avoid duplicate consecutive alerts
            if (_alerts.Count > 0 && _alerts[_alerts.Count - 1].Contains(alert.Substring(alert.IndexOf(']') + 1)))
                return;
            
            _alerts.Add(alert);
            _lstAlerts.Items.Add(alert);
            
            // Keep only last 100 alerts
            while (_alerts.Count > 100)
            {
                _alerts.RemoveAt(0);
                _lstAlerts.Items.RemoveAt(0);
            }
            
            // Scroll to bottom
            _lstAlerts.TopIndex = _lstAlerts.Items.Count - 1;
        }
        
        // ============================================================
        // Public Methods
        // ============================================================
        
        public void UpdateConfig(NOMADConfig config)
        {
            _config = config;
        }
        
        // ============================================================
        // Cleanup
        // ============================================================
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _pollTimer?.Dispose();
                _httpClient?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
