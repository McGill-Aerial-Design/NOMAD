using System;
using System.Drawing;
using System.Net.Http;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Jetson Health Monitor Tab for NOMAD Mission Planner Plugin.
    /// 
    /// Polls the Jetson /health API every 2 seconds and displays:
    /// - CPU Load (%)
    /// - GPU Load (%)
    /// - CPU Temperature (C)
    /// - GPU Temperature (C)
    /// - Status (OK/WARNING/CRITICAL)
    /// </summary>
    public class JetsonHealthTab : UserControl
    {
        private readonly HttpClient _httpClient;
        private readonly System.Threading.Timer _pollTimer;
        private string _jetsonBaseUrl = "http://127.0.0.1:8000";
        
        // UI Controls
        private Label _lblStatus;
        private ProgressBar _prgCpuLoad;
        private ProgressBar _prgGpuLoad;
        private ProgressBar _prgCpuTemp;
        private ProgressBar _prgGpuTemp;
        private Label _lblCpuLoad;
        private Label _lblGpuLoad;
        private Label _lblCpuTemp;
        private Label _lblGpuTemp;
        private Label _lblLastUpdate;
        private Button _btnRefresh;
        private TextBox _txtUrl;
        
        public JetsonHealthTab()
        {
            _httpClient = new HttpClient
            {
                Timeout = TimeSpan.FromSeconds(2)
            };
            
            InitializeUI();
            
            // Start polling every 2 seconds
            _pollTimer = new System.Threading.Timer(
                _ => PollJetsonHealth(),
                null,
                TimeSpan.FromSeconds(1), // Initial delay
                TimeSpan.FromSeconds(2)  // Period
            );
        }
        
        /// <summary>
        /// Set the Jetson base URL (e.g., Tailscale IP).
        /// </summary>
        public void SetJetsonUrl(string url)
        {
            if (!string.IsNullOrEmpty(url))
            {
                _jetsonBaseUrl = url.TrimEnd('/');
                _txtUrl.Text = _jetsonBaseUrl;
            }
        }
        
        private void InitializeUI()
        {
            this.BackColor = Color.FromArgb(45, 45, 48);
            this.Size = new Size(400, 450);
            this.AutoScroll = true;
            
            int yOffset = 15;
            
            // Title
            var lblTitle = new Label
            {
                Text = "Jetson Health Monitor",
                Location = new Point(15, yOffset),
                Size = new Size(370, 25),
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = Color.White
            };
            this.Controls.Add(lblTitle);
            yOffset += 35;
            
            // URL Input
            var lblUrl = new Label
            {
                Text = "Jetson URL:",
                Location = new Point(15, yOffset),
                AutoSize = true,
                ForeColor = Color.LightGray
            };
            this.Controls.Add(lblUrl);
            yOffset += 20;
            
            _txtUrl = new TextBox
            {
                Location = new Point(15, yOffset),
                Size = new Size(280, 25),
                Text = _jetsonBaseUrl,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            this.Controls.Add(_txtUrl);
            
            _btnRefresh = new Button
            {
                Text = "Update",
                Location = new Point(305, yOffset),
                Size = new Size(75, 25),
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
            };
            _btnRefresh.Click += (s, e) =>
            {
                _jetsonBaseUrl = _txtUrl.Text.TrimEnd('/');
                PollJetsonHealth();
            };
            this.Controls.Add(_btnRefresh);
            yOffset += 40;
            
            // Status
            _lblStatus = new Label
            {
                Text = "Status: Connecting...",
                Location = new Point(15, yOffset),
                Size = new Size(370, 25),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = Color.Yellow
            };
            this.Controls.Add(_lblStatus);
            yOffset += 35;
            
            // CPU Load
            AddMetric("CPU Load:", ref _prgCpuLoad, ref _lblCpuLoad, ref yOffset, Color.DodgerBlue);
            
            // GPU Load
            AddMetric("GPU Load:", ref _prgGpuLoad, ref _lblGpuLoad, ref yOffset, Color.LimeGreen);
            
            // CPU Temperature
            AddMetric("CPU Temp:", ref _prgCpuTemp, ref _lblCpuTemp, ref yOffset, Color.Orange, maxValue: 100);
            
            // GPU Temperature
            AddMetric("GPU Temp:", ref _prgGpuTemp, ref _lblGpuTemp, ref yOffset, Color.OrangeRed, maxValue: 100);
            
            // Last Update
            _lblLastUpdate = new Label
            {
                Text = "Last update: Never",
                Location = new Point(15, yOffset),
                Size = new Size(370, 20),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8)
            };
            this.Controls.Add(_lblLastUpdate);
        }
        
        private void AddMetric(string label, ref ProgressBar progressBar, ref Label valueLabel, ref int yOffset, Color color, int maxValue = 100)
        {
            var lbl = new Label
            {
                Text = label,
                Location = new Point(15, yOffset),
                AutoSize = true,
                ForeColor = Color.LightGray
            };
            this.Controls.Add(lbl);
            yOffset += 20;
            
            progressBar = new ProgressBar
            {
                Location = new Point(15, yOffset),
                Size = new Size(280, 20),
                Maximum = maxValue,
                ForeColor = color,
                Style = ProgressBarStyle.Continuous
            };
            this.Controls.Add(progressBar);
            
            valueLabel = new Label
            {
                Text = "-- %",
                Location = new Point(305, yOffset),
                Size = new Size(75, 20),
                ForeColor = Color.White,
                TextAlign = ContentAlignment.MiddleRight
            };
            this.Controls.Add(valueLabel);
            yOffset += 30;
        }
        
        private async void PollJetsonHealth()
        {
            try
            {
                var response = await _httpClient.GetAsync($"{_jetsonBaseUrl}/health");
                
                if (response.IsSuccessStatusCode)
                {
                    var json = await response.Content.ReadAsStringAsync();
                    var data = JObject.Parse(json);
                    
                    // Update UI on UI thread
                    this.BeginInvoke((Action)(() => UpdateHealthUI(data)));
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
        
        private void UpdateHealthUI(JObject data)
        {
            try
            {
                // Status
                var status = data["status"]?.ToString() ?? "unknown";
                var connected = data["connected"]?.Value<bool>() ?? false;
                
                _lblStatus.Text = $"Status: {status.ToUpper()}";
                _lblStatus.ForeColor = status == "ok" ? Color.LimeGreen : Color.Yellow;
                
                // CPU Load
                var cpuLoad = data["cpu_load"]?.Value<float>() ?? 0;
                _prgCpuLoad.Value = Math.Min((int)cpuLoad, 100);
                _lblCpuLoad.Text = $"{cpuLoad:F1}%";
                
                // GPU Load
                var gpuLoad = data["gpu_load"]?.Value<float>() ?? 0;
                _prgGpuLoad.Value = Math.Min((int)gpuLoad, 100);
                _lblGpuLoad.Text = $"{gpuLoad:F1}%";
                
                // CPU Temperature
                var cpuTemp = data["cpu_temp"]?.Value<float>() ?? 0;
                _prgCpuTemp.Value = Math.Min((int)cpuTemp, 100);
                _lblCpuTemp.Text = $"{cpuTemp:F1}C";
                _prgCpuTemp.ForeColor = cpuTemp > 85 ? Color.Red : cpuTemp > 75 ? Color.Orange : Color.LimeGreen;
                
                // GPU Temperature
                var gpuTemp = data["gpu_temp"]?.Value<float>() ?? 0;
                _prgGpuTemp.Value = Math.Min((int)gpuTemp, 100);
                _lblGpuTemp.Text = $"{gpuTemp:F1}C";
                _prgGpuTemp.ForeColor = gpuTemp > 85 ? Color.Red : gpuTemp > 75 ? Color.Orange : Color.LimeGreen;
                
                // Last update
                _lblLastUpdate.Text = $"Last update: {DateTime.Now:HH:mm:ss}";
            }
            catch (Exception ex)
            {
                _lblStatus.Text = $"Status: Parse Error";
                _lblStatus.ForeColor = Color.Red;
                System.Diagnostics.Debug.WriteLine($"Health UI update error: {ex.Message}");
            }
        }
        
        private void UpdateStatusError(string error)
        {
            _lblStatus.Text = $"Status: Error - {error}";
            _lblStatus.ForeColor = Color.Red;
            _lblLastUpdate.Text = $"Last update: {DateTime.Now:HH:mm:ss} (failed)";
        }
        
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
