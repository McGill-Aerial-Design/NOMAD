using System;
using System.Drawing;
using System.Net.Http;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Control panel for camera tilt servo and water shooter.
    /// Provides real-time servo angle control and shooter trigger functionality.
    /// </summary>
    public class ServoControlPanel : UserControl
    {
        private NOMADConfig _config;
        private HttpClient _httpClient;
        private TrackBar _tiltSlider;
        private Label _tiltLabel;
        private Label _tiltValueLabel;
        private Button _shooterButton;
        private Button _enableButton;
        private Button _disableButton;
        private Label _statusLabel;
        private Timer _updateTimer;
        private Timer _debounceTimer;
        private bool _isUpdating = false;
        private int _pendingTiltAngle = 90;
        private bool _servosEnabled = false;

        public ServoControlPanel()
        {
            _httpClient = new HttpClient { Timeout = TimeSpan.FromSeconds(5) };
            InitializeComponent();
        }

        public void Initialize(NOMADConfig config)
        {
            _config = config;
            
            // Start status polling
            _updateTimer = new Timer { Interval = 2000 };
            _updateTimer.Tick += async (s, e) => await RefreshStatusAsync();
            _updateTimer.Start();

            // Debounce timer for slider changes
            _debounceTimer = new Timer { Interval = 150 };
            _debounceTimer.Tick += async (s, e) =>
            {
                _debounceTimer.Stop();
                await SetTiltAngleAsync(_pendingTiltAngle);
            };

            // Initial status refresh
            _ = RefreshStatusAsync();
        }

        private void InitializeComponent()
        {
            BackColor = Color.FromArgb(30, 30, 35);
            Padding = new Padding(10);

            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 6,
                BackColor = Color.Transparent,
            };
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 30));  // Title
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 25));  // Tilt label
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 45));  // Slider
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 45));  // Shooter button
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 40));  // Enable/Disable buttons
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100)); // Status

            // Title
            var titleLabel = new Label
            {
                Text = "Servo Control",
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = Color.White,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleLeft,
            };
            mainLayout.Controls.Add(titleLabel, 0, 0);

            // Tilt label and value
            var tiltPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.LeftToRight,
                BackColor = Color.Transparent,
            };
            _tiltLabel = new Label
            {
                Text = "Camera Tilt:",
                ForeColor = Color.LightGray,
                Font = new Font("Segoe UI", 9),
                AutoSize = true,
                Margin = new Padding(0, 3, 5, 0),
            };
            _tiltValueLabel = new Label
            {
                Text = "90 deg",
                ForeColor = Color.Cyan,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                AutoSize = true,
                Margin = new Padding(0, 3, 0, 0),
            };
            tiltPanel.Controls.Add(_tiltLabel);
            tiltPanel.Controls.Add(_tiltValueLabel);
            mainLayout.Controls.Add(tiltPanel, 0, 1);

            // Tilt slider
            _tiltSlider = new TrackBar
            {
                Dock = DockStyle.Fill,
                Minimum = 0,
                Maximum = 180,
                Value = 90,
                TickFrequency = 30,
                SmallChange = 5,
                LargeChange = 15,
                BackColor = Color.FromArgb(40, 40, 45),
            };
            _tiltSlider.ValueChanged += OnTiltSliderChanged;
            mainLayout.Controls.Add(_tiltSlider, 0, 2);

            // Shooter button
            _shooterButton = new Button
            {
                Text = "SHOOT WATER",
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(200, 50, 50),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                FlatStyle = FlatStyle.Flat,
                Margin = new Padding(0, 5, 0, 5),
            };
            _shooterButton.FlatAppearance.BorderSize = 0;
            _shooterButton.Click += OnShooterClick;
            mainLayout.Controls.Add(_shooterButton, 0, 3);

            // Enable/Disable buttons panel
            var buttonPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.LeftToRight,
                BackColor = Color.Transparent,
                WrapContents = false,
            };

            _enableButton = new Button
            {
                Text = "Enable Servos",
                Width = 100,
                Height = 30,
                BackColor = Color.FromArgb(50, 150, 50),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Margin = new Padding(0, 0, 5, 0),
            };
            _enableButton.FlatAppearance.BorderSize = 0;
            _enableButton.Click += OnEnableClick;

            _disableButton = new Button
            {
                Text = "Disable Servos",
                Width = 100,
                Height = 30,
                BackColor = Color.FromArgb(100, 100, 100),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
            };
            _disableButton.FlatAppearance.BorderSize = 0;
            _disableButton.Click += OnDisableClick;

            buttonPanel.Controls.Add(_enableButton);
            buttonPanel.Controls.Add(_disableButton);
            mainLayout.Controls.Add(buttonPanel, 0, 4);

            // Status label
            _statusLabel = new Label
            {
                Text = "Status: Checking...",
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.TopLeft,
            };
            mainLayout.Controls.Add(_statusLabel, 0, 5);

            Controls.Add(mainLayout);
        }

        private void OnTiltSliderChanged(object sender, EventArgs e)
        {
            _pendingTiltAngle = _tiltSlider.Value;
            _tiltValueLabel.Text = $"{_pendingTiltAngle} deg";

            // Restart debounce timer
            _debounceTimer.Stop();
            _debounceTimer.Start();
        }

        private async Task SetTiltAngleAsync(int angle)
        {
            if (_config == null || _isUpdating) return;

            _isUpdating = true;
            try
            {
                var response = await _httpClient.PostAsync($"{_config.EffectiveBaseUrl}/api/servo/camera/tilt?angle={angle}", null);
                
                if (response.IsSuccessStatusCode)
                {
                    _statusLabel.Text = $"Tilt set to {angle} deg";
                    _statusLabel.ForeColor = Color.LightGreen;
                }
                else
                {
                    _statusLabel.Text = $"Failed to set tilt: {response.StatusCode}";
                    _statusLabel.ForeColor = Color.Orange;
                }
            }
            catch (Exception ex)
            {
                _statusLabel.Text = $"Error: {ex.Message}";
                _statusLabel.ForeColor = Color.Red;
            }
            finally
            {
                _isUpdating = false;
            }
        }

        private async void OnShooterClick(object sender, EventArgs e)
        {
            if (_config == null) return;

            _shooterButton.Enabled = false;
            _shooterButton.Text = "SHOOTING...";
            _shooterButton.BackColor = Color.FromArgb(150, 100, 50);

            try
            {
                var response = await _httpClient.PostAsync($"{_config.EffectiveBaseUrl}/api/servo/shooter/trigger?duration_ms=300", null);

                if (response.IsSuccessStatusCode)
                {
                    _statusLabel.Text = "Water shooter triggered!";
                    _statusLabel.ForeColor = Color.Cyan;
                }
                else
                {
                    _statusLabel.Text = $"Shooter failed: {response.StatusCode}";
                    _statusLabel.ForeColor = Color.Orange;
                }
            }
            catch (Exception ex)
            {
                _statusLabel.Text = $"Shooter error: {ex.Message}";
                _statusLabel.ForeColor = Color.Red;
            }
            finally
            {
                await Task.Delay(500);
                _shooterButton.Enabled = true;
                _shooterButton.Text = "SHOOT WATER";
                _shooterButton.BackColor = Color.FromArgb(200, 50, 50);
            }
        }

        private async void OnEnableClick(object sender, EventArgs e)
        {
            if (_config == null) return;

            try
            {
                var response = await _httpClient.PostAsync($"{_config.EffectiveBaseUrl}/api/servo/enable", null);

                if (response.IsSuccessStatusCode)
                {
                    _servosEnabled = true;
                    UpdateButtonStates();
                    _statusLabel.Text = "Servos enabled";
                    _statusLabel.ForeColor = Color.LightGreen;
                }
            }
            catch (Exception ex)
            {
                _statusLabel.Text = $"Enable error: {ex.Message}";
                _statusLabel.ForeColor = Color.Red;
            }
        }

        private async void OnDisableClick(object sender, EventArgs e)
        {
            if (_config == null) return;

            try
            {
                var response = await _httpClient.PostAsync($"{_config.EffectiveBaseUrl}/api/servo/disable", null);

                if (response.IsSuccessStatusCode)
                {
                    _servosEnabled = false;
                    UpdateButtonStates();
                    _statusLabel.Text = "Servos disabled (safe)";
                    _statusLabel.ForeColor = Color.Orange;
                }
            }
            catch (Exception ex)
            {
                _statusLabel.Text = $"Disable error: {ex.Message}";
                _statusLabel.ForeColor = Color.Red;
            }
        }

        private void UpdateButtonStates()
        {
            if (_servosEnabled)
            {
                _enableButton.BackColor = Color.FromArgb(100, 100, 100);
                _disableButton.BackColor = Color.FromArgb(200, 50, 50);
            }
            else
            {
                _enableButton.BackColor = Color.FromArgb(50, 150, 50);
                _disableButton.BackColor = Color.FromArgb(100, 100, 100);
            }

            _tiltSlider.Enabled = _servosEnabled;
            _shooterButton.Enabled = _servosEnabled;
        }

        private async Task RefreshStatusAsync()
        {
            if (_config == null) return;

            try
            {
                var response = await _httpClient.GetAsync($"{_config.EffectiveBaseUrl}/api/servo/status");

                if (response.IsSuccessStatusCode)
                {
                    var content = await response.Content.ReadAsStringAsync();
                    var json = JObject.Parse(content);

                    bool available = json["available"]?.Value<bool>() ?? false;

                    if (available)
                    {
                        var cameraTilt = json["servos"]?["camera_tilt"];
                        if (cameraTilt != null)
                        {
                            var angle = cameraTilt["angle"]?.Value<float>() ?? 90;
                            var enabled = cameraTilt["enabled"]?.Value<bool>() ?? false;

                            if (!_isUpdating && !_debounceTimer.Enabled)
                            {
                                _tiltSlider.Value = (int)Math.Round(angle);
                                _tiltValueLabel.Text = $"{(int)angle} deg";
                            }

                            _servosEnabled = enabled;
                            UpdateButtonStates();
                        }

                        _statusLabel.Text = "Servos connected";
                        _statusLabel.ForeColor = Color.LightGreen;
                    }
                    else
                    {
                        var error = json["error"]?.Value<string>() ?? "Not initialized";
                        _statusLabel.Text = $"Unavailable: {error}";
                        _statusLabel.ForeColor = Color.Orange;
                        _tiltSlider.Enabled = false;
                        _shooterButton.Enabled = false;
                    }
                }
                else
                {
                    _statusLabel.Text = "No response from Jetson";
                    _statusLabel.ForeColor = Color.Red;
                }
            }
            catch (HttpRequestException)
            {
                _statusLabel.Text = "Jetson disconnected";
                _statusLabel.ForeColor = Color.Red;
            }
            catch (Exception ex)
            {
                _statusLabel.Text = $"Error: {ex.Message}";
                _statusLabel.ForeColor = Color.Red;
            }
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _updateTimer?.Stop();
                _updateTimer?.Dispose();
                _debounceTimer?.Stop();
                _debounceTimer?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
