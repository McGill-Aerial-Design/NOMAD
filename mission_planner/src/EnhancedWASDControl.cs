// ============================================================
// Enhanced WASD Nudge Control for NOMAD Mission Planner Plugin
// ============================================================
// Provides keyboard-based drone control with visual feedback.
// 
// Control Mapping:
// - W/S: Forward/Backward movement
// - A/D: Left/Right strafe movement
// - Arrow Left/Right: Yaw rotation
// - Arrow Up/Down: Altitude change
//
// All movements use SET_POSITION_TARGET_LOCAL_NED MAVLink messages
// for smooth velocity-based control suitable for indoor/GPS-denied flight.
// ============================================================

using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Enhanced WASD control with visual keyboard feedback and MAVLink integration.
    /// </summary>
    public class EnhancedWASDControl : UserControl
    {
        // ============================================================
        // Constants
        // ============================================================
        
        private static readonly Color BG_COLOR = Color.FromArgb(30, 30, 33);
        private static readonly Color CARD_BG = Color.FromArgb(40, 40, 45);
        private static readonly Color KEY_INACTIVE = Color.FromArgb(60, 60, 65);
        private static readonly Color KEY_ACTIVE = Color.FromArgb(0, 122, 204);
        private static readonly Color TEXT_PRIMARY = Color.White;
        private static readonly Color TEXT_SECONDARY = Color.FromArgb(150, 150, 150);
        private static readonly Color SUCCESS_COLOR = Color.FromArgb(76, 175, 80);
        private static readonly Color WARNING_COLOR = Color.FromArgb(255, 152, 0);
        private static readonly Color ERROR_COLOR = Color.FromArgb(244, 67, 54);
        private static readonly Color ACCENT_COLOR = Color.FromArgb(0, 122, 204);
        private static readonly Color FOCUS_BORDER_COLOR = Color.FromArgb(220, 50, 50);  // Red border for focus
        
        // ============================================================
        // Fields
        // ============================================================
        
        private bool _enabled = false;
        private bool _mouseInside = false;  // Track if mouse is inside control area
        private float _moveSpeed = 0.5f;    // m/s for WASD
        private float _altSpeed = 0.3f;     // m/s for altitude
        private float _yawRate = 15.0f;     // deg/s for yaw
        
        // Current velocity/yaw command state
        private float _vx = 0f;  // Forward velocity
        private float _vy = 0f;  // Right velocity
        private float _vz = 0f;  // Down velocity (positive = down)
        private float _yawRateCmd = 0f;  // Yaw rate
        
        // Key state tracking
        private bool _keyW, _keyS, _keyA, _keyD;
        private bool _keyUp, _keyDown, _keyLeft, _keyRight;
        
        // UI Controls
        private Panel _keyboardPanel;
        private Panel _keyboardBorderPanel;  // Outer panel with border
        private Panel _statusPanel;
        private CheckBox _chkEnable;
        private Label _lblStatus;
        private Label _lblVelocity;
        private NumericUpDown _numMoveSpeed;
        private NumericUpDown _numAltSpeed;
        private NumericUpDown _numYawRate;
        
        // Keyboard visualization
        private Panel _keyW_Panel, _keyA_Panel, _keyS_Panel, _keyD_Panel;
        private Panel _keyUp_Panel, _keyDown_Panel, _keyLeft_Panel, _keyRight_Panel;
        
        // Command timer
        private System.Threading.Timer _commandTimer;
        
        // ============================================================
        // Properties
        // ============================================================
        
        /// <summary>
        /// Enable or disable WASD control.
        /// </summary>
        public bool ControlEnabled
        {
            get => _enabled;
            set
            {
                if (_enabled == value) return;
                _enabled = value;
                
                if (_enabled)
                {
                    StartCommandTimer();
                    UpdateStatus("ENABLED - Hover over keyboard area", SUCCESS_COLOR);
                }
                else
                {
                    StopCommandTimer();
                    ResetAllKeys();
                    UpdateStatus("Disabled", TEXT_SECONDARY);
                }
                
                if (_chkEnable != null) _chkEnable.Checked = value;
                UpdateKeyVisuals();
                UpdateBorderState();
            }
        }
        
        // ============================================================
        // Constructor
        // ============================================================
        
        public EnhancedWASDControl(float moveSpeed = 0.5f, float altSpeed = 0.3f, float yawRate = 15.0f)
        {
            _moveSpeed = moveSpeed;
            _altSpeed = altSpeed;
            _yawRate = yawRate;
            
            InitializeUI();
            
            // Set control properties for keyboard capture
            this.SetStyle(ControlStyles.Selectable, true);
            this.TabStop = true;
        }
        
        // ============================================================
        // UI Initialization
        // ============================================================
        
        private void InitializeUI()
        {
            this.BackColor = BG_COLOR;
            this.Dock = DockStyle.Fill;
            this.Padding = new Padding(10);
            this.MinimumSize = new Size(400, 400);
            
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 4,
                BackColor = Color.Transparent,
            };
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 50));   // Enable/Status
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));   // Keyboard visual
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 80));   // Settings
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 100));  // Payload controls
            
            // Top row: Enable checkbox and status
            var topPanel = CreateTopPanel();
            mainLayout.Controls.Add(topPanel, 0, 0);
            
            // Middle row: Keyboard visualization
            _keyboardPanel = CreateKeyboardVisual();
            mainLayout.Controls.Add(_keyboardPanel, 0, 1);
            
            // Settings row: Speed settings
            var settingsPanel = CreateSettingsPanel();
            mainLayout.Controls.Add(settingsPanel, 0, 2);
            
            // Bottom row: Payload controls
            var payloadPanel = CreatePayloadControlsPanel();
            mainLayout.Controls.Add(payloadPanel, 0, 3);
            
            this.Controls.Add(mainLayout);
        }
        
        private Panel CreateTopPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                Padding = new Padding(10, 5, 10, 5),
            };
            
            _chkEnable = new CheckBox
            {
                Text = "Enable Nudge Control",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(10, 12),
                AutoSize = true,
                Cursor = Cursors.Hand,
            };
            _chkEnable.CheckedChanged += (s, e) => ControlEnabled = _chkEnable.Checked;
            panel.Controls.Add(_chkEnable);
            
            _lblStatus = new Label
            {
                Text = "Disabled",
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(220, 14),
                AutoSize = true,
            };
            panel.Controls.Add(_lblStatus);
            
            return panel;
        }
        
        private Panel CreateKeyboardVisual()
        {
            // Outer border panel - shows focus state
            _keyboardBorderPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                Padding = new Padding(3),  // Border thickness
            };
            
            // Inner panel with actual keyboard layout
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                Padding = new Padding(12),
            };
            
            // Track mouse enter/leave for the entire control area
            _keyboardBorderPanel.MouseEnter += KeyboardArea_MouseEnter;
            _keyboardBorderPanel.MouseLeave += KeyboardArea_MouseLeave;
            panel.MouseEnter += KeyboardArea_MouseEnter;
            panel.MouseLeave += KeyboardArea_MouseLeave;
            
            // Two keyboard sections: WASD (left) and Arrows (right)
            var layout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                BackColor = Color.Transparent,
            };
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            layout.MouseEnter += KeyboardArea_MouseEnter;
            
            // WASD Section
            var wasdSection = CreateWASDSection();
            layout.Controls.Add(wasdSection, 0, 0);
            
            // Arrow Section
            var arrowSection = CreateArrowSection();
            layout.Controls.Add(arrowSection, 1, 0);
            
            panel.Controls.Add(layout);
            
            // Velocity display
            _lblVelocity = new Label
            {
                Text = "Velocity: 0.0, 0.0, 0.0 | Yaw: 0.0",
                Font = new Font("Consolas", 9),
                ForeColor = TEXT_SECONDARY,
                Dock = DockStyle.Bottom,
                Height = 25,
                TextAlign = ContentAlignment.MiddleCenter,
            };
            panel.Controls.Add(_lblVelocity);
            
            // Focus hint label
            var focusHint = new Label
            {
                Text = "Hover here and press keys to control",
                Font = new Font("Segoe UI", 8),
                ForeColor = TEXT_SECONDARY,
                Dock = DockStyle.Top,
                Height = 20,
                TextAlign = ContentAlignment.MiddleCenter,
            };
            panel.Controls.Add(focusHint);
            
            _keyboardBorderPanel.Controls.Add(panel);
            
            return _keyboardBorderPanel;
        }
        
        private void KeyboardArea_MouseEnter(object sender, EventArgs e)
        {
            _mouseInside = true;
            UpdateBorderState();
        }
        
        private void KeyboardArea_MouseLeave(object sender, EventArgs e)
        {
            // Check if mouse actually left the entire control area
            // (not just moved to a child control)
            Point mousePos = _keyboardBorderPanel.PointToClient(Control.MousePosition);
            if (!_keyboardBorderPanel.ClientRectangle.Contains(mousePos))
            {
                _mouseInside = false;
                ResetAllKeys();  // Release all keys when mouse leaves
                UpdateKeyVisuals();
                UpdateBorderState();
            }
        }
        
        private void UpdateBorderState()
        {
            if (_keyboardBorderPanel == null) return;
            
            if (_mouseInside && _enabled)
            {
                _keyboardBorderPanel.BackColor = FOCUS_BORDER_COLOR;  // Red border = active
            }
            else
            {
                _keyboardBorderPanel.BackColor = CARD_BG;  // No border = inactive
            }
        }
        
        private Panel CreateWASDSection()
        {
            var section = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Transparent,
            };
            
            var title = new Label
            {
                Text = "MOVEMENT (WASD)",
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(10, 5),
                AutoSize = true,
            };
            section.Controls.Add(title);
            
            int keySize = 48;  // Square keys
            int spacing = 6;   // Consistent spacing
            int startX = 40;
            int startY = 30;
            
            // W key (top center)
            _keyW_Panel = CreateKeyPanel("W", "FWD", startX + keySize + spacing, startY);
            section.Controls.Add(_keyW_Panel);
            
            // A key (left)
            _keyA_Panel = CreateKeyPanel("A", "LEFT", startX, startY + keySize + spacing);
            section.Controls.Add(_keyA_Panel);
            
            // S key (center)
            _keyS_Panel = CreateKeyPanel("S", "BACK", startX + keySize + spacing, startY + keySize + spacing);
            section.Controls.Add(_keyS_Panel);
            
            // D key (right)
            _keyD_Panel = CreateKeyPanel("D", "RIGHT", startX + (keySize + spacing) * 2, startY + keySize + spacing);
            section.Controls.Add(_keyD_Panel);
            
            return section;
        }
        
        private Panel CreateArrowSection()
        {
            var section = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Transparent,
            };
            
            var title = new Label
            {
                Text = "YAW/ALT (ARROWS)",
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(10, 5),
                AutoSize = true,
            };
            section.Controls.Add(title);
            
            int keySize = 48;  // Square keys
            int spacing = 6;   // Consistent spacing
            int startX = 40;
            int startY = 30;
            
            // Up arrow (altitude up) - centered above down
            _keyUp_Panel = CreateKeyPanel("[^]", "ALT+", startX + keySize + spacing, startY);
            section.Controls.Add(_keyUp_Panel);
            
            // Left arrow (yaw left)
            _keyLeft_Panel = CreateKeyPanel("[<]", "YAW-", startX, startY + keySize + spacing);
            section.Controls.Add(_keyLeft_Panel);
            
            // Down arrow (altitude down)
            _keyDown_Panel = CreateKeyPanel("[v]", "ALT-", startX + keySize + spacing, startY + keySize + spacing);
            section.Controls.Add(_keyDown_Panel);
            
            // Right arrow (yaw right)
            _keyRight_Panel = CreateKeyPanel("[>]", "YAW+", startX + (keySize + spacing) * 2, startY + keySize + spacing);
            section.Controls.Add(_keyRight_Panel);
            
            return section;
        }
        
        private Panel CreateKeyPanel(string keyText, string actionText, int x, int y)
        {
            int keySize = 48;  // Square key size
            
            var panel = new Panel
            {
                Location = new Point(x, y),
                Size = new Size(keySize, keySize),  // Square buttons
                BackColor = KEY_INACTIVE,
            };
            
            // Round corners
            panel.Paint += (s, e) =>
            {
                e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
                using (var path = RoundedRect(new Rectangle(0, 0, panel.Width - 1, panel.Height - 1), 5))
                using (var brush = new SolidBrush(panel.BackColor))
                {
                    e.Graphics.FillPath(brush, path);
                }
            };
            
            var keyLabel = new Label
            {
                Text = keyText,
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(0, 4),
                Size = new Size(keySize, 20),
                TextAlign = ContentAlignment.MiddleCenter,
                BackColor = Color.Transparent,
            };
            panel.Controls.Add(keyLabel);
            
            var actionLabel = new Label
            {
                Text = actionText,
                Font = new Font("Segoe UI", 7),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(0, 26),
                Size = new Size(keySize, 16),
                TextAlign = ContentAlignment.MiddleCenter,
                BackColor = Color.Transparent,
            };
            panel.Controls.Add(actionLabel);
            
            return panel;
        }
        
        private Panel CreateSettingsPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                Padding = new Padding(10, 5, 10, 5),
            };
            
            int x = 10;
            
            // Move speed
            var lblMove = new Label
            {
                Text = "Move (m/s):",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(x, 12),
                AutoSize = true,
            };
            panel.Controls.Add(lblMove);
            
            _numMoveSpeed = new NumericUpDown
            {
                Location = new Point(x + 75, 8),
                Size = new Size(60, 25),
                DecimalPlaces = 1,
                Minimum = 0.1M,
                Maximum = 2.0M,
                Increment = 0.1M,
                Value = (decimal)_moveSpeed,
                BackColor = Color.FromArgb(50, 50, 55),
                ForeColor = TEXT_PRIMARY,
            };
            _numMoveSpeed.ValueChanged += (s, e) => _moveSpeed = (float)_numMoveSpeed.Value;
            panel.Controls.Add(_numMoveSpeed);
            
            x += 150;
            
            // Alt speed
            var lblAlt = new Label
            {
                Text = "Alt (m/s):",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(x, 12),
                AutoSize = true,
            };
            panel.Controls.Add(lblAlt);
            
            _numAltSpeed = new NumericUpDown
            {
                Location = new Point(x + 60, 8),
                Size = new Size(60, 25),
                DecimalPlaces = 1,
                Minimum = 0.1M,
                Maximum = 1.0M,
                Increment = 0.1M,
                Value = (decimal)_altSpeed,
                BackColor = Color.FromArgb(50, 50, 55),
                ForeColor = TEXT_PRIMARY,
            };
            _numAltSpeed.ValueChanged += (s, e) => _altSpeed = (float)_numAltSpeed.Value;
            panel.Controls.Add(_numAltSpeed);
            
            x += 140;
            
            // Yaw rate
            var lblYaw = new Label
            {
                Text = "Yaw (deg/s):",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(x, 12),
                AutoSize = true,
            };
            panel.Controls.Add(lblYaw);
            
            _numYawRate = new NumericUpDown
            {
                Location = new Point(x + 75, 8),
                Size = new Size(60, 25),
                DecimalPlaces = 0,
                Minimum = 5,
                Maximum = 45,
                Increment = 5,
                Value = (decimal)_yawRate,
                BackColor = Color.FromArgb(50, 50, 55),
                ForeColor = TEXT_PRIMARY,
            };
            _numYawRate.ValueChanged += (s, e) => _yawRate = (float)_numYawRate.Value;
            panel.Controls.Add(_numYawRate);
            
            // Warning label
            var lblWarning = new Label
            {
                Text = "CAUTION: Use in Guided mode only. Keep RC ready for override.",
                Font = new Font("Segoe UI", 8),
                ForeColor = WARNING_COLOR,
                Location = new Point(10, 45),
                AutoSize = true,
            };
            panel.Controls.Add(lblWarning);
            
            return panel;
        }
        
        // ============================================================
        // Payload Controls Panel
        // ============================================================
        
        // TODO: Configure these GPIO pins once hardware is finalized
        private const int GPIO_PAYLOAD1_PIN = 0;  // TODO: Cube GPIO pin for payload 1 linear actuator
        private const int GPIO_PAYLOAD2_PIN = 0;  // TODO: Cube GPIO pin for payload 2 linear actuator
        private const int JETSON_WATER_GPIO = 0;  // TODO: Jetson GPIO pin for water pump trigger
        private const int JETSON_SERVO_PWM = 0;   // TODO: Jetson PWM channel for nozzle servo
        
        private TrackBar _nozzleServoSlider;
        private Label _lblNozzleValue;
        
        private Panel CreatePayloadControlsPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                Padding = new Padding(10, 5, 10, 5),
            };
            
            // Title label
            var titleLabel = new Label
            {
                Text = "PAYLOAD CONTROLS",
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(10, 5),
                AutoSize = true,
            };
            panel.Controls.Add(titleLabel);
            
            int x = 10;
            int y = 28;
            
            // Drop Payload 1 button
            var btnPayload1 = new Button
            {
                Text = "Drop Payload 1",
                Location = new Point(x, y),
                Size = new Size(110, 28),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = TEXT_PRIMARY,
                Font = new Font("Segoe UI", 9),
                Cursor = Cursors.Hand,
            };
            btnPayload1.FlatAppearance.BorderColor = Color.FromArgb(80, 80, 85);
            btnPayload1.Click += (s, e) => DropPayload(1);
            panel.Controls.Add(btnPayload1);
            x += 115;
            
            // Drop Payload 2 button
            var btnPayload2 = new Button
            {
                Text = "Drop Payload 2",
                Location = new Point(x, y),
                Size = new Size(110, 28),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = TEXT_PRIMARY,
                Font = new Font("Segoe UI", 9),
                Cursor = Cursors.Hand,
            };
            btnPayload2.FlatAppearance.BorderColor = Color.FromArgb(80, 80, 85);
            btnPayload2.Click += (s, e) => DropPayload(2);
            panel.Controls.Add(btnPayload2);
            x += 115;
            
            // Shoot Water button
            var btnWater = new Button
            {
                Text = "Shoot Water",
                Location = new Point(x, y),
                Size = new Size(100, 28),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(30, 100, 180),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Cursor = Cursors.Hand,
            };
            btnWater.FlatAppearance.BorderColor = Color.FromArgb(40, 120, 200);
            btnWater.Click += (s, e) => ShootWater();
            panel.Controls.Add(btnWater);
            
            // Second row: Nozzle servo
            y += 35;
            x = 10;
            
            // Nozzle servo label
            var lblNozzle = new Label
            {
                Text = "Nozzle Angle:",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(x, y + 5),
                AutoSize = true,
            };
            panel.Controls.Add(lblNozzle);
            x += 85;
            
            // Nozzle servo slider
            _nozzleServoSlider = new TrackBar
            {
                Location = new Point(x, y),
                Size = new Size(180, 28),
                Minimum = 0,
                Maximum = 180,
                Value = 90,
                TickFrequency = 30,
                BackColor = CARD_BG,
            };
            _nozzleServoSlider.ValueChanged += (s, e) => UpdateNozzleServo();
            panel.Controls.Add(_nozzleServoSlider);
            x += 185;
            
            // Servo value label
            _lblNozzleValue = new Label
            {
                Text = "90 deg",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(x, y + 5),
                AutoSize = true,
            };
            panel.Controls.Add(_lblNozzleValue);
            
            return panel;
        }
        
        /// <summary>
        /// Drop payload using Cube GPIO to actuate linear actuator.
        /// </summary>
        /// <param name="payloadNumber">1 or 2</param>
        private void DropPayload(int payloadNumber)
        {
            // TODO: Implement actual GPIO control through MAVLink DO_SET_RELAY or DO_SET_SERVO
            // The linear actuators are connected to Cube GPIO pins
            try
            {
                if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
                {
                    UpdateStatus("Not connected", ERROR_COLOR);
                    return;
                }
                
                // MAV_CMD_DO_SET_RELAY to toggle GPIO
                // TODO: Map payload number to correct relay/pin number
                int relayNumber = payloadNumber == 1 ? GPIO_PAYLOAD1_PIN : GPIO_PAYLOAD2_PIN;
                
                // Send MAV_CMD_DO_SET_RELAY (181)
                MainV2.comPort.doCommand(
                    MainV2.comPort.MAV.sysid,
                    MainV2.comPort.MAV.compid,
                    MAVLink.MAV_CMD.DO_SET_RELAY,
                    relayNumber,  // Relay number
                    1,            // Setting (1 = on/toggle)
                    0, 0, 0, 0, 0
                );
                
                UpdateStatus($"Dropped Payload {payloadNumber}", SUCCESS_COLOR);
            }
            catch (Exception ex)
            {
                UpdateStatus($"Payload error: {ex.Message}", ERROR_COLOR);
            }
        }
        
        /// <summary>
        /// Trigger water pump via Jetson GPIO.
        /// </summary>
        private async void ShootWater()
        {
            // TODO: Implement API call to Jetson to trigger water pump GPIO
            try
            {
                using (var client = new System.Net.Http.HttpClient { Timeout = TimeSpan.FromSeconds(5) })
                {
                    // TODO: Update with correct Jetson IP from config
                    // TODO: Create API endpoint on Jetson edge_core for water pump control
                    var response = await client.PostAsync(
                        $"http://100.64.0.1:5000/api/gpio/water_pump/trigger",
                        new System.Net.Http.StringContent(
                            "{\"duration_ms\": 500}",
                            System.Text.Encoding.UTF8,
                            "application/json"
                        )
                    );
                    
                    if (response.IsSuccessStatusCode)
                    {
                        UpdateStatus("Water pump triggered", SUCCESS_COLOR);
                    }
                    else
                    {
                        UpdateStatus("Water pump failed", ERROR_COLOR);
                    }
                }
            }
            catch (Exception ex)
            {
                UpdateStatus($"Water pump error: {ex.Message}", ERROR_COLOR);
            }
        }
        
        /// <summary>
        /// Update nozzle servo position via Jetson PWM.
        /// </summary>
        private async void UpdateNozzleServo()
        {
            if (_nozzleServoSlider == null || _lblNozzleValue == null) return;
            
            int angle = _nozzleServoSlider.Value;
            _lblNozzleValue.Text = $"{angle} deg";
            
            // TODO: Implement API call to Jetson to set servo position
            try
            {
                using (var client = new System.Net.Http.HttpClient { Timeout = TimeSpan.FromSeconds(2) })
                {
                    // TODO: Update with correct Jetson IP from config
                    // TODO: Create API endpoint on Jetson edge_core for servo control
                    await client.PostAsync(
                        $"http://100.64.0.1:5000/api/gpio/nozzle_servo/set",
                        new System.Net.Http.StringContent(
                            $"{{\"angle\": {angle}}}",
                            System.Text.Encoding.UTF8,
                            "application/json"
                        )
                    );
                }
            }
            catch
            {
                // Silent fail for servo updates - they're frequent
            }
        }
        
        // ============================================================
        // Keyboard Event Handling - Event-based with platform-specific backup
        // ============================================================
        
        // Windows API for checking key state directly (only works on Windows)
        [System.Runtime.InteropServices.DllImport("user32.dll")]
        private static extern short GetAsyncKeyState(int vKey);
        
        // Platform detection
        private static readonly bool _isWindows = Environment.OSVersion.Platform == PlatformID.Win32NT;
        
        // Virtual key codes
        private const int VK_W = 0x57;
        private const int VK_A = 0x41;
        private const int VK_S = 0x53;
        private const int VK_D = 0x44;
        private const int VK_UP = 0x26;
        private const int VK_DOWN = 0x28;
        private const int VK_LEFT = 0x25;
        private const int VK_RIGHT = 0x27;
        
        /// <summary>
        /// Check if a key is currently pressed using Windows API.
        /// Returns false on Linux (rely on event-based key tracking instead).
        /// </summary>
        private bool IsKeyPressed(int vKey)
        {
            if (!_isWindows) return true;  // On Linux, trust the event-based state
            
            try
            {
                return (GetAsyncKeyState(vKey) & 0x8000) != 0;
            }
            catch
            {
                return true;  // If API fails, trust event-based state
            }
        }
        
        /// <summary>
        /// Safety check using Windows API - ensures keys are actually released.
        /// Called periodically to catch any missed key releases.
        /// Also checks if mouse has left the control area.
        /// On Linux, this only handles mouse-leave checks since Win API is unavailable.
        /// </summary>
        private void CheckKeyReleases()
        {
            if (!_enabled) return;
            
            // If mouse is not inside control area, release all keys
            if (!_mouseInside)
            {
                bool hadKeysPressed = _keyW || _keyS || _keyA || _keyD || _keyUp || _keyDown || _keyLeft || _keyRight;
                if (hadKeysPressed)
                {
                    ResetAllKeys();
                    UpdateKeyVisuals();
                }
                return;
            }
            
            // On non-Windows platforms, skip Win32 API checks (rely on event-based tracking)
            if (!_isWindows) return;
            
            // Check if keys that we think are pressed are actually still pressed (Windows only)
            bool stateChanged = false;
            
            if (_keyW && !IsKeyPressed(VK_W)) { _keyW = false; stateChanged = true; }
            if (_keyA && !IsKeyPressed(VK_A)) { _keyA = false; stateChanged = true; }
            if (_keyS && !IsKeyPressed(VK_S)) { _keyS = false; stateChanged = true; }
            if (_keyD && !IsKeyPressed(VK_D)) { _keyD = false; stateChanged = true; }
            if (_keyUp && !IsKeyPressed(VK_UP)) { _keyUp = false; stateChanged = true; }
            if (_keyDown && !IsKeyPressed(VK_DOWN)) { _keyDown = false; stateChanged = true; }
            if (_keyLeft && !IsKeyPressed(VK_LEFT)) { _keyLeft = false; stateChanged = true; }
            if (_keyRight && !IsKeyPressed(VK_RIGHT)) { _keyRight = false; stateChanged = true; }
            
            if (stateChanged)
            {
                UpdateVelocitiesFromKeys();
                UpdateKeyVisuals();
            }
        }
        
        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            // Only process keys if enabled AND mouse is inside the control area
            if (!_enabled || !_mouseInside) return base.ProcessCmdKey(ref msg, keyData);
            
            bool handled = true;
            
            // Set key state on key down
            switch (keyData)
            {
                case Keys.W: _keyW = true; break;
                case Keys.S: _keyS = true; break;
                case Keys.A: _keyA = true; break;
                case Keys.D: _keyD = true; break;
                case Keys.Up: _keyUp = true; break;
                case Keys.Down: _keyDown = true; break;
                case Keys.Left: _keyLeft = true; break;
                case Keys.Right: _keyRight = true; break;
                default: handled = false; break;
            }
            
            if (handled)
            {
                UpdateVelocitiesFromKeys();
                UpdateKeyVisuals();
                return true;
            }
            
            return base.ProcessCmdKey(ref msg, keyData);
        }
        
        protected override void OnKeyUp(KeyEventArgs e)
        {
            if (!_enabled)
            {
                base.OnKeyUp(e);
                return;
            }
            
            // Reset the specific key that was released
            switch (e.KeyCode)
            {
                case Keys.W: _keyW = false; break;
                case Keys.S: _keyS = false; break;
                case Keys.A: _keyA = false; break;
                case Keys.D: _keyD = false; break;
                case Keys.Up: _keyUp = false; break;
                case Keys.Down: _keyDown = false; break;
                case Keys.Left: _keyLeft = false; break;
                case Keys.Right: _keyRight = false; break;
            }
            
            UpdateVelocitiesFromKeys();
            UpdateKeyVisuals();
            
            base.OnKeyUp(e);
        }
        
        protected override void OnLostFocus(EventArgs e)
        {
            // SAFETY: When control loses focus, immediately release all keys
            // This prevents the drone from continuing to move unexpectedly
            ResetAllKeys();
            UpdateStatus("Lost focus - Keys released", WARNING_COLOR);
            base.OnLostFocus(e);
        }
        
        protected override void OnLeave(EventArgs e)
        {
            // SAFETY: When leaving control, release all keys
            ResetAllKeys();
            base.OnLeave(e);
        }
        
        private void UpdateVelocitiesFromKeys()
        {
            // Forward/Back (X velocity in NED frame)
            if (_keyW && !_keyS)
                _vx = _moveSpeed;
            else if (_keyS && !_keyW)
                _vx = -_moveSpeed;
            else
                _vx = 0;
            
            // Left/Right strafe (Y velocity in NED frame)
            if (_keyD && !_keyA)
                _vy = _moveSpeed;
            else if (_keyA && !_keyD)
                _vy = -_moveSpeed;
            else
                _vy = 0;
            
            // Altitude (Z velocity in NED frame - positive is DOWN)
            if (_keyUp && !_keyDown)
                _vz = -_altSpeed;  // Up = negative Z
            else if (_keyDown && !_keyUp)
                _vz = _altSpeed;   // Down = positive Z
            else
                _vz = 0;
            
            // Yaw rate
            if (_keyRight && !_keyLeft)
                _yawRateCmd = _yawRate * (float)(Math.PI / 180.0);  // Convert to rad/s
            else if (_keyLeft && !_keyRight)
                _yawRateCmd = -_yawRate * (float)(Math.PI / 180.0);
            else
                _yawRateCmd = 0;
        }
        
        private void UpdateKeyVisuals()
        {
            if (InvokeRequired)
            {
                BeginInvoke(new Action(UpdateKeyVisuals));
                return;
            }
            
            bool enabled = _enabled;
            
            SetKeyColor(_keyW_Panel, enabled && _keyW);
            SetKeyColor(_keyA_Panel, enabled && _keyA);
            SetKeyColor(_keyS_Panel, enabled && _keyS);
            SetKeyColor(_keyD_Panel, enabled && _keyD);
            SetKeyColor(_keyUp_Panel, enabled && _keyUp);
            SetKeyColor(_keyDown_Panel, enabled && _keyDown);
            SetKeyColor(_keyLeft_Panel, enabled && _keyLeft);
            SetKeyColor(_keyRight_Panel, enabled && _keyRight);
            
            // Update velocity display
            _lblVelocity.Text = $"Velocity: {_vx:F1}, {_vy:F1}, {_vz:F1} | Yaw: {_yawRateCmd * 180 / Math.PI:F0} deg/s";
        }
        
        private void SetKeyColor(Panel keyPanel, bool active)
        {
            if (keyPanel == null) return;
            keyPanel.BackColor = active ? KEY_ACTIVE : KEY_INACTIVE;
            keyPanel.Invalidate();
        }
        
        private void ResetAllKeys()
        {
            _keyW = _keyS = _keyA = _keyD = false;
            _keyUp = _keyDown = _keyLeft = _keyRight = false;
            _vx = _vy = _vz = 0;
            _yawRateCmd = 0;
            UpdateKeyVisuals();
            SendVelocityCommand();  // Send zero velocity
        }
        
        // ============================================================
        // MAVLink Command Sending
        // ============================================================
        
        private void SendVelocityCommand()
        {
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
                return;
            
            try
            {
                // Create SET_POSITION_TARGET_LOCAL_NED message
                // This uses velocity control in the LOCAL_NED frame
                var msg = new MAVLink.mavlink_set_position_target_local_ned_t
                {
                    time_boot_ms = (uint)Environment.TickCount,
                    target_system = MainV2.comPort.MAV.sysid,
                    target_component = MainV2.comPort.MAV.compid,
                    coordinate_frame = (byte)MAVLink.MAV_FRAME.LOCAL_NED,
                    
                    // Type mask for velocity + yaw rate control
                    // Bits 0-2: Ignore position (set)
                    // Bits 3-5: Use velocity (clear)
                    // Bits 6-8: Ignore acceleration (set)
                    // Bit 9: Ignore force (set)
                    // Bit 10: Ignore yaw (set)
                    // Bit 11: Use yaw rate (clear)
                    // 0b0000_0101_1100_0111 = 0x05C7 - velocity + yaw rate
                    type_mask = 0x05C7,
                    
                    // Position (ignored)
                    x = 0, y = 0, z = 0,
                    
                    // Velocity (NED frame)
                    vx = _vx,   // North (forward)
                    vy = _vy,   // East (right)
                    vz = _vz,   // Down (positive down)
                    
                    // Acceleration (ignored)
                    afx = 0, afy = 0, afz = 0,
                    
                    // Yaw (ignored) and Yaw rate
                    yaw = 0,
                    yaw_rate = _yawRateCmd
                };
                
                // Send packet
                MainV2.comPort.sendPacket(msg, MainV2.comPort.MAV.sysid, MainV2.comPort.MAV.compid);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"WASD Nudge error: {ex.Message}");
            }
        }
        
        // ============================================================
        // Timer Management - Safety-First Design
        // ============================================================
        
        private void StartCommandTimer()
        {
            StopCommandTimer();
            
            // Timer does two things at 10 Hz:
            // 1. Polls keyboard state for reliable key tracking
            // 2. Sends velocity commands
            _commandTimer = new System.Threading.Timer(
                _ => CommandTimerTick(),
                null,
                TimeSpan.Zero,
                TimeSpan.FromMilliseconds(100)
            );
        }
        
        private void CommandTimerTick()
        {
            try
            {
                // Safety: Check for key releases using Windows API on UI thread
                if (IsHandleCreated && !IsDisposed)
                {
                    BeginInvoke(new Action(() => {
                        try
                        {
                            CheckKeyReleases();
                        }
                        catch { }
                    }));
                }
                
                // Send velocity command (even if zero - ensures clean stop)
                SendVelocityCommand();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"WASD timer error: {ex.Message}");
            }
        }
        
        private void StopCommandTimer()
        {
            _commandTimer?.Dispose();
            _commandTimer = null;
        }
        
        // ============================================================
        // Helpers
        // ============================================================
        
        private void UpdateStatus(string text, Color color)
        {
            if (InvokeRequired)
            {
                BeginInvoke(new Action(() => UpdateStatus(text, color)));
                return;
            }
            
            _lblStatus.Text = text;
            _lblStatus.ForeColor = color;
        }
        
        private GraphicsPath RoundedRect(Rectangle bounds, int radius)
        {
            int d = radius * 2;
            var path = new GraphicsPath();
            path.AddArc(bounds.X, bounds.Y, d, d, 180, 90);
            path.AddArc(bounds.Right - d, bounds.Y, d, d, 270, 90);
            path.AddArc(bounds.Right - d, bounds.Bottom - d, d, d, 0, 90);
            path.AddArc(bounds.X, bounds.Bottom - d, d, d, 90, 90);
            path.CloseFigure();
            return path;
        }
        
        // ============================================================
        // Cleanup
        // ============================================================
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                StopCommandTimer();
                ResetAllKeys();
            }
            base.Dispose(disposing);
        }
    }
}
