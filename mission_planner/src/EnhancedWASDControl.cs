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
        // Constants - delegated to NOMADTheme for consistency
        // ============================================================

        private static readonly Color BG_COLOR = NOMADTheme.BG_DARK;
        private static readonly Color CARD_BG = NOMADTheme.CARD_BG;
        private static readonly Color KEY_INACTIVE = NOMADTheme.CARD_BORDER;
        private static readonly Color KEY_ACTIVE = NOMADTheme.ACCENT;
        private static readonly Color TEXT_PRIMARY = NOMADTheme.TEXT_PRIMARY;
        private static readonly Color TEXT_SECONDARY = NOMADTheme.TEXT_SECONDARY;
        private static readonly Color SUCCESS_COLOR = NOMADTheme.SUCCESS;
        private static readonly Color WARNING_COLOR = NOMADTheme.WARNING;
        private static readonly Color ERROR_COLOR = NOMADTheme.ERROR;
        private static readonly Color ACCENT_COLOR = NOMADTheme.ACCENT;
        private static readonly Color FOCUS_BORDER_COLOR = NOMADTheme.FOCUS;
        
        // ============================================================
        // Fields
        // ============================================================
        
        private bool _enabled = false;
        private bool _mouseInside = false;  // Track if mouse is inside control area
        private float _moveSpeed = 0.5f;    // m/s for WASD
        private float _altSpeed = 0.3f;     // m/s for altitude
        private float _yawRate = 15.0f;     // deg/s for yaw
        
        // Target velocity from current key state (set by UpdateVelocitiesFromKeys)
        private float _vxTarget = 0f, _vyTarget = 0f, _vzTarget = 0f, _yawRateTarget = 0f;

        // Currently commanded velocity (ramped toward target to avoid step inputs
        // that make ArduPilot's pos_controller overshoot when keys are released).
        private float _vx = 0f;
        private float _vy = 0f;
        private float _vz = 0f;
        private float _yawRateCmd = 0f;
        
        // Key state tracking
        private bool _keyW, _keyS, _keyA, _keyD;
        private bool _keyUp, _keyDown, _keyLeft, _keyRight;
        
        // Configuration
        private NOMADConfig _config;
        
        // Connection manager for status display only (not used as safety gate)
        private JetsonConnectionManager _jetsonConnectionManager;
        private bool _jetsonConnected = false;
        
        // UI Controls
        private Panel _keyboardPanel;
        private Panel _keyboardBorderPanel;  // Outer panel with border
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
        /// SAFETY: Cannot enable when MAVLink is not connected.
        /// </summary>
        public bool ControlEnabled
        {
            get => _enabled;
            set
            {
                // SAFETY: Block enabling when MAVLink is not connected
                if (value && (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen))
                {
                    UpdateStatus("BLOCKED - MAVLink offline (safety)", ERROR_COLOR);
                    if (_chkEnable != null) _chkEnable.Checked = false;
                    return;
                }
                
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
        
        public EnhancedWASDControl(NOMADConfig config = null, float moveSpeed = 0.5f, float altSpeed = 0.3f, float yawRate = 15.0f, JetsonConnectionManager jetsonConnectionManager = null)
        {
            _config = config;
            _moveSpeed = moveSpeed;
            _altSpeed = altSpeed;
            _yawRate = yawRate;
            _jetsonConnectionManager = jetsonConnectionManager;
            
            // Subscribe to connection state changes for safety
            if (_jetsonConnectionManager != null)
            {
                _jetsonConnected = _jetsonConnectionManager.IsConnected;
                _jetsonConnectionManager.ConnectionStateChanged += OnJetsonConnectionStateChanged;
            }
            
            InitializeUI();
            
            // Set control properties for keyboard capture
            this.SetStyle(ControlStyles.Selectable, true);
            this.TabStop = true;
            
            // Update initial state based on connection
            UpdateConnectionState();
        }
        
        private void OnJetsonConnectionStateChanged(object sender, JetsonConnectionStateChangedEventArgs e)
        {
            _jetsonConnected = e.NewState == JetsonConnectionState.Connected;
            
            // SAFETY: Immediately disable controls when Jetson disconnects
            if (!_jetsonConnected && _enabled)
            {
                if (InvokeRequired)
                    BeginInvoke(new Action(() => DisableForSafety()));
                else
                    DisableForSafety();
            }
            else if (InvokeRequired)
            {
                BeginInvoke(new Action(UpdateConnectionState));
            }
            else
            {
                UpdateConnectionState();
            }
        }
        
        private void DisableForSafety()
        {
            ControlEnabled = false;
            UpdateStatus("DISABLED - MAVLink disconnected (safety)", ERROR_COLOR);
        }
        
        private void UpdateConnectionState()
        {
            bool mavlinkConnected = MainV2.comPort?.BaseStream?.IsOpen == true;

            if (_chkEnable != null)
            {
                // Grey out enable checkbox when MAVLink is disconnected
                _chkEnable.Enabled = mavlinkConnected;
                _chkEnable.ForeColor = mavlinkConnected ? TEXT_PRIMARY : TEXT_SECONDARY;
            }

            if (!mavlinkConnected && !_enabled)
            {
                UpdateStatus("MAVLink offline - controls disabled", WARNING_COLOR);
            }
        }
        
        // ============================================================
        // UI Initialization
        // ============================================================
        
        private void InitializeUI()
        {
            this.BackColor = BG_COLOR;
            this.Dock = DockStyle.Fill;
            this.Padding = new Padding(10);
            this.MinimumSize = new Size(400, 520);

            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 4,
                BackColor = Color.Transparent,
            };
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 40));   // Enable/Status
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 220));  // Keyboard visual
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 70));   // Settings
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));   // Payload controls — fills remaining
            
            // Top row: Enable checkbox and status
            var topPanel = CreateTopPanel();
            mainLayout.Controls.Add(topPanel, 0, 0);
            
            // Middle row: Keyboard visualization
            _keyboardPanel = CreateKeyboardVisual();
            mainLayout.Controls.Add(_keyboardPanel, 0, 1);
            
            // Settings row: Speed settings
            var settingsPanel = CreateSettingsPanel();
            mainLayout.Controls.Add(settingsPanel, 0, 2);
            
            // Bottom row: Payload controls (reusable component)
            var payloadPanel = new PayloadControlPanel(_config);
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
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(10, 8),
                AutoSize = true,
                Cursor = Cursors.Hand,
            };
            _chkEnable.CheckedChanged += (s, e) => ControlEnabled = _chkEnable.Checked;
            panel.Controls.Add(_chkEnable);

            _lblStatus = new Label
            {
                Text = "Disabled",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(210, 10),
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
                Text = "CAUTION: Guided mode only. Keep RC ready.",
                Font = new Font("Segoe UI", 8),
                ForeColor = WARNING_COLOR,
                Location = new Point(10, 38),
                AutoSize = true,
            };
            panel.Controls.Add(lblWarning);
            
            return panel;
        }
        
        // ============================================================
        // Payload Controls Panel
        // ============================================================

        // ============================================================
        // Keyboard Event Handling - Cross-platform with fallback
        // ============================================================
        
        // Windows API for checking key state directly (only works on Windows)
        // On Linux/Mac, we rely entirely on event-based key tracking
        [System.Runtime.InteropServices.DllImport("user32.dll", SetLastError = true)]
        private static extern short GetAsyncKeyState(int vKey);
        
        // Platform detection
        private static readonly bool _isWindows = Environment.OSVersion.Platform == PlatformID.Win32NT;
        private static bool _win32ApiAvailable = _isWindows;
        
        // Virtual key codes (Windows VK_ constants)
        private const int VK_W = 0x57;
        private const int VK_A = 0x41;
        private const int VK_S = 0x53;
        private const int VK_D = 0x44;
        private const int VK_UP = 0x26;
        private const int VK_DOWN = 0x28;
        private const int VK_LEFT = 0x25;
        private const int VK_RIGHT = 0x27;
        
        // Event-based key state tracking (cross-platform)
        // These are updated by KeyDown/KeyUp events and are the primary
        // source of truth on non-Windows platforms
        private bool _eventKeyW, _eventKeyA, _eventKeyS, _eventKeyD;
        private bool _eventKeyUp, _eventKeyDown, _eventKeyLeft, _eventKeyRight;
        
        /// <summary>
        /// Check if a key is currently pressed.
        /// Uses Windows API on Windows, event-based tracking on Linux/Mac.
        /// </summary>
        private bool IsKeyPressed(int vKey)
        {
            if (!_win32ApiAvailable)
            {
                // On non-Windows platforms, use event-based state
                return GetEventBasedKeyState(vKey);
            }
            
            try
            {
                return (GetAsyncKeyState(vKey) & 0x8000) != 0;
            }
            catch (EntryPointNotFoundException)
            {
                // Win32 API not available (e.g., running under Mono on Linux)
                _win32ApiAvailable = false;
                return GetEventBasedKeyState(vKey);
            }
            catch (DllNotFoundException)
            {
                // user32.dll not available
                _win32ApiAvailable = false;
                return GetEventBasedKeyState(vKey);
            }
            catch
            {
                // Any other API failure - fall back to event-based
                return GetEventBasedKeyState(vKey);
            }
        }
        
        /// <summary>
        /// Get key state from event-based tracking (cross-platform).
        /// </summary>
        private bool GetEventBasedKeyState(int vKey)
        {
            return vKey switch
            {
                VK_W => _eventKeyW,
                VK_A => _eventKeyA,
                VK_S => _eventKeyS,
                VK_D => _eventKeyD,
                VK_UP => _eventKeyUp,
                VK_DOWN => _eventKeyDown,
                VK_LEFT => _eventKeyLeft,
                VK_RIGHT => _eventKeyRight,
                _ => false
            };
        }
        
        /// <summary>
        /// Update event-based key state from a KeyDown event.
        /// </summary>
        private void UpdateEventKeyState(Keys key, bool pressed)
        {
            switch (key)
            {
                case Keys.W: _eventKeyW = pressed; break;
                case Keys.A: _eventKeyA = pressed; break;
                case Keys.S: _eventKeyS = pressed; break;
                case Keys.D: _eventKeyD = pressed; break;
                case Keys.Up: _eventKeyUp = pressed; break;
                case Keys.Down: _eventKeyDown = pressed; break;
                case Keys.Left: _eventKeyLeft = pressed; break;
                case Keys.Right: _eventKeyRight = pressed; break;
            }
        }
        
        /// <summary>
        /// Safety check - ensures keys are actually released.
        /// Called periodically to catch any missed key releases.
        /// Also checks if mouse has left the control area.
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
            if (!_win32ApiAvailable) return;
            
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
            
            // Set key state on key down (both internal state and event-based for cross-platform)
            switch (keyData)
            {
                case Keys.W: _keyW = true; UpdateEventKeyState(Keys.W, true); break;
                case Keys.S: _keyS = true; UpdateEventKeyState(Keys.S, true); break;
                case Keys.A: _keyA = true; UpdateEventKeyState(Keys.A, true); break;
                case Keys.D: _keyD = true; UpdateEventKeyState(Keys.D, true); break;
                case Keys.Up: _keyUp = true; UpdateEventKeyState(Keys.Up, true); break;
                case Keys.Down: _keyDown = true; UpdateEventKeyState(Keys.Down, true); break;
                case Keys.Left: _keyLeft = true; UpdateEventKeyState(Keys.Left, true); break;
                case Keys.Right: _keyRight = true; UpdateEventKeyState(Keys.Right, true); break;
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
            
            // Reset the specific key that was released (both internal and event-based state)
            switch (e.KeyCode)
            {
                case Keys.W: _keyW = false; UpdateEventKeyState(Keys.W, false); break;
                case Keys.S: _keyS = false; UpdateEventKeyState(Keys.S, false); break;
                case Keys.A: _keyA = false; UpdateEventKeyState(Keys.A, false); break;
                case Keys.D: _keyD = false; UpdateEventKeyState(Keys.D, false); break;
                case Keys.Up: _keyUp = false; UpdateEventKeyState(Keys.Up, false); break;
                case Keys.Down: _keyDown = false; UpdateEventKeyState(Keys.Down, false); break;
                case Keys.Left: _keyLeft = false; UpdateEventKeyState(Keys.Left, false); break;
                case Keys.Right: _keyRight = false; UpdateEventKeyState(Keys.Right, false); break;
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
            // Forward/Back (X velocity in body frame)
            if (_keyW && !_keyS)       _vxTarget = _moveSpeed;
            else if (_keyS && !_keyW)  _vxTarget = -_moveSpeed;
            else                        _vxTarget = 0;

            // Left/Right strafe (Y velocity in body frame)
            if (_keyD && !_keyA)       _vyTarget = _moveSpeed;
            else if (_keyA && !_keyD)  _vyTarget = -_moveSpeed;
            else                        _vyTarget = 0;

            // Altitude (Z velocity, positive = DOWN)
            if (_keyUp && !_keyDown)     _vzTarget = -_altSpeed;
            else if (_keyDown && !_keyUp) _vzTarget = _altSpeed;
            else                          _vzTarget = 0;

            // Yaw rate (rad/s)
            if (_keyRight && !_keyLeft)   _yawRateTarget = _yawRate * (float)(Math.PI / 180.0);
            else if (_keyLeft && !_keyRight) _yawRateTarget = -_yawRate * (float)(Math.PI / 180.0);
            else                              _yawRateTarget = 0;
        }

        // Linear ramp the commanded velocity toward the target. Called from the
        // 10 Hz command timer so each tick is ~100 ms. Slewing the setpoint
        // (rather than snapping it on key release) keeps ArduPilot's
        // pos_controller from overshooting and oscillating.
        private void RampVelocityTowardTarget()
        {
            const float linearStep = 0.15f;  // m/s per 100 ms  (full stop in ~3-4 ticks)
            float yawStep = 1.0f * (float)(Math.PI / 180.0); // ~10 deg/s per 100 ms

            _vx = ApproachFloat(_vx, _vxTarget, linearStep);
            _vy = ApproachFloat(_vy, _vyTarget, linearStep);
            _vz = ApproachFloat(_vz, _vzTarget, linearStep);
            _yawRateCmd = ApproachFloat(_yawRateCmd, _yawRateTarget, yawStep);
        }

        private static float ApproachFloat(float current, float target, float step)
        {
            if (current < target) return Math.Min(target, current + step);
            if (current > target) return Math.Max(target, current - step);
            return target;
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
            
            // Also reset event-based tracking (cross-platform)
            _eventKeyW = _eventKeyA = _eventKeyS = _eventKeyD = false;
            _eventKeyUp = _eventKeyDown = _eventKeyLeft = _eventKeyRight = false;
            
            _vx = _vy = _vz = 0;
            _yawRateCmd = 0;
            _vxTarget = _vyTarget = _vzTarget = 0;
            _yawRateTarget = 0;
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
                // Create SET_POSITION_TARGET_LOCAL_NED in BODY_OFFSET_NED:
                // velocity components are interpreted as forward/right/down
                // relative to the vehicle's current heading. ArduPilot Copter
                // explicitly supports frames 1, 7, 8, 9 — BODY_FRD (12) is
                // silently rejected on older firmwares, which made the WASD
                // packets a no-op.
                var msg = new MAVLink.mavlink_set_position_target_local_ned_t
                {
                    time_boot_ms = (uint)Environment.TickCount,
                    target_system = MainV2.comPort.MAV.sysid,
                    target_component = MainV2.comPort.MAV.compid,
                    coordinate_frame = (byte)MAVLink.MAV_FRAME.BODY_OFFSET_NED,

                    // Velocity + yaw_rate mask. Bits (LSB=0):
                    //   0,1,2  pos ignore (set)
                    //   3,4,5  vel use     (clear)
                    //   6,7,8  accel ignore (set)
                    //   9      FORCE_SET flag (clear — accel fields are not forces)
                    //   10     yaw ignore (set)
                    //   11     yaw_rate use (clear)
                    // = 0b0000_0101_1100_0111 = 0x05C7
                    type_mask = 0x05C7,
                    
                    // Position (ignored)
                    x = 0, y = 0, z = 0,
                    
                    // Velocity (body-offset NED frame): X=forward, Y=right, Z=down
                    vx = _vx,   // forward
                    vy = _vy,   // right
                    vz = _vz,   // down (positive down)
                    
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
                // SAFETY: Auto-disable if MAVLink drops while nudge is active
                bool mavlinkConnected = MainV2.comPort?.BaseStream?.IsOpen == true;
                if (!mavlinkConnected && _enabled)
                {
                    if (IsHandleCreated && !IsDisposed)
                        BeginInvoke(new Action(DisableForSafety));
                    return;
                }

                // Safety: Check for key releases using Windows API on UI thread
                if (IsHandleCreated && !IsDisposed)
                {
                    BeginInvoke(new Action(() => {
                        try { CheckKeyReleases(); }
                        catch { }
                    }));
                }

                // Slew commanded velocity toward the target from current keys,
                // then send. Ramping prevents the step input that previously
                // caused overshoot/oscillation on key release.
                RampVelocityTowardTarget();
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
