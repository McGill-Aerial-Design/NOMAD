// ============================================================
// NOMAD Payload Control Panel - Reusable Component
// ============================================================
// Drop servos, strap reel and camera tilt are all wired to the
// Cube Orange and commanded via MAVLink DO_SET_SERVO (primary).
// Edge Core HTTP is used as a fallback to command the Cube from the Jetson.
// ============================================================

using System;
using System.Drawing;
using System.Threading.Tasks;
using Timer = System.Windows.Forms.Timer;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Payload controls: 3 × drop servo, strap reel (hold-to-reel),
    /// water pump trigger, and ZED camera tilt slider.
    /// MAVLink DO_SET_SERVO is tried first; Edge Core's Cube command API is the fallback.
    /// </summary>
    public class PayloadControlPanel : UserControl
    {
        private static readonly Color CARD_BG        = NOMADTheme.CARD_BG;
        private static readonly Color TEXT_PRIMARY   = NOMADTheme.TEXT_PRIMARY;
        private static readonly Color TEXT_SECONDARY = NOMADTheme.TEXT_SECONDARY;
        private static readonly Color SUCCESS_COLOR  = NOMADTheme.SUCCESS;
        private static readonly Color WARNING_COLOR  = NOMADTheme.WARNING;
        private static readonly Color ERROR_COLOR    = NOMADTheme.ERROR;
        private static readonly Color ACCENT_COLOR   = NOMADTheme.ACCENT;

        // Drop button base color and armed-state colors (1 and 2 clicks in)
        private static readonly Color DROP_COLOR_IDLE    = Color.FromArgb(120, 50, 15);
        private static readonly Color DROP_COLOR_ARM1    = Color.FromArgb(200, 110, 0);   // 1st click — orange
        private static readonly Color DROP_COLOR_ARM2    = Color.FromArgb(220, 50,  0);   // 2nd click — red-orange
        private static readonly Color DROP_COLOR_DROPPED = Color.FromArgb(50, 90, 130);   // dropped — steel blue → retract mode

        // Safety limits
        private const int DROP_CLICKS_REQUIRED = 3;      // clicks needed to drop
        private const int DROP_RESET_MS        = 3000;   // ms before click count resets
        private const int REEL_SAFETY_MS       = 10_000; // max continuous reel time
        private const int TILT_SETTLE_MS       = 100;    // final send after slider stops moving

        private readonly NOMADConfig _config;

        // Drop safety
        private readonly Button[]  _dropButtons    = new Button[3];
        private readonly int[]     _dropClickCount  = { 0, 0, 0 };
        private readonly Timer[]   _dropResetTimers = new Timer[3];
        private readonly bool[]    _dropDropped     = { false, false, false }; // true = servo at PwmMax, retract available

        // Reel safety (indexed: 0 = reel 1, 1 = reel 2)
        private readonly bool[]  _reelActive       = new bool[2];
        private readonly Timer[] _reelSafetyTimers = new Timer[2];

        // Tilt debounce
        private TrackBar _tiltSlider;
        private Label    _lblTiltValue;
        private Timer    _tiltDebounceTimer;
        private bool     _suppressTiltEvent;

        private Label _lblStatus;

        // Shared tilt state so multiple panel instances stay in sync.
        private static int s_lastTiltPulseUs = 1250;
        private static event Action<int, PayloadControlPanel> CameraTiltChanged;

        /// <summary>
        /// Raise this to lock or unlock the camera tilt controls across all PayloadControlPanel instances.
        /// Task 2 autonomy fires true on entry, false on exit.
        /// </summary>
        public static event Action<bool> AutonomousModeChanged;
        public static void RaiseAutonomousModeChanged(bool isAutonomous) =>
            AutonomousModeChanged?.Invoke(isAutonomous);


        // ============================================================
        // Construction
        // ============================================================

        public PayloadControlPanel(NOMADConfig config)
        {
            _config = config;
            InitializeUI();
            CameraTiltChanged    += OnCameraTiltChangedExternally;
            AutonomousModeChanged += OnAutonomousModeChanged;
            this.Disposed += (s, e) =>
            {
                CameraTiltChanged    -= OnCameraTiltChangedExternally;
                AutonomousModeChanged -= OnAutonomousModeChanged;
                CleanupTimers();
            };
            ApplyTiltPulseQuietly(s_lastTiltPulseUs);
        }

        // ============================================================
        // UI Construction
        // ============================================================

        private void InitializeUI()
        {
            BackColor = CARD_BG;
            Dock = DockStyle.Fill;
            Padding = new Padding(10, 5, 10, 5);

            Controls.Add(new Label
            {
                Text = "PAYLOAD CONTROLS",
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(10, 5),
                AutoSize = true,
            });

            const int DROP_H = 26;
            const int DROP_W = 75;
            const int ROW_GAP = 5;

            // ---- Row 1: Drop buttons (triple-click required) + water pump ----
            int x = 10, y = 26;

            for (int i = 0; i < 3; i++)
            {
                int payload = i + 1;
                var btn = MakeButton($"Drop P{payload}", DROP_COLOR_IDLE, DROP_W, DROP_H);
                btn.Location = new Point(x, y);
                btn.Click += (s, e) => OnDropClick(payload);
                Controls.Add(btn);
                _dropButtons[i] = btn;
                x += DROP_W + 4;
            }

            var btnWater = MakeButton("Shoot Water", Color.FromArgb(30, 100, 180), 90, DROP_H);
            btnWater.Location = new Point(x, y);
            btnWater.Click += (s, e) => ShootWater();
            Controls.Add(btnWater);

            // ---- Row 2: Strap reel 1 (hold-to-reel, 10s safety limit) ----
            y += DROP_H + ROW_GAP;
            x = 10;

            Controls.Add(new Label
            {
                Text = "Reel P1:",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(x, y + 4),
                AutoSize = true,
            });
            x += 58;

            var btnReelIn = MakeButton("⬆ Reel In", Color.FromArgb(50, 100, 50), 85, DROP_H);
            btnReelIn.Location = new Point(x, y);
            btnReelIn.MouseDown  += (s, e) => StartReel(0, _config?.ReelPwmIn  ?? 2100);
            btnReelIn.MouseUp    += (s, e) => StopReel(0);
            btnReelIn.MouseLeave += (s, e) => { if (_reelActive[0]) StopReel(0); };
            Controls.Add(btnReelIn);
            x += 89;

            var btnReelOut = MakeButton("⬇ Reel Out", Color.FromArgb(50, 50, 100), 88, DROP_H);
            btnReelOut.Location = new Point(x, y);
            btnReelOut.MouseDown  += (s, e) => StartReel(0, _config?.ReelPwmOut ?? 900);
            btnReelOut.MouseUp    += (s, e) => StopReel(0);
            btnReelOut.MouseLeave += (s, e) => { if (_reelActive[0]) StopReel(0); };
            Controls.Add(btnReelOut);

            // ---- Row 3: Strap reel 2 ----
            y += DROP_H + ROW_GAP;
            x = 10;

            Controls.Add(new Label
            {
                Text = "Reel P2:",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(x, y + 4),
                AutoSize = true,
            });
            x += 58;

            var btnReel2In = MakeButton("⬆ Reel In", Color.FromArgb(50, 100, 50), 85, DROP_H);
            btnReel2In.Location = new Point(x, y);
            btnReel2In.MouseDown  += (s, e) => StartReel(1, _config?.Reel2PwmIn  ?? 2100);
            btnReel2In.MouseUp    += (s, e) => StopReel(1);
            btnReel2In.MouseLeave += (s, e) => { if (_reelActive[1]) StopReel(1); };
            Controls.Add(btnReel2In);
            x += 89;

            var btnReel2Out = MakeButton("⬇ Reel Out", Color.FromArgb(50, 50, 100), 88, DROP_H);
            btnReel2Out.Location = new Point(x, y);
            btnReel2Out.MouseDown  += (s, e) => StartReel(1, _config?.Reel2PwmOut ?? 900);
            btnReel2Out.MouseUp    += (s, e) => StopReel(1);
            btnReel2Out.MouseLeave += (s, e) => { if (_reelActive[1]) StopReel(1); };
            Controls.Add(btnReel2Out);

            // ---- Row 4: Camera tilt slider ----
            y += DROP_H + ROW_GAP;
            x = 10;

            Controls.Add(new Label
            {
                Text = "Cam Tilt:",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(x, y + 4),
                AutoSize = true,
            });
            x += 72;

            int tiltMin     = _config?.CameraTiltPwmMin ?? 700;
            int tiltMax     = _config?.CameraTiltPwmMax ?? 1450;
            int initialTilt = Math.Max(tiltMin, Math.Min(tiltMax, s_lastTiltPulseUs));

            _tiltSlider = new TrackBar
            {
                Location      = new Point(x, y - 2),
                Size          = new Size(170, 26),
                AutoSize      = false,   // prevent TrackBar from growing over the status label
                TickStyle     = TickStyle.None,
                Minimum       = tiltMin,
                Maximum       = tiltMax,
                Value         = initialTilt,
                SmallChange   = 10,
                LargeChange   = 50,
                BackColor     = CARD_BG,
            };
            _tiltSlider.ValueChanged += (s, e) => OnTiltSliderChanged();
            Controls.Add(_tiltSlider);
            x += 175;

            _lblTiltValue = new Label
            {
                Text     = $"{initialTilt} us",
                Font     = new Font("Segoe UI", 9),
                ForeColor = TEXT_PRIMARY,
                Location  = new Point(x, y + 4),
                AutoSize  = true,
            };
            Controls.Add(_lblTiltValue);
            x += 60;

            // Down / Center / Up preset buttons
            int tiltCenter = _config?.CameraTiltPwmNeutral ?? 1250;
            foreach (var (label, value) in new (string, int)[] { ("▼", tiltMin), ("●", tiltCenter), ("▲", tiltMax) })
            {
                int v = value;
                var btn = MakeButton(label, Color.FromArgb(60, 60, 70), 26, DROP_H);
                btn.Location = new Point(x, y);
                btn.Font = new Font("Segoe UI", 8);
                btn.Click += (s, e) => ApplyTiltPulse(v);
                Controls.Add(btn);
                x += 30;
            }

            // ---- Status label — below tilt row, full width ----
            y += DROP_H + ROW_GAP;
            _lblStatus = new Label
            {
                Text      = "",
                Font      = new Font("Segoe UI", 8),
                ForeColor = TEXT_SECONDARY,
                Location  = new Point(10, y),
                AutoSize  = true,
            };
            Controls.Add(_lblStatus);

            // Lock in the minimum height so the parent TableLayoutPanel row never clips content.
            this.MinimumSize = new Size(300, y + 20);
        }

        // ============================================================
        // Servo helpers
        // ============================================================

        // Returns (drop, retract) PWM for the configured min/max + mounting orientation.
        // Reversed servos drop at pwmMin and retract at pwmMax; non-reversed do the opposite.
        // Per-servo because P1 uses two opposing servos with independent orientations.
        private static (int drop, int retract) ServoPwm(int pwmMin, int pwmMax, bool reversed)
            => reversed ? (pwmMin, pwmMax) : (pwmMax, pwmMin);

        // ============================================================
        // Drop payload  —  3-click confirmation
        // ============================================================

        private void OnDropClick(int payloadNumber)
        {
            int idx = payloadNumber - 1;

            // Already dropped — single click retracts back to PwmMin so the
            // operator can wiggle the servo if a strap is caught.
            if (_dropDropped[idx])
            {
                ExecuteRetract(payloadNumber);
                return;
            }

            _dropClickCount[idx]++;

            // Restart the reset timer so clicks that come > 3s apart reset the count.
            _dropResetTimers[idx]?.Stop();
            _dropResetTimers[idx]?.Dispose();
            var resetTimer = new Timer { Interval = DROP_RESET_MS };
            resetTimer.Tick += (s, e) =>
            {
                resetTimer.Stop();
                resetTimer.Dispose();
                _dropResetTimers[idx] = null;
                _dropClickCount[idx] = 0;
                if (!IsDisposed && _dropButtons[idx] != null)
                    _dropButtons[idx].BackColor = DROP_COLOR_IDLE;
                SetStatus($"Payload {payloadNumber} drop cancelled (timeout)", TEXT_SECONDARY);
            };
            _dropResetTimers[idx] = resetTimer;
            resetTimer.Start();

            int count = _dropClickCount[idx];

            if (count < DROP_CLICKS_REQUIRED)
            {
                int remaining = DROP_CLICKS_REQUIRED - count;
                _dropButtons[idx].BackColor = count == 1 ? DROP_COLOR_ARM1 : DROP_COLOR_ARM2;
                SetStatus($"Payload {payloadNumber}: {remaining} more click{(remaining == 1 ? "" : "s")} to drop!", WARNING_COLOR);
                return;
            }

            // Third click — arm timer cleaned up, execute drop
            _dropResetTimers[idx]?.Stop();
            _dropResetTimers[idx]?.Dispose();
            _dropResetTimers[idx] = null;
            _dropClickCount[idx]  = 0;
            _dropButtons[idx].BackColor = DROP_COLOR_IDLE;

            ExecuteDrop(payloadNumber);
        }

        private async void ExecuteDrop(int payloadNumber)
        {
            int idx = payloadNumber - 1;
            int channel, pwmDrop;
            int channel2 = 0, pwmDrop2 = 2000;
            switch (payloadNumber)
            {
                case 1:
                    (pwmDrop,  _) = ServoPwm(_config?.Servo1PwmMin  ?? 1000, _config?.Servo1PwmMax  ?? 2000, _config?.Servo1Reversed  ?? false);
                    (pwmDrop2, _) = ServoPwm(_config?.Servo1bPwmMin ?? 1000, _config?.Servo1bPwmMax ?? 2000, _config?.Servo1bReversed ?? false);
                    channel  = _config?.Servo1Channel  ?? 0;
                    channel2 = _config?.Servo1bChannel ?? 0;
                    break;
                case 2:
                    (pwmDrop, _) = ServoPwm(_config?.Servo2PwmMin ?? 1000, _config?.Servo2PwmMax ?? 2000, _config?.Servo2Reversed ?? false);
                    channel = _config?.Servo2Channel ?? 0;
                    break;
                case 3:
                    (pwmDrop, _) = ServoPwm(_config?.Servo3PwmMin ?? 1000, _config?.Servo3PwmMax ?? 2000, _config?.Servo3Reversed ?? false);
                    channel = _config?.Servo3Channel ?? 0;
                    break;
                default: return;
            }

            if (channel <= 0)
            {
                SetStatus($"Payload {payloadNumber} channel not configured (see Settings > Servos)", WARNING_COLOR);
                return;
            }

            bool success = false;

            if (await CubeOutputController.SendServoPwmAsync(channel, pwmDrop))
            {
                if (channel2 > 0) await CubeOutputController.SendServoPwmAsync(channel2, pwmDrop2);
                string ch2info = channel2 > 0 ? $" + ch{channel2}" : "";
                SetStatus($"Payload {payloadNumber} dropped  (Cube ch{channel}{ch2info} {pwmDrop}us)", SUCCESS_COLOR);
                success = true;
            }
            else
            {
                SetStatus("Drop failed: Cube output command unavailable", ERROR_COLOR);
            }

            if (success)
            {
                _dropDropped[idx] = true;
                _dropButtons[idx].Text = $"Retract P{payloadNumber}";
                _dropButtons[idx].BackColor = DROP_COLOR_DROPPED;
            }
        }

        private async void ExecuteRetract(int payloadNumber)
        {
            int idx = payloadNumber - 1;
            int channel, pwmMin;
            int channel2 = 0, pwmMin2 = 1000;
            switch (payloadNumber)
            {
                case 1:
                    (_, pwmMin)  = ServoPwm(_config?.Servo1PwmMin  ?? 1000, _config?.Servo1PwmMax  ?? 2000, _config?.Servo1Reversed  ?? false);
                    (_, pwmMin2) = ServoPwm(_config?.Servo1bPwmMin ?? 1000, _config?.Servo1bPwmMax ?? 2000, _config?.Servo1bReversed ?? false);
                    channel  = _config?.Servo1Channel  ?? 0;
                    channel2 = _config?.Servo1bChannel ?? 0;
                    break;
                case 2:
                    (_, pwmMin) = ServoPwm(_config?.Servo2PwmMin ?? 1000, _config?.Servo2PwmMax ?? 2000, _config?.Servo2Reversed ?? false);
                    channel = _config?.Servo2Channel ?? 0;
                    break;
                case 3:
                    (_, pwmMin) = ServoPwm(_config?.Servo3PwmMin ?? 1000, _config?.Servo3PwmMax ?? 2000, _config?.Servo3Reversed ?? false);
                    channel = _config?.Servo3Channel ?? 0;
                    break;
                default: return;
            }

            if (channel <= 0)
            {
                SetStatus($"Payload {payloadNumber} channel not configured (see Settings > Servos)", WARNING_COLOR);
                return;
            }

            // Immediately restore button to drop-ready state so the operator can
            // drop again without waiting for the servo to physically finish moving.
            _dropDropped[idx] = false;
            _dropButtons[idx].Text = $"Drop P{payloadNumber}";
            _dropButtons[idx].BackColor = DROP_COLOR_IDLE;

            if (await CubeOutputController.SendServoPwmAsync(channel, pwmMin))
            {
                if (channel2 > 0) await CubeOutputController.SendServoPwmAsync(channel2, pwmMin2);
                string ch2info = channel2 > 0 ? $" + ch{channel2}" : "";
                SetStatus($"Payload {payloadNumber} retracted  (Cube ch{channel}{ch2info} {pwmMin}us)", SUCCESS_COLOR);
                return;
            }

            SetStatus("Retract failed: Cube output command unavailable", ERROR_COLOR);
        }

        // ============================================================
        // Strap reel  —  hold-to-reel, 10-second safety cut-off
        // ============================================================

        private void StartReel(int reelIdx, int pwmUs)
        {
            int channel = reelIdx == 0 ? (_config?.ReelServoChannel ?? 0) : (_config?.Reel2ServoChannel ?? 0);
            _reelActive[reelIdx] = true;

            _reelSafetyTimers[reelIdx]?.Stop();
            _reelSafetyTimers[reelIdx]?.Dispose();
            var t = new Timer { Interval = REEL_SAFETY_MS };
            t.Tick += (s, e) =>
            {
                t.Stop();
                t.Dispose();
                _reelSafetyTimers[reelIdx] = null;
                _reelActive[reelIdx] = false;
                SendServoNow(reelIdx == 0 ? (_config?.ReelServoChannel ?? 0) : (_config?.Reel2ServoChannel ?? 0), 1500);
                SetStatus($"Reel P{reelIdx + 1} stopped  (10s safety limit)", WARNING_COLOR);
            };
            _reelSafetyTimers[reelIdx] = t;
            t.Start();

            SendServoNow(channel, pwmUs);
            SetStatus($"Reel P{reelIdx + 1} ({pwmUs}µs) — hold button...", SUCCESS_COLOR);
        }

        private void StopReel(int reelIdx)
        {
            if (!_reelActive[reelIdx]) return;
            _reelActive[reelIdx] = false;

            _reelSafetyTimers[reelIdx]?.Stop();
            _reelSafetyTimers[reelIdx]?.Dispose();
            _reelSafetyTimers[reelIdx] = null;

            int channel = reelIdx == 0 ? (_config?.ReelServoChannel ?? 0) : (_config?.Reel2ServoChannel ?? 0);
            SendServoNow(channel, 1500);
            SetStatus($"Reel P{reelIdx + 1} stopped", TEXT_SECONDARY);
        }

        /// <summary>
        /// Fire-and-forget Cube servo command for time-critical paths (reel MouseDown/Up).
        /// </summary>
        private async void SendServoNow(int channel, int pwmUs)
        {
            if (channel <= 0) return;
            await CubeOutputController.SendServoPwmAsync(channel, pwmUs);
        }

        // ============================================================
        // Water pump
        // ============================================================

        private async void ShootWater()
        {
            int relay      = _config?.WaterPumpRelayNumber ?? 0;
            int durationMs = _config?.WaterPumpDurationMs  ?? 500;

            SetStatus($"Water pump firing  ({durationMs}ms)...", SUCCESS_COLOR);
            bool success = await CubeOutputController.FireRelayAsync(relay, durationMs);
            SetStatus(
                success ? "Water pump done" : "Pump failed: Cube relay command unavailable",
                success ? SUCCESS_COLOR : ERROR_COLOR);
        }

        // ============================================================
        // Camera tilt  —  stream on drag, settle-send on release
        // ============================================================

        private void OnTiltSliderChanged()
        {
            if (_tiltSlider == null || _lblTiltValue == null) return;

            int pulseUs = _tiltSlider.Value;
            _lblTiltValue.Text = $"{pulseUs} us";

            if (_suppressTiltEvent) return;

            s_lastTiltPulseUs = pulseUs;
            CameraTiltChanged?.Invoke(pulseUs, this);

            // Send immediately — drop if a previous command is still in-flight so
            // rapid drag events never queue up and cause lag.
            SendCameraTiltAsync(pulseUs, tryOnly: true);

            // Restart settle timer so the final resting value is always committed
            // even if the last few drag events were dropped.
            _tiltDebounceTimer?.Stop();
            _tiltDebounceTimer?.Dispose();
            _tiltDebounceTimer = new Timer { Interval = TILT_SETTLE_MS };
            _tiltDebounceTimer.Tick += (s, e) =>
            {
                _tiltDebounceTimer?.Stop();
                _tiltDebounceTimer?.Dispose();
                _tiltDebounceTimer = null;
                if (_tiltSlider != null && !_tiltSlider.IsDisposed)
                    SendCameraTiltAsync(_tiltSlider.Value, tryOnly: false);
            };
            _tiltDebounceTimer.Start();
        }

        private async void SendCameraTiltAsync(int pulseUs, bool tryOnly = false)
        {
            int channel = _config?.CameraTiltChannel ?? 0;

            await CubeOutputController.SendServoPwmAsync(channel, pulseUs, tryOnly);
        }

        private void OnCameraTiltChangedExternally(int pulseUs, PayloadControlPanel source)
        {
            if (source == this) return;
            if (IsDisposed || _tiltSlider == null) return;

            if (InvokeRequired) { BeginInvoke(new Action(() => ApplyTiltPulseQuietly(pulseUs))); return; }
            ApplyTiltPulseQuietly(pulseUs);
        }

        private void OnAutonomousModeChanged(bool isAutonomous)
        {
            if (IsDisposed) return;
            if (InvokeRequired) { BeginInvoke(new Action(() => OnAutonomousModeChanged(isAutonomous))); return; }

            if (_tiltSlider != null) _tiltSlider.Enabled = !isAutonomous;
            if (_lblTiltValue != null) _lblTiltValue.ForeColor = isAutonomous ? TEXT_SECONDARY : TEXT_PRIMARY;

            // Disable all tilt-row controls (preset buttons follow the slider's parent area)
            foreach (Control c in Controls)
            {
                if (c is Button btn && (btn.Text == "▼" || btn.Text == "●" || btn.Text == "▲"))
                    btn.Enabled = !isAutonomous;
            }

            if (isAutonomous)
                SetStatus("Camera tilt locked — autonomous mode active", WARNING_COLOR);
            else
                SetStatus("Camera tilt restored", TEXT_SECONDARY);
        }

        private void ApplyTiltPulse(int pulseUs)
        {
            ApplyTiltPulseQuietly(pulseUs);
            s_lastTiltPulseUs = pulseUs;
            CameraTiltChanged?.Invoke(pulseUs, this);
            SendCameraTiltAsync(pulseUs, tryOnly: false);
        }

        private void ApplyTiltPulseQuietly(int pulseUs)
        {
            if (_tiltSlider == null) return;
            int clamped = Math.Max(_tiltSlider.Minimum, Math.Min(_tiltSlider.Maximum, pulseUs));
            if (_tiltSlider.Value == clamped)
            {
                if (_lblTiltValue != null) _lblTiltValue.Text = $"{clamped} us";
                return;
            }
            _suppressTiltEvent = true;
            try { _tiltSlider.Value = clamped; }
            finally { _suppressTiltEvent = false; }
            if (_lblTiltValue != null) _lblTiltValue.Text = $"{clamped} us";
        }

        // ============================================================
        // Helpers
        // ============================================================

        private Button MakeButton(string text, Color backColor, int width, int height)
        {
            var btn = new Button
            {
                Text = text,
                Size = new Size(width, height),
                FlatStyle = FlatStyle.Flat,
                BackColor = backColor,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                Cursor = Cursors.Hand,
            };
            btn.FlatAppearance.BorderColor = Color.FromArgb(80, 80, 85);
            return btn;
        }

        private void SetStatus(string text, Color color)
        {
            if (InvokeRequired) { BeginInvoke(new Action(() => SetStatus(text, color))); return; }
            if (_lblStatus != null) { _lblStatus.Text = text; _lblStatus.ForeColor = color; }
        }

        private void CleanupTimers()
        {
            for (int i = 0; i < _dropResetTimers.Length; i++)
            {
                _dropResetTimers[i]?.Stop();
                _dropResetTimers[i]?.Dispose();
                _dropResetTimers[i] = null;
            }
            for (int i = 0; i < _reelSafetyTimers.Length; i++)
            {
                _reelSafetyTimers[i]?.Stop();
                _reelSafetyTimers[i]?.Dispose();
                _reelSafetyTimers[i] = null;
            }
            _tiltDebounceTimer?.Stop();
            _tiltDebounceTimer?.Dispose();
            _tiltDebounceTimer = null;
        }
    }
}
