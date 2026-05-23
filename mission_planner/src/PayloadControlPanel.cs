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
        private const int REEL_SAFETY_MS       = 10_000; // max continuous hold-to-reel time
        private const int TILT_SETTLE_MS       = 100;    // final send after slider stops moving

        // Full-spool reel constants — three-click confirm to start, fourth
        // click (or any click after start) cancels mid-spool.
        private const int FULL_REEL_DURATION_MS      = 80_000; // 1 min 20 s
        private const int FULL_REEL_CLICKS_REQUIRED  = 3;
        private const int FULL_REEL_CLICK_RESET_MS   = 3000;

        private readonly NOMADConfig _config;

        // Drop safety
        private readonly Button[]  _dropButtons    = new Button[3];
        private readonly int[]     _dropClickCount  = { 0, 0, 0 };
        private readonly Timer[]   _dropResetTimers = new Timer[3];
        private readonly bool[]    _dropDropped     = { false, false, false }; // true = servo at PwmMax, retract available

        // Reel safety (indexed: 0 = reel 1, 1 = reel 2)
        private readonly bool[]  _reelActive       = new bool[2];
        private readonly Timer[] _reelSafetyTimers = new Timer[2];

        // Full-spool reel state. Slot layout: 0 = reel-1 IN, 1 = reel-1 OUT,
        // 2 = reel-2 IN, 3 = reel-2 OUT. Each slot tracks its own arming
        // click count and its own active 80s countdown so the operator can
        // cancel mid-spool by clicking the running button again.
        private readonly Button[] _fullReelButtons        = new Button[4];
        private readonly string[] _fullReelLabels         = { "In Full", "Out Full", "In Full", "Out Full" };
        private readonly int[]    _fullReelClickCount     = new int[4];
        private readonly Timer[]  _fullReelClickReset     = new Timer[4];
        private readonly Timer[]  _fullReelCountdown      = new Timer[4];
        private readonly int[]    _fullReelRemainingMs    = new int[4];
        private readonly bool[]   _fullReelActive         = new bool[4];

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

        /// <summary>
        /// Update the shared tilt PWM and broadcast to every open PayloadControlPanel
        /// so each slider mirrors the new value. Does NOT send to the servo — that's
        /// the caller's responsibility (e.g. NomadJoystickService already streams
        /// SendServoPwmAsync at its own rate).
        /// </summary>
        public static void SetExternalTiltPulse(int pulseUs)
        {
            s_lastTiltPulseUs = pulseUs;
            CameraTiltChanged?.Invoke(pulseUs, null);
        }

        /// <summary>Last commanded tilt PWM (microseconds). Used by joystick service as its starting target.</summary>
        public static int LastTiltPulseUs => s_lastTiltPulseUs;

        // ============================================================
        // Cross-panel payload drop-state sync
        // ============================================================
        // External sources (NomadJoystickService, automated drops, second open
        // panel) need to push their drop/retract events here so every open
        // PayloadControlPanel mirrors the new visual state and the
        // joystick-toggle state machine stays consistent. The bool is true when
        // the servo is at its drop position.

        public static event Action<int, bool> PayloadDroppedStateChanged;

        // Authoritative dropped/retracted state per payload (idx 0..2). Mirrored
        // into each panel's _dropDropped on event delivery; consulted by
        // NomadJoystickService.ToggleDrop to decide direction on each switch flip.
        private static readonly bool[] s_payloadDropped = new bool[3];

        /// <summary>Latest known dropped state for the given payload index (0..2).</summary>
        public static bool IsPayloadDropped(int payloadIdx0)
            => payloadIdx0 >= 0 && payloadIdx0 < s_payloadDropped.Length && s_payloadDropped[payloadIdx0];

        /// <summary>
        /// Raise this when any source (panel, joystick service, autonomy) drops
        /// or retracts a payload, so every open panel and the joystick service
        /// agree on the toggle state. payloadIdx0 is zero-based (0=P1, 1=P2, 2=P3).
        /// </summary>
        public static void RaisePayloadDroppedState(int payloadIdx0, bool isDropped)
        {
            if (payloadIdx0 < 0 || payloadIdx0 >= s_payloadDropped.Length) return;
            s_payloadDropped[payloadIdx0] = isDropped;
            PayloadDroppedStateChanged?.Invoke(payloadIdx0, isDropped);
        }


        // ============================================================
        // Construction
        // ============================================================

        public PayloadControlPanel(NOMADConfig config)
        {
            _config = config;
            InitializeUI();
            CameraTiltChanged    += OnCameraTiltChangedExternally;
            AutonomousModeChanged += OnAutonomousModeChanged;
            PayloadDroppedStateChanged += OnPayloadDroppedStateChanged;
            this.Disposed += (s, e) =>
            {
                CameraTiltChanged    -= OnCameraTiltChangedExternally;
                AutonomousModeChanged -= OnAutonomousModeChanged;
                PayloadDroppedStateChanged -= OnPayloadDroppedStateChanged;
                CleanupTimers();
            };
            ApplyTiltPulseQuietly(s_lastTiltPulseUs);
            // Seed each drop button's visual from the shared state so a panel
            // opened after a drop already happened shows the correct label.
            for (int i = 0; i < _dropDropped.Length; i++)
                ApplyDropVisual(i, s_payloadDropped[i]);
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
            x += 92;

            CreateFullReelButton(0, x, y);
            x += 80;
            CreateFullReelButton(1, x, y);

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
            x += 92;

            CreateFullReelButton(2, x, y);
            x += 80;
            CreateFullReelButton(3, x, y);

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
            this.MinimumSize = new Size(470, y + 20);
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
                RaisePayloadDroppedState(idx, true);
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
            RaisePayloadDroppedState(idx, false);

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
            StopFullReel(reelIdx * 2, true);
            StopFullReel(reelIdx * 2 + 1, true);

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

        private void CreateFullReelButton(int slot, int x, int y)
        {
            if (slot < 0 || slot >= _fullReelButtons.Length) return;

            var btn = MakeButton(_fullReelLabels[slot], Color.FromArgb(85, 65, 35), 76, 26);
            btn.Location = new Point(x, y);
            btn.Click += (s, e) => OnFullReelClick(slot);
            Controls.Add(btn);
            _fullReelButtons[slot] = btn;
        }

        private void OnFullReelClick(int slot)
        {
            if (slot < 0 || slot >= _fullReelButtons.Length) return;

            if (_fullReelActive[slot])
            {
                StopFullReel(slot, true);
                return;
            }

            _fullReelClickCount[slot]++;
            if (_fullReelClickCount[slot] < FULL_REEL_CLICKS_REQUIRED)
            {
                StartFullReelClickReset(slot);
                UpdateFullReelButton(slot);
                SetStatus(
                    $"{_fullReelLabels[slot]} P{FullReelNumber(slot)} armed: {_fullReelClickCount[slot]}/{FULL_REEL_CLICKS_REQUIRED}",
                    WARNING_COLOR);
                return;
            }

            StartFullReel(slot);
        }

        private void StartFullReelClickReset(int slot)
        {
            _fullReelClickReset[slot]?.Stop();
            _fullReelClickReset[slot]?.Dispose();

            var resetTimer = new Timer { Interval = FULL_REEL_CLICK_RESET_MS };
            resetTimer.Tick += (s, e) =>
            {
                resetTimer.Stop();
                resetTimer.Dispose();
                _fullReelClickReset[slot] = null;
                _fullReelClickCount[slot] = 0;
                UpdateFullReelButton(slot);
            };
            _fullReelClickReset[slot] = resetTimer;
            resetTimer.Start();
        }

        private void StartFullReel(int slot)
        {
            int reelIdx = FullReelIndex(slot);
            int oppositeSlot = FullReelOppositeSlot(slot);
            int channel = reelIdx == 0 ? (_config?.ReelServoChannel ?? 0) : (_config?.Reel2ServoChannel ?? 0);

            if (channel <= 0)
            {
                _fullReelClickReset[slot]?.Stop();
                _fullReelClickReset[slot]?.Dispose();
                _fullReelClickReset[slot] = null;
                _fullReelClickCount[slot] = 0;
                UpdateFullReelButton(slot);
                SetStatus($"Reel P{reelIdx + 1} channel not configured (see Settings > Servos)", WARNING_COLOR);
                return;
            }

            if (_fullReelActive[oppositeSlot])
                StopFullReel(oppositeSlot, false);
            if (_reelActive[reelIdx])
                StopReel(reelIdx);

            _fullReelClickReset[slot]?.Stop();
            _fullReelClickReset[slot]?.Dispose();
            _fullReelClickReset[slot] = null;
            _fullReelClickCount[slot] = 0;

            _fullReelActive[slot] = true;
            _fullReelRemainingMs[slot] = FULL_REEL_DURATION_MS;
            UpdateFullReelButton(slot);

            int pwmUs = FullReelIsIn(slot)
                ? (reelIdx == 0 ? (_config?.ReelPwmIn ?? 2100) : (_config?.Reel2PwmIn ?? 2100))
                : (reelIdx == 0 ? (_config?.ReelPwmOut ?? 900) : (_config?.Reel2PwmOut ?? 900));

            SendServoNow(channel, pwmUs);
            SetStatus($"{_fullReelLabels[slot]} P{reelIdx + 1} running for 1:20  (click to cancel)", SUCCESS_COLOR);

            _fullReelCountdown[slot]?.Stop();
            _fullReelCountdown[slot]?.Dispose();
            var countdown = new Timer { Interval = 1000 };
            countdown.Tick += (s, e) =>
            {
                _fullReelRemainingMs[slot] -= 1000;
                if (_fullReelRemainingMs[slot] <= 0)
                {
                    StopFullReel(slot, false);
                    return;
                }
                UpdateFullReelButton(slot);
            };
            _fullReelCountdown[slot] = countdown;
            countdown.Start();
        }

        private void StopFullReel(int slot, bool cancelled)
        {
            if (slot < 0 || slot >= _fullReelButtons.Length) return;

            bool wasActive = _fullReelActive[slot];
            int reelIdx = FullReelIndex(slot);

            _fullReelActive[slot] = false;
            _fullReelRemainingMs[slot] = 0;
            _fullReelClickCount[slot] = 0;

            _fullReelClickReset[slot]?.Stop();
            _fullReelClickReset[slot]?.Dispose();
            _fullReelClickReset[slot] = null;

            _fullReelCountdown[slot]?.Stop();
            _fullReelCountdown[slot]?.Dispose();
            _fullReelCountdown[slot] = null;

            UpdateFullReelButton(slot);

            if (!wasActive) return;

            int channel = reelIdx == 0 ? (_config?.ReelServoChannel ?? 0) : (_config?.Reel2ServoChannel ?? 0);
            SendServoNow(channel, 1500);
            SetStatus(
                cancelled ? $"{_fullReelLabels[slot]} P{reelIdx + 1} cancelled" : $"{_fullReelLabels[slot]} P{reelIdx + 1} complete",
                cancelled ? WARNING_COLOR : SUCCESS_COLOR);
        }

        private void UpdateFullReelButton(int slot)
        {
            var btn = _fullReelButtons[slot];
            if (btn == null || btn.IsDisposed) return;

            if (_fullReelActive[slot])
            {
                int secondsRemaining = Math.Max(0, _fullReelRemainingMs[slot] / 1000);
                btn.Text = $"{FullReelShortLabel(slot)} {secondsRemaining}s";
                btn.BackColor = WARNING_COLOR;
                return;
            }

            btn.Text = _fullReelClickCount[slot] > 0
                ? $"{FullReelShortLabel(slot)} {_fullReelClickCount[slot]}/{FULL_REEL_CLICKS_REQUIRED}"
                : _fullReelLabels[slot];
            btn.BackColor = _fullReelClickCount[slot] > 0 ? Color.FromArgb(180, 95, 25) : Color.FromArgb(85, 65, 35);
        }

        private static int FullReelIndex(int slot) => slot / 2;
        private static int FullReelNumber(int slot) => FullReelIndex(slot) + 1;
        private static int FullReelOppositeSlot(int slot) => FullReelIndex(slot) * 2 + (FullReelIsIn(slot) ? 1 : 0);
        private static bool FullReelIsIn(int slot) => slot % 2 == 0;
        private static string FullReelShortLabel(int slot) => FullReelIsIn(slot) ? "In" : "Out";

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

        private void OnPayloadDroppedStateChanged(int payloadIdx0, bool isDropped)
        {
            if (IsDisposed) return;
            if (InvokeRequired) { BeginInvoke(new Action(() => OnPayloadDroppedStateChanged(payloadIdx0, isDropped))); return; }
            ApplyDropVisual(payloadIdx0, isDropped);
        }

        private void ApplyDropVisual(int payloadIdx0, bool isDropped)
        {
            if (payloadIdx0 < 0 || payloadIdx0 >= _dropButtons.Length) return;
            var btn = _dropButtons[payloadIdx0];
            if (btn == null) return;

            _dropDropped[payloadIdx0] = isDropped;
            // Cancel any in-progress click-arming on this button — the state
            // just got reset externally, so a half-armed sequence is stale.
            _dropResetTimers[payloadIdx0]?.Stop();
            _dropResetTimers[payloadIdx0]?.Dispose();
            _dropResetTimers[payloadIdx0] = null;
            _dropClickCount[payloadIdx0] = 0;

            int payloadNumber = payloadIdx0 + 1;
            if (isDropped)
            {
                btn.Text = $"Retract P{payloadNumber}";
                btn.BackColor = DROP_COLOR_DROPPED;
            }
            else
            {
                btn.Text = $"Drop P{payloadNumber}";
                btn.BackColor = DROP_COLOR_IDLE;
            }
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
            for (int i = 0; i < _fullReelClickReset.Length; i++)
            {
                _fullReelClickReset[i]?.Stop();
                _fullReelClickReset[i]?.Dispose();
                _fullReelClickReset[i] = null;

                _fullReelCountdown[i]?.Stop();
                _fullReelCountdown[i]?.Dispose();
                _fullReelCountdown[i] = null;
                _fullReelActive[i] = false;
                _fullReelClickCount[i] = 0;
            }
            _tiltDebounceTimer?.Stop();
            _tiltDebounceTimer?.Dispose();
            _tiltDebounceTimer = null;
        }
    }
}
