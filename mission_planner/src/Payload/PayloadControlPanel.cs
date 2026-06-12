// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Payload Control Panel - Reusable Component
// ============================================================
// Renders the configurable payloads from NOMADConfig.Payloads (drop servos,
// slider servos, relay/GPIO outputs) plus the dedicated strap-reel and ZED
// camera-tilt sections. Everything is wired to the Cube Orange and commanded via
// MAVLink DO_SET_SERVO / DO_SET_RELAY (primary), with Edge Core HTTP as fallback.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using Timer = System.Windows.Forms.Timer;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Payload controls: a configurable list of drop / slider / relay payloads,
    /// plus strap reels (hold-to-reel) and the ZED camera-tilt slider.
    /// MAVLink is tried first; Edge Core's Cube command API is the fallback.
    /// </summary>
    public partial class PayloadControlPanel : UserControl
    {
        private static readonly Color CARD_BG        = NOMADTheme.CARD_BG;
        private static readonly Color TEXT_PRIMARY   = NOMADTheme.TEXT_PRIMARY;
        private static readonly Color TEXT_SECONDARY = NOMADTheme.TEXT_SECONDARY;
        private static readonly Color SUCCESS_COLOR  = NOMADTheme.SUCCESS;
        private static readonly Color WARNING_COLOR  = NOMADTheme.WARNING;
        private static readonly Color ERROR_COLOR    = NOMADTheme.ERROR;
        private static readonly Color ACCENT_COLOR   = NOMADTheme.ACCENT;

        // Safety limits
        private const int REEL_SAFETY_MS = 10_000; // max continuous hold-to-reel time
        private const int TILT_SETTLE_MS = 100;    // final send after slider stops moving

        // Full-spool reel constants — three-click confirm to start, fourth
        // click (or any click after start) cancels mid-spool.
        private const int FULL_REEL_DURATION_MS     = 80_000; // 1 min 20 s
        private const int FULL_REEL_CLICKS_REQUIRED = 3;
        private const int FULL_REEL_CLICK_RESET_MS  = 3000;

        // Shared row geometry for the manually-laid-out sections.
        private const int ROW_H   = 26;
        private const int ROW_GAP = 5;

        private readonly NOMADConfig _config;

        // Reel safety (indexed: 0 = reel 1, 1 = reel 2)
        private readonly bool[]  _reelActive       = new bool[2];
        private readonly Timer[] _reelSafetyTimers = new Timer[2];

        // Full-spool reel state. Slot layout: 0 = reel-1 IN, 1 = reel-1 OUT,
        // 2 = reel-2 IN, 3 = reel-2 OUT. Each slot tracks its own arming
        // click count and its own active 80s countdown so the operator can
        // cancel mid-spool by clicking the running button again.
        private readonly Button[] _fullReelButtons     = new Button[4];
        private readonly string[] _fullReelLabels      = { "In Full", "Out Full", "In Full", "Out Full" };
        private readonly int[]    _fullReelClickCount   = new int[4];
        private readonly Timer[]  _fullReelClickReset   = new Timer[4];
        private readonly Timer[]  _fullReelCountdown    = new Timer[4];
        private readonly int[]    _fullReelRemainingMs  = new int[4];
        private readonly bool[]   _fullReelActive       = new bool[4];

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
        /// Autonomous mode fires true on entry, false on exit.
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
        // Construction
        // ============================================================

        public PayloadControlPanel(NOMADConfig config)
        {
            _config = config;
            InitializeUI();
            CameraTiltChanged          += OnCameraTiltChangedExternally;
            AutonomousModeChanged      += OnAutonomousModeChanged;
            PayloadDroppedStateChanged += OnPayloadDroppedStateChanged;
            this.Disposed += (s, e) =>
            {
                CameraTiltChanged          -= OnCameraTiltChangedExternally;
                AutonomousModeChanged      -= OnAutonomousModeChanged;
                PayloadDroppedStateChanged -= OnPayloadDroppedStateChanged;
                CleanupTimers();
            };
            ApplyTiltPulseQuietly(s_lastTiltPulseUs);
            // Seed each drop button's visual from the shared state so a panel
            // opened after a drop already happened shows the correct label.
            SeedDropVisuals();
        }

        // ============================================================
        // UI Construction
        // ============================================================

        private void InitializeUI()
        {
            BackColor = CARD_BG;
            Dock = DockStyle.Fill;
            Padding = new Padding(10, 5, 10, 5);
            AutoScroll = true;

            Controls.Add(new Label
            {
                Text = "PAYLOAD CONTROLS",
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(10, 5),
                AutoSize = true,
            });

            int y = 26;

            // Configurable payloads (drop / slider / relay) — see PayloadControlPanel.Drop.cs.
            BuildPayloadRows(ref y);

            // Dedicated strap-reel rows.
            BuildReelRow(0, ref y);
            BuildReelRow(1, ref y);

            // Dedicated ZED camera-tilt row.
            BuildTiltRow(ref y);

            // Status label — full width, below everything.
            _lblStatus = new Label
            {
                Text      = "",
                Font      = new Font("Segoe UI", 8),
                ForeColor = TEXT_SECONDARY,
                Location  = new Point(10, y),
                AutoSize  = true,
            };
            Controls.Add(_lblStatus);

            // Lock in the minimum height so the parent layout never clips content.
            this.MinimumSize = new Size(470, y + 24);
        }

        // ============================================================
        // Strap reel rows (UI)
        // ============================================================

        private void BuildReelRow(int reelIdx, ref int y)
        {
            int x = 10;
            Controls.Add(new Label
            {
                Text = $"Reel P{reelIdx + 1}:",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(x, y + 4),
                AutoSize = true,
            });
            x += 58;

            int pwmIn  = reelIdx == 0 ? (_config?.ReelPwmIn  ?? 2100) : (_config?.Reel2PwmIn  ?? 2100);
            int pwmOut = reelIdx == 0 ? (_config?.ReelPwmOut ?? 900)  : (_config?.Reel2PwmOut ?? 900);

            var btnIn = MakeButton("⬆ Reel In", Color.FromArgb(50, 100, 50), 85, ROW_H);
            btnIn.Location = new Point(x, y);
            btnIn.MouseDown  += (s, e) => StartReel(reelIdx, pwmIn);
            btnIn.MouseUp    += (s, e) => StopReel(reelIdx);
            btnIn.MouseLeave += (s, e) => { if (_reelActive[reelIdx]) StopReel(reelIdx); };
            Controls.Add(btnIn);
            x += 89;

            var btnOut = MakeButton("⬇ Reel Out", Color.FromArgb(50, 50, 100), 88, ROW_H);
            btnOut.Location = new Point(x, y);
            btnOut.MouseDown  += (s, e) => StartReel(reelIdx, pwmOut);
            btnOut.MouseUp    += (s, e) => StopReel(reelIdx);
            btnOut.MouseLeave += (s, e) => { if (_reelActive[reelIdx]) StopReel(reelIdx); };
            Controls.Add(btnOut);
            x += 92;

            CreateFullReelButton(reelIdx * 2, x, y);
            x += 80;
            CreateFullReelButton(reelIdx * 2 + 1, x, y);

            y += ROW_H + ROW_GAP;
        }

        // ============================================================
        // Camera tilt row (UI)
        // ============================================================

        private void BuildTiltRow(ref int y)
        {
            int x = 10;
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
                Location    = new Point(x, y - 2),
                Size        = new Size(170, ROW_H),
                AutoSize    = false,   // prevent TrackBar from growing over the status label
                TickStyle   = TickStyle.None,
                Minimum     = tiltMin,
                Maximum     = tiltMax,
                Value       = initialTilt,
                SmallChange = 10,
                LargeChange = 50,
                BackColor   = CARD_BG,
            };
            _tiltSlider.ValueChanged += (s, e) => OnTiltSliderChanged();
            Controls.Add(_tiltSlider);
            x += 175;

            _lblTiltValue = new Label
            {
                Text      = $"{initialTilt} us",
                Font      = new Font("Segoe UI", 9),
                ForeColor = TEXT_PRIMARY,
                Location  = new Point(x, y + 4),
                AutoSize  = true,
            };
            Controls.Add(_lblTiltValue);
            x += 60;

            int tiltCenter = _config?.CameraTiltPwmNeutral ?? 1250;
            foreach (var (label, value) in new (string, int)[] { ("▼", tiltMin), ("●", tiltCenter), ("▲", tiltMax) })
            {
                int v = value;
                var btn = MakeButton(label, Color.FromArgb(60, 60, 70), 26, ROW_H);
                btn.Location = new Point(x, y);
                btn.Font = new Font("Segoe UI", 8);
                btn.Click += (s, e) => ApplyTiltPulse(v);
                Controls.Add(btn);
                x += 30;
            }

            y += ROW_H + ROW_GAP;
        }

        // Strap-reel command logic lives in PayloadControlPanel.Reels.cs.


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

            UiAsync.RunSync(this, () => ApplyTiltPulseQuietly(pulseUs), "OnCameraTiltChangedExternally");
        }

        private void OnAutonomousModeChanged(bool isAutonomous)
        {
            if (IsDisposed) return;
            UiAsync.RunSync(this, () =>
            {
                if (_tiltSlider != null) _tiltSlider.Enabled = !isAutonomous;
                if (_lblTiltValue != null) _lblTiltValue.ForeColor = isAutonomous ? TEXT_SECONDARY : TEXT_PRIMARY;

                foreach (Control c in Controls)
                {
                    if (c is Button btn && (btn.Text == "▼" || btn.Text == "●" || btn.Text == "▲"))
                        btn.Enabled = !isAutonomous;
                }

                if (isAutonomous)
                    SetStatus("Camera tilt locked — autonomous mode active", WARNING_COLOR);
                else
                    SetStatus("Camera tilt restored", TEXT_SECONDARY);
            }, "OnAutonomousModeChanged");
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
            UiAsync.RunSync(this, () =>
            {
                if (_lblStatus != null) { _lblStatus.Text = text; _lblStatus.ForeColor = color; }
            }, "SetStatus");
        }

        private void CleanupTimers()
        {
            foreach (var t in _dropResetTimers.Values)
            {
                t?.Stop();
                t?.Dispose();
            }
            _dropResetTimers.Clear();

            foreach (var t in _sliderSettleTimers.Values)
            {
                t?.Stop();
                t?.Dispose();
            }
            _sliderSettleTimers.Clear();

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
