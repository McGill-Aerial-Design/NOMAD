// ============================================================
// NOMAD Task 2 Payload Control Panel
// ============================================================
// Slim payload panel for Task 2: only camera tilt slider and
// water-pump trigger. No drop servos, no strap reel — Task 2
// has no payloads to drop.
//
// Camera tilt is shared with the main PayloadControlPanel via
// the same static CameraTiltChanged event so dragging here
// updates Task 1 panels too. Tilt lock is driven by Task 2's
// spray-state polling (see NOMADTask2View).
// ============================================================

using System;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using Timer = System.Windows.Forms.Timer;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    public class Task2PayloadPanel : UserControl
    {
        private static readonly Color CARD_BG       = NOMADTheme.CARD_BG;
        private static readonly Color TEXT_PRIMARY  = NOMADTheme.TEXT_PRIMARY;
        private static readonly Color TEXT_SECONDARY= NOMADTheme.TEXT_SECONDARY;
        private static readonly Color SUCCESS_COLOR = NOMADTheme.SUCCESS;
        private static readonly Color WARNING_COLOR = NOMADTheme.WARNING;
        private static readonly Color ERROR_COLOR   = NOMADTheme.ERROR;
        private static readonly Color ACCENT_COLOR  = NOMADTheme.ACCENT;

        private const int SERVO_PULSE_MIN_US = 500;
        private const int SERVO_PULSE_MAX_US = 2500;
        private const int TILT_SETTLE_MS     = 100;

        private readonly NOMADConfig _config;

        private TrackBar _tiltSlider;
        private Label    _lblTiltValue;
        private Button   _btnDown;
        private Button   _btnCenter;
        private Button   _btnUp;
        private Button   _btnWater;
        private Label    _lblStatus;
        private Timer    _tiltDebounceTimer;
        private bool     _suppressTiltEvent;
        private bool     _tiltLocked;

        private static readonly SemaphoreSlim s_mavlinkLock = new SemaphoreSlim(1, 1);

        public Task2PayloadPanel(NOMADConfig config)
        {
            _config = config;
            InitializeUI();
            // Tilt lock is driven by NOMADTask2View's spray-state poll calling
            // SetTiltLocked() directly — no event subscription needed here.
            this.Disposed += (s, e) =>
            {
                _tiltDebounceTimer?.Stop();
                _tiltDebounceTimer?.Dispose();
            };
        }

        /// <summary>Lock or unlock the tilt slider (driven externally by spray-state polling).</summary>
        public void SetTiltLocked(bool locked)
        {
            if (IsDisposed) return;
            if (InvokeRequired) { BeginInvoke(new Action(() => SetTiltLocked(locked))); return; }
            if (_tiltLocked == locked) return;
            _tiltLocked = locked;

            if (_tiltSlider != null)  _tiltSlider.Enabled  = !locked;
            if (_btnDown    != null)  _btnDown.Enabled     = !locked;
            if (_btnCenter  != null)  _btnCenter.Enabled   = !locked;
            if (_btnUp      != null)  _btnUp.Enabled       = !locked;
            if (_lblTiltValue != null)
                _lblTiltValue.ForeColor = locked ? TEXT_SECONDARY : TEXT_PRIMARY;

            SetStatus(
                locked ? "Camera tilt locked — autonomous spray active" : "Camera tilt unlocked",
                locked ? WARNING_COLOR : TEXT_SECONDARY);
        }

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

            const int H = 28;
            int y = 28;
            int x = 10;

            // ---- Row 1: Shoot Water ----
            _btnWater = MakeButton("Shoot Water", Color.FromArgb(30, 100, 180), 130, H);
            _btnWater.Location = new Point(x, y);
            _btnWater.Click += (s, e) => ShootWater();
            Controls.Add(_btnWater);

            // ---- Row 2: Cam tilt slider ----
            y += H + 10;
            x = 10;

            Controls.Add(new Label
            {
                Text = "Cam Tilt:",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(x, y + 4),
                AutoSize = true,
            });
            x += 70;

            int tiltMin     = _config?.CameraTiltPwmMin ?? 700;
            int tiltMax     = _config?.CameraTiltPwmMax ?? 1450;
            int tiltCenter  = _config?.CameraTiltPwmNeutral ?? 1250;

            _tiltSlider = new TrackBar
            {
                Location    = new Point(x, y - 2),
                Size        = new Size(180, 28),
                AutoSize    = false,
                TickStyle   = TickStyle.None,
                Minimum     = tiltMin,
                Maximum     = tiltMax,
                Value       = Math.Max(tiltMin, Math.Min(tiltMax, tiltCenter)),
                SmallChange = 10,
                LargeChange = 50,
                BackColor   = CARD_BG,
            };
            _tiltSlider.ValueChanged += (s, e) => OnTiltSliderChanged();
            Controls.Add(_tiltSlider);
            x += 185;

            _lblTiltValue = new Label
            {
                Text     = $"{tiltCenter} us",
                Font     = new Font("Segoe UI", 9),
                ForeColor= TEXT_PRIMARY,
                Location = new Point(x, y + 4),
                AutoSize = true,
            };
            Controls.Add(_lblTiltValue);
            x += 60;

            _btnDown   = MakeButton("▼", Color.FromArgb(60, 60, 70), 28, H);
            _btnCenter = MakeButton("●", Color.FromArgb(60, 60, 70), 28, H);
            _btnUp     = MakeButton("▲", Color.FromArgb(60, 60, 70), 28, H);
            _btnDown.Location   = new Point(x, y);   x += 32;
            _btnCenter.Location = new Point(x, y);   x += 32;
            _btnUp.Location     = new Point(x, y);
            _btnDown.Click   += (s, e) => ApplyTilt(tiltMin);
            _btnCenter.Click += (s, e) => ApplyTilt(tiltCenter);
            _btnUp.Click     += (s, e) => ApplyTilt(tiltMax);
            Controls.Add(_btnDown);
            Controls.Add(_btnCenter);
            Controls.Add(_btnUp);

            y += H + 8;
            _lblStatus = new Label
            {
                Text = "",
                Font = new Font("Segoe UI", 8),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(10, y),
                AutoSize = true,
            };
            Controls.Add(_lblStatus);

            this.MinimumSize = new Size(300, y + 24);
        }

        // ============================================================
        // Camera tilt
        // ============================================================
        private void OnTiltSliderChanged()
        {
            if (_tiltSlider == null || _lblTiltValue == null) return;
            int pulseUs = _tiltSlider.Value;
            _lblTiltValue.Text = $"{pulseUs} us";
            if (_suppressTiltEvent) return;

            SendCameraTiltAsync(pulseUs, tryOnly: true);

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

        private void ApplyTilt(int pulseUs)
        {
            if (_tiltSlider == null) return;
            int clamped = Math.Max(_tiltSlider.Minimum, Math.Min(_tiltSlider.Maximum, pulseUs));
            _suppressTiltEvent = true;
            try { _tiltSlider.Value = clamped; }
            finally { _suppressTiltEvent = false; }
            if (_lblTiltValue != null) _lblTiltValue.Text = $"{clamped} us";
            SendCameraTiltAsync(clamped, tryOnly: false);
        }

        private async void SendCameraTiltAsync(int pulseUs, bool tryOnly = false)
        {
            int channel = _config?.CameraTiltChannel ?? 0;
            if (TrySendServoMAVLink(channel, pulseUs, tryOnly)) return;

            try
            {
                if (string.IsNullOrEmpty(_config?.EffectiveIP)) return;
                double angle = (pulseUs - SERVO_PULSE_MIN_US) * 180.0
                             / (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
                angle = Math.Max(0, Math.Min(180, angle));
                await JetsonApiService.PostAsync(
                    $"/api/servo/camera/tilt?angle={angle.ToString("0.##", System.Globalization.CultureInfo.InvariantCulture)}");
            }
            catch { }
        }

        private bool TrySendServoMAVLink(int channel, int pwmUs, bool tryOnly = false)
        {
            if (channel <= 0) return false;
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return false;

            byte sysid  = MainV2.comPort.MAV.sysid;
            byte compid = MainV2.comPort.MAV.compid;

            Task.Run(async () =>
            {
                bool acquired = tryOnly
                    ? await s_mavlinkLock.WaitAsync(0).ConfigureAwait(false)
                    : await s_mavlinkLock.WaitAsync(5000).ConfigureAwait(false);
                if (!acquired) return;
                try
                {
                    await MainV2.comPort.doCommandAsync(
                        sysid, compid,
                        MAVLink.MAV_CMD.DO_SET_SERVO,
                        channel, pwmUs, 0, 0, 0, 0, 0,
                        requireack: false, uicallback: null).ConfigureAwait(false);
                }
                catch { }
                finally { s_mavlinkLock.Release(); }
            });
            return true;
        }

        // ============================================================
        // Water pump
        // ============================================================
        private async void ShootWater()
        {
            int relay      = _config?.WaterPumpRelayNumber ?? 0;
            int durationMs = _config?.WaterPumpDurationMs  ?? 500;

            if (TrySendRelayMAVLink(relay, true))
            {
                SetStatus($"Water pump firing  ({durationMs}ms)...", SUCCESS_COLOR);
                await Task.Delay(durationMs);
                TrySendRelayMAVLink(relay, false);
                SetStatus("Water pump done", SUCCESS_COLOR);
                return;
            }

            SetStatus("Triggering water pump via API...", WARNING_COLOR);
            try
            {
                var resp = await JetsonApiService.PostAsync(
                    $"/api/servo/shooter/trigger?duration_ms={durationMs}");
                SetStatus(resp.IsSuccessStatusCode
                    ? "Water pump triggered"
                    : $"Pump failed: HTTP {(int)resp.StatusCode}",
                    resp.IsSuccessStatusCode ? SUCCESS_COLOR : ERROR_COLOR);
            }
            catch (Exception ex)
            {
                SetStatus($"Pump error: {ex.Message}", ERROR_COLOR);
            }
        }

        private bool TrySendRelayMAVLink(int relayNumber, bool on)
        {
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return false;
            byte sysid  = MainV2.comPort.MAV.sysid;
            byte compid = MainV2.comPort.MAV.compid;
            Task.Run(async () =>
            {
                await s_mavlinkLock.WaitAsync().ConfigureAwait(false);
                try
                {
                    await MainV2.comPort.doCommandAsync(
                        sysid, compid,
                        MAVLink.MAV_CMD.DO_SET_RELAY,
                        relayNumber, on ? 1 : 0, 0, 0, 0, 0, 0,
                        requireack: true, uicallback: null).ConfigureAwait(false);
                }
                catch { }
                finally { s_mavlinkLock.Release(); }
            });
            return true;
        }

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
    }
}
