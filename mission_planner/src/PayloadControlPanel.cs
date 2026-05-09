// ============================================================
// NOMAD Payload Control Panel - Reusable Component
// ============================================================
// Drop servos, strap reel and camera tilt are all wired to the
// Cube Orange and commanded via MAVLink DO_SET_SERVO (primary).
// Jetson HTTP API is used as a fallback when MAVLink is absent.
// ============================================================

using System;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Payload controls: 3 × drop servo, strap reel (hold-to-reel),
    /// water pump trigger, and ZED camera tilt slider.
    /// MAVLink DO_SET_SERVO is tried first; Jetson API is the fallback.
    /// </summary>
    public class PayloadControlPanel : UserControl
    {
        private static readonly Color CARD_BG       = NOMADTheme.CARD_BG;
        private static readonly Color TEXT_PRIMARY  = NOMADTheme.TEXT_PRIMARY;
        private static readonly Color TEXT_SECONDARY= NOMADTheme.TEXT_SECONDARY;
        private static readonly Color SUCCESS_COLOR = NOMADTheme.SUCCESS;
        private static readonly Color WARNING_COLOR = NOMADTheme.WARNING;
        private static readonly Color ERROR_COLOR   = NOMADTheme.ERROR;
        private static readonly Color ACCENT_COLOR  = NOMADTheme.ACCENT;

        // Used only for the Jetson API angle-conversion fallback.
        private const int SERVO_PULSE_MIN_US = 500;
        private const int SERVO_PULSE_MAX_US = 2500;

        private readonly NOMADConfig _config;
        private TrackBar _tiltSlider;
        private Label    _lblTiltValue;
        private Label    _lblStatus;
        private bool     _suppressTiltEvent;

        // Shared tilt state so multiple panel instances stay in sync
        // without each independently pushing stale setpoints.
        private static int s_lastTiltPulseUs = 1000;
        private static event Action<int, PayloadControlPanel> CameraTiltChanged;

        public PayloadControlPanel(NOMADConfig config)
        {
            _config = config;
            InitializeUI();
            CameraTiltChanged += OnCameraTiltChangedExternally;
            this.Disposed += (s, e) => CameraTiltChanged -= OnCameraTiltChangedExternally;
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

            var titleLabel = new Label
            {
                Text = "PAYLOAD CONTROLS",
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(10, 5),
                AutoSize = true,
            };
            Controls.Add(titleLabel);

            // ---- Row 1: Drop buttons + water pump ----
            int x = 10, y = 28;
            const int DROP_W = 75, DROP_H = 26;

            var btnDrop1 = MakeButton("Drop P1", Color.FromArgb(140, 60, 20), DROP_W, DROP_H);
            btnDrop1.Location = new Point(x, y);
            btnDrop1.Click += (s, e) => DropPayload(1);
            Controls.Add(btnDrop1);
            x += DROP_W + 4;

            var btnDrop2 = MakeButton("Drop P2", Color.FromArgb(140, 60, 20), DROP_W, DROP_H);
            btnDrop2.Location = new Point(x, y);
            btnDrop2.Click += (s, e) => DropPayload(2);
            Controls.Add(btnDrop2);
            x += DROP_W + 4;

            var btnDrop3 = MakeButton("Drop P3", Color.FromArgb(140, 60, 20), DROP_W, DROP_H);
            btnDrop3.Location = new Point(x, y);
            btnDrop3.Click += (s, e) => DropPayload(3);
            Controls.Add(btnDrop3);
            x += DROP_W + 8;

            var btnWater = MakeButton("Shoot Water", Color.FromArgb(30, 100, 180), 90, DROP_H);
            btnWater.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            btnWater.Location = new Point(x, y);
            btnWater.Click += (s, e) => ShootWater();
            Controls.Add(btnWater);

            // ---- Row 2: Strap reel (hold-to-reel) ----
            y += DROP_H + 7;
            x = 10;

            var lblReel = new Label
            {
                Text = "Reel P1:",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(x, y + 4),
                AutoSize = true,
            };
            Controls.Add(lblReel);
            x += 58;

            var btnReelIn = MakeButton("⬆ Reel In", Color.FromArgb(60, 110, 60), 85, DROP_H);
            btnReelIn.Location = new Point(x, y);
            btnReelIn.MouseDown += (s, e) => SendReelAsync(_config?.ReelPwmIn ?? 2100);
            btnReelIn.MouseUp   += (s, e) => SendReelAsync(1500);
            btnReelIn.MouseLeave+= (s, e) => SendReelAsync(1500); // safety: stop if cursor leaves
            Controls.Add(btnReelIn);
            x += 89;

            var btnReelOut = MakeButton("⬇ Reel Out", Color.FromArgb(60, 60, 110), 88, DROP_H);
            btnReelOut.Location = new Point(x, y);
            btnReelOut.MouseDown += (s, e) => SendReelAsync(_config?.ReelPwmOut ?? 900);
            btnReelOut.MouseUp   += (s, e) => SendReelAsync(1500);
            btnReelOut.MouseLeave+= (s, e) => SendReelAsync(1500);
            Controls.Add(btnReelOut);

            // ---- Row 3: Camera tilt slider ----
            y += DROP_H + 7;
            x = 10;

            var lblTilt = new Label
            {
                Text = "Cam Tilt:",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(x, y + 5),
                AutoSize = true,
            };
            Controls.Add(lblTilt);
            x += 72;

            int tiltMin = _config?.CameraTiltPwmMin ?? 700;
            int tiltMax = _config?.CameraTiltPwmMax ?? 1450;
            int initialTilt = Math.Max(tiltMin, Math.Min(tiltMax, s_lastTiltPulseUs));

            _tiltSlider = new TrackBar
            {
                Location = new Point(x, y),
                Size = new Size(170, 28),
                Minimum = tiltMin,
                Maximum = tiltMax,
                Value = initialTilt,
                TickFrequency = 50,
                SmallChange = 10,
                LargeChange = 50,
                BackColor = CARD_BG,
            };
            _tiltSlider.ValueChanged += (s, e) => OnTiltSliderChanged();
            Controls.Add(_tiltSlider);
            x += 175;

            _lblTiltValue = new Label
            {
                Text = $"{initialTilt} us",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(x, y + 5),
                AutoSize = true,
            };
            Controls.Add(_lblTiltValue);

            // ---- Status label ----
            y += DROP_H + 7;
            _lblStatus = new Label
            {
                Text = "",
                Font = new Font("Segoe UI", 8),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(10, y),
                AutoSize = true,
            };
            Controls.Add(_lblStatus);
        }

        // ============================================================
        // MAVLink servo helper
        // ============================================================

        /// <summary>
        /// Send a DO_SET_SERVO command via MAVLink.
        /// Returns true if the command was dispatched successfully.
        /// </summary>
        private bool TrySendServoMAVLink(int channel, int pwmUs)
        {
            try
            {
                if (channel <= 0) return false;
                if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
                    return false;

                MainV2.comPort.doCommand(
                    MainV2.comPort.MAV.sysid,
                    MainV2.comPort.MAV.compid,
                    MAVLink.MAV_CMD.DO_SET_SERVO,
                    channel, pwmUs, 0, 0, 0, 0, 0);

                return true;
            }
            catch
            {
                return false;
            }
        }

        // ============================================================
        // Drop payload
        // ============================================================

        private async void DropPayload(int payloadNumber)
        {
            int channel, pwmDrop;
            switch (payloadNumber)
            {
                case 1: channel = _config?.Servo1Channel ?? 0; pwmDrop = _config?.Servo1PwmMax ?? 2000; break;
                case 2: channel = _config?.Servo2Channel ?? 0; pwmDrop = _config?.Servo2PwmMax ?? 2000; break;
                case 3: channel = _config?.Servo3Channel ?? 0; pwmDrop = _config?.Servo3PwmMax ?? 2000; break;
                default: return;
            }

            if (channel <= 0)
            {
                SetStatus($"Payload {payloadNumber} servo channel not configured", WARNING_COLOR);
                return;
            }

            // Primary: MAVLink
            if (TrySendServoMAVLink(channel, pwmDrop))
            {
                SetStatus($"Payload {payloadNumber} dropped (MAVLink ch{channel} {pwmDrop}us)", SUCCESS_COLOR);
                return;
            }

            // Fallback: Jetson API
            SetStatus($"MAVLink unavailable — trying API fallback...", WARNING_COLOR);
            try
            {
                var resp = await JetsonApiService.PostAsync(
                    $"/api/servo/channel/{channel}/pwm?pwm={pwmDrop}");
                SetStatus(resp.IsSuccessStatusCode
                    ? $"Payload {payloadNumber} dropped (API ch{channel})"
                    : $"Drop failed: HTTP {(int)resp.StatusCode}",
                    resp.IsSuccessStatusCode ? SUCCESS_COLOR : ERROR_COLOR);
            }
            catch (Exception ex)
            {
                SetStatus($"Drop failed: {ex.Message}", ERROR_COLOR);
            }
        }

        // ============================================================
        // Strap reel
        // ============================================================

        private async void SendReelAsync(int pwmUs)
        {
            int channel = _config?.ReelServoChannel ?? 0;
            if (channel <= 0) return;

            if (TrySendServoMAVLink(channel, pwmUs))
            {
                SetStatus(pwmUs == 1500
                    ? "Reel stopped"
                    : $"Reeling ({pwmUs}us)", pwmUs == 1500 ? TEXT_SECONDARY : SUCCESS_COLOR);
                return;
            }

            // Fallback: Jetson API (fire-and-forget — don't block the MouseUp handler)
            try
            {
                await JetsonApiService.PostAsync(
                    $"/api/servo/channel/{channel}/pwm?pwm={pwmUs}");
            }
            catch { }
        }

        // ============================================================
        // Water pump
        // ============================================================

        private async void ShootWater()
        {
            int channel  = _config?.WaterPumpChannel  ?? 0;
            int pwmOn    = _config?.WaterPumpPwmOn    ?? 2000;
            int pwmOff   = _config?.WaterPumpPwmOff   ?? 1000;
            int durationMs = _config?.WaterPumpDurationMs ?? 500;

            // Primary: MAVLink pulse
            if (channel > 0 && TrySendServoMAVLink(channel, pwmOn))
            {
                SetStatus($"Water pump firing ({durationMs}ms)...", SUCCESS_COLOR);
                await Task.Delay(durationMs);
                TrySendServoMAVLink(channel, pwmOff);
                SetStatus("Water pump done", SUCCESS_COLOR);
                return;
            }

            // Fallback: Jetson API
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

        // ============================================================
        // Camera tilt
        // ============================================================

        private void OnTiltSliderChanged()
        {
            if (_tiltSlider == null || _lblTiltValue == null) return;

            int pulseUs = _tiltSlider.Value;
            _lblTiltValue.Text = $"{pulseUs} us";

            if (_suppressTiltEvent) return;

            // Sync all other panel instances to this setpoint.
            s_lastTiltPulseUs = pulseUs;
            CameraTiltChanged?.Invoke(pulseUs, this);

            SendCameraTiltAsync(pulseUs);
        }

        private async void SendCameraTiltAsync(int pulseUs)
        {
            int channel = _config?.CameraTiltChannel ?? 0;

            // Primary: MAVLink
            if (TrySendServoMAVLink(channel, pulseUs))
                return;

            // Fallback: Jetson API (angle-based endpoint)
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

        private void OnCameraTiltChangedExternally(int pulseUs, PayloadControlPanel source)
        {
            if (source == this) return;
            if (IsDisposed || _tiltSlider == null) return;

            if (InvokeRequired)
            {
                BeginInvoke(new Action(() => ApplyTiltPulseQuietly(pulseUs)));
                return;
            }
            ApplyTiltPulseQuietly(pulseUs);
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
            if (InvokeRequired)
            {
                BeginInvoke(new Action(() => SetStatus(text, color)));
                return;
            }
            if (_lblStatus != null)
            {
                _lblStatus.Text = text;
                _lblStatus.ForeColor = color;
            }
        }
    }
}
