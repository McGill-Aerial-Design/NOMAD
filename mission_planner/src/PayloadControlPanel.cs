// ============================================================
// NOMAD Payload Control Panel - Reusable Component
// ============================================================
// Provides payload drop (GPIO relay), water shooter, and nozzle 
// servo controls. Used by both NOMADVideoView and NOMADTask1View.
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Standalone payload controls panel: payload drop, water shooter, nozzle servo.
    /// </summary>
    public class PayloadControlPanel : UserControl
    {
        private static readonly Color CARD_BG = NOMADTheme.CARD_BG;
        private static readonly Color TEXT_PRIMARY = NOMADTheme.TEXT_PRIMARY;
        private static readonly Color TEXT_SECONDARY = NOMADTheme.TEXT_SECONDARY;
        private static readonly Color SUCCESS_COLOR = NOMADTheme.SUCCESS;
        private static readonly Color WARNING_COLOR = NOMADTheme.WARNING;
        private static readonly Color ERROR_COLOR = NOMADTheme.ERROR;
        private static readonly Color ACCENT_COLOR = NOMADTheme.ACCENT;

        private readonly NOMADConfig _config;
        private TrackBar _nozzleServoSlider;
        private Label _lblNozzleValue;
        private Label _lblStatus;

        private int GPIO_PAYLOAD1_PIN => _config?.GpioPayload1Pin ?? -1;
        private int GPIO_PAYLOAD2_PIN => _config?.GpioPayload2Pin ?? -1;

        public PayloadControlPanel(NOMADConfig config)
        {
            _config = config;
            InitializeUI();
        }

        private void InitializeUI()
        {
            BackColor = CARD_BG;
            Dock = DockStyle.Fill;
            Padding = new Padding(10, 5, 10, 5);

            // Title
            var titleLabel = new Label
            {
                Text = "PAYLOAD CONTROLS",
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(10, 5),
                AutoSize = true,
            };
            Controls.Add(titleLabel);

            int x = 10;
            int y = 28;

            // Drop Payload 1
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
            Controls.Add(btnPayload1);
            x += 115;

            // Drop Payload 2
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
            Controls.Add(btnPayload2);
            x += 115;

            // Shoot Water
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
            Controls.Add(btnWater);

            // Second row: Nozzle servo
            y += 35;
            x = 10;

            var lblNozzle = new Label
            {
                Text = "Nozzle Angle:",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(x, y + 5),
                AutoSize = true,
            };
            Controls.Add(lblNozzle);
            x += 85;

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
            Controls.Add(_nozzleServoSlider);
            x += 185;

            _lblNozzleValue = new Label
            {
                Text = "90 deg",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(x, y + 5),
                AutoSize = true,
            };
            Controls.Add(_lblNozzleValue);

            // Status label at bottom
            _lblStatus = new Label
            {
                Text = "",
                Font = new Font("Segoe UI", 8),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(10, y + 32),
                AutoSize = true,
            };
            Controls.Add(_lblStatus);
        }

        private void DropPayload(int payloadNumber)
        {
            try
            {
                int relayNumber = payloadNumber == 1 ? GPIO_PAYLOAD1_PIN : GPIO_PAYLOAD2_PIN;

                if (relayNumber < 0)
                {
                    SetStatus($"Payload {payloadNumber} GPIO not configured", WARNING_COLOR);
                    return;
                }

                if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
                {
                    SetStatus("Not connected", ERROR_COLOR);
                    return;
                }

                MainV2.comPort.doCommand(
                    MainV2.comPort.MAV.sysid,
                    MainV2.comPort.MAV.compid,
                    MAVLink.MAV_CMD.DO_SET_RELAY,
                    relayNumber, 1, 0, 0, 0, 0, 0);

                SetStatus($"Dropped Payload {payloadNumber}", SUCCESS_COLOR);
            }
            catch (Exception ex)
            {
                SetStatus($"Payload error: {ex.Message}", ERROR_COLOR);
            }
        }

        private async void ShootWater()
        {
            try
            {
                var jetsonIp = _config?.EffectiveIP;
                if (string.IsNullOrEmpty(jetsonIp))
                {
                    SetStatus("Jetson IP not configured", ERROR_COLOR);
                    return;
                }

                var response = await JetsonApiService.PostAsync(
                    "/api/servo/shooter/trigger?duration_ms=500");

                if (response.IsSuccessStatusCode)
                    SetStatus("Water pump triggered", SUCCESS_COLOR);
                else
                    SetStatus($"Water pump failed: {response.StatusCode}", ERROR_COLOR);
            }
            catch (Exception ex)
            {
                SetStatus($"Water pump error: {ex.Message}", ERROR_COLOR);
            }
        }

        private async void UpdateNozzleServo()
        {
            if (_nozzleServoSlider == null || _lblNozzleValue == null) return;

            int angle = _nozzleServoSlider.Value;
            _lblNozzleValue.Text = $"{angle} deg";

            try
            {
                var jetsonIp = _config?.EffectiveIP;
                if (string.IsNullOrEmpty(jetsonIp)) return;

                await JetsonApiService.PostAsync(
                    $"/api/servo/camera/tilt?angle={angle}");
            }
            catch { }
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
