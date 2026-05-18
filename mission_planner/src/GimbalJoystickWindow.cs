// ============================================================
// NOMAD Caddx Gimbal Joystick — Floating Dockable Window
// ============================================================
// Rate-controlled 2D joystick that streams MAV_CMD_DO_MOUNT_CONTROL
// angle commands (pitch/roll) to the Caddx brushless gimbal mount on
// the Cube Orange. Mode buttons send MAV_CMD_DO_MOUNT_CONFIGURE.
//
// This is independent from the ZED tilt servo (PayloadControlPanel),
// which is just a SERVOx output. The Caddx is configured as a real
// MNTx_* mount on ArduPilot.
// ============================================================

using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using Timer = System.Windows.Forms.Timer;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Floating window with a 2D rate joystick + mode buttons for the Caddx gimbal.
    /// Open with <see cref="ShowSingleton"/>; only one instance lives at a time.
    /// </summary>
    public class GimbalJoystickWindow : Form
    {
        private static GimbalJoystickWindow s_instance;

        public static void ShowSingleton(NOMADConfig config, IWin32Window owner = null)
        {
            if (s_instance != null && !s_instance.IsDisposed)
            {
                if (s_instance.WindowState == FormWindowState.Minimized)
                    s_instance.WindowState = FormWindowState.Normal;
                s_instance.BringToFront();
                s_instance.Activate();
                return;
            }
            s_instance = new GimbalJoystickWindow(config);
            if (owner != null) s_instance.Show(owner); else s_instance.Show();
        }

        // ============================================================
        // Tunables
        // ============================================================
        private const int   STREAM_HZ          = 20;
        private const float DEFAULT_MAX_RATE   = 60f;   // deg/sec at full stick
        private const float KEY_NUDGE_DEG      = 2.0f;
        // Mount limits pulled from GimbalController so both this window and the
        // physical NomadJoystickService stay in sync if the limits ever change.
        private const float PITCH_MIN_DEG      = GimbalController.PITCH_MIN_DEG;
        private const float PITCH_MAX_DEG      = GimbalController.PITCH_MAX_DEG;
        private const float ROLL_MIN_DEG       = GimbalController.ROLL_MIN_DEG;
        private const float ROLL_MAX_DEG       = GimbalController.ROLL_MAX_DEG;
        private const float STICK_DEADZONE     = 0.06f;

        // ============================================================
        // State
        // ============================================================
        private readonly NOMADConfig _config;

        // Stick: normalized [-1,1] x = roll rate, y = pitch rate (up = +pitch).
        private float _stickX, _stickY;
        // Local mirror of GimbalController target so display label code can read
        // without crossing threads. Authoritative state lives in GimbalController.
        private float _targetPitch, _targetRoll;
        private float _maxRateDegSec = DEFAULT_MAX_RATE;
        // Mount mode currently selected for the Caddx mount — mirrors GimbalController.
        private MountMode _mountMode = MountMode.MavlinkTargeting;
        private string _modeLabel = "MAVLINK";

        // Local alias kept for readability; the canonical type lives on GimbalController.
        private MountMode MapMode(GimbalController.MountMode m) => (MountMode)(int)m;
        private GimbalController.MountMode ToCtrl(MountMode m) => (GimbalController.MountMode)(int)m;

        private enum MountMode
        {
            Retract = 0,
            Neutral = 1,
            MavlinkTargeting = 2,
            RcTargeting = 3,
        }

        // UI
        private JoystickPad _pad;
        private Label _lblPitch, _lblRoll, _lblMode, _lblStatus, _lblRate;
        private TrackBar _trkRate;
        private Panel _ratePanel;
        private Button _btnRetract, _btnNeutral, _btnRcTgt, _btnMavTgt, _btnCenter, _btnLevel;
        private CheckBox _chkTopMost;
        private Timer _streamTimer;

        private GimbalJoystickWindow(NOMADConfig config)
        {
            _config = config;
            Text = "Caddx Gimbal Joystick";
            StartPosition = FormStartPosition.Manual;
            FormBorderStyle = FormBorderStyle.SizableToolWindow;
            BackColor = NOMADTheme.BG_DARK;
            ForeColor = NOMADTheme.TEXT_PRIMARY;
            Font = new Font("Segoe UI", 9);
            MinimumSize = new Size(360, 460);
            ClientSize = new Size(380, 500);
            ShowInTaskbar = false;
            KeyPreview = true;
            KeyDown += OnWindowKeyDown;

            // Position near top-right corner of screen
            try
            {
                var wa = Screen.PrimaryScreen.WorkingArea;
                Location = new Point(wa.Right - ClientSize.Width - 40, wa.Top + 80);
            }
            catch { }

            BuildUi();

            // Seed local mirror from any prior integrator state so reopening the
            // window doesn't snap the readout back to 0.
            _targetPitch = GimbalController.TargetPitchDeg;
            _targetRoll  = GimbalController.TargetRollDeg;

            _streamTimer = new Timer { Interval = 1000 / STREAM_HZ };
            _streamTimer.Tick += OnStreamTick;
            _streamTimer.Start();

            // Keep our readout in sync when the physical joystick service moves
            // the target while this window is open.
            GimbalController.TargetChanged += OnExternalTargetChanged;

            // Default to MAVLink targeting so the first joystick or keyboard input
            // immediately drives the mount.
            GimbalController.SetMode(ToCtrl(_mountMode));
            UpdateModeLabel();

            FormClosed += (s, e) =>
            {
                _streamTimer?.Stop();
                _streamTimer?.Dispose();
                _streamTimer = null;
                GimbalController.TargetChanged -= OnExternalTargetChanged;
                s_instance = null;
            };
        }

        private void OnExternalTargetChanged(float pitch, float roll)
        {
            if (IsDisposed) return;
            if (InvokeRequired) { BeginInvoke(new Action(() => OnExternalTargetChanged(pitch, roll))); return; }
            _targetPitch = pitch;
            _targetRoll = roll;
            if (_lblPitch != null) _lblPitch.Text = $"Pitch: {_targetPitch,+6:0.0}°";
            if (_lblRoll  != null) _lblRoll.Text  = $"Roll:  {_targetRoll,+6:0.0}°";
        }

        // ============================================================
        // UI
        // ============================================================
        private void BuildUi()
        {
            var title = new Label
            {
                Text = "CADDX GIMBAL — Rate Joystick",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                Location = new Point(12, 8),
                AutoSize = true,
            };
            Controls.Add(title);

            // Joystick pad
            _pad = new JoystickPad
            {
                Location = new Point(40, 36),
                Size = new Size(280, 220),
                BackColor = NOMADTheme.CARD_BG,
            };
            _pad.StickChanged += (x, y) => { _stickX = x; _stickY = y; };
            Controls.Add(_pad);

            // Target readouts
            _lblPitch = MakeReadout("Pitch:  +0.0°", new Point(20, 268));
            _lblRoll  = MakeReadout("Roll:   +0.0°", new Point(190, 268));
            _lblMode  = MakeReadout("Mode: MAVLINK", new Point(20, 290));
            Controls.Add(_lblPitch); Controls.Add(_lblRoll); Controls.Add(_lblMode);

            // Rate slider
            _ratePanel = new Panel
            {
                Location = new Point(12, 314),
                Size = new Size(356, 48),
                BackColor = Color.FromArgb(50, 50, 58),
            };
            _ratePanel.SendToBack();
            Controls.Add(_ratePanel);

            _ratePanel.Controls.Add(new Label
            {
                Text = "Max Rate (deg/s):",
                Location = new Point(8, 9),
                AutoSize = true,
                ForeColor = NOMADTheme.TEXT_SECONDARY,
            });
            _trkRate = new TrackBar
            {
                Location = new Point(126, 3),
                Size = new Size(140, 30),
                Minimum = 10, Maximum = 180,
                Value = (int)DEFAULT_MAX_RATE,
                TickStyle = TickStyle.None,
                BackColor = Color.FromArgb(50, 50, 58),
            };
            _trkRate.ValueChanged += (s, e) =>
            {
                _maxRateDegSec = _trkRate.Value;
                _lblRate.Text = $"{_trkRate.Value}";
            };
            _ratePanel.Controls.Add(_trkRate);
            _trkRate.SendToBack();
            _lblRate = new Label
            {
                Text = $"{_trkRate.Value}",
                Location = new Point(270, 9),
                AutoSize = true,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
            };
            _ratePanel.Controls.Add(_lblRate);

            // Mode buttons row 1: control modes (gimbal-manager flag presets)
            int y = 354;
            _btnMavTgt  = MakeButton("MAVLink Tgt", new Point(12,  y), 95, c => SetModePreset("MAVLINK", MountMode.MavlinkTargeting));
            _btnRcTgt   = MakeButton("RC Tgt",      new Point(112, y), 80, c => SetModePreset("RC", MountMode.RcTargeting));
            _btnNeutral = MakeButton("Neutral",     new Point(197, y), 80, c => SetModePreset("NEUTRAL", MountMode.Neutral));
            _btnRetract = MakeButton("Retract",     new Point(282, y), 80, c => SetModePreset("RETRACT", MountMode.Retract));
            Controls.Add(_btnMavTgt); Controls.Add(_btnRcTgt); Controls.Add(_btnNeutral); Controls.Add(_btnRetract);

            // Quick angle presets
            y = 388;
            _btnCenter = MakeButton("Center (0°/0°)",   new Point(12, y), 130, c => SnapAngles(0, 0));
            _btnLevel  = MakeButton("Look Down (-90°)", new Point(150, y), 130, c => SnapAngles(-90, _targetRoll));
            Controls.Add(_btnCenter); Controls.Add(_btnLevel);

            _chkTopMost = new CheckBox
            {
                Text = "Always on top",
                Location = new Point(285, y + 4),
                AutoSize = true,
                ForeColor = NOMADTheme.TEXT_SECONDARY,
            };
            _chkTopMost.CheckedChanged += (s, e) => TopMost = _chkTopMost.Checked;
            Controls.Add(_chkTopMost);

            _lblStatus = new Label
            {
                Text = "Ready",
                Location = new Point(12, 430),
                AutoSize = true,
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Font = new Font("Segoe UI", 8),
            };
            Controls.Add(_lblStatus);

            // Hint
            Controls.Add(new Label
            {
                Text = "Drag pad: pitch (Y) / roll (X). Arrow keys nudge both axes. Release = stop.",
                Location = new Point(12, 452),
                AutoSize = true,
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Font = new Font("Segoe UI", 8, FontStyle.Italic),
            });
        }

        private Label MakeReadout(string text, Point loc) => new Label
        {
            Text = text, Location = loc, AutoSize = true,
            Font = new Font("Consolas", 10, FontStyle.Bold),
            ForeColor = NOMADTheme.TEXT_PRIMARY,
        };

        private Button MakeButton(string text, Point loc, int width, Action<Button> onClick)
        {
            var btn = new Button
            {
                Text = text, Location = loc, Size = new Size(width, 28),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 60, 70),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                Cursor = Cursors.Hand,
            };
            btn.FlatAppearance.BorderColor = Color.FromArgb(80, 80, 85);
            btn.Click += (s, e) => onClick(btn);
            return btn;
        }

        // ============================================================
        // Stream loop — integrate stick → target angles, send MAVLink
        // ============================================================
        private void OnStreamTick(object sender, EventArgs e)
        {
            float dt = 1f / STREAM_HZ;
            float sx = Math.Abs(_stickX) < STICK_DEADZONE ? 0 : _stickX;
            float sy = Math.Abs(_stickY) < STICK_DEADZONE ? 0 : _stickY;

            bool active = sx != 0f || sy != 0f;

            // Delegate the rate→target-angle integration and the actual
            // DO_MOUNT_CONTROL send to GimbalController so this window and the
            // physical NomadJoystickService share one authoritative target.
            if (active)
            {
                GimbalController.ApplyStick(sx, sy, _maxRateDegSec, dt,
                    send: _mountMode == MountMode.MavlinkTargeting);
            }

            // Mirror controller state into our display fields (TargetChanged
            // also pushes this, but mirroring each tick keeps things visible
            // even when no stick motion fires the event).
            _targetPitch = GimbalController.TargetPitchDeg;
            _targetRoll  = GimbalController.TargetRollDeg;
            if (_lblPitch != null) _lblPitch.Text = $"Pitch: {_targetPitch,+6:0.0}°";
            if (_lblRoll  != null) _lblRoll.Text  = $"Roll:  {_targetRoll,+6:0.0}°";
        }

        // Helpers for the in-window snap / key-nudge buttons — both go through
        // GimbalController so the physical NomadJoystickService sees the same
        // target angles immediately.
        private void SendPitchRollAngle(float pitchDeg, float rollDeg)
        {
            GimbalController.SetTargetAngles(pitchDeg, rollDeg);
            GimbalController.SendPitchRollAngle(GimbalController.TargetPitchDeg, GimbalController.TargetRollDeg);
        }

        private void SendMountConfigure(MountMode mode)
        {
            GimbalController.SetMode(ToCtrl(mode));
        }

        // ============================================================
        // Mode preset — latches a flag combo and pings the gimbal manager once
        // ============================================================
        private void SetModePreset(string label, MountMode mode)
        {
            _mountMode = mode;
            _modeLabel = label;
            UpdateModeLabel();
            SetStatus($"Mode → {label}", NOMADTheme.TEXT_PRIMARY);

            SendMountConfigure(mode);
        }

        private void UpdateModeLabel()
        {
            if (_lblMode == null) return;
            _lblMode.Text = $"Mode: {_modeLabel}";
        }

        private void SnapAngles(float pitch, float roll)
        {
            if (_mountMode != MountMode.MavlinkTargeting)
            {
                _mountMode = MountMode.MavlinkTargeting;
                _modeLabel = "MAVLINK";
                UpdateModeLabel();
            }
            _targetPitch = pitch;
            _targetRoll = roll;
            SendPitchRollAngle(pitch, roll);
        }

        private void OnWindowKeyDown(object sender, KeyEventArgs e)
        {
            float pitchDelta = 0f;
            float rollDelta = 0f;

            switch (e.KeyCode)
            {
                case Keys.Up:
                    pitchDelta = KEY_NUDGE_DEG;
                    break;
                case Keys.Down:
                    pitchDelta = -KEY_NUDGE_DEG;
                    break;
                case Keys.Left:
                    rollDelta = KEY_NUDGE_DEG;
                    break;
                case Keys.Right:
                    rollDelta = -KEY_NUDGE_DEG;
                    break;
                default:
                    return;
            }

            if (_mountMode != MountMode.MavlinkTargeting)
            {
                _mountMode = MountMode.MavlinkTargeting;
                _modeLabel = "MAVLINK";
                UpdateModeLabel();
            }

            _targetPitch = Clamp(_targetPitch + pitchDelta, PITCH_MIN_DEG, PITCH_MAX_DEG);
            _targetRoll = Clamp(_targetRoll + rollDelta, ROLL_MIN_DEG, ROLL_MAX_DEG);
            SendPitchRollAngle(_targetPitch, _targetRoll);

            e.Handled = true;
            e.SuppressKeyPress = true;
        }

        private static float Clamp(float value, float min, float max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        private void SetStatus(string text, Color color)
        {
            if (InvokeRequired) { BeginInvoke(new Action(() => SetStatus(text, color))); return; }
            if (_lblStatus != null) { _lblStatus.Text = text; _lblStatus.ForeColor = color; }
        }

        // ============================================================
        // Joystick pad — custom control, mouse drag, springs to centre
        // ============================================================
        private class JoystickPad : Control
        {
            public event Action<float, float> StickChanged;
            private bool _dragging;
            private PointF _stickNorm; // [-1,1] each axis

            public JoystickPad()
            {
                DoubleBuffered = true;
                SetStyle(ControlStyles.UserPaint | ControlStyles.AllPaintingInWmPaint
                         | ControlStyles.OptimizedDoubleBuffer | ControlStyles.ResizeRedraw, true);
            }

            protected override void OnMouseDown(MouseEventArgs e)
            {
                if (e.Button != MouseButtons.Left) return;
                _dragging = true;
                UpdateFromMouse(e.Location);
                base.OnMouseDown(e);
            }

            protected override void OnMouseMove(MouseEventArgs e)
            {
                if (_dragging) UpdateFromMouse(e.Location);
                base.OnMouseMove(e);
            }

            protected override void OnMouseUp(MouseEventArgs e)
            {
                _dragging = false;
                _stickNorm = PointF.Empty;
                StickChanged?.Invoke(0, 0);
                Invalidate();
                base.OnMouseUp(e);
            }

            protected override void OnMouseLeave(EventArgs e)
            {
                if (_dragging)
                {
                    _dragging = false;
                    _stickNorm = PointF.Empty;
                    StickChanged?.Invoke(0, 0);
                    Invalidate();
                }
                base.OnMouseLeave(e);
            }

            private void UpdateFromMouse(Point p)
            {
                int cx = Width / 2, cy = Height / 2;
                int r = Math.Min(cx, cy) - 8;
                float dx = (p.X - cx) / (float)r;
                float dy = (p.Y - cy) / (float)r;
                float mag = (float)Math.Sqrt(dx * dx + dy * dy);
                if (mag > 1f) { dx /= mag; dy /= mag; }
                _stickNorm = new PointF(dx, dy);
                // Up-in-pixels means -Y, but operator expects "stick forward = look down" or
                // "stick up = look up". Convention: stick UP (negative pixel-y) → pitch UP (+).
                StickChanged?.Invoke(dx, -dy);
                Invalidate();
            }

            protected override void OnPaint(PaintEventArgs e)
            {
                var g = e.Graphics;
                g.SmoothingMode = SmoothingMode.AntiAlias;
                g.Clear(BackColor);

                int cx = Width / 2, cy = Height / 2;
                int r = Math.Min(cx, cy) - 8;

                // Outer ring
                using (var ringPen = new Pen(NOMADTheme.TEXT_SECONDARY, 2))
                    g.DrawEllipse(ringPen, cx - r, cy - r, r * 2, r * 2);

                // Inner cross + deadzone
                using (var p2 = new Pen(Color.FromArgb(60, 60, 70), 1))
                {
                    g.DrawLine(p2, cx - r, cy, cx + r, cy);
                    g.DrawLine(p2, cx, cy - r, cx, cy + r);
                    int dz = (int)(r * STICK_DEADZONE);
                    g.DrawEllipse(p2, cx - dz, cy - dz, dz * 2, dz * 2);
                }

                // Stick puck
                int px = cx + (int)(_stickNorm.X * r);
                int py = cy + (int)(_stickNorm.Y * r);
                using (var brush = new SolidBrush(NOMADTheme.ACCENT))
                    g.FillEllipse(brush, px - 12, py - 12, 24, 24);
                using (var pen = new Pen(Color.White, 2))
                    g.DrawEllipse(pen, px - 12, py - 12, 24, 24);

                // Axis labels
                using (var brush = new SolidBrush(NOMADTheme.TEXT_SECONDARY))
                using (var f = new Font("Segoe UI", 8))
                {
                    var sz = g.MeasureString("PITCH+", f);
                    g.DrawString("PITCH+", f, brush, cx - sz.Width / 2, cy - r - sz.Height - 1);
                    g.DrawString("PITCH-", f, brush, cx - sz.Width / 2, cy + r + 1);
                    g.DrawString("ROLL+",  f, brush, cx - r - g.MeasureString("ROLL+", f).Width - 2, cy - 7);
                    g.DrawString("ROLL-",  f, brush, cx + r + 2, cy - 7);
                }
            }
        }
    }
}
