// ============================================================
// NOMAD Caddx Gimbal Joystick — Floating Dockable Window
// ============================================================
// Rate-controlled 2D joystick that streams MAV_CMD_DO_MOUNT_CONTROL
// angle commands (pitch/yaw) to the Caddx brushless gimbal mount on
// the Cube Orange. Mode buttons send MAV_CMD_DO_MOUNT_CONFIGURE.
//
// This is independent from the ZED tilt servo (PayloadControlPanel),
// which is just a SERVOx output. The Caddx is configured as a real
// MNTx_* mount on ArduPilot.
// ============================================================

using System;
using System.Drawing;
using System.Drawing.Drawing2D;
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
        // Matches MNT1_PITCH_MIN/MAX, MNT1_YAW_MIN/MAX on the Caddx mount.
        private const float PITCH_MIN_DEG      = -90f;
        private const float PITCH_MAX_DEG      =  90f;
        private const float YAW_MIN_DEG        = -170f;
        private const float YAW_MAX_DEG        =  170f;
        private const float STICK_DEADZONE     = 0.06f;

        // ============================================================
        // State
        // ============================================================
        private readonly NOMADConfig _config;

        // Stick: normalized [-1,1] x = yaw rate, y = pitch rate (up = +pitch).
        private float _stickX, _stickY;
        // Integrated target angles (deg). Caddx serial driver in AP only accepts
        // angle commands, so we integrate stick rate → angle locally.
        private float _targetPitch, _targetYaw;
        private float _maxRateDegSec = DEFAULT_MAX_RATE;
        // Gimbal device id (0 = broadcast / primary). MNT1 = 1, MNT2 = 2 if multi-mount.
        private byte _gimbalDeviceId = 0;
        // Latching gimbal-manager flags: pitch+yaw lock to world-frame by default.
        private GimbalManagerFlags _flags = GimbalManagerFlags.PITCH_LOCK | GimbalManagerFlags.YAW_LOCK;
        private string _modeLabel = "MAVLINK";

        // Mirror of mavlink_msgs GIMBAL_MANAGER_FLAGS bitmask — kept local so we
        // don't depend on this enum being present in the bundled MAVLink build.
        [Flags]
        private enum GimbalManagerFlags : uint
        {
            NONE          = 0,
            RETRACT       = 1,
            NEUTRAL       = 2,
            ROLL_LOCK     = 4,
            PITCH_LOCK    = 8,
            YAW_LOCK      = 16,
            RC_EXCLUSIVE  = 32,
            RC_MIXED      = 64,
        }

        // UI
        private JoystickPad _pad;
        private Label _lblPitch, _lblYaw, _lblMode, _lblStatus, _lblRate;
        private TrackBar _trkRate;
        private Button _btnRetract, _btnNeutral, _btnRcTgt, _btnMavTgt, _btnCenter, _btnLevel;
        private CheckBox _chkTopMost;
        private Timer _streamTimer;

        // Serialize MAVLink to avoid contention with PayloadControlPanel's lock.
        // We use the same approach (Task.Run + non-blocking dispatch).
        private bool _inflight;

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

            // Position near top-right corner of screen
            try
            {
                var wa = Screen.PrimaryScreen.WorkingArea;
                Location = new Point(wa.Right - ClientSize.Width - 40, wa.Top + 80);
            }
            catch { }

            BuildUi();

            _streamTimer = new Timer { Interval = 1000 / STREAM_HZ };
            _streamTimer.Tick += OnStreamTick;
            _streamTimer.Start();

            // No explicit "mode set" call needed for gimbal-manager v2 — sending
            // PITCHYAW with pitch/yaw lock flags is sufficient. Optionally we could
            // call DO_GIMBAL_MANAGER_CONFIGURE to claim primary control sysid.
            UpdateModeLabel();

            FormClosed += (s, e) =>
            {
                _streamTimer?.Stop();
                _streamTimer?.Dispose();
                _streamTimer = null;
                s_instance = null;
            };
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
            _lblYaw   = MakeReadout("Yaw:    +0.0°", new Point(190, 268));
            _lblMode  = MakeReadout("Mode: MAVLINK", new Point(20, 290));
            Controls.Add(_lblPitch); Controls.Add(_lblYaw); Controls.Add(_lblMode);

            // Rate slider
            Controls.Add(new Label
            {
                Text = "Max Rate (deg/s):",
                Location = new Point(20, 322),
                AutoSize = true,
                ForeColor = NOMADTheme.TEXT_SECONDARY,
            });
            _trkRate = new TrackBar
            {
                Location = new Point(150, 318),
                Size = new Size(140, 30),
                Minimum = 10, Maximum = 180,
                Value = (int)DEFAULT_MAX_RATE,
                TickStyle = TickStyle.None,
                BackColor = NOMADTheme.BG_DARK,
            };
            _trkRate.ValueChanged += (s, e) =>
            {
                _maxRateDegSec = _trkRate.Value;
                _lblRate.Text = $"{_trkRate.Value}";
            };
            Controls.Add(_trkRate);
            _lblRate = new Label
            {
                Text = $"{_trkRate.Value}",
                Location = new Point(295, 322),
                AutoSize = true,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
            };
            Controls.Add(_lblRate);

            // Mode buttons row 1: control modes (gimbal-manager flag presets)
            int y = 354;
            _btnMavTgt  = MakeButton("MAVLink Tgt", new Point(12,  y), 95, c => SetModePreset("MAVLINK",
                                            GimbalManagerFlags.PITCH_LOCK | GimbalManagerFlags.YAW_LOCK));
            _btnRcTgt   = MakeButton("RC Tgt",      new Point(112, y), 80, c => SetModePreset("RC",
                                            GimbalManagerFlags.RC_EXCLUSIVE));
            _btnNeutral = MakeButton("Neutral",     new Point(197, y), 80, c => SetModePreset("NEUTRAL",
                                            GimbalManagerFlags.NEUTRAL));
            _btnRetract = MakeButton("Retract",     new Point(282, y), 80, c => SetModePreset("RETRACT",
                                            GimbalManagerFlags.RETRACT));
            Controls.Add(_btnMavTgt); Controls.Add(_btnRcTgt); Controls.Add(_btnNeutral); Controls.Add(_btnRetract);

            // Quick angle presets
            y = 388;
            _btnCenter = MakeButton("Center (0°/0°)",   new Point(12, y), 130, c => SnapAngles(0, 0));
            _btnLevel  = MakeButton("Look Down (-90°)", new Point(150, y), 130, c => SnapAngles(-90, _targetYaw));
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
                Text = "Drag pad: pitch (Y) / yaw (X). Release = stop.",
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

            // Integrate rate → angle (Caddx driver only accepts angle commands).
            if (active)
            {
                _targetPitch += sy * _maxRateDegSec * dt;
                _targetYaw   += sx * _maxRateDegSec * dt;

                if (_targetPitch < PITCH_MIN_DEG) _targetPitch = PITCH_MIN_DEG;
                if (_targetPitch > PITCH_MAX_DEG) _targetPitch = PITCH_MAX_DEG;
                if (_targetYaw   < YAW_MIN_DEG)   _targetYaw   = YAW_MIN_DEG;
                if (_targetYaw   > YAW_MAX_DEG)   _targetYaw   = YAW_MAX_DEG;
            }

            _lblPitch.Text = $"Pitch: {_targetPitch,+6:0.0}°";
            _lblYaw.Text   = $"Yaw:   {_targetYaw,+6:0.0}°";

            // RETRACT/NEUTRAL latching modes: no streaming.
            if ((_flags & (GimbalManagerFlags.RETRACT | GimbalManagerFlags.NEUTRAL)) != 0)
                return;

            // Only send while the stick is active. Once released, the gimbal holds
            // its last commanded angle on its own — no need to keep pinging.
            if (active)
                SendPitchYawAngle(_targetPitch, _targetYaw);
        }

        // Send DO_GIMBAL_MANAGER_PITCHYAW with absolute angles (rates = NaN).
        // Caddx serial mount driver in ArduPilot only honours the angle path.
        private void SendPitchYawAngle(float pitchDeg, float yawDeg)
        {
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen) return;
            if (_inflight) return; // drop overlapping frames during fast drag
            _inflight = true;

            byte sysid  = MainV2.comPort.MAV.sysid;
            byte compid = MainV2.comPort.MAV.compid;
            // Always include PITCH_LOCK | YAW_LOCK for absolute earth-frame angles.
            float flagsF = (float)(uint)(_flags | GimbalManagerFlags.PITCH_LOCK | GimbalManagerFlags.YAW_LOCK);
            byte devId = _gimbalDeviceId;

            Task.Run(async () =>
            {
                try
                {
                    await MainV2.comPort.doCommandAsync(
                        sysid, compid,
                        MAVLink.MAV_CMD.DO_GIMBAL_MANAGER_PITCHYAW,
                        pitchDeg, yawDeg,
                        float.NaN, float.NaN,
                        flagsF, 0f, devId,
                        requireack: false, uicallback: null).ConfigureAwait(false);
                }
                catch (Exception ex)
                {
                    BeginInvoke(new Action(() => SetStatus("Angle send failed: " + ex.Message, NOMADTheme.ERROR)));
                }
                finally { _inflight = false; }
            });
        }

        // ============================================================
        // Mode preset — latches a flag combo and pings the gimbal manager once
        // ============================================================
        private void SetModePreset(string label, GimbalManagerFlags flags)
        {
            _flags = flags;
            _modeLabel = label;
            UpdateModeLabel();
            SetStatus($"Mode → {label}", NOMADTheme.TEXT_PRIMARY);

            // One-shot kick: send current angles so the mode flag takes effect.
            SendPitchYawAngle(_targetPitch, _targetYaw);
        }

        private void UpdateModeLabel()
        {
            if (_lblMode == null) return;
            _lblMode.Text = $"Mode: {_modeLabel}";
        }

        private void SnapAngles(float pitch, float yaw)
        {
            // Presets only make sense in MAVLink-controlled flag states.
            if ((_flags & (GimbalManagerFlags.RETRACT | GimbalManagerFlags.NEUTRAL | GimbalManagerFlags.RC_EXCLUSIVE)) != 0)
            {
                _flags = GimbalManagerFlags.PITCH_LOCK | GimbalManagerFlags.YAW_LOCK;
                _modeLabel = "MAVLINK";
                UpdateModeLabel();
            }
            _targetPitch = pitch;
            _targetYaw = yaw;
            SendPitchYawAngle(pitch, yaw);
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
                    g.DrawString("YAW-",  f, brush, cx - r - g.MeasureString("YAW-", f).Width - 2, cy - 7);
                    g.DrawString("YAW+",  f, brush, cx + r + 2, cy - 7);
                }
            }
        }
    }
}
