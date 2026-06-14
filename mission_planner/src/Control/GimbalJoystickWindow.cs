// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Gimbal Joystick — Floating Dockable Window
// ============================================================
// Rate-controlled 2D joystick that streams MAV_CMD_DO_MOUNT_CONTROL angle
// commands (pitch/roll) to a brushless gimbal mount on the autopilot. Mode
// buttons send MAV_CMD_DO_MOUNT_CONFIGURE. Works with any DO_MOUNT_CONTROL
// mount configured as an MNTx_* mount on ArduPilot.
//
// This is independent from the ZED tilt servo (PayloadControlPanel), which is
// just a SERVOx output. The command construction lives in GimbalCommand and the
// shared send/integrator in GimbalController, so this window is pure UI.
//
// Layout is fully dynamic: a docked TableLayoutPanel with a fill joystick pad
// (which scales itself in OnPaint) and AutoSize rows that reflow, so the window
// fits any size/aspect ratio without overlap.
// ============================================================

using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;
using Timer = System.Windows.Forms.Timer;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Floating window with a 2D rate joystick + mode buttons for the camera gimbal.
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
        private const int STREAM_HZ = 20;
        private const float KEY_NUDGE_DEG = 2.0f;
        // Mount limits pulled from GimbalController so both this window and the
        // physical NomadJoystickService stay in sync if the limits ever change.
        private const float PITCH_MIN_DEG = GimbalController.PITCH_MIN_DEG;
        private const float PITCH_MAX_DEG = GimbalController.PITCH_MAX_DEG;
        private const float ROLL_MIN_DEG = GimbalController.ROLL_MIN_DEG;
        private const float ROLL_MAX_DEG = GimbalController.ROLL_MAX_DEG;
        private const float STICK_DEADZONE = 0.06f;

        // ============================================================
        // State
        // ============================================================
        private readonly NOMADConfig _config;

        // Stick: normalized [-1,1] x = roll rate, y = pitch rate (up = +pitch).
        private float _stickX, _stickY;
        // Local mirror of GimbalController target so display label code can read
        // without crossing threads. Authoritative state lives in GimbalController.
        private float _targetPitch, _targetRoll;
        // Mount mode currently selected — mirrors GimbalController.
        private MountMode _mountMode = MountMode.MavlinkTargeting;
        private string _modeLabel = "MAVLINK";

        // UI
        private JoystickPad _pad;
        private Label _lblPitch, _lblRoll, _lblMode, _lblStatus, _lblRate;
        private TrackBar _trkRate;
        private Button _btnRetract, _btnNeutral, _btnRcTgt, _btnMavTgt, _btnCenter, _btnLevel;
        private CheckBox _chkTopMost;
        private Timer _streamTimer;

        private GimbalJoystickWindow(NOMADConfig config)
        {
            _config = config;
            Text = "Gimbal Joystick";
            StartPosition = FormStartPosition.Manual;
            FormBorderStyle = FormBorderStyle.SizableToolWindow;
            BackColor = NOMADTheme.BG_DARK;
            ForeColor = NOMADTheme.TEXT_PRIMARY;
            Font = NOMADTheme.Font();
            MinimumSize = new Size(320, 500);
            ClientSize = new Size(380, 560);
            Padding = new Padding(NOMADTheme.GAP);
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
            _targetRoll = GimbalController.TargetRollDeg;

            _streamTimer = new Timer { Interval = 1000 / STREAM_HZ };
            _streamTimer.Tick += OnStreamTick;
            _streamTimer.Start();

            // Keep our readout in sync when the physical joystick service moves
            // the target while this window is open.
            GimbalController.TargetChanged += OnExternalTargetChanged;
            // Mirror max-rate edits made elsewhere (settings dialog, physical service).
            GimbalController.MaxRateChanged += OnExternalMaxRateChanged;

            // Default to MAVLink targeting so the first joystick or keyboard input
            // immediately drives the mount.
            GimbalController.SetMode(_mountMode);
            UpdateModeLabel();

            FormClosed += (s, e) =>
            {
                _streamTimer?.Stop();
                _streamTimer?.Dispose();
                _streamTimer = null;
                GimbalController.TargetChanged -= OnExternalTargetChanged;
                GimbalController.MaxRateChanged -= OnExternalMaxRateChanged;
                s_instance = null;
            };
        }

        private void OnExternalTargetChanged(float pitch, float roll)
        {
            if (IsDisposed) return;
            UiAsync.RunSync(this, () =>
            {
                _targetPitch = pitch;
                _targetRoll = roll;
                if (_lblPitch != null) _lblPitch.Text = $"Pitch: {_targetPitch,+6:0.0}°";
                if (_lblRoll != null) _lblRoll.Text = $"Roll: {_targetRoll,+6:0.0}°";
            }, "OnExternalTargetChanged");
        }

        private void OnExternalMaxRateChanged(float rate)
        {
            if (IsDisposed) return;
            UiAsync.RunSync(this, () =>
            {
                if (_trkRate == null) return;
                int v = (int)Math.Round(rate);
                if (v < _trkRate.Minimum) v = _trkRate.Minimum;
                if (v > _trkRate.Maximum) v = _trkRate.Maximum;
                if (_trkRate.Value != v) _trkRate.Value = v; // ValueChanged fires; the controller setter dedupes.
                if (_lblRate != null) _lblRate.Text = $"{v}";
            }, "OnExternalMaxRateChanged");
        }

        // ============================================================
        // UI — a docked TableLayoutPanel of AutoSize rows + a fill pad row, so
        // everything reflows and nothing overlaps at any window size.
        // ============================================================
        private void BuildUi()
        {
            var root = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 7,
                BackColor = Color.Transparent,
                Padding = new Padding(0),
                Margin = new Padding(0),
            };
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));   // 0 title
            root.RowStyles.Add(new RowStyle(SizeType.Percent, 100f)); // 1 joystick pad (fills)
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));   // 2 readouts
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));   // 3 rate slider
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));   // 4 mode + preset buttons
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));   // 5 status
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));   // 6 hint

            var title = new Label
            {
                Text = "GIMBAL — Rate Joystick",
                Font = NOMADTheme.Font(NOMADTheme.SIZE_HEADING, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                AutoSize = true,
                Margin = new Padding(2, 2, 0, NOMADTheme.GAP),
            };
            root.Controls.Add(title, 0, 0);

            // Joystick pad — fills the flexible row and stays circular via OnPaint.
            _pad = new JoystickPad
            {
                Dock = DockStyle.Fill,
                Margin = new Padding(0, 0, 0, NOMADTheme.GAP),
                MinimumSize = new Size(180, 160),
                BackColor = NOMADTheme.CARD_BG,
            };
            _pad.StickChanged += (x, y) => { _stickX = x; _stickY = y; };
            root.Controls.Add(_pad, 0, 1);

            // Target readouts — a wrapping flow so they never clip.
            var readouts = new FlowLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                WrapContents = true,
                Margin = new Padding(0, 0, 0, NOMADTheme.GAP),
            };
            _lblPitch = MakeReadout("Pitch: +0.0°");
            _lblRoll = MakeReadout("Roll: +0.0°");
            _lblMode = MakeReadout("Mode: MAVLINK");
            readouts.Controls.Add(_lblPitch);
            readouts.Controls.Add(_lblRoll);
            readouts.Controls.Add(_lblMode);
            root.Controls.Add(readouts, 0, 2);

            root.Controls.Add(BuildRateRow(), 0, 3);
            root.Controls.Add(BuildButtonRows(), 0, 4);

            _lblStatus = new Label
            {
                Text = "Ready",
                AutoSize = true,
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL),
                Margin = new Padding(2, NOMADTheme.GAP, 0, 2),
            };
            root.Controls.Add(_lblStatus, 0, 5);

            var hint = new Label
            {
                Text = "Drag pad: pitch (Y) / roll (X). Arrow keys nudge both axes. Release = stop.",
                AutoSize = true,
                MaximumSize = new Size(0, 0),
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL, FontStyle.Italic),
                Margin = new Padding(2, 0, 0, 0),
            };
            root.Controls.Add(hint, 0, 6);

            Controls.Add(root);
        }

        // Rate slider row: "Max Rate (deg/s):" | [====slider====] | value.
        private TableLayoutPanel BuildRateRow()
        {
            var row = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                ColumnCount = 3,
                RowCount = 1,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                BackColor = NOMADTheme.PANEL_ALT,
                Padding = new Padding(NOMADTheme.GAP, 4, NOMADTheme.GAP, 4),
                Margin = new Padding(0, 0, 0, NOMADTheme.GAP),
            };
            row.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            row.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            row.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));

            var lbl = new Label
            {
                Text = "Max Rate (deg/s):",
                AutoSize = true,
                Anchor = AnchorStyles.Left,
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Margin = new Padding(0, 0, NOMADTheme.GAP, 0),
            };
            row.Controls.Add(lbl, 0, 0);

            _trkRate = new TrackBar
            {
                Dock = DockStyle.Fill,
                Minimum = (int)GimbalController.MIN_MAX_RATE_DEG_SEC,
                Maximum = (int)GimbalController.MAX_MAX_RATE_DEG_SEC,
                Value = (int)Math.Round(GimbalController.MaxRateDegSec),
                TickStyle = TickStyle.None,
                BackColor = NOMADTheme.PANEL_ALT,
                Margin = new Padding(0),
            };
            _trkRate.ValueChanged += (s, e) =>
            {
                // Single source of truth — controller setter fires MaxRateChanged
                // which all other UIs subscribe to.
                GimbalController.MaxRateDegSec = _trkRate.Value;
                _lblRate.Text = $"{_trkRate.Value}";
            };
            row.Controls.Add(_trkRate, 1, 0);

            _lblRate = new Label
            {
                Text = $"{_trkRate.Value}",
                AutoSize = true,
                Anchor = AnchorStyles.Right,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Margin = new Padding(NOMADTheme.GAP, 0, 0, 0),
            };
            row.Controls.Add(_lblRate, 2, 0);
            return row;
        }

        // Mode + preset buttons, each in a wrapping flow so they re-pack when the
        // window is narrow instead of running off the edge.
        private FlowLayoutPanel BuildButtonRows()
        {
            var flow = new FlowLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                WrapContents = true,
                FlowDirection = FlowDirection.LeftToRight,
                Margin = new Padding(0, 0, 0, 0),
                Padding = new Padding(0),
            };

            _btnMavTgt = MakeButton("MAVLink Tgt", c => SetModePreset("MAVLINK", MountMode.MavlinkTargeting));
            _btnRcTgt = MakeButton("RC Tgt", c => SetModePreset("RC", MountMode.RcTargeting));
            _btnNeutral = MakeButton("Neutral", c => SetModePreset("NEUTRAL", MountMode.Neutral));
            _btnRetract = MakeButton("Retract", c => SetModePreset("RETRACT", MountMode.Retract));
            _btnCenter = MakeButton("Center 0°/0°", c => SnapAngles(0, 0));
            _btnLevel = MakeButton("Look Down −90°", c => SnapAngles(-90, _targetRoll));
            flow.Controls.Add(_btnMavTgt);
            flow.Controls.Add(_btnRcTgt);
            flow.Controls.Add(_btnNeutral);
            flow.Controls.Add(_btnRetract);
            flow.Controls.Add(_btnCenter);
            flow.Controls.Add(_btnLevel);

            _chkTopMost = new CheckBox
            {
                Text = "Always on top",
                AutoSize = true,
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Margin = new Padding(NOMADTheme.GAP, 4, 0, 0),
            };
            _chkTopMost.CheckedChanged += (s, e) => TopMost = _chkTopMost.Checked;
            flow.Controls.Add(_chkTopMost);
            return flow;
        }

        private Label MakeReadout(string text) => new Label
        {
            Text = text,
            AutoSize = true,
            Font = NOMADTheme.Mono(NOMADTheme.SIZE_LARGE, FontStyle.Bold),
            ForeColor = NOMADTheme.TEXT_PRIMARY,
            Margin = new Padding(0, 0, NOMADTheme.PAD, 0),
        };

        private Button MakeButton(string text, Action<Button> onClick)
        {
            var btn = new Button
            {
                Text = text,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                Padding = new Padding(6, 3, 6, 3),
                Margin = new Padding(0, 0, NOMADTheme.GAP, NOMADTheme.GAP),
                FlatStyle = FlatStyle.Flat,
                BackColor = NOMADTheme.BUTTON_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL, FontStyle.Bold),
                Cursor = Cursors.Hand,
            };
            btn.FlatAppearance.BorderColor = NOMADTheme.CARD_BORDER;
            btn.Click += (s, e) => onClick(btn);
            return btn;
        }

        // ============================================================
        // Stream loop — integrate stick → target angles, send MAVLink
        // ============================================================
        private void OnStreamTick(object sender, EventArgs e)
        {
            float dt = 1f / STREAM_HZ;
            float sx = GimbalCommand.ApplyDeadzone(_stickX, STICK_DEADZONE);
            float sy = GimbalCommand.ApplyDeadzone(_stickY, STICK_DEADZONE);

            bool active = sx != 0f || sy != 0f;

            // Delegate the rate→target-angle integration and the actual
            // DO_MOUNT_CONTROL send to GimbalController so this window and the
            // physical NomadJoystickService share one authoritative target.
            if (active)
            {
                GimbalController.ApplyStick(sx, sy, dt,
                    send: _mountMode == MountMode.MavlinkTargeting);
            }

            // Mirror controller state into our display fields (TargetChanged
            // also pushes this, but mirroring each tick keeps things visible
            // even when no stick motion fires the event).
            _targetPitch = GimbalController.TargetPitchDeg;
            _targetRoll = GimbalController.TargetRollDeg;
            if (_lblPitch != null) _lblPitch.Text = $"Pitch: {_targetPitch,+6:0.0}°";
            if (_lblRoll != null) _lblRoll.Text = $"Roll: {_targetRoll,+6:0.0}°";
        }

        // Helpers for the in-window snap / key-nudge buttons — both go through
        // GimbalController so the physical NomadJoystickService sees the same
        // target angles immediately.
        private void SendPitchRollAngle(float pitchDeg, float rollDeg)
        {
            GimbalController.SetTargetAngles(pitchDeg, rollDeg);
            GimbalController.SendPitchRollAngle(GimbalController.TargetPitchDeg, GimbalController.TargetRollDeg);
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

            GimbalController.SetMode(mode);
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
            UiAsync.RunSync(this, () =>
            {
                if (_lblStatus != null) { _lblStatus.Text = text; _lblStatus.ForeColor = color; }
            }, "SetStatus");
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
                if (r <= 0) return;
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
                if (r <= 0) return;

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
                using (var f = new Font(NOMADTheme.FONT_FAMILY, NOMADTheme.SIZE_SMALL))
                {
                    var sz = g.MeasureString("PITCH+", f);
                    g.DrawString("PITCH+", f, brush, cx - sz.Width / 2, cy - r - sz.Height - 1);
                    g.DrawString("PITCH-", f, brush, cx - sz.Width / 2, cy + r + 1);
                    g.DrawString("ROLL+", f, brush, cx - r - g.MeasureString("ROLL+", f).Width - 2, cy - 7);
                    g.DrawString("ROLL-", f, brush, cx + r + 2, cy - 7);
                }
            }
        }
    }
}
