// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Gimbal Joystick — Floating Dockable Window
// ============================================================
// Rate-controlled joystick requests profile-mapped targets through the selected Mission Planner connection.
//
// This is independent from the camera tilt servo (ActuatorControlPanel), which is
// just a SERVOx output. Angle integration lives in GimbalCommand and the shared
// target/request state lives in GimbalController, so this window is pure UI.
//
// Layout is fully dynamic: a docked TableLayoutPanel with a fill joystick pad
// (which scales itself in OnPaint) and AutoSize rows that reflow, so the window
// fits any size/aspect ratio without overlap.
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;
using Timer = System.Windows.Forms.Timer;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Floating window with a 2D rate joystick + mode buttons for the camera gimbal.
    /// Open with <see cref="ShowSingleton"/>; only one instance lives at a time.
    /// </summary>
    [global::MissionPlanner.Utilities.PreventTheming]
    public class GimbalJoystickWindow : Form
    {
        private static GimbalJoystickWindow s_instance;

        internal static void CloseSingleton()
        {
            s_instance?.Close();
        }

        public static void ShowSingleton(NOMADConfig config, IWin32Window owner = null)
        {
            if (!OutputController.IsCurrentProfile(config))
            {
                return;
            }
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
        private const float STICK_DEADZONE = 0.06f;

        // ============================================================
        // State
        // ============================================================
        // Stick: normalized [-1,1], using this aircraft's screen-axis mapping.
        private float _stickX, _stickY;
        // Local mirror of GimbalController target so display label code can read
        // without crossing threads. Authoritative state lives in GimbalController.
        private float _targetPitch, _targetRoll, _targetYaw;
        // Mount mode currently selected — mirrors GimbalController.
        private MountMode _mountMode = MountMode.MavlinkTargeting;
        private string _modeLabel = "MAVLINK";

        // UI
        private JoystickPad _pad;
        private System.Windows.Controls.TextBlock _lblPitch, _lblRoll, _lblYaw, _lblMode, _lblRate;
        private System.Windows.Controls.Slider _trkRate;
        private System.Windows.Controls.Button _btnRetract, _btnNeutral, _btnRcTgt, _btnMavTgt, _btnCenter, _btnLevel;
        private System.Windows.Controls.CheckBox _chkTopMost, _chkArrowKeys;
        private Timer _streamTimer;

        private GimbalJoystickWindow(NOMADConfig config)
        {
            Text = "Gimbal Joystick - " + config.AircraftName;
            _mountMode = GimbalController.SelectedMode;
            StartPosition = FormStartPosition.Manual;
            FormBorderStyle = FormBorderStyle.SizableToolWindow;
            BackColor = NOMADTheme.BG_DARK;
            ForeColor = NOMADTheme.TEXT_PRIMARY;
            Font = NOMADTheme.Font();
            MinimumSize = new Size(300, 470);
            ClientSize = new Size(340, 520);
            Padding = new Padding(NOMADTheme.GAP);
            ShowInTaskbar = false;

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
            _targetYaw = GimbalController.TargetYawDeg;

            _streamTimer = new Timer { Interval = 1000 / STREAM_HZ };
            _streamTimer.Tick += OnStreamTick;
            _streamTimer.Start();

            // Keep our readout in sync when the physical joystick service moves
            // the target while this window is open.
            GimbalController.TargetChanged += OnExternalTargetChanged;
            // Mirror max-rate edits made elsewhere (settings dialog, physical service).
            GimbalController.MaxRateChanged += OnExternalMaxRateChanged;
            GimbalController.ModeChanged += OnExternalModeChanged;
            GimbalArrowKeyFilter.EnabledChanged += OnArrowKeysEnabledChanged;

            UpdateModeLabel();

            FormClosed += (s, e) =>
            {
                _streamTimer?.Stop();
                _streamTimer?.Dispose();
                _streamTimer = null;
                GimbalController.TargetChanged -= OnExternalTargetChanged;
                GimbalController.MaxRateChanged -= OnExternalMaxRateChanged;
                GimbalController.ModeChanged -= OnExternalModeChanged;
                GimbalArrowKeyFilter.EnabledChanged -= OnArrowKeysEnabledChanged;
                s_instance = null;
            };
        }

        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            if ((keyData & Keys.Modifiers) == Keys.None && GimbalArrowKeyFilter.TryHandleMessage(ref msg))
                return true;
            return base.ProcessCmdKey(ref msg, keyData);
        }

        private void OnExternalTargetChanged(float pitch, float roll)
        {
            if (IsDisposed) return;
            UiAsync.RunSync(this, () =>
            {
                _targetPitch = pitch;
                _targetRoll = roll;
                _targetYaw = GimbalController.TargetYawDeg;
                if (_lblPitch != null) _lblPitch.Text = $"Pitch: {_targetPitch,+6:0.0}°";
                if (_lblRoll != null) _lblRoll.Text = $"Roll: {_targetRoll,+6:0.0}°";
                if (_lblYaw != null) _lblYaw.Text = $"Yaw: {_targetYaw,+6:0.0}°";
            }, "OnExternalTargetChanged");
        }

        private void OnExternalMaxRateChanged(float rate)
        {
            if (IsDisposed) return;
            UiAsync.RunSync(this, () =>
            {
                if (_trkRate == null) return;
                int v = (int)Math.Round(rate);
                if (v < _trkRate.Minimum) v = (int)_trkRate.Minimum;
                if (v > _trkRate.Maximum) v = (int)_trkRate.Maximum;
                if (_trkRate.Value != v) _trkRate.Value = v; // ValueChanged fires; the controller setter dedupes.
                if (_lblRate != null) _lblRate.Text = $"{v}";
            }, "OnExternalMaxRateChanged");
        }

        private void OnExternalModeChanged(MountMode mode)
        {
            if (IsDisposed) return;
            UiAsync.RunSync(this, () =>
            {
                _mountMode = mode;
                _modeLabel = ModeLabel(mode);
                UpdateModeLabel();
                HighlightModeButtons();
            }, "OnExternalModeChanged");
        }

        private void OnArrowKeysEnabledChanged(bool enabled)
        {
            if (IsDisposed) return;
            UiAsync.RunSync(this, () =>
            {
                if (_chkArrowKeys != null && _chkArrowKeys.IsChecked != enabled)
                    _chkArrowKeys.IsChecked = enabled;
            }, "OnArrowKeysEnabledChanged");
        }

        private void BuildUi()
        {
            var root = new System.Windows.Controls.DockPanel { Background = WpfTheme.Card };
            var title = WpfUi.Section("GIMBAL — Rate Joystick");
            System.Windows.Controls.DockPanel.SetDock(title, System.Windows.Controls.Dock.Top);
            root.Children.Add(title);
            var bottom = new System.Windows.Controls.StackPanel();
            System.Windows.Controls.DockPanel.SetDock(bottom, System.Windows.Controls.Dock.Bottom);
            var readouts = new System.Windows.Controls.WrapPanel();
            _lblPitch = MakeReadout("Pitch: +0.0°"); _lblRoll = MakeReadout("Roll: +0.0°");
            _lblYaw = MakeReadout("Yaw: +0.0°"); _lblMode = MakeReadout("Selected mode: MAVLINK");
            readouts.Children.Add(_lblPitch); readouts.Children.Add(_lblRoll);
            readouts.Children.Add(_lblYaw); readouts.Children.Add(_lblMode);
            bottom.Children.Add(readouts);
            var rate = new System.Windows.Controls.DockPanel();
            var rateTitle = WpfUi.Text("Max Rate (deg/s):");
            System.Windows.Controls.DockPanel.SetDock(rateTitle, System.Windows.Controls.Dock.Left);
            rate.Children.Add(rateTitle);
            _lblRate = WpfUi.Text("");
            System.Windows.Controls.DockPanel.SetDock(_lblRate, System.Windows.Controls.Dock.Right);
            rate.Children.Add(_lblRate);
            _trkRate = new System.Windows.Controls.Slider { Minimum = GimbalController.MIN_MAX_RATE_DEG_SEC,
                Maximum = GimbalController.MAX_MAX_RATE_DEG_SEC, Value = Math.Round(GimbalController.MaxRateDegSec),
                TickFrequency = 1, IsSnapToTickEnabled = true };
            System.Windows.Automation.AutomationProperties.SetName(_trkRate, "Max Rate (deg/s)");
            _lblRate.Text = $"{_trkRate.Value}";
            _trkRate.ValueChanged += (sender, args) =>
            { GimbalController.MaxRateDegSec = (float)_trkRate.Value; _lblRate.Text = $"{_trkRate.Value}"; };
            rate.Children.Add(_trkRate); bottom.Children.Add(rate);
            var buttons = new System.Windows.Controls.WrapPanel();
            _btnMavTgt = WpfUi.Action("MAVLink Tgt", () => SetModePreset("MAVLINK", MountMode.MavlinkTargeting));
            _btnRcTgt = WpfUi.Action("RC Tgt", () => SetModePreset("RC", MountMode.RcTargeting));
            _btnNeutral = WpfUi.Action("Neutral", () => SetModePreset("NEUTRAL", MountMode.Neutral));
            _btnRetract = WpfUi.Action("Retract", () => SetModePreset("RETRACT", MountMode.Retract));
            _btnCenter = WpfUi.Action("Center", () => SnapAngles(0, 0, 0));
            _btnLevel = WpfUi.Action("Look Down −90°", () => SnapAngles(-90,
                GimbalController.TargetRollDeg, GimbalController.TargetYawDeg));
            buttons.Children.Add(_btnMavTgt); buttons.Children.Add(_btnRcTgt); buttons.Children.Add(_btnNeutral);
            buttons.Children.Add(_btnRetract); buttons.Children.Add(_btnCenter); buttons.Children.Add(_btnLevel);
            _chkTopMost = new System.Windows.Controls.CheckBox { Content = "Always on top" };
            _chkTopMost.Checked += (sender, args) => TopMost = true;
            _chkTopMost.Unchecked += (sender, args) => TopMost = false;
            buttons.Children.Add(_chkTopMost);
            _chkArrowKeys = new System.Windows.Controls.CheckBox { Content = "Arrow keys across Mission Planner",
                IsChecked = GimbalArrowKeyFilter.Enabled };
            System.Windows.RoutedEventHandler change = (sender, args) =>
            {
                try { GimbalArrowKeyFilter.SetEnabled(_chkArrowKeys.IsChecked == true); }
                catch (Exception error)
                {
                    _chkArrowKeys.IsChecked = GimbalArrowKeyFilter.Enabled;
                    Log.Warn("Arrow key setting was not saved: " + error.Message);
                    NotificationService.Shared?.AddNotification(NotificationSeverity.Warning, NotificationCategory.System,
                        "Arrow key setting was not saved", error.Message);
                }
            };
            _chkArrowKeys.Checked += change; _chkArrowKeys.Unchecked += change;
            buttons.Children.Add(_chkArrowKeys); bottom.Children.Add(buttons);
            root.Children.Add(bottom);
            _pad = new JoystickPad(STICK_DEADZONE) { MinWidth = 150, MinHeight = 150,
                HorizontalLabel = GimbalController.ScreenHorizontalAxis, VerticalLabel = GimbalController.ScreenVerticalAxis,
                HorizontalInverted = GimbalController.ScreenHorizontalInverted,
                VerticalInverted = GimbalController.ScreenVerticalInverted };
            _pad.StickChanged += (x, y) => { _stickX = x; _stickY = y; };
            root.Children.Add(_pad);
            Controls.Add(WpfUi.Host(root));
            HighlightModeButtons();
        }

        private void HighlightModeButtons()
        {
            bool servo = OutputController.GimbalAxes?.Count > 0;
            _btnMavTgt.Content = servo ? "Servo targets" : "MAVLink Tgt";
            _btnMavTgt.IsEnabled = _btnRcTgt.IsEnabled = _btnNeutral.IsEnabled = _btnRetract.IsEnabled = !servo;
            SetModeButtonState(_btnMavTgt, _mountMode == MountMode.MavlinkTargeting);
            SetModeButtonState(_btnRcTgt, _mountMode == MountMode.RcTargeting);
            SetModeButtonState(_btnNeutral, _mountMode == MountMode.Neutral);
            SetModeButtonState(_btnRetract, _mountMode == MountMode.Retract);
        }

        private static void SetModeButtonState(System.Windows.Controls.Button button, bool active)
        {
            button.Background = active ? WpfTheme.Accent : WpfTheme.Card;
            button.BorderBrush = active ? WpfTheme.Accent : WpfTheme.Outline;
        }

        private static System.Windows.Controls.TextBlock MakeReadout(string text)
        {
            var result = WpfUi.Text(text);
            result.Margin = new System.Windows.Thickness(0, 0, NOMADTheme.PAD, 0);
            return result;
        }

        // ============================================================
        // Stream loop — integrate stick → target angles, send MAVLink
        // ============================================================
        private readonly GimbalInputState _gimbalInput = new GimbalInputState();
        private void OnStreamTick(object sender, EventArgs e)
        {
            float dt = 1f / STREAM_HZ;
            float sx = GimbalCommand.ApplyDeadzone(_stickX, STICK_DEADZONE);
            float sy = GimbalCommand.ApplyDeadzone(_stickY, STICK_DEADZONE);

            bool active = _gimbalInput.CanMove(true, sx, sy, OutputController.Generation);

            if (active)
            {
                GimbalController.ApplyScreenStick(sx, sy, dt,
                    send: _mountMode == MountMode.MavlinkTargeting);
            }

            // Mirror controller state into our display fields (TargetChanged
            // also pushes this, but mirroring each tick keeps things visible
            // even when no stick motion fires the event).
            _targetPitch = GimbalController.TargetPitchDeg;
            _targetRoll = GimbalController.TargetRollDeg;
            _targetYaw = GimbalController.TargetYawDeg;
            if (_lblPitch != null) _lblPitch.Text = $"Pitch: {_targetPitch,+6:0.0}°";
            if (_lblRoll != null) _lblRoll.Text = $"Roll: {_targetRoll,+6:0.0}°";
            if (_lblYaw != null) _lblYaw.Text = $"Yaw: {_targetYaw,+6:0.0}°";
        }

        // ============================================================
        // Mode preset — apply the selected mode immediately. Target requests wait
        // for native dispatch; vehicle acceptance remains unverified.
        // ============================================================
        private void SetModePreset(string label, MountMode mode)
        {
            _mountMode = mode;
            _modeLabel = label;
            UpdateModeLabel();
            HighlightModeButtons();

            GimbalController.SetMode(mode);
        }

        private static string ModeLabel(MountMode mode)
        {
            switch (mode)
            {
                case MountMode.RcTargeting:
                    return "RC";
                case MountMode.Neutral:
                    return "NEUTRAL";
                case MountMode.Retract:
                    return "RETRACT";
                default:
                    return "MAVLINK";
            }
        }

        private void UpdateModeLabel()
        {
            if (_lblMode == null) return;
            _lblMode.Text = $"Selected mode: {_modeLabel}";
        }

        private void SnapAngles(float pitch, float roll, float yaw) =>
            _ = GimbalController.RequestScreenTargetAsync(pitch, roll, yaw);

    }
}
