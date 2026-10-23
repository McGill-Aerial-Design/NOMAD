// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Drawing;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        private readonly VideoFlightInput _flightInput = new VideoFlightInput();
        private VideoFlightController _flightController;
        private Button _flyButton;
        private PictureBox _flightBox;
        private Form _flightForm;
        private bool _flyEnabled, _flightPreparing, _cursorHidden;
        private volatile bool _flightCaptured;
        private long _flightRevision, _flightVideoGeneration, _flightUiTicks, _flightVideoTicks, _flightLastSample;
        private Rectangle _previousCursorClip;
        private Point _previousCursorPosition;
        internal Func<VideoFlightKeys> FlightKeys { get; set; } = ReadFlightKeys;
        internal Func<bool> FlightForeground { get; set; }
        internal Func<Point> FlightPointer { get; set; } = () => Cursor.Position;
        internal Action<bool, PictureBox> FlightCursor { get; set; }
        internal bool FlightCaptured => _flightCaptured;
        internal string FlightStatus { get; private set; } = "Fly off";
        internal int FlightSubmitted => _flightController?.Submitted ?? 0;
        internal Task FlightRelease { get; private set; } = Task.CompletedTask;

        [DllImport("user32.dll")]
        private static extern short GetAsyncKeyState(int key);
        [DllImport("user32.dll")]
        private static extern IntPtr GetForegroundWindow();

        private void InitializeFlight()
        {
            if (!_showControls) { return; }
            _flyButton = ControlFactory.Button("Fly", (sender, args) => SetFlightEnabled(!_flyEnabled), width: 55, height: 28);
            _flyButton.AccessibleName = "Fly with keyboard and mouse";
            _videoToolTips.SetToolTip(_flyButton,
                "Armed, airborne Copter already in GUIDED only. Click video to capture; Escape releases. " +
                "WASD move, Left/Right yaw, Space up, Shift down, mouse aims gimbal. No arming or takeoff.");
            var controls = (FlowLayoutPanel)Toolbar.GetControlFromPosition(0, 1);
            controls.Controls.Add(_flyButton);
        }

        internal void SetFlightEnabled(bool enabled)
        {
            ValidateUiThread();
            if (!_showControls || IsDisposed) { return; }
            ReleaseFlight("Flight controls released");
            _flyEnabled = enabled;
            if (enabled) { RetireTracking("Fly enabled; region tracking retired"); }
            FlightStatus = enabled ? "Fly: click video to capture · Escape releases" : "Fly off";
            _flyButton.Text = enabled ? "Fly on" : "Fly";
            _lblVisionStatus.Visible = !enabled;
            _lblStatus.Text = FlightStatus;
        }

        internal async Task<bool> BeginFlightCaptureAsync(PictureBox box = null)
        {
            ValidateUiThread();
            box = box ?? _fullscreenBox ?? _videoBox;
            if (!_flyEnabled || _flightCaptured || _flightPreparing || VideoFlightInput.Captured) { return false; }
            if (!FlightVideoReady())
            {
                FlightStatus = "Fly needs fresh video and calibrated gimbal limits with a one-frame queue.";
                _lblStatus.Text = FlightStatus;
                return false;
            }
            if (!IsFlightForeground(box)) { return false; }
            _flightPreparing = true;
            long revision = ++_flightRevision;
            long videoGeneration = _session.Generation;
            var controller = new VideoFlightController(_aircraft, _flightInput, FlightInputCurrent);
            FlightStatus = "Checking flight support…";
            try
            {
                bool ready = await controller.PrepareAsync().ConfigureAwait(false);
                if (IsDisposed || !IsHandleCreated) { return false; }
                return (bool)Invoke(new Func<bool>(() => CompleteFlightCapture(controller, box,
                    revision, videoGeneration, ready)));
            }
            catch (InvalidOperationException) when (IsDisposed || !IsHandleCreated) { return false; }
        }

        private bool CompleteFlightCapture(VideoFlightController controller, PictureBox box,
            long revision, long videoGeneration, bool ready)
        {
            ValidateUiThread();
            try
            {
                if (revision != _flightRevision || !_flyEnabled || IsDisposed) { return false; }
                FlightStatus = controller.Status;
                if (!ready || !FlightVideoReady() || _session.Generation != videoGeneration ||
                    !IsFlightForeground(box) || !_flightInput.Claim()) { return false; }
                RetireTracking("Mouse flight capture retired tracking");
                if (_dragBox != null) { _dragBox.Capture = false; _dragBox = null; }
                _flightController = controller;
                _flightBox = box;
                _flightForm = box.FindForm();
                _flightVideoGeneration = videoGeneration;
                _flightLastSample = Stopwatch.GetTimestamp();
                _flightCaptured = true;
                _flightForm.Deactivate += FlightFocusLost;
                _flightForm.Move += FlightFocusLost;
                _flightBox.Resize += FlightFocusLost;
                Application.AddMessageFilter(this);
                SetFlightCursor(true, box);
                FlightStatus = "Release keys · Esc to release";
                return true;
            }
            catch (Exception)
            { ReleaseFlight("Capture unavailable; controls released"); return false; }
            finally
            {
                _flightPreparing = false;
                if (!IsDisposed) { _lblStatus.Text = FlightStatus; }
            }
        }

        private bool FlightVideoReady() => _aircraft != null && _input.TrackingCalibrated &&
            _input.DropOldFrames && _input.QueueMaxBuffers == 1 && _displayedVideoFrame?.Fresh == true &&
            _displayedVideoFrame.Generation == _session.Generation && _session.State == VideoState.Streaming &&
            GimbalController.TargetPitchDeg >= _input.PitchMinDeg && GimbalController.TargetPitchDeg <= _input.PitchMaxDeg &&
            GimbalController.TargetYawDeg >= _input.YawMinDeg && GimbalController.TargetYawDeg <= _input.YawMaxDeg;

        private bool IsFlightForeground(PictureBox box)
        {
            if (FlightForeground != null) { return FlightForeground(); }
            var form = box?.FindForm();
            return Visible && box?.Visible == true && form?.Visible == true && form.ContainsFocus &&
                GetForegroundWindow() == form.Handle;
        }

        private bool FlightInputCurrent() => _flightCaptured && _flightInput.IsOwner &&
            VideoFlightController.Fresh(Interlocked.Read(ref _flightUiTicks), .25) &&
            VideoFlightController.Fresh(Interlocked.Read(ref _flightVideoTicks), .5) &&
            _session.Generation == _flightVideoGeneration && _session.State == VideoState.Streaming;

        internal void SampleFlight()
        {
            if (!_flightCaptured) { return; }
            if (!_flightController.Active)
            { ReleaseFlight(_flightController.Status); return; }
            if (!FlightVideoReady() || _session.Generation != _flightVideoGeneration)
            { ReleaseFlight("Video unavailable · released"); return; }
            if (!IsFlightForeground(_flightBox))
            { ReleaseFlight("Focus lost · released"); return; }
            if (!_flightController.AircraftCurrent())
            { ReleaseFlight("Aircraft changed · released"); return; }
            long now = Stopwatch.GetTimestamp();
            double elapsed = (now - _flightLastSample) / (double)Stopwatch.Frequency;
            if (elapsed > .25)
            { ReleaseFlight("Input stalled · released"); return; }
            if (elapsed < .05) { return; }
            _flightLastSample = now;
            Interlocked.Exchange(ref _flightUiTicks, now);
            Interlocked.Exchange(ref _flightVideoTicks, _displayedVideoFrame.ReceivedTicks);
            if (!_flightInput.TrySample(FlightKeys(), out var velocity)) { RecenterFlightCursor(); return; }
            _flightController.Sample(velocity);
            MoveFlightGimbal(elapsed);
            FlightStatus = _flightController.Status;
            _lblStatus.Text = FlightStatus;
        }

        private void MoveFlightGimbal(double elapsed)
        {
            Point center = _flightBox.PointToScreen(new Point(_flightBox.Width / 2, _flightBox.Height / 2));
            Point pointer = FlightPointer();
            int dx = pointer.X - center.X, dy = pointer.Y - center.Y;
            RecenterFlightCursor();
            if (dx == 0 && dy == 0) { return; }
            float step = _input.TrackingMaxRateDegSec * (float)elapsed;
            float pitch = GimbalCommand.Clamp(GimbalController.TargetPitchDeg +
                GimbalCommand.Clamp(-dy * .15f * _input.PitchSign, -step, step), _input.PitchMinDeg, _input.PitchMaxDeg);
            float yaw = GimbalCommand.Clamp(GimbalController.TargetYawDeg +
                GimbalCommand.Clamp(dx * .15f * _input.YawSign, -step, step), _input.YawMinDeg, _input.YawMaxDeg);
            var controller = _flightController;
            _ = GimbalController.RequestManualPitchYawAsync(pitch, yaw,
                () => controller.Active && FlightInputCurrent() && controller.AircraftCurrent());
        }

        internal void ReleaseFlight(string reason = "Released")
        {
            ++_flightRevision;
            _flightCaptured = false;
            var controller = _flightController;
            if (controller != null) { FlightRelease = controller.StopAsync(); }
            else { _flightInput.Release(); }
            Application.RemoveMessageFilter(this);
            if (_flightForm != null)
            {
                _flightForm.Deactivate -= FlightFocusLost;
                _flightForm.Move -= FlightFocusLost;
            }
            if (_flightBox != null)
            {
                _flightBox.Resize -= FlightFocusLost;
                SetFlightCursor(false, _flightBox);
            }
            _flightBox = null;
            _flightForm = null;
            FlightStatus = reason;
        }

        private void FlightFocusLost(object sender, EventArgs args) => ReleaseFlight("Focus lost; controls released");
        private void SetFlightCursor(bool captured, PictureBox box)
        {
            if (FlightCursor != null) { FlightCursor(captured, box); return; }
            if (captured)
            {
                _previousCursorClip = Cursor.Clip;
                _previousCursorPosition = Cursor.Position;
                Cursor.Clip = box.RectangleToScreen(box.ClientRectangle);
                Cursor.Hide(); _cursorHidden = true;
                RecenterFlightCursor();
            }
            else
            {
                Cursor.Clip = _previousCursorClip;
                if (_cursorHidden) { Cursor.Show(); _cursorHidden = false; }
                Cursor.Position = _previousCursorPosition;
            }
        }

        private void RecenterFlightCursor()
        {
            if (FlightCursor != null || _flightBox == null) { return; }
            Cursor.Position = _flightBox.PointToScreen(new Point(_flightBox.Width / 2, _flightBox.Height / 2));
        }

        public bool PreFilterMessage(ref Message message)
        {
            if (!_flightCaptured || message.Msg < 0x100 || message.Msg > 0x105) { return false; }
            Keys key = (Keys)message.WParam.ToInt32();
            if (key == Keys.Escape) { ReleaseFlight(); return true; }
            return FlightKey(key) != VideoFlightKeys.None;
        }

        private static VideoFlightKeys ReadFlightKeys()
        {
            VideoFlightKeys result = VideoFlightKeys.None;
            foreach (Keys key in new[] { Keys.W, Keys.S, Keys.A, Keys.D, Keys.Left, Keys.Right, Keys.Space, Keys.ShiftKey })
            { if ((GetAsyncKeyState((int)key) & 0x8000) != 0) { result |= FlightKey(key); } }
            return result;
        }
        private static VideoFlightKeys FlightKey(Keys key)
        {
            switch (key)
            {
                case Keys.W: return VideoFlightKeys.Forward;
                case Keys.S: return VideoFlightKeys.Back;
                case Keys.A: return VideoFlightKeys.Left;
                case Keys.D: return VideoFlightKeys.Right;
                case Keys.Left: return VideoFlightKeys.YawLeft;
                case Keys.Right: return VideoFlightKeys.YawRight;
                case Keys.Space: return VideoFlightKeys.Up;
                case Keys.ShiftKey: case Keys.LShiftKey: case Keys.RShiftKey: return VideoFlightKeys.Down;
                default: return VideoFlightKeys.None;
            }
        }
    }
}
