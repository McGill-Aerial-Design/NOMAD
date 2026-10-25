// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        private readonly object _visionGate = new object();
        private RectangleF? _selectedRegion;
        private string _selectedLabel;
        private bool _regionReady, _trackingVisible;
        private long _regionRevision, _trackingClaim, _selectionStartedTicks;
        private readonly Queue<TrackingTransition> _trackingEvents = new Queue<TrackingTransition>();
        private Label _lblVisionStatus;
        private Button _detectButton, _trackButton, _stopTrackingButton;
        private bool _tracking;
        private string _trackingStatus = "Click or drag to select a target";
        private long _trackingRevision, _trackingManualRevision, _latestSequence, _latestReceivedTicks;
        private long _lastStepTicks, _lastRequestTicks, _trackingVideoGeneration;
        private long _commandGeneration = -1, _commandVideoGeneration = -1;
        private int _trackingInFlight, _submittedTrackingCommands;
        private float _trackingPitch, _trackingYaw, _commandedPitch, _commandedYaw;
        private float _trackingAircraftHeading;
        private float _trackingUnappliedYawChange;
        private OutputController.NativeTarget _trackingTarget;
        private MAVState _trackingMav;
        internal VideoFrame DisplayedVideoFrame => _displayedVideoFrame;
        internal string TrackingStatus { get { lock (_visionGate) { return _trackingStatus; } } }
        internal RectangleF? SelectedRegion { get { lock (_visionGate) { return _selectedRegion; } } }
        internal bool RegionReady { get { lock (_visionGate) { return _regionReady; } } }
        private bool WaitingForRegionInitialization
        {
            get { lock (_visionGate) { return _selectedRegion.HasValue && !_regionReady && !_tracking &&
                (Stopwatch.GetTimestamp() - _selectionStartedTicks) / (double)Stopwatch.Frequency < 2; } }
        }
        internal bool IsTracking { get { lock (_visionGate) { return _tracking; } } }
        internal int SubmittedTrackingCommands => Volatile.Read(ref _submittedTrackingCommands);
        internal float CommandedPitchDeg { get { lock (_visionGate) { return _commandedPitch; } } }
        internal float CommandedYawDeg { get { lock (_visionGate) { return _commandedYaw; } } }
        internal string ObservationDirectory { get; set; }
        internal string LastObservationPath { get; private set; }

        private void InitializeVision()
        {
            if (!_showControls) { return; }
            GimbalController.ManualInput += OnManualGimbalInput;
            OutputController.ConnectionChanged += OnAircraftChanged;
            AttachSelectionMouse(_videoBox);
            var controls = new FlowLayoutPanel
            { Dock = DockStyle.Fill, AutoSize = true, WrapContents = true, Margin = Padding.Empty };
            _detectButton = ControlFactory.Button("Detect", (sender, args) => EnableDetection(!_session.DetectionEnabled),
                width: 65, height: 28);
            _detectButton.MinimumSize = _detectButton.Size;
            _detectButton.AutoSize = true;
            _detectButton.AutoSizeMode = AutoSizeMode.GrowAndShrink;
            _trackButton = ControlFactory.Button("Track", (sender, args) => StartTracking(), width: 60, height: 28);
            _stopTrackingButton = ControlFactory.Button("Stop tracking", (sender, args) => StopTracking(),
                width: 100, height: 28);
            var save = ControlFactory.Button("Save observation", (sender, args) => SaveObservation(), width: 130, height: 28);
            var history = ControlFactory.Button("History", ShowTrackingHistory, width: 70, height: 28);
            controls.Controls.AddRange(new Control[] { _detectButton, _trackButton, _stopTrackingButton, save, history });
            _videoToolTips.SetToolTip(_detectButton, "Detect objects in video. Detection alone never moves the gimbal.");
            _videoToolTips.SetToolTip(_trackButton, "Follow the selected region with the gimbal.");
            _videoToolTips.SetToolTip(_stopTrackingButton, "Stop automatic gimbal tracking.");
            _videoToolTips.SetToolTip(save, "Save the current image and observation details.");
            _videoToolTips.SetToolTip(history, "Review the last 32 selection and tracking transitions.");
            Toolbar.RowCount = 2;
            Toolbar.Controls.Add(controls, 0, 1);
            Toolbar.SetColumnSpan(controls, 2);
            _lblVisionStatus = new Label
            { Dock = DockStyle.Bottom, Height = 36, ForeColor = Color.Gainsboro, TextAlign = ContentAlignment.MiddleLeft };
            Controls.Add(_lblVisionStatus);
            UpdateVisionStatus();
        }

        internal void EnableDetection(bool enabled)
        {
            ValidateUiThread();
            if (!_showControls || IsDisposed) { return; }
            _session.ConfigureDetection(enabled, _input.DetectorModelProfilePath);
            lock (_visionGate) { RecordTrackingEvent(enabled ? "detection-enabled" : "detection-disabled"); }
            UpdateVisionStatus();
            _videoBox.Invalidate(); _fullscreenBox?.Invalidate();
        }

        internal bool SelectTargetAt(Point point, Size viewport)
        {
            ValidateUiThread();
            var frame = _displayedVideoFrame;
            if (frame == null || !frame.Fresh ||
                frame.Generation != _session.Generation ||
                !VideoGeometry.TryImagePoint(point, frame.Image.Size, viewport, out var pixel)) { return false; }
            var matches = _session.DetectionEnabled
                ? frame.Detections.Where(detection => detection.Bounds.Contains(pixel)).ToArray()
                : Array.Empty<VideoDetection>();
            float minimum = 8 * Math.Max(1, Math.Max(frame.Image.Width, frame.Image.Height) / 640f);
            bool aided = matches.Length == 1 && matches[0].Bounds.Width >= minimum && matches[0].Bounds.Height >= minimum;
            var bounds = aided ? matches[0].Bounds : VideoGeometry.DefaultRegion(pixel, frame.Image.Size);
            return RequestRegion(frame, bounds, aided ? matches[0].Label : null);
        }

        internal bool SelectRegion(RectangleF imageRegion)
        {
            ValidateUiThread();
            var frame = _displayedVideoFrame;
            return frame != null && RequestRegion(frame, imageRegion, null);
        }

        private bool RequestRegion(VideoFrame frame, RectangleF bounds, string label)
        {
            RetireTracking("Initializing selected region");
            long revision = _session.RequestRegion(frame, bounds);
            if (revision == 0)
            { SetTrackingStatus("Select a fresh, larger region after visual tracking is ready"); return false; }
            lock (_visionGate)
            {
                _selectedRegion = bounds; _selectedLabel = label; _regionRevision = revision; _regionReady = false;
                _selectionStartedTicks = Stopwatch.GetTimestamp();
                _latestSequence = frame.Sequence; _latestReceivedTicks = frame.ReceivedTicks;
                _trackingStatus = "Initializing selected region";
                RecordTrackingEvent("target-selected");
            }
            UpdateVisionStatus();
            _videoBox.Invalidate(); _fullscreenBox?.Invalidate();
            return true;
        }

        internal bool StartTracking()
        {
            ValidateUiThread();
            if (!Visible || FindForm()?.Visible != true) { return false; }
            if (!_input.TrackingCalibrated)
            { SetTrackingStatus("Calibrate this input in Settings → Video before tracking"); return false; }
            if (!_input.DropOldFrames || _input.QueueMaxBuffers != 1)
            { SetTrackingStatus("Tracking needs Drop old frames and a one-frame queue in Settings → Video"); return false; }
            float currentPitch = GimbalController.TargetPitchDeg, currentYaw = GimbalController.TargetYawDeg;
            if (currentPitch < _input.PitchMinDeg || currentPitch > _input.PitchMaxDeg ||
                currentYaw < _input.YawMinDeg || currentYaw > _input.YawMaxDeg)
            { SetTrackingStatus("Move the manual target inside the calibrated limits before tracking"); return false; }
            var frame = _displayedVideoFrame;
            if (frame?.Fresh != true || !RegionReady || !SelectedRegion.HasValue ||
                frame.Region?.Revision != _regionRevision || !frame.Region.Found ||
                !_session.IsRegionCurrent(_regionRevision)) { return false; }
            var target = _aircraft == null ? null : OutputController.CaptureTarget(_aircraft);
            var mav = target?.Connection.MAV;
            if (!TryAircraftHeading(target, mav, out float heading))
            { SetTrackingStatus("Tracking requires fresh aircraft attitude and heartbeat"); return false; }
            long manual = GimbalController.ManualRevision;
            lock (_visionGate)
            {
                if (!_selectedRegion.HasValue || !_regionReady) { return false; }
                ++_trackingRevision;
                _tracking = true;
                _trackingVisible = true;
                _trackingClaim = GimbalController.ClaimTracking();
                _trackingTarget = target; _trackingMav = mav;
                _trackingManualRevision = manual;
                _trackingVideoGeneration = frame.Generation;
                _trackingPitch = currentPitch; _trackingYaw = currentYaw;
                _trackingAircraftHeading = heading;
                _trackingUnappliedYawChange = 0;
                _lastStepTicks = frame.ReceivedTicks; _lastRequestTicks = 0;
                _trackingStatus = "Tracking selected region";
                RecordTrackingEvent("tracking-started");
            }
            UpdateVisionStatus();
            return true;
        }

        internal void StopTracking() => RetireTracking("Tracking stopped; select an object again");
        private void OnManualGimbalInput() => RetireTracking("Manual gimbal input; select an object again");
        private void OnAircraftChanged() => RetireTracking("Aircraft or connection changed; select again");

        private void RetireTracking(string reason)
        {
            lock (_visionGate)
            {
                bool changed = _tracking || _selectedRegion.HasValue;
                ++_trackingRevision;
                _tracking = false;
                _selectedRegion = null; _selectedLabel = null; _regionReady = false;
                _trackingTarget = null; _trackingMav = null;
                _trackingStatus = reason;
                if (changed) { RecordTrackingEvent("tracking-retired"); }
            }
            _session?.RetireRegion();
        }

        private void SetTrackingStatus(string status)
        { lock (_visionGate) { _trackingStatus = status; } UpdateVisionStatus(); }

        private void RetireTrackingRequest(long revision, string reason)
        {
            lock (_visionGate)
            {
                if (_tracking && revision == _trackingRevision) { RetireTracking(reason); }
            }
        }

        private void RecordTrackingEvent(string transition)
        {
            while (_trackingEvents.Count >= 32) { _trackingEvents.Dequeue(); }
            _trackingEvents.Enqueue(new TrackingTransition
            { Transition = transition, Reason = _trackingStatus, Utc = DateTime.UtcNow, Sequence = _latestSequence });
        }

        private void ShowTrackingHistory(object sender, EventArgs args)
        {
            TrackingTransition[] events;
            lock (_visionGate) { events = _trackingEvents.Reverse().ToArray(); }
            var menu = new ContextMenuStrip
            { BackColor = NOMADTheme.CARD_BG, ForeColor = NOMADTheme.TEXT_PRIMARY, Font = NOMADTheme.Font() };
            foreach (var item in events)
            { menu.Items.Add(new ToolStripLabel(item.Utc.ToString("HH:mm:ss") + " UTC  " + item.Reason)); }
            if (events.Length == 0) { menu.Items.Add(new ToolStripLabel("No tracking events")); }
            menu.Closed += (source, eventArgs) => menu.Dispose();
            var button = (Control)sender;
            menu.Show(button, new Point(0, button.Height));
        }

        private static bool FreshAircraft(OutputController.NativeTarget target, MAVState mav)
        {
            if (target == null || mav == null || !target.IsCurrent || !ReferenceEquals(target.Connection.MAV, mav))
            { return false; }
            var packet = mav.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.HEARTBEAT);
            if (packet == null || packet.sysid != target.System || packet.compid != target.Component ||
                packet.rxtime.Kind != DateTimeKind.Utc) { return false; }
            double age = (DateTime.UtcNow - packet.rxtime).TotalSeconds;
            return age >= 0 && age <= 3.5 && packet.ToStructure<MAVLink.mavlink_heartbeat_t>().autopilot ==
                (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA;
        }

        private bool TrackingInputCurrent(long revision, long sequence)
        {
            OutputController.NativeTarget target;
            MAVState mav;
            long received, videoGeneration, regionRevision, claim;
            lock (_visionGate)
            {
                if (!_tracking || !_trackingVisible || revision != _trackingRevision || sequence != _latestSequence)
                { return false; }
                target = _trackingTarget; mav = _trackingMav;
                received = _latestReceivedTicks; videoGeneration = _trackingVideoGeneration;
                regionRevision = _regionRevision; claim = _trackingClaim;
            }
            double age = (Stopwatch.GetTimestamp() - received) / (double)Stopwatch.Frequency;
            return age >= 0 && age < .5 && _session.State == VideoState.Streaming &&
                claim == GimbalController.CurrentTrackingClaim && _session.IsRegionCurrent(regionRevision) &&
                _session.Generation == videoGeneration &&
                TryAircraftHeading(target, mav, out _);
        }

        private static bool TryAircraftHeading(OutputController.NativeTarget target, MAVState mav, out float heading)
        {
            heading = 0;
            if (!FreshAircraft(target, mav)) { return false; }
            var packet = mav.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.ATTITUDE);
            if (packet == null || packet.sysid != target.System || packet.compid != target.Component ||
                packet.rxtime.Kind != DateTimeKind.Utc) { return false; }
            double age = (DateTime.UtcNow - packet.rxtime).TotalSeconds;
            var attitude = packet.ToStructure<MAVLink.mavlink_attitude_t>();
            return VideoTrackingAngles.TryPredictHeading(attitude.yaw, attitude.roll, attitude.pitch,
                attitude.pitchspeed, attitude.yawspeed, age, out heading);
        }

        private void UpdateVisionFrame(VideoFrame frame)
        {
            lock (_visionGate)
            {
                if (!_selectedRegion.HasValue) { return; }
                if (frame.Region?.Revision != _regionRevision) { return; }
                // Initialization may outlive the displayed frame; only a fresh later update can establish readiness.
                if (!frame.Fresh && frame.Region.Found && WaitingForRegionInitialization) { return; }
                if (!frame.Fresh || !frame.Region.Found || !frame.Region.Bounds.HasValue)
                {
                    RetireTracking(!frame.Fresh ? "Video stale; select again" :
                        frame.Region.Status ?? "Target lost; select again");
                    return;
                }
                _selectedRegion = frame.Region.Bounds; _regionReady = true;
                if (!_tracking) { _trackingStatus = "Region locked; press Track to move the gimbal"; }
                _latestSequence = frame.Sequence; _latestReceivedTicks = frame.ReceivedTicks;
            }
            _ = DriveTrackingAsync(frame);
        }

        private async Task DriveTrackingAsync(VideoFrame frame)
        {
            long revision, manual, generation;
            float pitch, yaw, headingChange, compensation;
            OutputController.NativeTarget target;
            MAVState mav;
            lock (_visionGate) { target = _trackingTarget; mav = _trackingMav; }
            if (!TryAircraftHeading(target, mav, out float heading)) { return; }
            lock (_visionGate)
            {
                long now = Stopwatch.GetTimestamp();
                double dt = (frame.ReceivedTicks - _lastStepTicks) / (double)Stopwatch.Frequency;
                headingChange = VideoTrackingAngles.HeadingChangeDegrees(_trackingAircraftHeading, heading) +
                    _trackingUnappliedYawChange;
                float step = _input.TrackingMaxRateDegSec * (float)Math.Min(dt, .2);
                compensation = GimbalCommand.Clamp(headingChange, -step, step);
                if (!_tracking || !_selectedRegion.HasValue || !_regionReady ||
                    (now - _lastRequestTicks) / (double)Stopwatch.Frequency < .05 ||
                    !VideoTrackingAngles.TryStep(_input, frame.Image.Size, _selectedRegion.Value,
                        _trackingPitch, _trackingYaw, dt, out pitch, out yaw,
                        compensation)) { return; }
                revision = _trackingRevision; manual = _trackingManualRevision; generation = _trackingTarget.Generation;
                _lastStepTicks = frame.ReceivedTicks;
            }
            if (!TrackingInputCurrent(revision, frame.Sequence) || Interlocked.Exchange(ref _trackingInFlight, 1) != 0)
            { return; }
            try
            {
                lock (_visionGate) { _lastRequestTicks = Stopwatch.GetTimestamp(); }
                var result = await GimbalController.RequestTrackingTargetAsync(pitch, yaw, generation, manual,
                    () => TrackingInputCurrent(revision, frame.Sequence)).ConfigureAwait(false);
                if (result.Submitted)
                {
                    Interlocked.Increment(ref _submittedTrackingCommands);
                    lock (_visionGate)
                    {
                        _commandedPitch = pitch; _commandedYaw = yaw;
                        _commandGeneration = generation; _commandVideoGeneration = frame.Generation;
                        if (_tracking && revision == _trackingRevision)
                        {
                            _trackingPitch = pitch; _trackingYaw = yaw; _trackingAircraftHeading = heading;
                            _trackingUnappliedYawChange = yaw == _input.YawMinDeg || yaw == _input.YawMaxDeg
                                ? 0 : headingChange - compensation;
                            _trackingStatus = yaw == _input.YawMinDeg || yaw == _input.YawMaxDeg
                                ? "Tracking · pan target limit"
                                : pitch == _input.PitchMinDeg || pitch == _input.PitchMaxDeg
                                    ? "Tracking · tilt target limit" : "Tracking selected region";
                        }
                    }
                }
                // A known pre-dispatch busy result drops this sample; only a new fresh frame may try again.
                else if (result.Outcome != GimbalRequestOutcome.Busy)
                { RetireTrackingRequest(revision, "Gimbal request not confirmed; select again before retrying"); }
            }
            catch (Exception) { RetireTrackingRequest(revision, "Gimbal request failed; check before selecting again"); }
            finally { Interlocked.Exchange(ref _trackingInFlight, 0); }
        }

        private void UpdateVisionStatus()
        {
            if (!_showControls || _lblVisionStatus == null) { return; }
            long revision, sequence;
            lock (_visionGate) { revision = _trackingRevision; sequence = _latestSequence; }
            if (IsTracking && !TrackingInputCurrent(revision, sequence))
            { RetireTracking("Video or aircraft telemetry stale; select again"); }
            if (RegionReady && !_session.IsRegionCurrent(_regionRevision))
            { RetireTracking("Target lost; select again"); }
            if (SelectedRegion.HasValue && !IsTracking && !WaitingForRegionInitialization &&
                (!RegionReady || _displayedVideoFrame?.Fresh != true))
            { RetireTracking("Selection expired; select the current image again"); }
            string error = _session.TrackerError ?? _session.VisionError;
            _lblVisionStatus.Text = IsTracking ? TrackingStatus.Replace(" selected region", "") :
                error != null ? "Vision unavailable" : WaitingForRegionInitialization ? "Selecting target" :
                !_input.TrackingCalibrated ? "Calibration required" : RegionReady ? "Target selected" : "Select a target";
            string detail = TrackingStatus + (error == null ? "" : " | " + error) +
                (!_input.TrackingCalibrated ? " | Calibration: Settings → Video" : "");
            _videoToolTips.SetToolTip(_lblStatus, detail);
            _videoToolTips.SetToolTip(_lblVisionStatus, detail);
            _detectButton.Text = _session.DetectionEnabled ? "Detect on" : "Detect";
            _trackButton.Enabled = _input.TrackingCalibrated && RegionReady && !IsTracking;
            _stopTrackingButton.Enabled = IsTracking || SelectedRegion.HasValue;
        }

        private void PaintDetections(PictureBox box, Graphics graphics)
        {
            var frame = _displayedVideoFrame;
            if (frame == null || !frame.Fresh) { return; }
            var area = VideoGeometry.ImageRectangle(frame.Image.Size, box.ClientSize);
            float scale = area.Width / frame.Image.Width;
            foreach (var detection in _session.DetectionEnabled ? frame.Detections : Array.Empty<VideoDetection>())
            {
                var bounds = detection.Bounds;
                var display = new RectangleF(area.X + bounds.X * scale, area.Y + bounds.Y * scale,
                    bounds.Width * scale, bounds.Height * scale);
                using (var pen = new Pen(Color.LimeGreen, 2))
                {
                    graphics.DrawRectangle(pen, display.X, display.Y, display.Width, display.Height);
                    graphics.DrawString(detection.Label + " " + detection.Confidence.ToString("0.00"), Font,
                        Brushes.LimeGreen, display.X, Math.Max(area.Y, display.Y - Font.Height));
                }
            }
            PaintSelectedRegion(box, graphics, area, scale);
        }
    }

    internal sealed class TrackingTransition
    {
        public string Transition { get; set; }
        public string Reason { get; set; }
        public DateTime Utc { get; set; }
        public long Sequence { get; set; }
    }
}
