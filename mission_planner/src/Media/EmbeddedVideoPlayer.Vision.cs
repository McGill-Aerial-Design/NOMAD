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
using WpfButton = System.Windows.Controls.Button;
using WpfComboBox = System.Windows.Controls.ComboBox;
using WpfTextBlock = System.Windows.Controls.TextBlock;
using System.Windows.Automation;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        private readonly object _visionGate = new object();
        private RectangleF? _selectedRegion;
        private PointF? _selectedAimPoint;
        private string _selectedLabel;
        private string _selectionDetectorIdentity;
        private OutputController.NativeTarget _selectionTarget;
        private MAVState _selectionMav;
        private VisualTrackingMode _trackingMode;
        private WpfComboBox _trackingModeSelector;
        private WpfButton _clearAreaButton;
        private readonly System.Windows.Controls.ContextMenu _trackingHistoryMenu = new System.Windows.Controls.ContextMenu
        { Background = WpfTheme.Card, Foreground = WpfTheme.Text };
        private bool _selectionInitialized, _trackingPaused;
        private bool _regionReady, _trackingVisible;
        private long _regionRevision, _trackingClaim, _selectionStartedTicks;
        private readonly Queue<TrackingTransition> _trackingEvents = new Queue<TrackingTransition>();
        private WpfTextBlock _lblVisionStatus;
        private WpfButton _detectButton, _trackButton;
        private bool _tracking;
        private string _trackingStatus = "Click or drag to select a target";
        private string _trackingFeedback;
        private long _trackingRevision, _trackingManualRevision, _latestSequence, _latestReceivedTicks;
        private VideoTrackingSample _latestTrackingSample;
        private long _lastStepTicks, _lastRequestTicks, _nextRequestTicks, _trackingVideoGeneration;
        private long _commandGeneration = -1, _commandVideoGeneration = -1;
        private int _trackingInFlight, _submittedTrackingCommands;
        private float _trackingPitch, _trackingYaw, _commandedPitch, _commandedYaw;
        private float _trackingAircraftHeading;
        private float _trackingUnappliedYawChange;
        private ServoTrackingController _servoTrackingController;
        private OutputController.NativeTarget _trackingTarget;
        private MAVState _trackingMav;
        internal VideoFrame DisplayedVideoFrame => _displayedVideoFrame;
        internal string TrackingStatus { get { lock (_visionGate) { return _trackingStatus; } } }
        internal RectangleF? SelectedRegion { get { lock (_visionGate) { return _selectedRegion; } } }
        internal bool RegionReady { get { lock (_visionGate) { return _regionReady; } } }
        private bool WaitingForRegionInitialization
        {
            get { lock (_visionGate) { return _selectedRegion.HasValue && !_selectionInitialized &&
                (_session.TrackerLoading ||
                    (Stopwatch.GetTimestamp() - _selectionStartedTicks) / (double)Stopwatch.Frequency < 2); } }
        }
        internal bool IsTracking { get { lock (_visionGate) { return _tracking; } } }
        internal VisualTrackingMode TrackingMode => _trackingMode;
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
            _trackingModeSelector = VideoControls.TrackingModeSelector;
            _trackingModeSelector.SelectedIndex = (int)_trackingMode;
            _trackingModeSelector.SelectionChanged += (sender, args) =>
                SetTrackingMode((VisualTrackingMode)_trackingModeSelector.SelectedIndex);
            _clearAreaButton = VideoControls.ClearButton;
            _clearAreaButton.Click += (sender, args) =>
            { RetireTracking("Target cleared; select again"); UpdateVisionStatus(); };
            _detectButton = VideoControls.DetectButton;
            _detectButton.Click += (sender, args) => EnableDetection(!_session.DetectionEnabled);
            _trackButton = VideoControls.TrackButton;
            _trackButton.Click += (sender, args) =>
            {
                if (IsTracking) { StopTracking(); }
                else { StartTracking(); }
                UpdateVisionStatus();
            };
            VideoControls.SaveButton.Click += async (sender, args) => await SaveObservationAsync();
            VideoControls.HistoryButton.Click += ShowTrackingHistory;
            VideoControls.MoreButton.Click += (sender, args) =>
            {
                bool expand = VisionOptions.Visibility != System.Windows.Visibility.Visible;
                VisionOptions.Visibility = expand ? System.Windows.Visibility.Visible : System.Windows.Visibility.Collapsed;
                VideoControls.MoreButton.Content = expand ? "Less ▴" : "More ▾";
                PerformLayout();
            };
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

        internal void UpdateDetectionModel(string path)
        {
            ValidateUiThread();
            _input.DetectorModelProfilePath = path ?? "";
            _session.ConfigureDetection(_session.DetectionEnabled, _input.DetectorModelProfilePath);
            UpdateVisionStatus();
            _videoBox.Invalidate(); _fullscreenBox?.Invalidate();
        }

        private bool CurrentDetections(VideoFrame frame) => frame != null && _session.DetectionEnabled &&
            frame.DetectionRevision == _session.DetectionRevision;

        internal bool SelectTargetAt(Point point, Size viewport)
        {
            ValidateUiThread();
            var frame = _displayedVideoFrame;
            if (frame == null || !frame.Fresh ||
                frame.Generation != _session.Generation ||
                !VideoGeometry.TryImagePoint(point, frame.Image.Size, viewport, out var pixel)) { return false; }
            var matches = CurrentDetections(frame)
                ? frame.Detections.Where(detection => detection.Bounds.Contains(pixel)).ToArray()
                : Array.Empty<VideoDetection>();
            float minimum = 8 * Math.Max(1, Math.Max(frame.Image.Width, frame.Image.Height) / 640f);
            bool aided = _trackingMode == VisualTrackingMode.Object && matches.Length == 1 &&
                matches[0].Bounds.Width >= minimum && matches[0].Bounds.Height >= minimum;
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
            long revision = _session.RequestRegion(frame, bounds, _trackingMode);
            if (revision == 0)
            { SetTrackingStatus("Select a fresh, larger region after visual tracking is ready"); return false; }
            var selectionTarget = _aircraft == null ? null : OutputController.CaptureTarget(_aircraft);
            var selectionMav = selectionTarget?.Connection.MAV;
            lock (_visionGate)
            {
                _selectedRegion = bounds; _selectedLabel = label; _regionRevision = revision; _regionReady = false;
                _latestTrackingSample = null;
                _selectedAimPoint = null; _selectionInitialized = false;
                _selectionDetectorIdentity = label == null ? null : frame.DetectorIdentity;
                _selectionTarget = selectionTarget; _selectionMav = selectionMav;
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
            _trackingFeedback = null;
            if (!Visible || FindForm()?.Visible != true) { return false; }
            if (!_input.TrackingCalibrated)
            { SetTrackingStatus("Calibrate this input in Settings → Video before tracking"); return false; }
            if (_input.ServoGimbalAxes.Count > 0 && !_input.ServoGimbalAxes.Any(axis => axis.Axis == "Pitch" || axis.Axis == "Yaw"))
            { SetTrackingStatus("Automatic tracking needs a pitch or yaw gimbal axis"); return false; }
            if (_input.ServoGimbalAxes.Any(axis => axis.Axis == "Roll") && Math.Abs(GimbalController.TargetRollDeg) > .1)
            { SetTrackingStatus("Level the gimbal roll before automatic tracking"); return false; }
            if (!_input.DropOldFrames || _input.QueueMaxBuffers != 1)
            { SetTrackingStatus("Tracking needs Drop old frames and a one-frame queue in Settings → Video"); return false; }
            float currentPitch = GimbalController.TargetPitchDeg, currentYaw = GimbalController.TargetYawDeg;
            if (currentPitch < _input.PitchMinDeg || currentPitch > _input.PitchMaxDeg ||
                currentYaw < _input.YawMinDeg || currentYaw > _input.YawMaxDeg)
            { SetTrackingStatus("Move the manual target inside the calibrated limits before tracking"); return false; }
            VideoTrackingSample sample;
            lock (_visionGate) { sample = _latestTrackingSample; }
            if (sample?.Fresh != true || !RegionReady || !SelectedRegion.HasValue ||
                sample.Region?.Revision != _regionRevision || !sample.Region.Found ||
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
                _trackingPaused = false;
                _trackingVisible = true;
                _trackingClaim = GimbalController.ClaimTracking();
                _trackingTarget = target; _trackingMav = mav;
                _trackingManualRevision = manual;
                _trackingVideoGeneration = sample.Generation;
                _trackingPitch = currentPitch; _trackingYaw = currentYaw;
                _trackingAircraftHeading = heading;
                _trackingUnappliedYawChange = 0;
                _servoTrackingController = default;
                _lastStepTicks = sample.ReceivedTicks; _lastRequestTicks = _nextRequestTicks = 0;
                _trackingStatus = "Tracking selected region";
                RecordTrackingEvent("tracking-started");
            }
            UpdateVisionStatus();
            return true;
        }

        internal void StopTracking() => DisarmTracking("Tracking stopped");
        private void OnManualGimbalInput() => DisarmTracking("Manual gimbal input");
        private void OnAircraftChanged() => RetireTracking("Aircraft or connection changed; select again");

        private void RetireTracking(string reason)
        {
            lock (_visionGate)
            {
                bool changed = _tracking || _selectedRegion.HasValue;
                ++_trackingRevision;
                _tracking = false;
                _selectedRegion = null; _selectedLabel = null; _regionReady = false;
                _latestTrackingSample = null;
                _selectionDetectorIdentity = null; _selectedAimPoint = null;
                _selectionTarget = null; _selectionMav = null;
                _selectionInitialized = false; _trackingPaused = false; _trackingUnappliedYawChange = 0;
                _servoTrackingController = default;
                _trackingTarget = null; _trackingMav = null;
                _trackingStatus = reason;
                _trackingFeedback = null;
                if (changed) { RecordTrackingEvent("tracking-retired"); }
            }
            _session?.RetireRegion();
        }

        private void SetTrackingStatus(string status)
        { lock (_visionGate) { _trackingStatus = _trackingFeedback = status; } UpdateVisionStatus(); }

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
            var menu = _trackingHistoryMenu;
            menu.IsOpen = false;
            menu.Items.Clear();
            foreach (var item in events)
            { menu.Items.Add(new System.Windows.Controls.MenuItem
                { Header = item.Utc.ToString("HH:mm:ss") + " UTC  " + item.Reason }); }
            if (events.Length == 0) { menu.Items.Add(new System.Windows.Controls.MenuItem { Header = "No tracking events" }); }
            menu.PlacementTarget = (System.Windows.UIElement)sender;
            menu.Placement = System.Windows.Controls.Primitives.PlacementMode.Bottom;
            menu.IsOpen = true;
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

        private bool TrackingInputCurrent(long revision, long sequence, long? receivedSnapshot = null)
        {
            OutputController.NativeTarget target;
            MAVState mav;
            long received, videoGeneration, regionRevision, claim;
            lock (_visionGate)
            {
                if (!_tracking || _trackingPaused || !_regionReady || !_trackingVisible ||
                    revision != _trackingRevision || (receivedSnapshot.HasValue ? sequence > _latestSequence : sequence != _latestSequence))
                { return false; }
                target = _trackingTarget; mav = _trackingMav;
                received = receivedSnapshot ?? _latestReceivedTicks; videoGeneration = _trackingVideoGeneration;
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

        private bool FreshTrackingVideo() => _displayedVideoFrame?.Fresh == true &&
            _displayedVideoFrame.Generation == _session.Generation && _session.State == VideoState.Streaming &&
            _session.HasRegion(_regionRevision);

        private void UpdateVisionSample(VideoTrackingSample sample)
        {
            lock (_visionGate)
            {
                if (!_selectedRegion.HasValue) { return; }
                if (sample.Generation != _session.Generation || sample.Region?.Revision != _regionRevision ||
                    sample.Sequence <= _latestSequence) { return; }
                // Initialization may outlive the displayed frame; only a fresh later update can establish readiness.
                if (!sample.Fresh && sample.Region.Found && WaitingForRegionInitialization) { return; }
                if (!sample.Fresh)
                {
                    if (_tracking && FreshTrackingVideo()) { PauseTracking("Tracking delayed; waiting for verified image"); }
                    else { DisarmTracking("Video stale"); }
                    return;
                }
                if (_tracking && (sample.ReceivedTicks - _latestReceivedTicks) /
                    (double)Stopwatch.Frequency >= .5)
                {
                    if (FreshTrackingVideo()) { PauseTracking("Tracking delayed; waiting for verified image"); }
                    else { DisarmTracking("Video receipt gap"); }
                }
                _selectionInitialized = true;
                _latestTrackingSample = sample;
                if (sample.Region.State == VisualTrackingState.Searching || sample.Region.State == VisualTrackingState.Outside)
                {
                    PauseTracking(sample.Region.Status);
                    _latestSequence = sample.Sequence; _latestReceivedTicks = sample.ReceivedTicks;
                    return;
                }
                if (!sample.Region.Found || !sample.Region.Bounds.HasValue || !sample.Region.AimPoint.HasValue)
                { RetireTracking(sample.Region.Status ?? "Target lost; select again"); return; }
                _selectedRegion = sample.Region.Bounds; _regionReady = true;
                _selectedAimPoint = sample.Region.AimPoint;
                if (!_tracking) { _trackingStatus = "Region locked; press Track to move the gimbal"; }
                _latestSequence = sample.Sequence; _latestReceivedTicks = sample.ReceivedTicks;
            }
            if (ResumeTracking(sample)) { return; }
            _ = DriveTrackingAsync(sample);
        }

        private async Task DriveTrackingAsync(VideoTrackingSample sample)
        {
            long revision, manual, generation, stepTicks;
            float pitch, yaw, headingChange, compensation;
            ServoTrackingController servoController;
            OutputController.NativeTarget target;
            MAVState mav;
            lock (_visionGate) { target = _trackingTarget; mav = _trackingMav; }
            if (!TryAircraftHeading(target, mav, out float heading)) { return; }
            lock (_visionGate)
            {
                long now = Stopwatch.GetTimestamp();
                double dt = (sample.ReceivedTicks - _lastStepTicks) / (double)Stopwatch.Frequency;
                headingChange = VideoTrackingAngles.HeadingChangeDegrees(_trackingAircraftHeading, heading) +
                    _trackingUnappliedYawChange;
                float step = _input.TrackingMaxRateDegSec * (float)Math.Min(dt, .2);
                compensation = GimbalCommand.Clamp(headingChange, -step, step);
                if (!_tracking || _trackingPaused || !_selectedAimPoint.HasValue || !_regionReady ||
                    now + Stopwatch.Frequency / 100 < _nextRequestTicks) { return; }
                // Propose a value copy: busy/retired requests must not advance PID state.
                servoController = _servoTrackingController;
                stepTicks = _lastStepTicks;
                bool stepped = _input.ServoGimbalAxes.Count > 0
                    ? servoController.TryStep(_input, sample.ImageSize, _selectedAimPoint.Value,
                        _trackingPitch, _trackingYaw, dt, out pitch, out yaw,
                        compensation, _trackingMode == VisualTrackingMode.Object)
                    : VideoTrackingAngles.TryStep(_input, sample.ImageSize, _selectedAimPoint.Value,
                        _trackingPitch, _trackingYaw, dt, out pitch, out yaw,
                        compensation, _trackingMode == VisualTrackingMode.Object);
                if (!stepped)
                {
                    if (_trackingMode == VisualTrackingMode.Object)
                    { _trackingAircraftHeading = heading; _trackingUnappliedYawChange = 0; }
                    return;
                }
                revision = _trackingRevision; manual = _trackingManualRevision; generation = _trackingTarget.Generation;
            }
            if (!TrackingInputCurrent(revision, sample.Sequence) || Interlocked.Exchange(ref _trackingInFlight, 1) != 0)
            { return; }
            try
            {
                lock (_visionGate)
                {
                    if (stepTicks != _lastStepTicks) { return; }
                    _lastRequestTicks = Stopwatch.GetTimestamp();
                    // Keep a 30 Hz schedule with 10 ms jitter allowance; never accumulate overdue sends.
                    _nextRequestTicks = Math.Max(_lastRequestTicks, _nextRequestTicks) + Stopwatch.Frequency / 30;
                }
                var result = await GimbalController.RequestTrackingTargetAsync(pitch, yaw, generation, manual,
                    () => TrackingInputCurrent(revision, sample.Sequence,
                        _input.ServoGimbalAxes.Count > 0 ? sample.ReceivedTicks : (long?)null), _input.Name).ConfigureAwait(false);
                if (result.Submitted)
                {
                    Interlocked.Increment(ref _submittedTrackingCommands);
                    lock (_visionGate)
                    {
                        _commandedPitch = pitch; _commandedYaw = yaw;
                        _commandGeneration = generation; _commandVideoGeneration = sample.Generation;
                        if (_tracking && revision == _trackingRevision)
                        {
                            _trackingPitch = pitch; _trackingYaw = yaw; _trackingAircraftHeading = heading;
                            _servoTrackingController = servoController;
                            _lastStepTicks = sample.ReceivedTicks;
                            // Carry rate clipping; TryStep's deliberate Object attenuation is already excluded.
                            _trackingUnappliedYawChange = (_input.ServoGimbalAxes.Count > 0 &&
                                !_input.ServoGimbalAxes.Any(axis => axis.Axis == "Yaw")) ||
                                yaw == _input.YawMinDeg || yaw == _input.YawMaxDeg
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
            if (SelectedRegion.HasValue && _selectionTarget != null && (!_selectionTarget.IsCurrent ||
                !ReferenceEquals(_selectionTarget.Connection.MAV, _selectionMav)))
            { RetireTracking("Aircraft or connection changed; select again"); }
            if (IsTracking && (_session.Generation != _trackingVideoGeneration ||
                _trackingClaim != GimbalController.CurrentTrackingClaim ||
                _trackingManualRevision != GimbalController.ManualRevision ||
                !TryAircraftHeading(_trackingTarget, _trackingMav, out _)))
            { DisarmTracking("Video, aircraft telemetry or control ownership changed"); }
            if (IsTracking && _latestTrackingSample?.Fresh != true)
            {
                if (FreshTrackingVideo()) { PauseTracking("Tracking delayed; waiting for verified image"); }
                else { DisarmTracking("Video stale"); }
            }
            if (RegionReady && !_session.IsRegionCurrent(_regionRevision))
            {
                lock (_visionGate)
                {
                    if (_session.HasRegion(_regionRevision))
                    { PauseTracking("Target unverified; searching original reference"); }
                    else { RetireTracking("Target lost; select again"); }
                }
            }
            if (_selectionInitialized && SelectedRegion.HasValue && !_session.HasRegion(_regionRevision))
            { RetireTracking("Selected reference retired; select again"); }
            if (SelectedRegion.HasValue && !IsTracking && !WaitingForRegionInitialization &&
                !_selectionInitialized)
            { RetireTracking("Selection expired; select the current image again"); }
            if (SelectedRegion.HasValue && RegionReady && _latestTrackingSample?.Fresh != true)
            { DisarmTracking("Video stale"); }
            string error = _session.TrackerError ?? _session.VisionError;
            _lblVisionStatus.Text = _trackingFeedback ?? (IsTracking ? TrackingStatus.Replace(" selected region", "") :
                _session.TrackerLoading ? "Loading tracker" : error != null ? error : WaitingForRegionInitialization ? "Selecting target" :
                SelectedRegion.HasValue && !RegionReady ? _trackingMode + " searching" :
                !_input.TrackingCalibrated ? "Gimbal tracking needs calibration: Settings → Video" :
                RegionReady ? "Target selected · Start tracking to move the gimbal" : "Click or drag on the video to select a target");
            string detail = TrackingStatus + (error == null ? "" : " | " + error) +
                MotionEstimateDetail() +
                (!_input.TrackingCalibrated ? " | Calibration: Settings → Video" : "");
            _lblStatus.ToolTip = detail;
            _lblVisionStatus.ToolTip = detail;
            _detectButton.Content = _session.DetectionLoading ? "Loading…" : _session.DetectionEnabled ? "Detect on" : "Detect";
            VideoControls.TrackCaption.Text = IsTracking ? "Stop tracking" : "Start tracking";
            AutomationProperties.SetName(_trackButton, VideoControls.TrackCaption.Text);
            _trackButton.IsEnabled = IsTracking || (_input.TrackingCalibrated && RegionReady &&
                _latestTrackingSample?.Fresh == true &&
                _session.IsRegionCurrent(_regionRevision));
            _trackButton.ToolTip = IsTracking ? "Stop automatic gimbal tracking." :
                "Follow the verified selection with the gimbal.";
            _clearAreaButton.IsEnabled = SelectedRegion.HasValue;
        }

        private void PaintDetections(PictureBox box, Graphics graphics)
        {
            var frame = _displayedVideoFrame;
            if (frame == null || !frame.Fresh) { return; }
            var area = VideoGeometry.ImageRectangle(frame.Image.Size, box.ClientSize);
            float scale = area.Width / frame.Image.Width;
            foreach (var detection in CurrentDetections(frame) ? frame.Detections : Array.Empty<VideoDetection>())
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
