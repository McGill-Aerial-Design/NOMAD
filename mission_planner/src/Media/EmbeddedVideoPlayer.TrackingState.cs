// SPDX-License-Identifier: Apache-2.0
using System.Drawing;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        internal void SetTrackingMode(VisualTrackingMode mode)
        {
            ValidateUiThread();
            if (_trackingMode == mode) { return; }
            RetireTracking("Tracking mode changed; select again"); _trackingMode = mode;
            if (_trackingModeSelector != null) { _trackingModeSelector.SelectedIndex = (int)mode; }
            if (_clearAreaButton != null) { _clearAreaButton.Visible = mode == VisualTrackingMode.Area; }
            UpdateVisionStatus();
        }

        private void DisarmTracking(string reason)
        {
            if (_trackingMode != VisualTrackingMode.Area || !SelectedRegion.HasValue)
            { RetireTracking(reason + "; select again"); return; }
            lock (_visionGate)
            {
                ++_trackingRevision; _tracking = false; _trackingPaused = false;
                _trackingTarget = null; _trackingMav = null; _trackingUnappliedYawChange = 0;
                _regionReady = _regionReady && _displayedVideoFrame?.Fresh == true &&
                    _session.IsRegionCurrent(_regionRevision);
                if (!_regionReady) { _selectedAimPoint = null; }
                _lastStepTicks = _lastRequestTicks = 0;
                _trackingStatus = reason + "; area retained, press Track when verified";
                RecordTrackingEvent("tracking-disarmed");
            }
        }

        private void PauseAreaTracking(string reason)
        {
            _regionReady = false; _selectedAimPoint = null;
            if (_tracking && !_trackingPaused)
            { ++_trackingRevision; _trackingPaused = true; RecordTrackingEvent("tracking-paused"); }
            _trackingUnappliedYawChange = 0; _lastStepTicks = _lastRequestTicks = 0;
            _trackingStatus = reason;
        }

        private bool ResumeAreaTracking(VideoFrame frame)
        {
            long revision;
            OutputController.NativeTarget target;
            MAVState mav;
            lock (_visionGate)
            {
                if (!_tracking || !_trackingPaused) { return false; }
                revision = _trackingRevision; target = _trackingTarget; mav = _trackingMav;
            }
            // Native dispatch holds its output lock before entering our input guard.
            bool validHeading = TryAircraftHeading(target, mav, out float heading);
            float pitch = GimbalController.TargetPitchDeg, yaw = GimbalController.TargetYawDeg;
            lock (_visionGate)
            {
                if (!_tracking || !_trackingPaused || revision != _trackingRevision ||
                    !ReferenceEquals(target, _trackingTarget) || !ReferenceEquals(mav, _trackingMav) ||
                    frame.Region?.Revision != _regionRevision || frame.Sequence != _latestSequence ||
                    !_regionReady || !frame.Fresh) { return true; }
                if (!validHeading) { DisarmTracking("Aircraft attitude unavailable"); return true; }
                _trackingAircraftHeading = heading; _trackingUnappliedYawChange = 0;
                _trackingPitch = pitch; _trackingYaw = yaw;
                _lastStepTicks = frame.ReceivedTicks; _lastRequestTicks = 0; _trackingPaused = false;
                _trackingStatus = "Area verified; tracking resumed"; RecordTrackingEvent("tracking-resumed");
            }
            return true;
        }
    }
}
