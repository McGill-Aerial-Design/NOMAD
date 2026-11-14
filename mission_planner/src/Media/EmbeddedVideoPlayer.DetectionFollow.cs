// SPDX-License-Identifier: Apache-2.0
using System.Diagnostics;
using System.Drawing;

namespace NOMAD.MissionPlanner
{
    // Detection-led following: a clicked, re-found or automatically chosen detection seeds the visual tracker,
    // and the gimbal moves only after that tracker verifies the region and StartTracking's interlocks pass.
    public partial class EmbeddedVideoPlayer
    {
        private const double FollowMemorySeconds = 3;
        private volatile bool _followWhenReady, _autoFollow;
        private string _followLabel;
        private RectangleF _followBounds;
        private long _followSeenTicks;

        internal bool AutoFollow => _autoFollow;

        internal void SetAutoFollow(bool enabled)
        {
            ValidateUiThread();
            if (_autoFollow == enabled) { return; }
            _autoFollow = enabled;
            lock (_visionGate) { RecordTrackingEvent(enabled ? "auto-follow-enabled" : "auto-follow-disabled"); }
            if (enabled && !_session.DetectionEnabled) { EnableDetection(true); }
            UpdateVisionStatus();
        }

        // Operator intent (Stop, Clear, manual gimbal, aircraft or video change) ends every automatic follow.
        private void CancelFollow()
        {
            ForgetFollowTarget();
            if (!_autoFollow) { return; }
            _autoFollow = false;
            lock (_visionGate) { RecordTrackingEvent("auto-follow-disabled"); }
        }

        private void ForgetFollowTarget() { _followWhenReady = false; _followLabel = null; }

        private bool FollowDetection(VideoFrame frame, VideoDetection detection)
        {
            if (!RequestRegion(frame, detection.Bounds, detection.Label)) { return false; }
            _followWhenReady = true;
            return true;
        }

        // Runs once per UI frame after the latest tracking sample has been applied.
        private void UpdateDetectionFollow()
        {
            if (!_showControls) { return; }
            RectangleF? region;
            string label;
            bool ready;
            lock (_visionGate) { region = _selectedRegion; label = _selectedLabel; ready = _regionReady; }
            if (region.HasValue)
            {
                if (label != null && ready && (_followWhenReady || IsTracking))
                { _followLabel = label; _followBounds = region.Value; _followSeenTicks = Stopwatch.GetTimestamp(); }
                // An explicit refusal (calibration, limits, telemetry) ends the request; a transient one retries.
                if (_followWhenReady && ready && !IsTracking && (StartTracking() || _trackingFeedback != null))
                { _followWhenReady = false; }
                return;
            }
            _followWhenReady = false;
            var frame = _displayedVideoFrame;
            if (frame?.Fresh != true || frame.Generation != _session.Generation || !CurrentDetections(frame)) { return; }
            float minimum = DetectionTargeting.MinimumSize(frame.Image.Size);
            VideoDetection next = null;
            if (_followLabel != null &&
                (Stopwatch.GetTimestamp() - _followSeenTicks) / (double)Stopwatch.Frequency <= FollowMemorySeconds)
            { next = DetectionTargeting.Reacquire(frame.Detections, _followLabel, _followBounds, minimum); }
            else { _followLabel = null; }
            if (next == null && _autoFollow && _trackingMode == VisualTrackingMode.Object)
            { next = DetectionTargeting.Auto(frame.Detections, frame.Image.Size, minimum); }
            if (next != null) { FollowDetection(frame, next); }
        }

        // Yellow is the verified tracker region; green corners and a caption show the detector agreeing with it.
        private static void PaintConfirmation(Graphics graphics, RectangleF display, VideoDetection detection, Font font)
        {
            float arm = System.Math.Max(6, System.Math.Min(display.Width, display.Height) / 4);
            using (var pen = new Pen(Color.LimeGreen, 3))
            {
                foreach (var corner in new[] { new PointF(display.Left, display.Top), new PointF(display.Right, display.Top),
                    new PointF(display.Left, display.Bottom), new PointF(display.Right, display.Bottom) })
                {
                    float x = corner.X == display.Left ? arm : -arm, y = corner.Y == display.Top ? arm : -arm;
                    graphics.DrawLine(pen, corner, new PointF(corner.X + x, corner.Y));
                    graphics.DrawLine(pen, corner, new PointF(corner.X, corner.Y + y));
                }
            }
            graphics.DrawString(detection.Label + " " + detection.Confidence.ToString("0.00") + " · tracked", font,
                Brushes.Yellow, display.X, System.Math.Max(0, display.Y - font.Height));
        }
    }
}
