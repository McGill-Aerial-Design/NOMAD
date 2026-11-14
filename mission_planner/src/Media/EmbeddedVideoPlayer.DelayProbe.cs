// SPDX-License-Identifier: Apache-2.0
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    // Camera delay check: tilt the gimbal a small step and time how long the still scene takes to change on video.
    // The result includes command transit and mount response, which also delay what tracking sees.
    public partial class EmbeddedVideoPlayer
    {
        private const double SettleTimeoutSeconds = 5, ChangeTimeoutSeconds = 3;
        private const int StillFrames = 8;
        private const float StillNoiseLimit = 8;
        private bool _measuringDelay;

        internal bool MeasuringDelay => _measuringDelay;

        // Returns the delay in milliseconds, or null after showing why the check stopped.
        internal async Task<int?> MeasureCameraDelayAsync()
        {
            ValidateUiThread();
            if (_measuringDelay) { return null; }
            if (_session.State != VideoState.Streaming || _displayedVideoFrame?.Fresh != true)
            { SetTrackingStatus("Play live video before measuring camera delay"); return null; }
            CancelFollow();
            RetireTracking("Measuring camera delay");
            _measuringDelay = true;
            float pitch = GimbalController.TargetPitchDeg, roll = GimbalController.TargetRollDeg;
            float yaw = GimbalController.TargetYawDeg, current = pitch;
            float moved = CameraDelayProbe.StepTarget(pitch, _input.PitchMinDeg, _input.PitchMaxDeg);
            var trials = new List<double>();
            try
            {
                for (int trial = 0; trial < CameraDelayProbe.Trials; trial++)
                {
                    SetTrackingStatus("Measuring camera delay " + (trial + 1) + "/" + CameraDelayProbe.Trials +
                        "; keep the scene still");
                    var still = await StillSceneAsync();
                    if (still.Baseline == null)
                    { SetTrackingStatus("Camera delay check stopped: the scene or video is not still"); return null; }
                    float target = trial % 2 == 0 ? moved : pitch;
                    long sent = Stopwatch.GetTimestamp();
                    var result = await GimbalController.RequestScreenTargetAsync(target, roll, yaw);
                    ValidateUiThread();
                    if (!result.Submitted)
                    { SetTrackingStatus("Camera delay check stopped: " + result.Message); return null; }
                    current = target;
                    long? changed = await FirstChangeAsync(still.Baseline, still.Threshold, sent);
                    if (!changed.HasValue)
                    { SetTrackingStatus("No scene change within 3 s; check that the gimbal moves this camera"); return null; }
                    trials.Add((changed.Value - sent) / (double)Stopwatch.Frequency);
                }
                int delay = CameraDelayProbe.Estimate(trials, _input.LatencyMs);
                SetTrackingStatus("Camera delay measured: " + delay + " ms");
                return delay;
            }
            finally
            {
                _measuringDelay = false;
                if (current != pitch) { _ = GimbalController.RequestScreenTargetAsync(pitch, roll, yaw); }
            }
        }

        internal void ApplyCameraDelay(int milliseconds)
        {
            ValidateUiThread();
            _input.CameraDelayEstimateMs = milliseconds;
            _session.ConfigureTracking(_input);
        }

        // Waits for consecutive quiet frames, then returns the last as the baseline and a noise-based threshold.
        private async Task<(float[] Baseline, float Threshold)> StillSceneAsync()
        {
            var quiet = new Queue<float>();
            float[] previous = null;
            long sequence = -1;
            var clock = Stopwatch.StartNew();
            while (clock.Elapsed.TotalSeconds < SettleTimeoutSeconds)
            {
                var frame = await NextFrameAsync(sequence);
                if (frame == null) { break; }
                sequence = frame.Sequence;
                var sample = CameraDelayProbe.Sample(frame.Image);
                if (previous != null)
                {
                    float difference = CameraDelayProbe.Difference(previous, sample);
                    if (difference > StillNoiseLimit) { quiet.Clear(); }
                    else { quiet.Enqueue(difference); }
                    if (quiet.Count == StillFrames) { return (sample, CameraDelayProbe.ChangeThreshold(quiet.ToArray())); }
                }
                previous = sample;
            }
            return (null, 0);
        }

        private async Task<long?> FirstChangeAsync(float[] baseline, float threshold, long sentTicks)
        {
            long sequence = _displayedVideoFrame?.Sequence ?? -1;
            var clock = Stopwatch.StartNew();
            while (clock.Elapsed.TotalSeconds < ChangeTimeoutSeconds)
            {
                var frame = await NextFrameAsync(sequence);
                if (frame == null) { return null; }
                sequence = frame.Sequence;
                if (frame.ReceivedTicks > sentTicks &&
                    CameraDelayProbe.Difference(baseline, CameraDelayProbe.Sample(frame.Image)) > threshold)
                { return frame.ReceivedTicks; }
            }
            return null;
        }

        // The next fresh displayed frame; frames are only read on the UI thread, where they are replaced.
        private async Task<VideoFrame> NextFrameAsync(long afterSequence)
        {
            var clock = Stopwatch.StartNew();
            while (clock.Elapsed.TotalSeconds < 1)
            {
                await Task.Delay(5);
                ValidateUiThread();
                if (IsDisposed || _session.State != VideoState.Streaming) { return null; }
                var frame = _displayedVideoFrame;
                if (frame?.Fresh == true && frame.Sequence > afterSequence) { return frame; }
            }
            return null;
        }
    }
}
