// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using System.Diagnostics;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    internal enum VideoState { Stopped, Starting, Streaming, Stopping, Disposed }

    // The worker alone touches the pipeline; consumers own frames returned by TakeFrame.
    internal sealed partial class VideoSession : IDisposable
    {
        private readonly object _gate = new object();
        private readonly bool _isolated;
        private bool _plainRecovery;
        private VideoTrackingSample _pendingTracking;
        private long _trackingSequence;
        private readonly Func<IVideoPipeline> _createPipeline;
        private CancellationTokenSource _cancellation;
        private Task _worker = Task.CompletedTask;
        private VideoFrame _pendingFrame;
        private readonly Func<string, CancellationToken, IVideoDetector> _createDetector;
        private CancellationTokenSource _visionCancellation;
        private bool _detectionEnabled;
        private string _modelProfilePath = "", _visionError;
        private long _detectionRevision, _sequence;
        private long _generation;
        private VideoState _state;
        private string _error;
        private readonly NOMADConfig _trackingAircraft;
        private VideoInputProfile _trackingProfile;

        public VideoSession(Func<IVideoPipeline> createPipeline,
            Func<string, CancellationToken, IVideoDetector> createDetector = null,
            Func<CancellationToken, VisualTrackingMode, IVisualTracker> createTracker = null,
            NOMADConfig trackingAircraft = null, VideoInputProfile trackingProfile = null)
        {
            _createPipeline = createPipeline;
            _isolated = createPipeline == MediaWorkerPipeline.Create;
            if (_isolated) { MediaWorkerHost.Failed += OnMediaWorkerFailed; }
            _createDetector = createDetector ?? VisionDetectorFactory.Create;
            _createTracker = createTracker;
            _trackingAircraft = trackingAircraft; _trackingProfile = trackingProfile?.Copy() ?? new VideoInputProfile();
        }

        internal void ConfigureTracking(VideoInputProfile profile)
        { lock (_gate) { _trackingProfile = profile.Copy(); RetireRegion(); } }

        public VideoState State { get { lock (_gate) { return _state; } } }
        public string Error { get { lock (_gate) { return _error; } } }
        internal string VisionError { get { lock (_gate) { return _visionError; } } }
        internal bool DetectionEnabled { get { lock (_gate) { return _detectionEnabled; } } }
        internal void ConfigureDetection(bool enabled, string modelProfilePath)
        {
            lock (_gate)
            {
                CancelLater(_visionCancellation);
                ++_detectionRevision;
                _detectionEnabled = enabled;
                _modelProfilePath = modelProfilePath ?? "";
                _visionError = null;
                _detectionLoading = false;
                ClearPendingFrame();
            }
        }
        internal Task Completion { get { lock (_gate) { return _worker; } } }
        internal long Generation { get { lock (_gate) { return _generation; } } }
        public bool IsActive => State == VideoState.Starting || State == VideoState.Streaming;

        public bool Start(string pipeline, string shareKey = null)
        {
            lock (_gate)
            {
                if (_state != VideoState.Stopped || !_worker.IsCompleted)
                {
                    return false;
                }
                if (_isolated) { MediaWorkerHost.RequestStart(); }
                _plainRecovery = false;
                _cancellation?.Dispose();
                _cancellation = new CancellationTokenSource();
                var token = _cancellation.Token;
                var generation = ++_generation;
                _error = null;
                _state = VideoState.Starting;
                _worker = Task.Run(() => Run(pipeline, shareKey, generation, token));
                return true;
            }
        }

        private void Run(string pipeline, string shareKey, long generation, CancellationToken token)
        {
            try
            {
                while (!token.IsCancellationRequested)
                {
                    try
                    {
                        using (var resource = shareKey == null ? _createPipeline() :
                            SharedVideoCapture.Create(shareKey, _createPipeline))
                        {
                            resource.Start(pipeline, token);
                            token.ThrowIfCancellationRequested();
                            lock (_gate)
                            {
                                generation = _generation;
                                if (_state == VideoState.Stopping || _state == VideoState.Disposed) { return; }
                                _state = VideoState.Streaming;
                            }
                            ReadFrames(resource, generation, token);
                        }
                        return;
                    }
                    catch (Exception error) when (_isolated && IsWorkerFailure(error) &&
                        MediaWorkerHost.CanRecover && !token.IsCancellationRequested)
                    {
                        lock (_gate)
                        {
                            if (_state == VideoState.Stopping || _state == VideoState.Disposed) { return; }
                            _state = VideoState.Starting;
                            generation = _generation;
                        }
                    }
                }
            }
            catch (OperationCanceledException) when (token.IsCancellationRequested) { }
            catch (Exception error)
            { lock (_gate) { if (_state != VideoState.Disposed && !(token.IsCancellationRequested && IsWorkerFailure(error)))
                { _error = error.Message; } } }
            finally
            {
                lock (_gate)
                {
                    if (_state != VideoState.Stopping && _state != VideoState.Disposed)
                    {
                        RetireRegion(); ClearPendingFrame(); _state = VideoState.Stopped;
                        _cancellation?.Dispose(); _cancellation = null;
                    }
                }
            }
        }

        private static bool IsWorkerFailure(Exception error) => error is MediaWorkerException ||
            (error.InnerException != null && IsWorkerFailure(error.InnerException));

        private void OnMediaWorkerFailed()
        {
            lock (_gate)
            {
                if (_state == VideoState.Stopped || _state == VideoState.Stopping || _state == VideoState.Disposed) { return; }
                ++_generation; ++_detectionRevision; _plainRecovery = true; CancelLater(_visionCancellation); _detectionEnabled = false; _detectionLoading = false;
                RetireRegion(); ClearPendingFrame(); _pendingTracking = null;
                _visionError = "Media worker stopped; detection and tracking are off. Select again after video recovers.";
                _state = VideoState.Starting;
            }
        }

        private void ReadFrames(IVideoPipeline pipeline, long generation, CancellationToken token)
        {
            using (var source = CancellationTokenSource.CreateLinkedTokenSource(token))
            using (var motionHistory = new VideoMotionHistory(_trackingAircraft))
            {
                var detection = new DetectionWorker(this, source.Token);
                var tracking = new RegionWorker(this, source.Token);
                var display = new DisplayWindow(this, generation, source.Token);
                var detector = new AnalysisLane((candidate, frame) =>
                {
                    try
                    {
                        var current = detection.Current(candidate.Frame.DetectionRevision,
                            candidate.Detect, candidate.Model);
                        var values = candidate.Detect && current != null
                            ? current.Detect(frame.Image, detection.Token) : Array.Empty<VideoDetection>();
                        lock (candidate.Gate)
                        { candidate.Detections = values; candidate.Identity = current?.Identity ?? ""; }
                    }
                    catch (OperationCanceledException) { }
                    catch (Exception error) { detection.Fail(candidate.Frame.DetectionRevision, error); }
                    finally { lock (candidate.Gate) { candidate.DetectionDone = true; } }
                }, source.Token, cleanup: detection.Dispose);
                var tracker = new AnalysisLane((candidate, frame) =>
                {
                    try
                    {
                        var region = tracking.Update(frame.Image, frame.ReceivedTicks, frame.Sequence, frame.Motion);
                        lock (candidate.Gate) { candidate.Region = region; }
                        if (region != null)
                        { PublishTrackingSample(generation, new VideoTrackingSample(frame.Image.Size, frame.Sequence,
                            generation, frame.ReceivedTicks, frame.ReceivedUtc, region, frame.Motion)); }
                    }
                    catch (OperationCanceledException) { }
                    catch (Exception error) { Log.Error("Tracking unavailable: " + error.GetType().Name); }
                    finally { lock (candidate.Gate) { candidate.TrackingDone = true; } }
                }, source.Token, tracking.Warm, tracking.Dispose);
                try
                {
                    while (!source.IsCancellationRequested)
                    {
                        var image = pipeline.ReadFrame(source.Token, out long received, out DateTime utc);
                        if (image == null) { return; }
                        VideoFrame frame = null;
                        try
                        {
                            image = MediaPixels.Own(image, received);
                            if (_trackingAircraft != null) { motionHistory.Capture(received); }
                            var motion = motionHistory.Snapshot(received, _trackingProfile.CameraDelayEstimateMs);
                            bool enabled; string path; long revision;
                            lock (_gate) { enabled = _detectionEnabled; path = _modelProfilePath; revision = _detectionRevision; }
                            frame = new VideoFrame(image, Array.Empty<VideoDetection>(), Interlocked.Increment(ref _sequence),
                                generation, received, utc, "", detectionRevision: revision, motion: motion);
                            image = null;
                            var candidate = new DisplayCandidate(frame, enabled, path);
                            detector.Offer(candidate); tracker.Offer(candidate); display.Offer(candidate);
                            frame = null;
                        }
                        finally { frame?.Dispose(); MediaPixels.Dispose(image); }
                    }
                }
                finally
                {
                    source.Cancel(); detector.Stop(); tracker.Stop(); display.Stop();
                    Task.WhenAll(detector.Completion, tracker.Completion, display.Completion).GetAwaiter().GetResult();
                }
            }
        }

        internal void PublishTrackingSample(long generation, VideoTrackingSample sample)
        {
            lock (_gate)
            {
                if (generation != _generation || sample.Generation != generation || _state != VideoState.Streaming ||
                    sample.Region == null || sample.Region.Revision != _selectionRevision ||
                    sample.Sequence <= _trackingSequence) { return; }
                _trackingSequence = sample.Sequence; _pendingTracking = sample;
            }
        }

        internal VideoTrackingSample TakeTrackingSample()
        { lock (_gate) { var result = _pendingTracking; _pendingTracking = null; return result; } }

        private static void CancelLater(CancellationTokenSource source)
        {
            if (source == null) { return; }
            Task.Run(() => { try { source.Cancel(); } catch (ObjectDisposedException) { } });
        }

        internal void PublishFrame(long generation, Bitmap frame) =>
            PublishVideoFrame(generation, new VideoFrame(frame, Array.Empty<VideoDetection>(),
                Interlocked.Increment(ref _sequence), generation, Stopwatch.GetTimestamp(), DateTime.UtcNow, ""));

        internal void PublishVideoFrame(long generation, VideoFrame frame)
        {
            lock (_gate)
            {
                if (generation != _generation || _state != VideoState.Streaming)
                {
                    frame.Dispose();
                    return;
                }
                ClearPendingFrame();
                _pendingFrame = frame;
                if (frame.Region != null) { PublishTrackingSample(generation, new VideoTrackingSample(frame)); }
            }
        }

        public VideoFrame TakeFrame()
        {
            lock (_gate)
            {
                var frame = _pendingFrame;
                _pendingFrame = null;
                return frame;
            }
        }

        public void Stop() => Stop(false);
        public void Dispose() => Stop(true);

        private void Stop(bool disposing)
        {
            lock (_gate)
            {
                long generation = ++_generation;
                RetireRegion(); ClearPendingFrame(); _pendingTracking = null;
                _state = disposing || _state == VideoState.Disposed ? VideoState.Disposed : VideoState.Stopping;
                if (disposing && _isolated) { MediaWorkerHost.Failed -= OnMediaWorkerFailed; }
                var source = _cancellation;
                CancelLater(source);
                var prior = _worker;
                _worker = Task.Run(() => FinishStop(prior, source, generation));
            }
        }

        private async Task FinishStop(Task worker, CancellationTokenSource source, long generation)
        {
            await Task.Yield();
            try { await worker.ConfigureAwait(false); }
            finally
            {
                lock (_gate)
                {
                    source?.Dispose();
                    if (ReferenceEquals(_cancellation, source)) { _cancellation = null; }
                    if (_generation == generation && _state != VideoState.Disposed) { _state = VideoState.Stopped; }
                }
            }
        }

        private void ClearPendingFrame()
        {
            _pendingFrame?.Dispose();
            _pendingFrame = null;
        }
    }
}
