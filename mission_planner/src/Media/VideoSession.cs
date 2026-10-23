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
    internal interface IVideoPipeline : IDisposable
    {
        void Start(string pipeline, CancellationToken cancellation);
        Bitmap ReadFrame(CancellationToken cancellation);
    }

    internal enum VideoState { Stopped, Starting, Streaming, Stopping, Disposed }

    // The worker alone touches the pipeline; consumers own frames returned by TakeFrame.
    internal sealed partial class VideoSession : IDisposable
    {
        private readonly object _gate = new object();
        private readonly object _stopGate = new object();
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

        public VideoSession(Func<IVideoPipeline> createPipeline,
            Func<string, CancellationToken, IVideoDetector> createDetector = null,
            Func<CancellationToken, IVisualTracker> createTracker = null)
        {
            _createPipeline = createPipeline;
            _createDetector = createDetector ?? VisionDetectorFactory.Create;
            _createTracker = createTracker;
        }

        public VideoState State { get { lock (_gate) { return _state; } } }
        public string Error { get { lock (_gate) { return _error; } } }
        internal string VisionError { get { lock (_gate) { return _visionError; } } }
        internal bool DetectionEnabled { get { lock (_gate) { return _detectionEnabled; } } }
        internal void ConfigureDetection(bool enabled, string modelProfilePath)
        {
            lock (_gate)
            {
                _visionCancellation?.Cancel();
                ++_detectionRevision;
                _detectionEnabled = enabled;
                _modelProfilePath = modelProfilePath ?? "";
                _visionError = null;
                ClearPendingFrame();
            }
        }
        internal Task Completion { get { lock (_gate) { return _worker; } } }
        internal long Generation { get { lock (_gate) { return _generation; } } }
        public bool IsActive => State == VideoState.Starting || State == VideoState.Streaming;

        public bool Start(string pipeline)
        {
            lock (_gate)
            {
                if (_state != VideoState.Stopped || !_worker.IsCompleted)
                {
                    return false;
                }
                _cancellation?.Dispose();
                _cancellation = new CancellationTokenSource();
                var token = _cancellation.Token;
                var generation = ++_generation;
                _error = null;
                _state = VideoState.Starting;
                _worker = Task.Run(() => Run(pipeline, generation, token));
                return true;
            }
        }

        private void Run(string pipeline, long generation, CancellationToken token)
        {
            try
            {
                token.ThrowIfCancellationRequested();
                using (var resource = _createPipeline())
                {
                    resource.Start(pipeline, token);
                    token.ThrowIfCancellationRequested();
                    lock (_gate)
                    {
                        if (generation != _generation)
                        {
                            return;
                        }
                        _state = VideoState.Streaming;
                    }
                    ReadFrames(resource, generation, token);
                }
            }
            catch (OperationCanceledException) when (token.IsCancellationRequested) { }
            catch (Exception ex)
            {
                lock (_gate)
                {
                    if (generation == _generation || _state == VideoState.Stopping || _state == VideoState.Disposed)
                    {
                        _error = ex.Message;
                    }
                }
            }
            finally
            {
                lock (_gate)
                {
                    if (generation == _generation)
                    {
                        RetireRegion();
                        ClearPendingFrame();
                        _cancellation?.Dispose();
                        _cancellation = null;
                        _state = VideoState.Stopped;
                    }
                }
            }
        }

        private void ReadFrames(IVideoPipeline pipeline, long generation, CancellationToken token)
        {
            IVideoDetector detector = null;
            CancellationTokenSource vision = null;
            long detectorRevision = -1;
            using (var tracking = new RegionWorker(this, token))
            try
            {
                while (!token.IsCancellationRequested)
                {
                    var image = pipeline.ReadFrame(token);
                    if (image == null) { return; }
                    VideoFrame processed = null;
                    long received = Stopwatch.GetTimestamp();
                    DateTime utc = DateTime.UtcNow;
                    long sequence = Interlocked.Increment(ref _sequence);
                    VideoRegionResult region = null;
                    long revision;
                    bool enabled;
                    string path;
                    lock (_gate)
                    { revision = _detectionRevision; enabled = _detectionEnabled; path = _modelProfilePath; }
                    IReadOnlyList<VideoDetection> detections = Array.Empty<VideoDetection>();
                    string identity = "";
                    try
                    {
                        region = tracking.Update(image, received, sequence);
                        if (revision != detectorRevision)
                        {
                            ReleaseDetector(ref detector, ref vision);
                            detectorRevision = revision;
                            if (enabled)
                            {
                                vision = CancellationTokenSource.CreateLinkedTokenSource(token);
                                lock (_gate)
                                {
                                    if (revision != _detectionRevision) { vision.Cancel(); }
                                    _visionCancellation = vision;
                                }
                                detector = _createDetector(path, vision.Token);
                            }
                        }
                        if (enabled && detector != null)
                        {
                            detections = detector.Detect(image, vision.Token);
                            identity = detector.Identity;
                        }
                        token.ThrowIfCancellationRequested();
                        processed = new VideoFrame(image, detections, sequence,
                            generation, received, utc, identity, region);
                        image = null;
                        lock (_gate)
                        {
                            if (revision == _detectionRevision)
                            { PublishVideoFrame(generation, processed); processed = null; }
                        }
                    }
                    catch (OperationCanceledException)
                    {
                        token.ThrowIfCancellationRequested();
                    }
                    catch (Exception) when (!token.IsCancellationRequested)
                    {
                        // Inference errors must not terminate plain playback or expose model/operator paths.
                        lock (_gate)
                        {
                            if (revision == _detectionRevision)
                            {
                                _detectionEnabled = false;
                                ++_detectionRevision;
                                _visionError = "Vision unavailable; plain video continues. Review the model profile.";
                            }
                        }
                        if (image != null)
                        {
                            PublishVideoFrame(generation, new VideoFrame(image, Array.Empty<VideoDetection>(),
                                sequence, generation, received, utc, "", region));
                            image = null;
                        }
                    }
                    finally { processed?.Dispose(); image?.Dispose(); }
                }
            }
            finally
            {
                ReleaseDetector(ref detector, ref vision);
            }
        }

        private void ReleaseDetector(ref IVideoDetector detector, ref CancellationTokenSource vision)
        {
            var resource = detector;
            var source = vision;
            detector = null; vision = null;
            lock (_gate) { if (ReferenceEquals(_visionCancellation, source)) { _visionCancellation = null; } }
            try { resource?.Dispose(); }
            catch (Exception) { lock (_gate) { _visionError = "Vision cleanup failed; review the model before enabling again."; } }
            finally { source?.Dispose(); }
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
            lock (_stopGate)
            {
                StopWorker(disposing);
            }
        }

        private void StopWorker(bool disposing)
        {
            Task worker;
            lock (_gate)
            {
                ++_generation;
                RetireRegion();
                _state = disposing || _state == VideoState.Disposed ? VideoState.Disposed : VideoState.Stopping;
                _cancellation?.Cancel();
                worker = _worker;
            }
            // No worker invokes a consumer or needs its UI thread to finish.
            worker.GetAwaiter().GetResult();
            lock (_gate)
            {
                ClearPendingFrame();
                _cancellation?.Dispose();
                _cancellation = null;
                if (_state != VideoState.Disposed)
                {
                    _state = VideoState.Stopped;
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
