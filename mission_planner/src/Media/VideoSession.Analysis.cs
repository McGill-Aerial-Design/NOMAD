// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    internal sealed partial class VideoSession
    {
        private sealed class DisplayCandidate
        {
            internal readonly object Gate = new object();
            internal readonly VideoFrame Frame;
            internal readonly bool Detect;
            internal readonly string Model;
            internal bool DetectionDone, TrackingDone;
            internal IReadOnlyList<VideoDetection> Detections = Array.Empty<VideoDetection>();
            internal string Identity = "";
            internal VideoRegionResult Region;
            internal DisplayCandidate(VideoFrame frame, bool detect, string model)
            { Frame = frame; Detect = detect; Model = model; DetectionDone = !detect; }
        }

        // A lane owns one executing frame and one replaceable pending frame, never a historical FIFO.
        private sealed class AnalysisLane
        {
            private readonly object _gate = new object();
            private readonly AutoResetEvent _ready = new AutoResetEvent(false);
            private readonly Action<DisplayCandidate, VideoFrame> _process;
            private readonly CancellationToken _token;
            private readonly Action _initialize, _cleanup;
            private DisplayCandidate _pending;
            private VideoFrame _lease;
            private bool _stopped;
            internal Task Completion { get; }
            internal AnalysisLane(Action<DisplayCandidate, VideoFrame> process, CancellationToken token,
                Action initialize = null, Action cleanup = null)
            {
                _process = process; _token = token; _initialize = initialize; _cleanup = cleanup;
                Completion = Task.Factory.StartNew(Run, CancellationToken.None,
                    TaskCreationOptions.LongRunning, TaskScheduler.Default);
            }
            internal void Offer(DisplayCandidate candidate)
            {
                lock (_gate)
                {
                    if (_stopped) { return; }
                    _lease?.Dispose(); _pending = candidate; _lease = candidate.Frame.Retain();
                    _ready.Set();
                }
            }
            private void Run()
            {
                try
                {
                    _initialize?.Invoke();
                    while (!_token.IsCancellationRequested)
                    {
                        DisplayCandidate candidate; VideoFrame lease;
                        lock (_gate)
                        {
                            if (_stopped) { return; }
                            candidate = _pending; lease = _lease; _pending = null; _lease = null;
                        }
                        if (candidate == null)
                        { WaitHandle.WaitAny(new[] { _ready, _token.WaitHandle }); continue; }
                        using (lease) { _process(candidate, lease); }
                    }
                }
                finally
                {
                    try { _cleanup?.Invoke(); }
                    finally
                    { lock (_gate) { _stopped = true; _lease?.Dispose(); _lease = null; _pending = null; _ready.Dispose(); } }
                }
            }
            internal void Stop()
            { lock (_gate) { if (_stopped) { return; } _stopped = true; _ready.Set(); } }
        }

        private sealed class DisplayWindow
        {
            private readonly object _gate = new object();
            private readonly Queue<DisplayCandidate> _pending = new Queue<DisplayCandidate>();
            private readonly VideoSession _owner;
            private readonly long _generation;
            private readonly CancellationToken _token;
            private bool _stopped;
            internal Task Completion { get; }
            internal DisplayWindow(VideoSession owner, long generation, CancellationToken token)
            {
                _owner = owner; _generation = generation; _token = token;
                Completion = Task.Factory.StartNew(Run, CancellationToken.None,
                    TaskCreationOptions.LongRunning, TaskScheduler.Default);
            }
            internal void Offer(DisplayCandidate candidate)
            {
                lock (_gate)
                {
                    if (_stopped) { candidate.Frame.Dispose(); return; }
                    if (_pending.Count == 4) { Emit(_pending.Dequeue(), true); }
                    _pending.Enqueue(candidate);
                }
            }
            private void Run()
            {
                try
                {
                    while (!_token.WaitHandle.WaitOne(5))
                    {
                        lock (_gate)
                        {
                            if (_stopped) { return; }
                            while (_pending.Count != 0)
                            {
                                var next = _pending.Peek();
                                bool ready;
                                lock (next.Gate) { ready = next.DetectionDone && next.TrackingDone; }
                                if (!ready && next.Frame.AgeSeconds < .1) { break; }
                                Emit(_pending.Dequeue(), !ready);
                            }
                        }
                    }
                }
                finally
                {
                    lock (_gate)
                    { _stopped = true; while (_pending.Count != 0) { _pending.Dequeue().Frame.Dispose(); } }
                }
            }
            private void Emit(DisplayCandidate candidate, bool plain)
            {
                using (var frame = candidate.Frame)
                {
                    IReadOnlyList<VideoDetection> detections; VideoRegionResult region; string identity;
                    lock (candidate.Gate)
                    {
                        detections = plain ? Array.Empty<VideoDetection>() : candidate.Detections;
                        region = plain ? null : candidate.Region; identity = plain ? "" : candidate.Identity;
                    }
                    long revision = _owner.DetectionRevision;
                    if (revision != frame.DetectionRevision) { detections = Array.Empty<VideoDetection>(); identity = ""; }
                    var image = MediaPixels.Clone(frame.Image);
                    try
                    {
                        var envelope = new VideoFrame(image, detections, frame.Sequence, frame.Generation,
                            frame.ReceivedTicks, frame.ReceivedUtc, identity, region, revision, frame.Motion);
                        image = null; _owner.PublishVideoFrame(_generation, envelope);
                    }
                    finally { MediaPixels.Dispose(image); }
                }
            }
            internal void Stop() { lock (_gate) { _stopped = true; } }
        }
    }
}
