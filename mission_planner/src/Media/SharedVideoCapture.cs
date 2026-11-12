// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    // One native receiver per source: a camera handle is exclusive and a UDP port delivers each datagram to only one
    // socket, so Dashboard, Video and HUD share decoded pixels (never tracking or display ownership). Keys come from
    // VideoPipeline.ShareKey and are compared exactly.
    internal static class SharedVideoCapture
    {
        private static readonly object Gate = new object();
        private static readonly Dictionary<string, Capture> Captures = new Dictionary<string, Capture>(StringComparer.Ordinal);

        internal static IVideoPipeline Create(string key, Func<IVideoPipeline> factory)
        {
            if (string.IsNullOrEmpty(key)) { throw new ArgumentException("A shared video source needs a key.", nameof(key)); }
            return new Viewer(key, factory);
        }

        private sealed class Capture
        {
            internal readonly string Key, Pipeline;
            internal readonly Func<IVideoPipeline> Factory;
            internal readonly List<Viewer> Viewers = new List<Viewer>();
            internal readonly CancellationTokenSource Cancellation = new CancellationTokenSource();
            internal readonly TaskCompletionSource<bool> Ready =
                new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
            internal Task Worker;
            internal Exception StartError, Failure;
            internal bool Closing;

            internal Capture(string key, string pipeline, Func<IVideoPipeline> factory)
            { Key = key; Pipeline = pipeline; Factory = factory; }

            internal void Run()
            {
                Exception failure = null;
                bool started = false;
                var token = Cancellation.Token;
                try
                {
                    token.ThrowIfCancellationRequested();
                    using (var native = Factory())
                    {
                        native.Start(Pipeline, token);
                        token.ThrowIfCancellationRequested();
                        started = true; Ready.TrySetResult(true);
                        while (!token.IsCancellationRequested)
                        {
                            var image = native.ReadFrame(token, out long ticks, out DateTime utc);
                            try
                            {
                                if (image == null) { break; }
                                token.ThrowIfCancellationRequested();
                                Viewer[] viewers;
                                lock (Gate) { viewers = Viewers.ToArray(); }
                                foreach (var viewer in viewers) { viewer.Publish(image, ticks, utc); }
                            }
                            finally { MediaPixels.Dispose(image); }
                        }
                    }
                }
                catch (OperationCanceledException) when (token.IsCancellationRequested) { }
                catch (Exception error) { failure = error; }
                finally
                {
                    Failure = failure;
                    if (!started) { StartError = failure; }
                    Viewer[] viewers;
                    lock (Gate)
                    {
                        Closing = true;
                        viewers = Viewers.ToArray();
                        if (Captures.TryGetValue(Key, out var current) && ReferenceEquals(current, this))
                        { Captures.Remove(Key); }
                    }
                    foreach (var viewer in viewers) { viewer.End(failure); }
                    Ready.TrySetResult(false);
                }
            }
        }

        private sealed class Viewer : IVideoPipeline
        {
            private readonly object _gate = new object();
            private readonly string _key;
            private readonly Func<IVideoPipeline> _factory;
            private Capture _capture;
            private Bitmap _pending;
            private long _receivedTicks;
            private DateTime _receivedUtc;
            private Exception _error;
            private bool _ended, _disposed;

            internal Viewer(string key, Func<IVideoPipeline> factory) { _key = key; _factory = factory; }

            public void Start(string pipeline, CancellationToken token)
            {
                while (true)
                {
                    token.ThrowIfCancellationRequested();
                    Task closing = null;
                    lock (Gate)
                    {
                        if (_capture != null || _disposed) { throw new InvalidOperationException("Video viewer was retired."); }
                        if (Captures.TryGetValue(_key, out var capture) && capture.Closing) { closing = capture.Worker; }
                        else
                        {
                            if (capture == null)
                            {
                                capture = new Capture(_key, pipeline, _factory);
                                Captures.Add(_key, capture);
                                capture.Worker = Task.Factory.StartNew(capture.Run, CancellationToken.None,
                                    TaskCreationOptions.LongRunning, TaskScheduler.Default);
                            }
                            else if (!string.Equals(capture.Pipeline, pipeline, StringComparison.OrdinalIgnoreCase))
                            { throw new InvalidOperationException("This video input is already playing with different receiver settings."); }
                            _capture = capture;
                            capture.Viewers.Add(this);
                        }
                    }
                    if (closing != null) { closing.Wait(token); continue; }
                    _capture.Ready.Task.Wait(token);
                    token.ThrowIfCancellationRequested();
                    if (_capture.StartError != null)
                    { throw new InvalidOperationException(_capture.StartError.Message, _capture.StartError); }
                    return;
                }
            }

            internal void Publish(Bitmap image, long ticks, DateTime utc)
            {
                lock (_gate)
                {
                    if (_disposed || _ended) { return; }
                    var copy = MediaPixels.Clone(image);
                    MediaPixels.Dispose(_pending); _pending = copy;
                    _receivedTicks = ticks; _receivedUtc = utc;
                    Monitor.PulseAll(_gate);
                }
            }

            internal void End(Exception error)
            {
                lock (_gate)
                {
                    _ended = true; _error = error;
                    MediaPixels.Dispose(_pending); _pending = null;
                    Monitor.PulseAll(_gate);
                }
            }

            public Bitmap ReadFrame(CancellationToken token, out long receivedTicks, out DateTime receivedUtc)
            {
                lock (_gate)
                {
                    while (true)
                    {
                        token.ThrowIfCancellationRequested();
                        if (_disposed) { throw new ObjectDisposedException(nameof(Viewer)); }
                        if (_ended)
                        {
                            receivedTicks = 0; receivedUtc = default;
                            if (_error != null) { throw new InvalidOperationException(_error.Message, _error); }
                            return null;
                        }
                        if (_pending != null)
                        {
                            var image = _pending; _pending = null;
                            receivedTicks = _receivedTicks; receivedUtc = _receivedUtc;
                            return image;
                        }
                        Monitor.Wait(_gate, 50);
                    }
                }
            }

            public void Dispose()
            {
                lock (_gate)
                {
                    if (_disposed) { return; }
                    _disposed = true; MediaPixels.Dispose(_pending); _pending = null;
                    Monitor.PulseAll(_gate);
                }
                Capture capture = _capture;
                if (capture == null) { return; }
                bool last;
                lock (Gate)
                {
                    capture.Viewers.Remove(this);
                    last = capture.Viewers.Count == 0;
                    if (last) { capture.Closing = true; }
                }
                if (!last) { return; }
                try
                {
                    try { capture.Cancellation.Cancel(); }
                    // The native worker never calls a viewer's UI and cannot wait for this thread.
                    finally { capture.Worker.GetAwaiter().GetResult(); }
                    if (capture.Failure != null)
                    { throw new InvalidOperationException(capture.Failure.Message, capture.Failure); }
                }
                finally { capture.Cancellation.Dispose(); }
            }
        }
    }
}
