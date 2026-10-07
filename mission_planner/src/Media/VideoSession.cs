// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
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
    internal sealed class VideoSession : IDisposable
    {
        private readonly object _gate = new object();
        private readonly object _stopGate = new object();
        private readonly Func<IVideoPipeline> _createPipeline;
        private CancellationTokenSource _cancellation;
        private Task _worker = Task.CompletedTask;
        private Bitmap _pendingFrame;
        private long _generation;
        private VideoState _state;
        private string _error;

        public VideoSession(Func<IVideoPipeline> createPipeline)
        {
            _createPipeline = createPipeline;
        }

        public VideoState State { get { lock (_gate) { return _state; } } }
        public string Error { get { lock (_gate) { return _error; } } }
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
            while (!token.IsCancellationRequested)
            {
                var frame = pipeline.ReadFrame(token);
                if (frame == null)
                {
                    return;
                }
                PublishFrame(generation, frame);
            }
        }

        internal void PublishFrame(long generation, Bitmap frame)
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

        public Bitmap TakeFrame()
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
