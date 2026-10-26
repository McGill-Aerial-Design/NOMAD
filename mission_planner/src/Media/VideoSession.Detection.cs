// SPDX-License-Identifier: Apache-2.0
using System;
using System.Threading;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    internal sealed partial class VideoSession
    {
        internal long DetectionRevision { get { lock (_gate) { return _detectionRevision; } } }
        internal bool DetectionLoading { get { lock (_gate) { return _detectionLoading; } } }
        private bool _detectionLoading;

        // Only model construction is asynchronous. Inference and exact frame ownership stay on the decode worker.
        private sealed class DetectionWorker : IDisposable
        {
            private readonly VideoSession _owner;
            private readonly CancellationToken _token;
            private Task<IVideoDetector> _load;
            private CancellationTokenSource _source;
            private IVideoDetector _detector;
            private long _revision = -1;

            internal DetectionWorker(VideoSession owner, CancellationToken token) { _owner = owner; _token = token; }

            internal IVideoDetector Current(long revision, bool enabled, string path)
            {
                if (_revision != revision)
                {
                    _source?.Cancel();
                    ReleaseDetector();
                }
                if (_load != null)
                {
                    if (!_load.IsCompleted) { return null; }
                    try
                    {
                        var loaded = _load.GetAwaiter().GetResult();
                        if (_revision == revision && enabled && !_source.IsCancellationRequested) { _detector = loaded; }
                        else { loaded.Dispose(); }
                    }
                    catch (OperationCanceledException) { _token.ThrowIfCancellationRequested(); }
                    catch (Exception) { Fail(_revision); }
                    finally { _load = null; }
                }
                if (_revision != revision)
                {
                    ClearSource();
                    _revision = revision;
                    if (enabled)
                    {
                        _source = CancellationTokenSource.CreateLinkedTokenSource(_token);
                        var source = _source;
                        lock (_owner._gate)
                        {
                            _owner._visionCancellation = source;
                            if (revision != _owner._detectionRevision) { source.Cancel(); }
                            else { _owner._detectionLoading = true; }
                        }
                        _load = Task.Run(() => Load(path, source.Token));
                    }
                }
                if (_detector != null)
                {
                    lock (_owner._gate)
                    { if (_revision == _owner._detectionRevision) { _owner._detectionLoading = false; } }
                }
                return _detector;
            }

            private IVideoDetector Load(string path, CancellationToken token)
            {
                IVideoDetector detector = null;
                try
                {
                    token.ThrowIfCancellationRequested();
                    detector = _owner._createDetector(path, token) ??
                        throw new InvalidOperationException("The detector factory returned no model.");
                    token.ThrowIfCancellationRequested();
                    var result = detector; detector = null; return result;
                }
                finally { detector?.Dispose(); }
            }

            internal CancellationToken Token => _source?.Token ?? _token;

            internal void Fail(long revision)
            {
                lock (_owner._gate)
                {
                    if (revision != _owner._detectionRevision) { return; }
                    _owner._detectionEnabled = false;
                    _owner._detectionLoading = false;
                    ++_owner._detectionRevision;
                    _owner._visionError = "Selected model unavailable; video continues. Review the model or press Detect to retry.";
                }
            }

            private void ClearSource()
            {
                lock (_owner._gate)
                { if (ReferenceEquals(_owner._visionCancellation, _source)) { _owner._visionCancellation = null; } }
                _source?.Dispose(); _source = null;
            }

            private void ReleaseDetector()
            {
                var detector = _detector; _detector = null;
                try { detector?.Dispose(); }
                catch (Exception) { Fail(_revision); }
            }

            public void Dispose()
            {
                _source?.Cancel();
                try
                {
                    if (_load != null)
                    {
                        try { _load.GetAwaiter().GetResult()?.Dispose(); }
                        catch (OperationCanceledException) { }
                        catch (Exception) { Fail(_revision); }
                    }
                    ReleaseDetector();
                }
                finally
                {
                    ClearSource();
                    lock (_owner._gate) { _owner._detectionLoading = false; }
                }
            }
        }
    }
}
