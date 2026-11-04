// SPDX-License-Identifier: Apache-2.0
using System;
using System.Runtime.CompilerServices;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    internal static class VisualTrackerFactory
    {
        internal static IVisualTracker Create(CancellationToken cancellationToken,
            VisualTrackingMode mode = VisualTrackingMode.Object, bool gpuObjectTracker = false)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (mode != VisualTrackingMode.Object && mode != VisualTrackingMode.Area)
            { throw new ArgumentOutOfRangeException(nameof(mode)); }
            if (gpuObjectTracker && mode == VisualTrackingMode.Object)
            {
                // Explicit choice: a failure is reported, never silently replaced by the CPU tracker.
                try { return CreateGpu(cancellationToken); }
                catch (Exception) when (cancellationToken.IsCancellationRequested)
                { throw new OperationCanceledException(cancellationToken); }
                catch (Exception error) when (!(error is OutOfMemoryException))
                { throw new InvalidOperationException("GPU Object tracker could not load: " + error.Message); }
            }
            try
            {
                VisionRuntime.EnsureOpenCvLoaded(cancellationToken);
                return CreateLoaded(mode);
            }
            catch (Exception) when (cancellationToken.IsCancellationRequested)
            { throw new OperationCanceledException(cancellationToken); }
            catch (Exception error) when (!(error is OutOfMemoryException))
            { throw new InvalidOperationException("Visual tracker could not load. Windows x64 is required."); }
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static IVisualTracker CreateGpu(CancellationToken token) =>
            new EfficientTamVisualTracker(VisionRuntime.EnsureTrackerModel(token), token);

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static IVisualTracker CreateLoaded(VisualTrackingMode mode) =>
            mode == VisualTrackingMode.Area ? (IVisualTracker)new AreaVisualTracker() : new CsrtVisualTracker();
    }
}
