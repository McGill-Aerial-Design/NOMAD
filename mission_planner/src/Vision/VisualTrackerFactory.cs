// SPDX-License-Identifier: Apache-2.0
using System;
using System.Runtime.CompilerServices;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    internal static class VisualTrackerFactory
    {
        internal static IVisualTracker Create(CancellationToken cancellationToken,
            VisualTrackingMode mode = VisualTrackingMode.Object)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (mode != VisualTrackingMode.Object && mode != VisualTrackingMode.Area)
            { throw new ArgumentOutOfRangeException(nameof(mode)); }
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
        private static IVisualTracker CreateLoaded(VisualTrackingMode mode) =>
            mode == VisualTrackingMode.Area ? (IVisualTracker)new AreaVisualTracker() : new CsrtVisualTracker();
    }
}
