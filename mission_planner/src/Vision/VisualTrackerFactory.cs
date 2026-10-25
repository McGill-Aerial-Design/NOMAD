// SPDX-License-Identifier: Apache-2.0
using System;
using System.Runtime.CompilerServices;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    internal static class VisualTrackerFactory
    {
        internal static IVisualTracker Create(CancellationToken cancellationToken)
        {
            cancellationToken.ThrowIfCancellationRequested();
            try
            {
                VisionRuntime.EnsureOpenCvLoaded(cancellationToken);
                return CreateLoaded();
            }
            catch (Exception) when (cancellationToken.IsCancellationRequested)
            { throw new OperationCanceledException(cancellationToken); }
            catch (Exception error) when (!(error is OutOfMemoryException))
            { throw new InvalidOperationException("Visual tracker could not load. Windows x64 is required."); }
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static IVisualTracker CreateLoaded() { return new CsrtVisualTracker(); }
    }
}
