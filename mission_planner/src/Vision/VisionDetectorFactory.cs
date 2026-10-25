// SPDX-License-Identifier: Apache-2.0
using System;
using System.Runtime.CompilerServices;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    internal static class VisionDetectorFactory
    {
        internal static IVideoDetector Create(string modelProfilePath, CancellationToken cancellationToken)
        {
            cancellationToken.ThrowIfCancellationRequested();
            try
            {
                var profile = string.IsNullOrWhiteSpace(modelProfilePath)
                    ? VisionModelProfile.Builtin() : VisionModelProfile.Load(modelProfilePath);
                VisionRuntime.EnsureLoaded(cancellationToken);
                if (string.IsNullOrWhiteSpace(modelProfilePath))
                { profile.Model = VisionRuntime.BuiltinModelPath; }
                return CreateLoaded(profile, cancellationToken);
            }
            catch (Exception) when (cancellationToken.IsCancellationRequested)
            { throw new OperationCanceledException(cancellationToken); }
            catch (Exception error) when (!(error is OutOfMemoryException))
            {
                // Native/model errors can contain operator paths; the UI gets one safe diagnostic.
                throw new InvalidOperationException("Vision model could not load. Check the model profile, " +
                    "SHA256 and supported YOLOX format; Windows x64 is required.");
            }
        }

        // ORT types must be resolved only after the embedded runtime loader is installed.
        [MethodImpl(MethodImplOptions.NoInlining)]
        private static IVideoDetector CreateLoaded(VisionModelProfile profile, CancellationToken token)
        { return new YoloXVideoDetector(profile, token); }
    }
}
