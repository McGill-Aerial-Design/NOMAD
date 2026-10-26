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

        // Exercise the real model/hash/tensor checks without exposing native errors containing operator paths.
        internal static void ValidateModel(VisionModelProfile profile, CancellationToken token)
        {
            try
            {
                profile.Validate();
                VisionRuntime.EnsureLoaded(token);
                using (CreateLoaded(profile, token)) { }
            }
            catch (Exception) when (token.IsCancellationRequested) { throw new OperationCanceledException(token); }
            catch (FormatException error) { throw new InvalidOperationException(error.Message); }
            catch (Exception error) when (!(error is OutOfMemoryException))
            {
                throw new InvalidOperationException("ONNX weights could not load. Check the local file, SHA256 and " +
                    "raw YOLOX export; Windows x64 is required.");
            }
        }

        // ORT types must be resolved only after the embedded runtime loader is installed.
        [MethodImpl(MethodImplOptions.NoInlining)]
        private static IVideoDetector CreateLoaded(VisionModelProfile profile, CancellationToken token)
        { return new YoloXVideoDetector(profile, token); }
    }
}
