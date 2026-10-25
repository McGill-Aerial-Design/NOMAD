// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    internal interface IVideoDetector : IDisposable
    {
        string Identity { get; }
        IReadOnlyList<VideoDetection> Detect(Bitmap frame, CancellationToken cancellationToken);
    }

    internal sealed class VideoDetection
    {
        internal int ClassId { get; }
        internal string Label { get; }
        internal float Confidence { get; }
        internal RectangleF Bounds { get; }

        internal VideoDetection(int classId, string label, float confidence, RectangleF bounds)
        {
            if (classId < 0 || string.IsNullOrWhiteSpace(label) || !Finite(confidence) ||
                confidence < 0 || confidence > 1 || !Finite(bounds.X) || !Finite(bounds.Y) ||
                !Finite(bounds.Width) || !Finite(bounds.Height) || bounds.X < 0 || bounds.Y < 0 ||
                bounds.Width <= 0 || bounds.Height <= 0)
            { throw new ArgumentException("Invalid video detection."); }
            ClassId = classId;
            Label = label;
            Confidence = confidence;
            Bounds = bounds;
        }

        internal static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
