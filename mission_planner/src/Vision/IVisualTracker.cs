// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    internal enum VisualTrackingMode { Object, Area }
    internal enum VisualTrackingState { Lost, Searching, Outside, Verified }

    internal readonly struct VisualFrameContext
    {
        internal PointF? SearchHint { get; }
        internal VisualFrameContext(PointF? searchHint) { SearchHint = searchHint; }
        internal static float PriorSearchRadius(Size image, RectangleF referenceSelection) =>
            Math.Max(64 * Math.Max(1, Math.Max(image.Width, image.Height) / 640f),
                Math.Max(referenceSelection.Width, referenceSelection.Height) * 1.25f);
    }

    internal sealed class VisualTrackingResult
    {
        internal VisualTrackingState State { get; }
        internal RectangleF? Bounds { get; }
        internal PointF? AimPoint { get; }
        internal PointF? MotionSearchHint { get; }
        internal string Status { get; }
        internal bool Found => State == VisualTrackingState.Verified;

        internal VisualTrackingResult(VisualTrackingState state, string status, RectangleF? bounds = null,
            PointF? aimPoint = null, PointF? motionSearchHint = null)
        {
            if (!Enum.IsDefined(typeof(VisualTrackingState), state) || string.IsNullOrWhiteSpace(status) ||
                (bounds.HasValue && (!VideoDetection.Finite(bounds.Value.X) || !VideoDetection.Finite(bounds.Value.Y) ||
                !VideoDetection.Finite(bounds.Value.Width) || !VideoDetection.Finite(bounds.Value.Height) ||
                bounds.Value.X < 0 || bounds.Value.Y < 0 || bounds.Value.Width <= 0 || bounds.Value.Height <= 0)) ||
                (aimPoint.HasValue && !Finite(aimPoint.Value)) ||
                (motionSearchHint.HasValue && !Finite(motionSearchHint.Value)))
            { throw new ArgumentException("Invalid visual tracking result."); }
            if (state == VisualTrackingState.Verified && (!bounds.HasValue || !aimPoint.HasValue))
            { throw new ArgumentException("Verified tracking requires image bounds and an aim point."); }
            State = state; Status = status; Bounds = bounds; AimPoint = aimPoint; MotionSearchHint = motionSearchHint;
        }

        private static bool Finite(PointF point) => VideoDetection.Finite(point.X) && VideoDetection.Finite(point.Y);
    }

    // One frame worker owns the tracker. Each call borrows its bitmap only until the call returns.
    internal interface IVisualTracker : IDisposable
    {
        void Initialize(Bitmap frame, RectangleF bounds, CancellationToken cancellationToken,
            VisualFrameContext context = default);
        VisualTrackingResult Update(Bitmap frame, CancellationToken cancellationToken,
            VisualFrameContext context = default);
    }
}
