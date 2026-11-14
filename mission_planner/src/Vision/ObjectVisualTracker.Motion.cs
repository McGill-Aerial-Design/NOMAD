// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Linq;
using System.Threading;
using OpenCvSharp;

namespace NOMAD.MissionPlanner
{
    internal sealed partial class ObjectVisualTracker
    {
        private Mat _flowImage;
        private Point2f[] _flowPoints;
        private Rectangle _flowBounds;

        // Motion only proposes a search window; measured foreground and retained appearance still authorize the aim.
        // A displacement guess (ViTTrack's) lets the pyramid refine fast motion instead of searching from rest; the
        // backward pass still starts from the forward result, so the consistency check is unchanged.
        private Rectangle? FlowProposal(Mat image, CancellationToken token, PointF guess = default)
        {
            if (_flowImage == null || _flowPoints == null || _flowPoints.Length < 6) { return null; }
            // Pyramids cover the old and guessed object places plus the 4-level, 21-px window reach (~80 px), not
            // the whole frame; point differences and distance ratios are unchanged by the region's origin.
            var moving = _flowBounds; moving.Offset((int)Math.Round(guess.X), (int)Math.Round(guess.Y));
            var region = Rectangle.Union(_flowBounds, moving); region.Inflate(96, 96);
            region.Intersect(new Rectangle(0, 0, image.Width, image.Height));
            if (region.Width < 16 || region.Height < 16) { return null; }
            var points = _flowPoints.Select(p => new Point2f(p.X - region.X, p.Y - region.Y)).ToArray();
            using (var colour = new Mat(image, NativeBox(region)))
            using (var before = new Mat(_flowImage, NativeBox(region)))
            using (var gray = new Mat())
            using (var source = Mat.FromArray(points))
            using (var next = guess.IsEmpty ? new Mat() :
                Mat.FromArray(points.Select(p => new Point2f(p.X + guess.X, p.Y + guess.Y)).ToArray()))
            using (var back = new Mat())
            using (var forward = new Mat())
            using (var backward = new Mat())
            using (var error = new Mat())
            {
                Cv2.CvtColor(colour, gray, ColorConversionCodes.BGR2GRAY);
                var criteria = new TermCriteria(CriteriaTypes.Count | CriteriaTypes.Eps, 10, .03);
                Cv2.CalcOpticalFlowPyrLK(before, gray, source, next, forward, error,
                    new OpenCvSharp.Size(21, 21), 3, criteria,
                    guess.IsEmpty ? OpticalFlowFlags.None : OpticalFlowFlags.UseInitialFlow);
                token.ThrowIfCancellationRequested();
                Cv2.CalcOpticalFlowPyrLK(gray, before, next, back, backward, error,
                    new OpenCvSharp.Size(21, 21), 3, criteria);
                token.ThrowIfCancellationRequested();
                next.GetArray(out Point2f[] moved); back.GetArray(out Point2f[] returned);
                forward.GetArray(out byte[] good); backward.GetArray(out byte[] reversed);
                var accepted = Enumerable.Range(0, points.Length).Where(i => good[i] != 0 && reversed[i] != 0 &&
                    VideoDetection.Finite(moved[i].X) && VideoDetection.Finite(moved[i].Y) &&
                    Math.Abs(returned[i].X - points[i].X) <= .75 &&
                    Math.Abs(returned[i].Y - points[i].Y) <= .75).ToArray();
                if (accepted.Length < 6) { return null; }
                var dx = accepted.Select(i => moved[i].X - points[i].X).OrderBy(x => x).ToArray();
                var dy = accepted.Select(i => moved[i].Y - points[i].Y).OrderBy(y => y).ToArray();
                // MedianFlow scale: the median change of tracked point-pair distances follows an approaching or
                // receding object, so a projected outline keeps its size between segmentations.
                var ratios = new System.Collections.Generic.List<double>();
                for (int a = 0; a < accepted.Length; a++)
                for (int b = a + 1; b < accepted.Length; b++)
                {
                    int i = accepted[a], j = accepted[b];
                    double span = Math.Sqrt(Math.Pow(points[i].X - points[j].X, 2) +
                        Math.Pow(points[i].Y - points[j].Y, 2));
                    if (span < 4) { continue; }
                    ratios.Add(Math.Sqrt(Math.Pow(moved[i].X - moved[j].X, 2) +
                        Math.Pow(moved[i].Y - moved[j].Y, 2)) / span);
                }
                ratios.Sort();
                double scale = ratios.Count < 6 ? 1 : Math.Max(.8, Math.Min(1.25, ratios[ratios.Count / 2]));
                float width = (float)(_flowBounds.Width * scale), height = (float)(_flowBounds.Height * scale);
                float cx = _flowBounds.X + _flowBounds.Width / 2f + dx[dx.Length / 2];
                float cy = _flowBounds.Y + _flowBounds.Height / 2f + dy[dy.Length / 2];
                return Rectangle.Round(new RectangleF(cx - width / 2, cy - height / 2, width, height));
            }
        }

        // An unsegmented projection only seeds flow once its masked identity was verified on a ViTTrack proposal.
        private void RememberFlow(Mat image, ObjectForeground target, bool verifiedProjection = false)
        {
            _flowImage?.Dispose(); _flowImage = null; _flowPoints = null;
            if (ThinShape(target) || (target.Projected && !verifiedProjection)) { return; }
            var gray = new Mat();
            try
            {
                Cv2.CvtColor(image, gray, ColorConversionCodes.BGR2GRAY);
                var bounds = NativeBox(target.Bounds);
                using (var local = new Mat(gray, bounds))
                using (var mask = new Mat(target.Mask, bounds))
                {
                    _flowPoints = Cv2.GoodFeaturesToTrack(local, 64, .01, 4, mask, 3, false, .04)
                        .Select(p => new Point2f(p.X + bounds.X, p.Y + bounds.Y)).ToArray();
                }
                _flowBounds = target.Bounds; _flowImage = gray; gray = null;
            }
            finally { gray?.Dispose(); }
        }
    }
}
