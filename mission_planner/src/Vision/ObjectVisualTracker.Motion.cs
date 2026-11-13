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
        private Rectangle? FlowProposal(Mat image, CancellationToken token)
        {
            if (_flowImage == null || _flowPoints == null || _flowPoints.Length < 6) { return null; }
            using (var gray = new Mat())
            using (var source = Mat.FromArray(_flowPoints))
            using (var next = new Mat())
            using (var back = new Mat())
            using (var forward = new Mat())
            using (var backward = new Mat())
            using (var error = new Mat())
            {
                Cv2.CvtColor(image, gray, ColorConversionCodes.BGR2GRAY);
                var criteria = new TermCriteria(CriteriaTypes.Count | CriteriaTypes.Eps, 10, .03);
                Cv2.CalcOpticalFlowPyrLK(_flowImage, gray, source, next, forward, error,
                    new OpenCvSharp.Size(21, 21), 3, criteria);
                token.ThrowIfCancellationRequested();
                Cv2.CalcOpticalFlowPyrLK(gray, _flowImage, next, back, backward, error,
                    new OpenCvSharp.Size(21, 21), 3, criteria);
                token.ThrowIfCancellationRequested();
                next.GetArray(out Point2f[] moved); back.GetArray(out Point2f[] returned);
                forward.GetArray(out byte[] good); backward.GetArray(out byte[] reversed);
                var accepted = Enumerable.Range(0, _flowPoints.Length).Where(i => good[i] != 0 && reversed[i] != 0 &&
                    VideoDetection.Finite(moved[i].X) && VideoDetection.Finite(moved[i].Y) &&
                    Math.Abs(returned[i].X - _flowPoints[i].X) <= .75 &&
                    Math.Abs(returned[i].Y - _flowPoints[i].Y) <= .75).ToArray();
                if (accepted.Length < 6) { return null; }
                var dx = accepted.Select(i => moved[i].X - _flowPoints[i].X).OrderBy(x => x).ToArray();
                var dy = accepted.Select(i => moved[i].Y - _flowPoints[i].Y).OrderBy(y => y).ToArray();
                var bounds = _flowBounds;
                bounds.Offset((int)Math.Round(dx[dx.Length / 2]), (int)Math.Round(dy[dy.Length / 2]));
                return bounds;
            }
        }

        private void RememberFlow(Mat image, ObjectForeground target)
        {
            _flowImage?.Dispose(); _flowImage = null; _flowPoints = null;
            if (ThinShape(target) || target.Projected) { return; }
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
