// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;
using OpenCvSharp;

namespace NOMAD.MissionPlanner
{
    // Fixed operator template: rejected frames never train the model or advance its accepted search position.
    internal sealed class CpuTemplateTracker : IDisposable
    {
        private readonly InferenceSession _session;
        private readonly DenseTensor<float> _template;
        private RectangleF _bounds;
        private RectangleF? _confirmation;
        private int _matches;
        private bool _verified;
        private bool _recovering;
        private bool _retired;
        private long _unverifiedSince;
        private Mat _appearance;
        internal string Diagnostic { get; private set; }
        internal bool HasCandidate => _verified || _confirmation.HasValue;

        internal CpuTemplateTracker(string model, Mat image, Rect selection, CancellationToken token)
        {
            using (var options = new SessionOptions { IntraOpNumThreads = 2, InterOpNumThreads = 1 })
            using (token.Register(() => options.SetLoadCancellationFlag(true)))
            { token.ThrowIfCancellationRequested(); _session = new InferenceSession(model, options); }
            try
            {
                token.ThrowIfCancellationRequested();
                if (_session.InputMetadata.Count != 2 || _session.OutputMetadata.Count != 3 ||
                    !_session.InputMetadata["template"].Dimensions.SequenceEqual(new[] { 1, 3, 128, 128 }) ||
                    !_session.InputMetadata["search"].Dimensions.SequenceEqual(new[] { 1, 3, 256, 256 }) ||
                    new[] { "output1", "output2", "output3" }.Where((name, index) =>
                        _session.OutputMetadata[name].ElementType != typeof(float) ||
                        _session.OutputMetadata[name].Dimensions.Aggregate(1, (a, b) => a * b) != (index == 0 ? 256 : 512)).Any())
                { throw new FormatException("Unexpected CPU template tracker tensors."); }
                _bounds = new RectangleF(selection.X, selection.Y, selection.Width, selection.Height);
                _template = Crop(image, _bounds, 2, 128, out _);
            }
            catch { _session.Dispose(); throw; }
        }

        internal VisualTrackingResult Update(Mat image, CancellationToken token, bool discontinuous)
        {
            token.ThrowIfCancellationRequested();
            if (_unverifiedSince != 0 && (Stopwatch.GetTimestamp() - _unverifiedSince) / (double)Stopwatch.Frequency >= 5)
            { _retired = true; }
            if (_retired) { return new VisualTrackingResult(VisualTrackingState.Lost, "Object identity lost; select again"); }
            if (discontinuous) { Reject(); }
            // Consistent proposals may guide confirmation; only the third match advances accepted bounds.
            var search = Crop(image, _confirmation ?? _bounds, _recovering ? 6 : 4, 256, out RectangleF crop);
            float[] confidence, sizes, offsets;
            using (var options = new RunOptions())
            using (token.Register(() => options.Terminate = true))
            {
                token.ThrowIfCancellationRequested();
                try
                {
                    using (var outputs = _session.Run(new[] { NamedOnnxValue.CreateFromTensor("template", _template),
                        NamedOnnxValue.CreateFromTensor("search", search) }, new[] { "output1", "output2", "output3" }, options))
                    {
                        confidence = outputs.First(x => x.Name == "output1").AsTensor<float>().ToArray();
                        sizes = outputs.First(x => x.Name == "output2").AsTensor<float>().ToArray();
                        offsets = outputs.First(x => x.Name == "output3").AsTensor<float>().ToArray();
                    }
                }
                catch (OnnxRuntimeException) when (token.IsCancellationRequested) { throw new OperationCanceledException(token); }
            }
            token.ThrowIfCancellationRequested();
            if (confidence.Length != 256 || sizes.Length != 512 || offsets.Length != 512 ||
                confidence.Concat(sizes).Concat(offsets).Any(x => !VideoDetection.Finite(x)))
            { throw new InvalidOperationException("Invalid CPU template prediction."); }
            int best = 0; double score = 0;
            for (int index = 0; index < 256; index++)
            {
                double window = .25 * (1 - Math.Cos(2 * Math.PI * (index % 16 + 1) / 17)) *
                    (1 - Math.Cos(2 * Math.PI * (index / 16 + 1) / 17));
                if (confidence[index] * window > score) { score = confidence[index] * window; best = index; }
            }
            var candidate = Prediction(crop, best, sizes, offsets);
            float competing = CompetingScore(crop, candidate, best, confidence, sizes, offsets, image.Width, image.Height);
            double ratio = candidate.Width * candidate.Height / (_bounds.Width * _bounds.Height);
            var anchor = _confirmation ?? _bounds;
            double shift = Math.Sqrt(Math.Pow(candidate.X + candidate.Width / 2 - anchor.X - anchor.Width / 2, 2) +
                Math.Pow(candidate.Y + candidate.Height / 2 - anchor.Y - anchor.Height / 2, 2));
            Diagnostic = "CPU template score=" + confidence[best].ToString("F3") + "; rank=" + score.ToString("F3") +
                "; competing=" + competing.ToString("F3") + "; bounds=" + candidate + "; ratio=" + ratio.ToString("F2") +
                "; shift=" + shift.ToString("F2");
            // The position window ranks candidates; it must not lower the confidence of a correctly moving object.
            if (ratio < (_verified ? .45 : .16) || ratio > (_verified ? 2.2 : 6.25) ||
                shift > Math.Max(anchor.Width, anchor.Height) * (_verified ? .75 : 1) || candidate.Width < 2 || candidate.Height < 2 ||
                candidate.Left < 0 || candidate.Top < 0 || candidate.Right > image.Width || candidate.Bottom > image.Height)
            { Reject(); return Searching("Object template unverified; retaining original selection"); }
            using (var observed = Appearance(image, candidate))
            {
                float similarity = -1;
                if (_appearance != null)
                {
                    using (var correlation = new Mat())
                    {
                        Cv2.MeanStdDev(_appearance, out Scalar _, out Scalar deviation);
                        Cv2.MatchTemplate(observed, _appearance, correlation, TemplateMatchModes.CCoeffNormed);
                        similarity = deviation.Val0 >= 2 ? correlation.At<float>(0) : -1;
                        Diagnostic += "; appearance=" + similarity.ToString("F3");
                        if (!VideoDetection.Finite(similarity) || similarity < .3)
                        { Reject(); return Searching("Returning object appearance differs; identity unverified"); }
                    }
                }
                bool witnessed = _verified && similarity >= .5 && shift <= Math.Max(_bounds.Width, _bounds.Height) * .5;
                if (competing >= .3 && competing >= confidence[best] * .8)
                {
                    // Retirement needs evidence that the original is present, not just competing peaks during occlusion.
                    if (_appearance != null && confidence[best] >= .3)
                    {
                        _retired = true;
                        return new VisualTrackingResult(VisualTrackingState.Lost, "Competing objects; identity lost, select again");
                    }
                    Reject(); return Searching("Competing object templates; identity unverified");
                }
                if (confidence[best] < (witnessed ? .3 : .5))
                { Reject(); return Searching("Object template unverified; retaining original selection"); }
                // A wider search locates a returning target; confirmation uses the model's trained crop scale.
                _recovering = false;
                if (!_verified)
                {
                    if (_confirmation.HasValue && Overlap(_confirmation.Value, candidate) < .5) { _matches = 0; }
                    _confirmation = candidate;
                    if (++_matches < 3) { return Searching("Confirming CPU object template (" + _matches + "/3)"); }
                    _verified = true;
                }
                _confirmation = null;
                _bounds = candidate;
                _unverifiedSince = 0;
                _appearance?.Dispose(); _appearance = observed.Clone();
            }
            return new VisualTrackingResult(VisualTrackingState.Verified, "Selected object template verified", candidate,
                new PointF(candidate.X + candidate.Width / 2, candidate.Y + candidate.Height / 2));
        }

        private void Reject()
        {
            _verified = false; _recovering = true; _matches = 0; _confirmation = null;
            if (_unverifiedSince == 0) { _unverifiedSince = Stopwatch.GetTimestamp(); }
        }
        private static RectangleF Prediction(RectangleF crop, int index, float[] sizes, float[] offsets)
        {
            float width = sizes[index], height = sizes[256 + index];
            float x = (index % 16 + offsets[index]) / 16, y = (index / 16 + offsets[256 + index]) / 16;
            return new RectangleF(crop.X + (x - width / 2) * crop.Width,
                crop.Y + (y - height / 2) * crop.Height, width * crop.Width, height * crop.Height);
        }

        internal static float CompetingScore(RectangleF crop, RectangleF candidate, int best, float[] confidence,
            float[] sizes, float[] offsets, int width, int height)
        {
            float strongest = 0;
            for (int index = 0; index < 256; index++)
            {
                if (index == best || confidence[index] < .3 || confidence[index] <= strongest) { continue; }
                var other = Prediction(crop, index, sizes, offsets);
                double ratio = (double)other.Width * other.Height / ((double)candidate.Width * candidate.Height);
                if (other.Width < 2 || other.Height < 2 || other.Left < 0 || other.Top < 0 ||
                    other.Right > width || other.Bottom > height || ratio < .45 || ratio > 2.2) { continue; }
                // Different grid cells can regress the same object; compare decoded boxes, not cell positions.
                if (Overlap(candidate, other) < .25) { strongest = confidence[index]; }
            }
            return strongest;
        }

        private static double Overlap(RectangleF a, RectangleF b)
        {
            var overlap = RectangleF.Intersect(a, b);
            double area = overlap.Width * overlap.Height;
            return area / (a.Width * a.Height + b.Width * b.Height - area);
        }
        private static VisualTrackingResult Searching(string status) => new VisualTrackingResult(VisualTrackingState.Searching, status);


        private static DenseTensor<float> Crop(Mat image, RectangleF bounds, int factor, int side, out RectangleF region)
        {
            int size = Math.Max(16, Math.Min(2048, (int)Math.Ceiling(Math.Sqrt(bounds.Width * bounds.Height) * factor)));
            int x = (int)(bounds.X + (bounds.Width - size) / 2), y = (int)(bounds.Y + (bounds.Height - size) / 2);
            region = new RectangleF(x, y, size, size);
            var clipped = Rect.Intersect(new Rect(x, y, size, size), new Rect(0, 0, image.Width, image.Height));
            using (var roi = new Mat(image, clipped))
            using (var padded = new Mat())
            using (var resized = new Mat())
            {
                Cv2.CopyMakeBorder(roi, padded, clipped.Y - y, y + size - clipped.Bottom,
                    clipped.X - x, x + size - clipped.Right, BorderTypes.Constant);
                Cv2.Resize(padded, resized, new OpenCvSharp.Size(side, side));
                resized.GetArray(out Vec3b[] pixels);
                var values = new float[3 * side * side];
                var mean = new[] { .485f, .456f, .406f }; var deviation = new[] { .229f, .224f, .225f };
                for (int channel = 0; channel < 3; channel++)
                for (int index = 0; index < pixels.Length; index++)
                { values[channel * pixels.Length + index] = (pixels[index][channel] / 255f - mean[channel]) / deviation[channel]; }
                return new DenseTensor<float>(values, new[] { 1, 3, side, side });
            }
        }

        private static Mat Appearance(Mat image, RectangleF bounds)
        {
            var result = new Mat();
            try
            {
                var box = new Rect((int)bounds.X, (int)bounds.Y,
                    Math.Max(1, Math.Min(image.Width, (int)Math.Ceiling(bounds.Right)) - (int)bounds.X),
                    Math.Max(1, Math.Min(image.Height, (int)Math.Ceiling(bounds.Bottom)) - (int)bounds.Y));
                using (var crop = new Mat(image, box))
                using (var gray = new Mat())
                {
                    Cv2.CvtColor(crop, gray, ColorConversionCodes.BGR2GRAY);
                    Cv2.Resize(gray, result, new OpenCvSharp.Size(48, 48));
                }
                return result;
            }
            catch { result.Dispose(); throw; }
        }

        public void Dispose() { _session.Dispose(); _appearance?.Dispose(); }
    }
}
