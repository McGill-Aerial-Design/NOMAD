// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;

namespace NOMAD.MissionPlanner
{
    internal sealed class YoloXVideoDetector : IVideoDetector
    {
        private readonly VisionModelProfile _profile;
        private readonly InferenceSession _session;
        private readonly object _gate = new object();
        private readonly string _inputName;
        private readonly string _outputName;
        private bool _disposed;
        public string Identity => _profile.Identity;

        internal YoloXVideoDetector(VisionModelProfile profile, CancellationToken token)
        {
            profile.Validate();
            _profile = profile;
            byte[] model = profile.ReadModel(token);
            using (var options = new SessionOptions { IntraOpNumThreads = 2, InterOpNumThreads = 1 })
            using (token.Register(() => options.SetLoadCancellationFlag(true)))
            {
                token.ThrowIfCancellationRequested();
                _session = new InferenceSession(model, options);
            }
            try
            {
                token.ThrowIfCancellationRequested();
                if (_session.InputMetadata.Count != 1 || _session.OutputMetadata.Count != 1)
                { throw new FormatException("YOLOX requires exactly one input and one output."); }
                var input = _session.InputMetadata.Single();
                var output = _session.OutputMetadata.Single();
                ValidateTensor(input.Value, new[] { 1, 3, profile.InputSize, profile.InputSize });
                ValidateTensor(output.Value, new[] { 1, profile.Rows, profile.Labels.Length + 5 });
                _inputName = input.Key;
                _outputName = output.Key;
            }
            catch { _session.Dispose(); throw; }
        }

        private static void ValidateTensor(NodeMetadata metadata, int[] shape)
        {
            if (!metadata.IsTensor || metadata.ElementType != typeof(float) ||
                !metadata.Dimensions.SequenceEqual(shape))
            { throw new FormatException("YOLOX requires a float tensor shaped [" + string.Join(",", shape) +
                "]; the model declares [" + string.Join(",", metadata.Dimensions) + "]. Check input size, labels and raw grid export."); }
        }

        public IReadOnlyList<VideoDetection> Detect(Bitmap frame, CancellationToken token)
        {
            if (frame == null) { throw new ArgumentNullException(nameof(frame)); }
            lock (_gate)
            {
                if (_disposed) { throw new ObjectDisposedException(nameof(YoloXVideoDetector)); }
                token.ThrowIfCancellationRequested();
                float ratio;
                var data = Preprocess(frame, _profile.InputSize, token, out ratio);
                var tensor = new DenseTensor<float>(data, new[] { 1, 3, _profile.InputSize, _profile.InputSize });
                var inputs = new[] { NamedOnnxValue.CreateFromTensor(_inputName, tensor) };
                using (var options = new RunOptions())
                using (token.Register(() => options.Terminate = true))
                {
                    try
                    {
                        token.ThrowIfCancellationRequested();
                        using (var output = _session.Run(inputs, new[] { _outputName }, options))
                        {
                            var prediction = output.Single().AsTensor<float>();
                            var expected = new[] { 1, _profile.Rows, _profile.Labels.Length + 5 };
                            if (!prediction.Dimensions.SequenceEqual(expected))
                            { throw new FormatException("YOLOX output dimensions changed."); }
                            return Decode(prediction.ToArray(), _profile, frame.Size, ratio, token);
                        }
                    }
                    catch (Exception) when (token.IsCancellationRequested)
                    { throw new OperationCanceledException(token); }
                }
            }
        }

        internal static float[] Preprocess(Bitmap frame, int size, CancellationToken token, out float ratio)
        {
            if (frame.Width < 1 || frame.Height < 1 || frame.Width > 16384 || frame.Height > 16384 ||
                (long)frame.Width * frame.Height > 64 * 1024 * 1024)
            { throw new ArgumentException("Video frame exceeds vision limits."); }
            ratio = Math.Min((float)size / frame.Width, (float)size / frame.Height);
            using (var scaled = new Bitmap(size, size, PixelFormat.Format24bppRgb))
            {
                using (var graphics = Graphics.FromImage(scaled))
                {
                    graphics.Clear(Color.FromArgb(114, 114, 114));
                    graphics.DrawImage(frame, new Rectangle(0, 0, Math.Max(1, (int)(frame.Width * ratio)),
                        Math.Max(1, (int)(frame.Height * ratio))));
                }
                var bits = scaled.LockBits(new Rectangle(0, 0, size, size), ImageLockMode.ReadOnly,
                    PixelFormat.Format24bppRgb);
                try
                {
                    var values = new float[3 * size * size];
                    var row = new byte[size * 3];
                    for (int y = 0; y < size; y++)
                    {
                        token.ThrowIfCancellationRequested();
                        Marshal.Copy(IntPtr.Add(bits.Scan0, y * bits.Stride), row, 0, row.Length);
                        for (int x = 0; x < size; x++)
                        {
                            values[y * size + x] = row[x * 3];
                            values[size * size + y * size + x] = row[x * 3 + 1];
                            values[2 * size * size + y * size + x] = row[x * 3 + 2];
                        }
                    }
                    return values;
                }
                finally { scaled.UnlockBits(bits); }
            }
        }

        internal static IReadOnlyList<VideoDetection> Decode(float[] values, VisionModelProfile profile,
            Size frame, float ratio, CancellationToken token)
        {
            int columns = profile.Labels.Length + 5;
            if (values == null || values.Length != profile.Rows * columns || !VideoDetection.Finite(ratio) ||
                ratio <= 0 || frame.Width < 1 || frame.Height < 1)
            { throw new FormatException("Invalid YOLOX output."); }
            var candidates = new List<VideoDetection>();
            int row = 0;
            foreach (int stride in profile.Strides)
            {
                for (int y = 0; y < profile.InputSize / stride; y++)
                {
                    token.ThrowIfCancellationRequested();
                    for (int x = 0; x < profile.InputSize / stride; x++, row++)
                    { AddCandidate(values, row * columns, x, y, stride, ratio, profile, frame, candidates); }
                }
            }
            candidates.Sort((left, right) => right.Confidence.CompareTo(left.Confidence));
            var selected = new List<VideoDetection>();
            foreach (var candidate in candidates)
            {
                token.ThrowIfCancellationRequested();
                if (!selected.Any(item => item.ClassId == candidate.ClassId &&
                    IntersectionOverUnion(item.Bounds, candidate.Bounds) > profile.NmsThreshold))
                { selected.Add(candidate); }
                if (selected.Count == 100) { break; }
            }
            return selected.AsReadOnly();
        }

        private static void AddCandidate(float[] data, int offset, int x, int y, int stride, float ratio,
            VisionModelProfile profile, Size frame, List<VideoDetection> candidates)
        {
            int label = 0;
            for (int c = 0; c < profile.Labels.Length; c++)
            {
                float probability = data[offset + 5 + c];
                if (!VideoDetection.Finite(probability) || probability < 0 || probability > 1) { return; }
                if (probability > data[offset + 5 + label]) { label = c; }
            }
            float objectness = data[offset + 4];
            float score = objectness * data[offset + 5 + label];
            if (!VideoDetection.Finite(objectness) || objectness < 0 || objectness > 1 ||
                score < profile.ScoreThreshold || !profile.AllowedClasses.Contains(label)) { return; }
            double centerX = (data[offset] + x) * stride / ratio;
            double centerY = (data[offset + 1] + y) * stride / ratio;
            double width = Math.Exp(data[offset + 2]) * stride / ratio;
            double height = Math.Exp(data[offset + 3]) * stride / ratio;
            if (double.IsNaN(centerX) || double.IsInfinity(centerX) || double.IsNaN(centerY) ||
                double.IsInfinity(centerY) || double.IsNaN(width) || double.IsInfinity(width) ||
                double.IsNaN(height) || double.IsInfinity(height)) { return; }
            float left = (float)Math.Max(0, Math.Min(frame.Width, centerX - width / 2));
            float top = (float)Math.Max(0, Math.Min(frame.Height, centerY - height / 2));
            float right = (float)Math.Max(0, Math.Min(frame.Width, centerX + width / 2));
            float bottom = (float)Math.Max(0, Math.Min(frame.Height, centerY + height / 2));
            if (right - left < 1 || bottom - top < 1) { return; }
            candidates.Add(new VideoDetection(label, profile.Labels[label], score,
                RectangleF.FromLTRB(left, top, right, bottom)));
        }

        internal static float IntersectionOverUnion(RectangleF left, RectangleF right)
        {
            var overlap = RectangleF.Intersect(left, right);
            float intersection = Math.Max(0, overlap.Width) * Math.Max(0, overlap.Height);
            return intersection / (left.Width * left.Height + right.Width * right.Height - intersection);
        }

        public void Dispose()
        {
            lock (_gate)
            {
                if (_disposed) { return; }
                _disposed = true;
                _session.Dispose();
            }
        }
    }
}
