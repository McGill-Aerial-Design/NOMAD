// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    // Opt-in Object tracker: EfficientTAM video memory on the NVIDIA GPU through DirectML. The model predicts every
    // frame, but memory is written only for admitted frames, so an occluder, removal or jump is never learned.
    // After a miss or stale-video gap, three consecutive consistent admissions are required before steering resumes.
    internal sealed class EfficientTamVisualTracker : IVisualTracker
    {
        // Presence is the model's own decision (logit > 0). Its IoU estimate was >= 0.59 on every saved positive
        // (orbit 379, approach 89 frames), so 0.5 is a conservative floor, not a tuned threshold.
        private const float MinimumIou = .5f;
        // Per-frame size change bounds compound over frames since the last admission (at most four).
        private const double GrowthPerFrame = 1.6;
        private const int ConfirmationFrames = 3, MaximumGap = 4;
        // Video memory cannot tell identical objects apart; after this many unadmitted frames an identical decoy
        // near the stale position is as likely as the selected object (a classical live orbit locked one 13.6 s
        // after loss). Re-acquisition then requires reselection.
        private const int MaximumMissedFrames = 15;
        private readonly EfficientTamCore _core;
        private Size _imageSize;
        private int _side, _left, _top, _missed, _confirmations;
        private bool _initialized, _recovering, _disposed;
        private RectangleF _last;
        private double _lastArea;
        internal string LastDiagnostic { get; private set; }
        internal string Provider => _core.Provider;
        internal int CommittedFrames => _core.CommittedFrames;

        internal EfficientTamVisualTracker(string modelDirectory, CancellationToken token)
        {
            VisionRuntime.EnsureDirectMlLoaded(token);
            token.ThrowIfCancellationRequested();
            _core = new EfficientTamCore(modelDirectory, true);
        }

        public void Initialize(Bitmap frame, RectangleF bounds, CancellationToken token, VisualFrameContext context = default)
        {
            CheckDisposed();
            _initialized = false;
            token.ThrowIfCancellationRequested();
            VisualTrackingImage.ValidateFrame(frame);
            VisualTrackingImage.ValidateSelection(frame, bounds);
            _imageSize = frame.Size;
            // Object-relative crop: about four selection diameters, never larger than the short image side.
            int shortSide = Math.Min(frame.Width, frame.Height);
            _side = Math.Min(shortSide, Math.Max(Math.Min(256, shortSide),
                (int)Math.Ceiling(4 * Math.Sqrt(bounds.Width * bounds.Height))));
            Centre(bounds.X + bounds.Width / 2, bounds.Y + bounds.Height / 2);
            float scale = EfficientTamCore.Side / (float)_side;
            var prediction = _core.Prompt(Crop(frame), (bounds.Left - _left) * scale, (bounds.Top - _top) * scale,
                (bounds.Right - _left) * scale, (bounds.Bottom - _top) * scale);
            token.ThrowIfCancellationRequested();
            if (!Measure(prediction, out RectangleF found, out double area) || Overlap(found, bounds) <= 0)
            { throw new ArgumentException("Cannot separate the selected object; drag a tighter outline around it."); }
            _core.Commit(prediction);
            _last = found; _lastArea = area; _missed = 0; _confirmations = 0; _recovering = false;
            Centre(found.X + found.Width / 2, found.Y + found.Height / 2);
            _initialized = true;
        }

        public VisualTrackingResult Update(Bitmap frame, CancellationToken token, VisualFrameContext context = default)
        {
            CheckDisposed();
            token.ThrowIfCancellationRequested();
            VisualTrackingImage.ValidateFrame(frame);
            if (!_initialized) { return Result(VisualTrackingState.Lost, "Select a distinctive object"); }
            if (frame.Size != _imageSize)
            { _initialized = false; return Result(VisualTrackingState.Lost, "Image size changed; select the object again"); }
            if (context.Discontinuous) { _recovering = true; _confirmations = 0; }
            if (_missed > MaximumMissedFrames)
            { return Result(VisualTrackingState.Searching, "Object lost too long to re-identify; select it again"); }
            var clock = System.Diagnostics.Stopwatch.StartNew();
            float[] image = Crop(frame);
            double cropMs = clock.Elapsed.TotalMilliseconds;
            var prediction = _core.Track(image);
            double modelMs = clock.Elapsed.TotalMilliseconds - cropMs;
            token.ThrowIfCancellationRequested();
            bool measured = Measure(prediction, out RectangleF found, out double area);
            string reason = Admit(prediction, measured, found, area);
            var stages = _core.StageMilliseconds;
            LastDiagnostic += string.Format(System.Globalization.CultureInfo.InvariantCulture,
                "; ms crop={0:F0} model={1:F0} (enc={2:F0} attn={3:F0} dec={4:F0}) host={5:F0}",
                cropMs, modelMs, stages[0], stages[1], stages[2], clock.Elapsed.TotalMilliseconds - cropMs - modelMs);
            if (reason != null)
            {
                _missed++; _recovering = true; _confirmations = 0;
                return Result(VisualTrackingState.Searching, "Object unverified (" + reason + "); memory unchanged");
            }
            if (_recovering && ++_confirmations < ConfirmationFrames)
            {
                // Follow the candidate without learning it or steering until confirmation completes.
                _last = found; _lastArea = area; _missed = 0;
                Centre(found.X + found.Width / 2, found.Y + found.Height / 2);
                return Result(VisualTrackingState.Searching, "Confirming object (" + _confirmations + "/3)");
            }
            _core.Commit(prediction);
            _recovering = false; _confirmations = 0; _missed = 0;
            _last = found; _lastArea = area;
            Centre(found.X + found.Width / 2, found.Y + found.Height / 2);
            var aim = new PointF(found.X + found.Width / 2, found.Y + found.Height / 2);
            LastDiagnostic += "; admitted; commit ms=" + _core.StageMilliseconds[3].ToString("F0");
            return new VisualTrackingResult(VisualTrackingState.Verified, "Object video memory verified", found, aim);
        }

        // Returns null when admitted, otherwise the rejection reason.
        private string Admit(EfficientTamCore.Prediction prediction, bool measured, RectangleF found, double area)
        {
            LastDiagnostic = Diagnostic(prediction, area);
            if (!prediction.Present || !measured) { return "not present"; }
            if (prediction.Iou < MinimumIou) { return "low mask confidence"; }
            int gap = Math.Min(MaximumGap, _missed + 1);
            double limit = Math.Pow(GrowthPerFrame, gap);
            if (area > _lastArea * limit || area < _lastArea / limit) { return "size jump"; }
            double size = Math.Max(_last.Width, _last.Height);
            double dx = found.X + found.Width / 2 - (_last.X + _last.Width / 2);
            double dy = found.Y + found.Height / 2 - (_last.Y + _last.Height / 2);
            if (Math.Sqrt(dx * dx + dy * dy) > size * gap) { return "position jump"; }
            return null;
        }

        private bool Measure(EfficientTamCore.Prediction prediction, out RectangleF bounds, out double area)
        {
            bounds = RectangleF.Empty; area = 0;
            if (!prediction.Present) { return false; }
            float[] high = EfficientTamCore.Upsample(prediction.LowRes);
            int n = EfficientTamCore.Side, left = n, top = n, right = -1, bottom = -1, count = 0;
            for (int y = 0; y < n; y++)
            for (int x = 0; x < n; x++)
            {
                if (high[y * n + x] < 0) { continue; }
                count++;
                if (x < left) { left = x; }
                if (x > right) { right = x; }
                if (y < top) { top = y; }
                if (y > bottom) { bottom = y; }
            }
            if (count < 4) { return false; }
            float scale = _side / (float)n;
            bounds = new RectangleF(_left + left * scale, _top + top * scale, (right - left + 1) * scale, (bottom - top + 1) * scale);
            area = count * scale * scale;
            return true;
        }

        private void Centre(float x, float y)
        {
            _left = Math.Max(0, Math.Min(_imageSize.Width - _side, (int)Math.Round(x - _side / 2f)));
            _top = Math.Max(0, Math.Min(_imageSize.Height - _side, (int)Math.Round(y - _side / 2f)));
        }

        private float[] Crop(Bitmap frame)
        {
            int n = EfficientTamCore.Side;
            using (var crop = new Bitmap(n, n, PixelFormat.Format24bppRgb))
            {
                using (var graphics = Graphics.FromImage(crop))
                {
                    graphics.InterpolationMode = _side == n ? InterpolationMode.NearestNeighbor : InterpolationMode.HighQualityBicubic;
                    graphics.PixelOffsetMode = PixelOffsetMode.Half;
                    graphics.CompositingMode = CompositingMode.SourceCopy;
                    graphics.DrawImage(frame, new Rectangle(0, 0, n, n), new Rectangle(_left, _top, _side, _side), GraphicsUnit.Pixel);
                }
                var rgb = new byte[n * n * 3];
                var data = crop.LockBits(new Rectangle(0, 0, n, n), ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);
                try
                {
                    var row = new byte[n * 3];
                    for (int y = 0; y < n; y++)
                    {
                        Marshal.Copy(data.Scan0 + y * data.Stride, row, 0, row.Length);
                        for (int x = 0; x < n; x++)
                        {
                            // GDI+ stores BGR; the model expects RGB.
                            rgb[(y * n + x) * 3] = row[x * 3 + 2];
                            rgb[(y * n + x) * 3 + 1] = row[x * 3 + 1];
                            rgb[(y * n + x) * 3 + 2] = row[x * 3];
                        }
                    }
                }
                finally { crop.UnlockBits(data); }
                return EfficientTamCore.Normalize(rgb);
            }
        }

        private static string Diagnostic(EfficientTamCore.Prediction prediction, double area) =>
            "objectness=" + prediction.ObjectScore.ToString("F2") + "; iou=" + prediction.Iou.ToString("F3") +
            "; area=" + area.ToString("F0");

        private static double Overlap(RectangleF a, RectangleF b)
        {
            var both = RectangleF.Intersect(a, b);
            return both.IsEmpty ? 0 : both.Width * both.Height / (a.Width * a.Height + b.Width * b.Height - both.Width * both.Height);
        }

        private VisualTrackingResult Result(VisualTrackingState state, string status) => new VisualTrackingResult(state, status);

        private void CheckDisposed()
        { if (_disposed) { throw new ObjectDisposedException(nameof(EfficientTamVisualTracker)); } }

        public void Dispose()
        {
            if (_disposed) { return; }
            _disposed = true;
            _core.Dispose();
        }
    }
}
