// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading;
using OpenCvSharp;

namespace NOMAD.MissionPlanner
{
    internal sealed partial class ObjectVisualTracker
    {
        private ObjectForeground _originalView, _workingView, _previousView;
        private FrozenAppearance _workingAppearance;
        private ObjectRecoveryReference _workingReference;
        private Rectangle _lastObjectBounds;
        private int _lastObjectArea, _acceptedViews, _heldFrames, _heldConfirmations, _frames, _previousFrame, _rememberedFrame;
        // ponytail: frame-count bounds (~0.5 s / ~1.5 s at 10-20 fps); time bounds if frame rates vary widely.
        private const int MaximumHeldFrames = 10;
        // Position alone binds identity only briefly: a live orbit locked onto an identical decoy 13.6 s after loss.
        private const int MaximumNearbyFrames = 15;
        // ViTTrack proposes each box of a separated, non-thin object in 2-4 ms; GrabCut segmentation costs 10-35 ms.
        private CpuTemplateTracker _proposer;
        private int _proposedFrames;
        // ponytail: fixed refresh cadence; refresh on a falling identity margin if segmentation still dominates.
        // Every few proposed frames segment once, keeping the outline, previous-view chain and learned views fresh.
        private const int ProposedFramesPerRefresh = 3;
        private double _objectAngle;
        private PointF _appearanceAnchor;
        private double[] _lastColour;
        private PointF _objectCentre, _objectMotion;
        private bool _hasObjectCentre;
        internal int LearnedViews { get; private set; }
        internal string LastObjectDiagnostic { get; private set; }

        private void InitializeObject(Mat image, OpenCvSharp.Rect selection, bool smallSelection, CancellationToken token)
        {
            LearnedViews = 0;
            var region = new Rectangle(selection.X, selection.Y, selection.Width, selection.Height);
            var point = new PointF(region.X + region.Width / 2f, region.Y + region.Height / 2f);
            bool selectedThin = Math.Max(selection.Width, selection.Height) >= 2 * Math.Min(selection.Width, selection.Height);
            if (!selectedThin && Math.Max(selection.Width, selection.Height) <= 48)
            {
                string model = VisionRuntime.EnsureCpuTemplateModel(token);
                _templateTracker = CreateTemplate(model, image, selection, token);
                return;
            }
            _originalView = ObjectForeground.TryExtract(image, region, point, null, token,
                smallSelection ? 3.5 : 1.8, restrictToSelection: !smallSelection);
            bool tiny = Math.Max(selection.Width, selection.Height) <= 48 &&
                !(_originalView != null && selectedThin && ThinShape(_originalView));
            var outline = region;
            outline.Inflate(Math.Max(1, region.Width / 10), Math.Max(1, region.Height / 10));
            bool useTemplate = _originalView == null || (!(selectedThin && ThinShape(_originalView)) &&
                (_originalView.Bounds.Width * _originalView.Bounds.Height < selection.Width * selection.Height * .25 ||
                !outline.Contains(_originalView.Bounds) || tiny));
            if (useTemplate)
            {
                string model = VisionRuntime.EnsureCpuTemplateModel(token);
                _templateTracker = CreateTemplate(model, image, selection, token);
                if (tiny || _originalView == null)
                { _originalView?.Dispose(); _originalView = null; return; }
            }
            if (_originalView == null)
            { return; }
            using (var mapping = Cv2.GetPerspectiveTransform(_originalView.Quad, PatchCorners()))
            {
                var aim = Cv2.PerspectiveTransform(new[] { new Point2f(point.X, point.Y) }, mapping)[0];
                _appearanceAnchor = new PointF(aim.X, aim.Y);
            }
            var objectBounds = NativeBox(_originalView.Bounds);
            _appearance = new FrozenAppearance(image, objectBounds, _originalView.Mask);
            _reference = new ObjectRecoveryReference(image, objectBounds, _appearance, _originalView.Mask);
            if (_reference.CanLocate && _templateTracker != null && Math.Max(selection.Width, selection.Height) > 48)
            {
                // Distinctive foreground geometry is stronger evidence than a generic model proposal.
                _templateTracker.Dispose(); _templateTracker = null;
            }
            if (_templateTracker == null && !ThinShape(_originalView))
            { _proposer = CreateTemplate(VisionRuntime.EnsureCpuTemplateModel(token), image, objectBounds, token); }
            Remember(_originalView); RememberFlow(image, _originalView); _following = true;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static CpuTemplateTracker CreateTemplate(string model, Mat image, Rect selection, CancellationToken token) =>
            new CpuTemplateTracker(model, image, selection, token);

        private VisualTrackingResult FollowObject(Mat image, CancellationToken token)
        {
            _frames++;
            if (!_following) { return RecoverObject(image, token); }
            var previous = _workingView ?? _originalView;
            bool thin = ThinShape(previous);
            var predicted = new PointF(_objectCentre.X + _objectMotion.X, _objectCentre.Y + _objectMotion.Y);
            var fallback = new Rectangle((int)Math.Round(predicted.X - _lastObjectBounds.Width / 2f),
                (int)Math.Round(predicted.Y - _lastObjectBounds.Height / 2f), _lastObjectBounds.Width, _lastObjectBounds.Height);
            var proposed = thin || _heldFrames > 0 ? null : Propose(image, fallback, token);
            if (proposed.HasValue && _proposedFrames < ProposedFramesPerRefresh)
            {
                var accepted = AcceptProposal(image, previous, proposed.Value, fallback, token);
                if (accepted != null) { _proposedFrames++; return accepted; }
            }
            _proposedFrames = 0;
            var search = thin ? fallback : proposed ?? FlowProposal(image, token) ?? fallback;
            var seed = ForegroundSeed(image, search);
            // Pad the prior so a turning object's outline can regrow instead of ratcheting to the last tight box.
            var fresh = ObjectForeground.TryExtract(image, Padded(search), seed, _objectAngle, token,
                expectedColour: thin ? _lastColour : null, restrictToSelection: !thin, fast: true);
            if (fresh == null || !Continuous(fresh, search, fallback) || !Recognizes(fresh, .55, true))
            {
                fresh?.Dispose();
                fresh = ObjectForeground.TryExtract(image, Padded(search), seed, _objectAngle, token,
                    expectedColour: thin ? _lastColour : null, restrictToSelection: !thin);
            }
            if (fresh == null && !thin)
            {
                fresh = ObjectForeground.TryExtract(image, Padded(fallback), ForegroundSeed(image, fallback),
                    _objectAngle, token, restrictToSelection: true);
            }
            ObjectForeground projected = null;
            try
            {
                var observed = fresh;
                bool freshSupport = fresh != null;
                bool freshContinuous = fresh != null && Continuous(fresh, search, fallback);
                bool freshAccepted = freshContinuous && Recognizes(fresh, .55, true);
                string freshState = fresh == null ? "none" : !freshContinuous ? "jump " + fresh.Bounds + "/" + fresh.Area :
                    !freshAccepted ? "unrecognized " + fresh.Bounds : "accepted";
                bool continuous = freshContinuous, recognized = freshAccepted;
                if (!thin && !freshAccepted)
                {
                    projected = ObjectForeground.ProjectPrevious(image, previous, search, token);
                    continuous = projected != null && Continuous(projected, search, fallback);
                    recognized = continuous && Recognizes(projected, .55, true);
                    if (!recognized)
                    {
                        projected?.Dispose();
                        projected = ObjectForeground.ProjectPrevious(image, previous, fallback, token);
                        continuous = projected != null && Continuous(projected, search, fallback);
                        recognized = continuous && Recognizes(projected, .55, true);
                    }
                    observed = projected; freshSupport = false;
                }
                LastObjectDiagnostic = (observed == null ? "No separable foreground" :
                    "Foreground " + observed.Bounds + ", area=" + observed.Area + "; fresh=" + freshState +
                    "; continuity=" + continuous + "; recognized=" + recognized) +
                    "; " + ProcessingImageSize.Width + "x" + ProcessingImageSize.Height;
                if (!continuous || !recognized || (_heldFrames > 0 && !freshSupport))
                {
                    // Feature geometry can still confirm an unchanged pose when segmentation fragments. Its
                    // projected box must agree with the predicted outline: on small objects it inflates.
                    var precise = _heldFrames > 0 ? null : _reference.Refine(image, NativeBox(search), token) ??
                        _workingReference?.Refine(image, NativeBox(search), token);
                    if (precise.HasValue && Math.Max(Overlap(ManagedBox(precise.Value), search),
                        Overlap(ManagedBox(precise.Value), fallback)) >= .5)
                    { return GeometryResult(precise.Value); }
                    // A brief appearance miss keeps the local track near the last verified state without steering;
                    // unverified foreground never updates that state or trains references.
                    if (freshContinuous && ++_heldFrames <= MaximumHeldFrames)
                    {
                        _heldConfirmations = 0; _acceptedViews = 0;
                        return Searching("Object appearance unconfirmed; holding local track");
                    }
                    ResetLocalTrack(); return Searching("Object obscured; searching retained appearances");
                }
                if (_heldFrames > 0 && ++_heldConfirmations < 3)
                {
                    Remember(observed);
                    return Searching("Confirming held object (" + _heldConfirmations + "/3)");
                }
                _heldFrames = 0; _heldConfirmations = 0;
                // Whole-object contract: measured foreground bounds, not an extrapolated reference projection.
                var visible = GeometryResult(NativeBox(observed.Bounds));
                _acceptedViews = freshSupport ? _acceptedViews + 1 : 0;
                if (_acceptedViews >= 3) { PromoteObject(image, observed, visible, token); _acceptedViews = 0; }
                Remember(observed);
                RememberFlow(image, observed);
                if (freshSupport) { _previousView?.Dispose(); _previousView = observed.Clone(); _previousFrame = _frames; }
                return visible;
            }
            finally { fresh?.Dispose(); projected?.Dispose(); }
        }

        // ViTTrack's box near the motion prediction, only when it is confident, uncontested and continuous: a jump to
        // a look-alike elsewhere must not seed segmentation or identity.
        private Rectangle? Propose(Mat image, Rectangle predicted, CancellationToken token)
        {
            if (_proposer == null) { return null; }
            var proposal = _proposer.Propose(image, predicted, 4, token);
            var box = Rectangle.Round(proposal.Bounds);
            bool contested = proposal.Competing >= .3 && proposal.Competing >= proposal.Confidence * .8;
            double limit = 6 + Math.Max(_lastObjectBounds.Width, _lastObjectBounds.Height) * .65;
            double shift = Math.Sqrt(Math.Pow(box.X + box.Width / 2f - predicted.X - predicted.Width / 2f, 2) +
                Math.Pow(box.Y + box.Height / 2f - predicted.Y - predicted.Height / 2f, 2));
            bool inside = box.Width >= 2 && box.Height >= 2 && box.Left >= 0 && box.Top >= 0 &&
                box.Right <= image.Width && box.Bottom <= image.Height;
            return proposal.Confidence >= .5 && !contested && shift <= limit && inside ? box : (Rectangle?)null;
        }

        // ViTTrack locates the object; optical flow seeded with its displacement aligns the retained outline to a
        // pixel, which masked fine texture needs. The projection must pass the same continuity and masked identity
        // (whole object and aim point) checks as an unsegmented frame of the segmentation path below.
        private VisualTrackingResult AcceptProposal(Mat image, ObjectForeground previous, Rectangle proposed,
            Rectangle predicted, CancellationToken token)
        {
            var guess = new PointF(proposed.X + proposed.Width / 2f - _flowBounds.X - _flowBounds.Width / 2f,
                proposed.Y + proposed.Height / 2f - _flowBounds.Y - _flowBounds.Height / 2f);
            var aligned = FlowProposal(image, token, guess);
            var boxes = aligned.HasValue && aligned != proposed ? new[] { aligned.Value, proposed } : new[] { proposed };
            foreach (var box in boxes)
            using (var projected = ObjectForeground.ProjectPrevious(image, previous, box, token))
            {
                if (projected == null || !Continuous(projected, predicted, predicted) || !Recognizes(projected, .55, true))
                { continue; }
                LastObjectDiagnostic = "ViTTrack proposal " + proposed + (box == proposed ? "" : " aligned " + box) +
                    " recognized; " + ProcessingImageSize.Width + "x" + ProcessingImageSize.Height;
                Remember(projected); RememberFlow(image, projected, verifiedProjection: true);
                return GeometryResult(NativeBox(box));
            }
            return null;
        }

        private static Rectangle Padded(Rectangle box)
        {
            box.Inflate(Math.Max(2, box.Width / 4), Math.Max(2, box.Height / 4));
            return box;
        }

        private VisualTrackingResult RecoverObject(Mat image, CancellationToken token)
        {
            var original = _reference.Find(image, token);
            var working = _workingReference?.Find(image, token);
            if (original.HasValue && working.HasValue && Overlap(ManagedBox(original.Value), ManagedBox(working.Value)) < .25)
            { _recoveryMatches = 0; return Searching("Competing Object appearances; identity unverified"); }
            var candidate = working ?? original ?? NearbyObject(image, token);
            if (!candidate.HasValue)
            { _recoveryMatches = 0; _recoveryBounds = null; return Searching("Object unverified; searching retained appearances"); }
            var region = ManagedBox(candidate.Value);
            if (_recoveryBounds.HasValue && Overlap(_recoveryBounds.Value, region) < .5) { _recoveryMatches = 0; }
            _recoveryBounds = region;
            if (++_recoveryMatches < 3) { return Searching("Confirming retained object (" + _recoveryMatches + "/3)"); }
            using (var observed = ObjectForeground.TryExtract(image, region,
                new PointF(region.X + region.Width / 2f, region.Y + region.Height / 2f), _objectAngle, token))
            {
                if (observed != null && Recognizes(observed, .65))
                { Remember(observed); RememberFlow(image, observed); }
                else
                {
                    _lastObjectBounds = region;
                    _objectCentre = new PointF(region.X + region.Width / 2f, region.Y + region.Height / 2f);
                }
            }
            _following = true;
            _objectMotion = PointF.Empty;
            _recoveryMatches = 0; _recoveryBounds = null;
            return GeometryResult(candidate.Value);
        }

        // Small objects carry too few global ORB features; look near the last verified position instead.
        // Exactly one appearance-verified foreground must exist there, so a nearby duplicate fails closed.
        private OpenCvSharp.Rect? NearbyObject(Mat image, CancellationToken token)
        {
            if (!_hasObjectCentre || _lastColour == null || _lastObjectArea >= 48 * 48) { return null; }
            if (_frames - _rememberedFrame > MaximumNearbyFrames)
            { LastObjectDiagnostic = "Nearby search expired; global identity required"; return null; }
            int size = Math.Max(8, Math.Max(_lastObjectBounds.Width, _lastObjectBounds.Height));
            var window = new Rect((int)_objectCentre.X - 3 * size, (int)_objectCentre.Y - 3 * size, 6 * size, 6 * size);
            window = Rect.Intersect(window, new Rect(0, 0, image.Width, image.Height));
            if (window.Width < size || window.Height < size) { return null; }
            var objectSize = new OpenCvSharp.Size(_lastObjectBounds.Width, _lastObjectBounds.Height);
            var seeds = _appearance.Peaks(image, window, objectSize, 3);
            if (_workingAppearance != null) { seeds.AddRange(_workingAppearance.Peaks(image, window, objectSize, 3)); }
            Rectangle? found = null;
            int extracted = 0, recognized = 0;
            foreach (var seed in seeds)
            {
                token.ThrowIfCancellationRequested();
                var box = new Rectangle(seed.X - _lastObjectBounds.Width / 2, seed.Y - _lastObjectBounds.Height / 2,
                    _lastObjectBounds.Width, _lastObjectBounds.Height);
                using (var candidate = ObjectForeground.TryExtract(image, Padded(box), new PointF(seed.X, seed.Y), _objectAngle,
                    token, restrictToSelection: true))
                {
                    double ratio = candidate == null ? 0 : candidate.Area / (double)_lastObjectArea;
                    if (candidate == null || ratio < .45 || ratio > 2.2) { continue; }
                    extracted++;
                    // Retained views may differ in scale after a gap; the nearby area filter above bounds size.
                    if (!Recognizes(candidate, .55) && !SameOutline(_originalView, candidate, double.MaxValue) &&
                        !(_workingView != null && SameOutline(_workingView, candidate, double.MaxValue)) &&
                        !(_previousView != null && SameOutline(_previousView, candidate, double.MaxValue))) { continue; }
                    recognized++;
                    if (found.HasValue && Overlap(found.Value, candidate.Bounds) < .25)
                    { LastObjectDiagnostic = "Nearby search: competing appearances"; return null; }
                    found = found ?? candidate.Bounds;
                }
            }
            LastObjectDiagnostic = "Nearby search: seeds=" + seeds.Count + "; foregrounds=" + extracted +
                "; recognized=" + recognized;
            return found.HasValue ? NativeBox(found.Value) : (OpenCvSharp.Rect?)null;
        }

        // Continuity is measured from the motion prediction (optical flow or last velocity), not the last position:
        // a fast object can move more than its own size per frame while its acceleration stays small.
        private bool Continuous(ObjectForeground candidate, Rectangle search, Rectangle fallback)
        {
            double ratio = candidate.Area / (double)_lastObjectArea;
            double limit = 6 + Math.Max(_lastObjectBounds.Width, _lastObjectBounds.Height) * .65;
            return ratio >= .45 && ratio <= 2.2 &&
                (Shift(candidate, search) <= limit || Shift(candidate, fallback) <= limit);
        }

        private static double Shift(ObjectForeground candidate, Rectangle expected) =>
            Math.Sqrt(Math.Pow(candidate.Centre.X - (expected.X + expected.Width / 2f), 2) +
                Math.Pow(candidate.Centre.Y - (expected.Y + expected.Height / 2f), 2));

        private bool Recognizes(ObjectForeground candidate, double threshold, bool localContinuous = false)
        {
            // The previous verified frame only chains gradual turning on a continuous local track; recovery and
            // learning still need the original or working view, and an abrupt occluder differs from it.
            return (_workingView != null && MatchesView(_workingView, candidate, threshold)) ||
                MatchesView(_originalView, candidate, threshold) ||
                (localContinuous && ((_previousView != null && (MatchesView(_previousView, candidate, threshold) ||
                SameOutline(_previousView, candidate, Math.Pow(1.33, Math.Min(4, _frames - _previousFrame))))) ||
                ThinAppearance(_workingView ?? _originalView, candidate)));
        }

        // Below the 48x48 patch's pixel count the patch only upsamples the object, so NCC measures interpolation
        // noise. Such small objects still change outline shape and colour only gradually between frames.
        // Size may change by at most maximumScale; on the local chain it compounds per frame since the previous
        // verified view, so an approach can grow after a brief break but a static fragment cannot collapse.
        private static bool SameOutline(ObjectForeground previous, ObjectForeground candidate, double maximumScale)
        {
            if (candidate.Projected || previous.Area >= 48 * 48 || candidate.Area >= 48 * 48) { return false; }
            double ratio = candidate.Area / (double)previous.Area, colour = 0;
            if (ratio < 1 / maximumScale || ratio > maximumScale) { return false; }
            for (int channel = 0; channel < 3; channel++)
            { colour += Math.Pow(previous.ColourMean[channel] - candidate.ColourMean[channel], 2); }
            double aspect = candidate.Bounds.Width * (double)previous.Bounds.Height /
                (candidate.Bounds.Height * (double)previous.Bounds.Width);
            if (aspect < .8 || aspect > 1.25 || Math.Sqrt(colour / 3) > 15) { return false; }
            // Bounds-normalized axis-aligned outlines: a blob's min-area-rectangle angle is unstable frame to frame.
            using (var p = Outline(previous))
            using (var c = Outline(candidate))
            using (var both = new Mat())
            {
                Cv2.BitwiseAnd(p, c, both);
                return Cv2.CountNonZero(both) >= Math.Max(Cv2.CountNonZero(p), Cv2.CountNonZero(c)) * .75;
            }
        }

        private static Mat Outline(ObjectForeground view)
        {
            var outline = new Mat();
            using (var crop = new Mat(view.Mask, NativeBox(view.Bounds)))
            { Cv2.Resize(crop, outline, new OpenCvSharp.Size(24, 24), interpolation: InterpolationFlags.Nearest); }
            return outline;
        }

        private static bool ThinShape(ObjectForeground view)
        {
            double Length(Point2f a, Point2f b) => Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2));
            return Length(view.Quad[1], view.Quad[2]) > 4 * Length(view.Quad[0], view.Quad[1]);
        }

        private static bool ThinAppearance(ObjectForeground reference, ObjectForeground candidate)
        {
            if (!ThinShape(reference) || !ThinShape(candidate)) { return false; }
            int aCount = 0, bCount = 0, overlap = 0;
            var aBins = new int[64]; var bBins = new int[64];
            for (int y = 0; y < 48; y++)
            for (int x = 0; x < 48; x++)
            {
                bool a = reference.PatchMask.At<byte>(y, x) != 0, b = candidate.PatchMask.At<byte>(y, x) != 0;
                if (a)
                { var c = reference.PatchColor.At<Vec3b>(y, x); aBins[(c.Item0 / 64) * 16 + (c.Item1 / 64) * 4 + c.Item2 / 64]++; aCount++; }
                if (b)
                { var c = candidate.PatchColor.At<Vec3b>(y, x); bBins[(c.Item0 / 64) * 16 + (c.Item1 / 64) * 4 + c.Item2 / 64]++; bCount++; }
                if (a && b) { overlap++; }
            }
            if (aCount == 0 || bCount == 0 || overlap < Math.Max(aCount, bCount) * .7 ||
                aBins.Max() < aCount * .6 || bBins.Max() < bCount * .6) { return false; }
            double colour = 0;
            for (int channel = 0; channel < 3; channel++)
            { colour += Math.Pow(reference.ColourMean[channel] - candidate.ColourMean[channel], 2); }
            return Math.Sqrt(colour / 3) <= 25;
        }

        private bool MatchesView(ObjectForeground reference, ObjectForeground candidate, double threshold) =>
            FrozenAppearance.MatchesMasked(reference.PatchGray, candidate.PatchGray, reference.PatchColor, candidate.PatchColor,
                reference.PatchMask, candidate.PatchMask, threshold, _appearanceAnchor);

        private void PromoteObject(Mat image, ObjectForeground candidate, VisualTrackingResult visible, CancellationToken token)
        {
            ObjectForeground view = null;
            FrozenAppearance appearance = null;
            ObjectRecoveryReference reference = null;
            try
            {
                token.ThrowIfCancellationRequested();
                if (!visible.Found) { return; }
                if (MatchesView(_workingView ?? _originalView, candidate, .9)) { return; }
                view = candidate.Clone();
                var box = visible.Bounds.HasValue ? Rectangle.Round(visible.Bounds.Value) : candidate.Bounds;
                appearance = new FrozenAppearance(image, NativeBox(box), candidate.Mask);
                reference = new ObjectRecoveryReference(image, NativeBox(box), appearance, candidate.Mask);
                _workingReference?.Dispose(); _workingAppearance?.Dispose(); _workingView?.Dispose();
                _workingReference = reference; reference = null;
                _workingAppearance = appearance; appearance = null;
                _workingView = view; view = null; LearnedViews++;
            }
            finally { reference?.Dispose(); appearance?.Dispose(); view?.Dispose(); }
        }

        private void Remember(ObjectForeground target)
        {
            _objectMotion = _hasObjectCentre ? new PointF(target.Centre.X - _objectCentre.X, target.Centre.Y - _objectCentre.Y) : PointF.Empty;
            _objectCentre = target.Centre; _hasObjectCentre = true; _rememberedFrame = _frames;
            _lastObjectBounds = target.Bounds; _lastObjectArea = target.Area;
            _objectAngle = target.AngleDegrees; _lastColour = target.ColourMean;
        }

        private PointF ForegroundSeed(Mat image, Rectangle proposal)
        {
            int cx = proposal.X + proposal.Width / 2, cy = proposal.Y + proposal.Height / 2;
            int radius = Math.Max(3, Math.Min(20, Math.Min(proposal.Width, proposal.Height) / 3));
            double best = double.MaxValue;
            var seed = new PointF(cx, cy);
            for (int y = Math.Max(0, cy - radius); y <= Math.Min(image.Height - 1, cy + radius); y++)
            for (int x = Math.Max(0, cx - radius); x <= Math.Min(image.Width - 1, cx + radius); x++)
            {
                var pixel = image.At<Vec3b>(y, x);
                double colour = 0;
                for (int channel = 0; channel < 3; channel++) { colour += Math.Pow(pixel[channel] - _lastColour[channel], 2); }
                double score = Math.Sqrt(colour) + 2 * Math.Sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
                if (score < best) { best = score; seed = new PointF(x, y); }
            }
            return seed;
        }

        private static Point2f[] PatchCorners() => new[] { new Point2f(0, 0), new Point2f(47, 0), new Point2f(47, 47), new Point2f(0, 47) };
        private static VisualTrackingResult GeometryResult(OpenCvSharp.Rect box) => new VisualTrackingResult(
            VisualTrackingState.Verified, "Object foreground geometry verified", new RectangleF(box.X, box.Y, box.Width, box.Height),
            new PointF(box.X + box.Width / 2f, box.Y + box.Height / 2f));
        private static OpenCvSharp.Rect NativeBox(Rectangle box) => new OpenCvSharp.Rect(box.X, box.Y, box.Width, box.Height);
        private static Rectangle ManagedBox(OpenCvSharp.Rect box) => new Rectangle(box.X, box.Y, box.Width, box.Height);
    }
}
