"""
task2_circle_verify.py - Task 2 Circle Detection & Change Verification

Circle detection for indoor fire extinguishing targets.
Detects pale purple/blue red-cabbage-paper circles on white backing, then
verifies spray success by comparing before/after circle snapshots.

Competition context (AEAC 2026 Task 2):
- Targets are circles (5-30cm dia) on white plastic backing
- Before spray: purple (red cabbage juice dyed paper)
- After spray: turns blue (pH reaction with baking soda water)
- Verification: blue/cyan chemical response indicates successful hit

Detection approach:
- Task 2 HSV/Lab segmentation for non-white purple/blue paper on white backing
- Grayscale Hough Circle Transform fallback
- Contour-based circularity fallback

Verification approach:
- Before spray: detect circles, store properties (position, radius, ROI pixels)
- After spray: detect circles, match by proximity, compare
- Primary metric: blue/cyan response increases within matched circle ROI
- Supporting metric: pixel difference ratio within matched circle ROI

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger("edge_core.task2_circle_verify")


def _circles_iou_xy(cx1: float, cy1: float, r1: float,
                    cx2: float, cy2: float, r2: float) -> float:
    """Intersection-over-Union of two circles."""
    d = ((cx1 - cx2) ** 2 + (cy1 - cy2) ** 2) ** 0.5
    if d >= r1 + r2:
        return 0.0
    if d <= abs(r1 - r2):
        rs = min(r1, r2); rl = max(r1, r2)
        return (rs * rs) / (rl * rl)
    r1s = r1 * r1; r2s = r2 * r2
    try:
        a1 = r1s * math.acos((d * d + r1s - r2s) / (2.0 * d * r1))
        a2 = r2s * math.acos((d * d + r2s - r1s) / (2.0 * d * r2))
        a3 = 0.5 * math.sqrt(
            (-d + r1 + r2) * (d + r1 - r2) * (d - r1 + r2) * (d + r1 + r2)
        )
    except (ValueError, ZeroDivisionError):
        return 0.0
    inter = a1 + a2 - a3
    union = math.pi * r1s + math.pi * r2s - inter
    if union <= 0.0:
        return 0.0
    return inter / union


@dataclass
class DetectedCircle:
    """A circle detected in a Task 2 image (color-agnostic)."""
    cx: int          # center x (pixels)
    cy: int          # center y (pixels)
    radius: int      # radius (pixels)
    method: str      # "hough" or "contour"
    confidence: float  # detection confidence [0, 1]
    # ROI data for before/after comparison
    roi_mask: Optional[np.ndarray] = None  # circular mask for this circle
    mean_color: Optional[np.ndarray] = None  # mean BGR of circle interior


@dataclass
class CircleSnapshot:
    """Snapshot of all detected circles in an image (before or after spray)."""
    circles: List[DetectedCircle]
    image: Optional[np.ndarray] = None     # full image (for pixel comparison)
    image_path: Optional[str] = None       # path to saved image


@dataclass
class CircleComparison:
    """Result of comparing before/after circle snapshots."""
    change_ratio: float        # fraction of pixels that changed (0.0 - 1.0)
    verified: bool             # chemical blue response detected
    matched_pairs: int         # number of before/after circle pairs matched
    details: str = ""          # human-readable summary
    blue_after_ratio: float = 0.0
    blue_increase_ratio: float = 0.0


class Task2CircleDetector:
    """
    Circle detector for Task 2 indoor red-cabbage-paper targets.

    Uses three methods:
    1. HSV/Lab color segmentation for pale purple/blue paper on white backing
    2. Hough Circle Transform on grayscale
    3. Contour circularity on Canny edges

    This is intentionally not YOLO-based; the target class is simple and the
    exact paper/lighting can be tuned with deterministic thresholds.
    """

    def __init__(
        self,
        min_radius_px: int = 15,
        max_radius_px: int = 300,
        min_circularity: float = 0.42,
        min_solidity: float = 0.68,
        blur_kernel: int = 5,
        # Accepted-but-ignored kwargs for back-compat with callers that were
        # built against the old Hough/contour pipeline.
        hough_dp: float = 1.5,
        hough_param1: int = 100,
        hough_param2: int = 40,
        canny_low: int = 50,
        canny_high: int = 150,
    ):
        self.min_radius_px = min_radius_px
        self.max_radius_px = max_radius_px
        self.min_circularity = min_circularity
        self.min_solidity = min_solidity
        self.blur_kernel = blur_kernel
        _ = (hough_dp, hough_param1, hough_param2, canny_low, canny_high)

    def detect(self, bgr_image: np.ndarray) -> List[DetectedCircle]:
        """Detect cabbage-paper circles in a BGR image.

        Mirrors edge_core/ros/simple_video_bridge.py::_detect_shape_circles
        (the Hough+density pipeline). Used here on saved before/after
        snapshots, so the ZED depth gate is absent -- physical-diameter
        validation runs at the live overlay path in the bridge. All other
        gates (colour mask, skin reject, density, backing) match.
        """
        if bgr_image is None or bgr_image.size == 0:
            return []

        h, w = bgr_image.shape[:2]
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2LAB)
        s_ch = hsv[:, :, 1]; v_ch = hsv[:, :, 2]
        a_ch = lab[:, :, 1].astype(np.int16)
        b_ch = lab[:, :, 2].astype(np.int16)
        chroma = np.abs(a_ch - 128) + np.abs(b_ch - 128)

        mauve = (
            (a_ch > 132) & (b_ch < 134) & (chroma >= 4)
            & (v_ch >= 70) & (v_ch <= 245)
        )
        blue = (
            (a_ch < 124) & (b_ch < 128) & (chroma >= 8)
            & (v_ch >= 60) & (v_ch <= 230)
        )
        not_white = ~((s_ch <= 25) & (v_ch >= 195))
        target = ((mauve | blue) & not_white).astype(np.uint8)
        skin = ((a_ch > 130) & (b_ch > 135) & (chroma >= 6)).astype(np.uint8)

        backing_raw = (
            (s_ch <= 40) & (v_ch >= 175) & (chroma <= 22)
        ).astype(np.uint8) * 255
        k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        backing = cv2.morphologyEx(backing_raw, cv2.MORPH_CLOSE, k5, iterations=2)
        backing = cv2.morphologyEx(backing, cv2.MORPH_OPEN, k5, iterations=1)
        backing_bool = backing > 0

        # See bridge: Hough on the chroma channel, not grayscale.
        chroma_u8 = np.clip(chroma, 0, 80).astype(np.uint8) * 3
        chroma_u8 = cv2.medianBlur(chroma_u8, 5)
        min_r = max(self.min_radius_px, int(round(min(h, w) * 0.015)))
        max_r = min(self.max_radius_px, min(h, w) // 2)

        circle_lists = []
        try:
            cr = cv2.HoughCircles(
                chroma_u8, cv2.HOUGH_GRADIENT, dp=1.2,
                minDist=max(20, min_r * 2),
                param1=80, param2=22,
                minRadius=min_r, maxRadius=max_r,
            )
        except cv2.error as exc:
            logger.warning(f"HoughCircles failed: {exc}")
            cr = None
        if cr is not None:
            circle_lists.append(cr[0])

        # CC fallback using centroid + area-equivalent radius (more
        # accurate centering than minEnclosingCircle on slightly-irregular
        # mask blobs).
        k_open_cc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        k_close_cc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        target_clean = cv2.morphologyEx(target * 255, cv2.MORPH_OPEN, k_open_cc)
        target_closed = cv2.morphologyEx(target_clean, cv2.MORPH_CLOSE, k_close_cc)
        n_lbl, lbl, st, _ = cv2.connectedComponentsWithStats(target_closed, connectivity=8)
        cc_circles = []
        min_area_cc = max(math.pi * min_r * min_r * 0.4, 200)
        max_area_cc = math.pi * max_r * max_r
        for i in range(1, n_lbl):
            area = int(st[i, cv2.CC_STAT_AREA])
            if area < min_area_cc or area > max_area_cc:
                continue
            comp = (lbl == i).astype(np.uint8)
            M = cv2.moments(comp)
            if M["m00"] <= 0:
                continue
            ccx = M["m10"] / M["m00"]
            ccy = M["m01"] / M["m00"]
            r_eq = math.sqrt(area / math.pi)
            bw_ = st[i, cv2.CC_STAT_WIDTH]
            bh_ = st[i, cv2.CC_STAT_HEIGHT]
            bbox_r = max(bw_, bh_) / 2.0
            if bbox_r <= 0 or (r_eq / bbox_r) < 0.70:
                continue
            cc_circles.append([float(ccx), float(ccy), float(r_eq + 3.0)])
        if cc_circles:
            circle_lists.append(np.array(cc_circles, dtype=np.float32))

        if not circle_lists:
            return []
        circles_raw = [np.concatenate(circle_lists, axis=0)]
        if circles_raw[0].size == 0:
            return []

        yy, xx = np.ogrid[:h, :w]
        cands: List[Tuple[float, DetectedCircle]] = []
        for ccx, ccy, cr in circles_raw[0]:
            cx_i = int(ccx); cy_i = int(ccy); r_i = int(cr)
            if not (0 <= cx_i < w and 0 <= cy_i < h):
                continue
            if r_i < min_r or r_i > max_r:
                continue
            if (cx_i - r_i) < 2 or (cy_i - r_i) < 2 or (cx_i + r_i) >= (w - 2) or (cy_i + r_i) >= (h - 2):
                continue

            d2 = (xx - cx_i) ** 2 + (yy - cy_i) ** 2
            inside = d2 <= (r_i * 0.85) ** 2
            ic = int(inside.sum())
            if ic <= 0:
                continue
            t_inside = int((target & inside).sum())
            color_density = t_inside / ic
            if color_density < 0.30:
                continue
            if t_inside < 200:
                continue

            sk_inside = int((skin & inside).sum())
            if (sk_inside / ic) > 0.30:
                continue

            ring = (d2 >= (r_i * 1.10) ** 2) & (d2 <= (r_i * 1.85) ** 2)
            rc = int(ring.sum())
            backing_ratio = 0.0
            if rc > 0:
                rb = int((backing_bool & ring).sum())
                backing_ratio = rb / rc

            if backing_ratio < 0.15:
                continue

            size_factor = min(r_i / 60.0, 1.0)
            confidence = min(
                0.99,
                0.05
                + min(color_density, 0.65) * 0.30
                + min(backing_ratio, 0.70) * 0.40
                + size_factor * 0.20,
            )

            det = self._make_detection(cx_i, cy_i, r_i, bgr_image, "hough_density_v3")
            if det is None:
                continue
            det.confidence = float(confidence)
            cands.append((confidence, det))

        cands.sort(key=lambda kv: -kv[0])
        detections: List[DetectedCircle] = []
        for _, det in cands:
            keep = True
            for k in detections:
                dist = ((det.cx - k.cx) ** 2 + (det.cy - k.cy) ** 2) ** 0.5
                close = dist < (det.radius + k.radius) * 0.7
                iou = _circles_iou_xy(det.cx, det.cy, det.radius,
                                     k.cx, k.cy, k.radius)
                if close or iou > 0.15:
                    keep = False
                    break
            if keep:
                detections.append(det)
            if len(detections) >= 6:
                break
        return detections
    def _make_detection(
        self,
        cx: int,
        cy: int,
        radius: int,
        bgr_image: np.ndarray,
        method: str,
    ) -> Optional[DetectedCircle]:
        """Build a DetectedCircle with ROI mask and mean color."""
        h, w = bgr_image.shape[:2]

        # Create circular mask for this detection
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(mask, (cx, cy), radius, 255, -1)

        # Mean color inside the circle
        mean_color = cv2.mean(bgr_image, mask=mask)[:3]
        mean_color = np.array(mean_color)

        # Confidence is overwritten by the caller with the fused score from
        # the cabbage-paper pipeline. The default below only applies to legacy
        # callers that still pass an unrecognised method string.
        confidence = {
            "cabbage_v2": 0.75,
        }.get(method, 0.50)

        return DetectedCircle(
            cx=cx,
            cy=cy,
            radius=radius,
            method=method,
            confidence=confidence,
            roi_mask=mask,
            mean_color=mean_color,
        )

    def _overlaps_existing(
        self,
        det: DetectedCircle,
        existing: List[DetectedCircle],
        dist_factor: float = 0.7,
    ) -> bool:
        """Check if detection overlaps with existing ones."""
        for e in existing:
            dist = math.sqrt((det.cx - e.cx) ** 2 + (det.cy - e.cy) ** 2)
            if dist < max(det.radius, e.radius) * dist_factor:
                return True
        return False


class ColorAgnosticCircleVerifier:
    """
    Color-agnostic before/after spray verifier.

    Detects any circular shape (no hue gates) in both images, matches them
    by proximity, and reports verified=True when the mean Lab color delta
    (deltaE) or mean BGR difference inside the matched ROI exceeds the
    configured threshold. Use this when the target paper colour is not
    known ahead of time (Task 2 dry-run / variant chemistry).
    """

    def __init__(
        self,
        change_threshold: float = 0.20,
        match_distance_px: int = 50,
        pixel_diff_threshold: int = 30,
    ):
        # change_threshold is interpreted as a fraction (e.g. 0.20 == 20%).
        # We use it two ways:
        #   1. Fraction of pixels in the ROI that exceed pixel_diff_threshold.
        #   2. Mean Lab deltaE inside the ROI, normalised by a 100-unit scale.
        # Passing either gate flips verified=True.
        self.change_threshold = change_threshold
        self.match_distance_px = match_distance_px
        self.pixel_diff_threshold = pixel_diff_threshold
        self._detector = Task2CircleDetector()

    def capture_snapshot(
        self, bgr_image: np.ndarray, image_path: Optional[str] = None
    ) -> CircleSnapshot:
        circles = self._detector.detect(bgr_image)
        return CircleSnapshot(circles=circles, image=bgr_image, image_path=image_path)

    def capture_snapshot_from_file(self, image_path: str) -> CircleSnapshot:
        img = cv2.imread(image_path)
        if img is None:
            logger.warning(f"Cannot read image: {image_path}")
            return CircleSnapshot(circles=[], image_path=image_path)
        return self.capture_snapshot(img, image_path)

    def compare(
        self,
        before: CircleSnapshot,
        after: CircleSnapshot,
    ) -> CircleComparison:
        if before.image is None or after.image is None:
            return CircleComparison(
                change_ratio=0.0, verified=False, matched_pairs=0,
                details="Missing image data for pixel comparison",
            )

        h, w = before.image.shape[:2]
        after_img = after.image
        if after_img.shape[:2] != (h, w):
            after_img = cv2.resize(after_img, (w, h))

        # Match circles when both images have detections; otherwise fall back
        # to the union of detected circles (after only, before only).
        matched: List[Tuple[DetectedCircle, DetectedCircle]] = []
        if before.circles and after.circles:
            matched = self._match_circles(before.circles, after.circles)

        # Choose ROIs to measure. If no match, use whatever circle we have.
        roi_circles: List[DetectedCircle] = []
        if matched:
            roi_circles = [m[0] for m in matched]
        elif before.circles:
            roi_circles = before.circles
        elif after.circles:
            roi_circles = after.circles
        else:
            return CircleComparison(
                change_ratio=0.0, verified=False, matched_pairs=0,
                details="No circles detected in either image",
            )

        best_ratio = 0.0
        best_delta_e = 0.0
        best_details = ""

        for c in roi_circles:
            ratio, delta_e, detail = self._measure_roi_change(
                c, before.image, after_img
            )
            score = max(ratio, delta_e / 100.0)
            best_score = max(best_ratio, best_delta_e / 100.0)
            if score > best_score:
                best_ratio = ratio
                best_delta_e = delta_e
                best_details = detail

        verified = (
            best_ratio >= self.change_threshold
            or (best_delta_e / 100.0) >= self.change_threshold
        )

        return CircleComparison(
            change_ratio=best_ratio,
            verified=verified,
            matched_pairs=len(matched),
            details=best_details,
        )

    def _match_circles(
        self,
        before_circles: List[DetectedCircle],
        after_circles: List[DetectedCircle],
    ) -> List[Tuple[DetectedCircle, DetectedCircle]]:
        matched = []
        used_after = set()
        for b in before_circles:
            best_dist = float("inf")
            best_idx = -1
            for i, a in enumerate(after_circles):
                if i in used_after:
                    continue
                dist = math.sqrt((b.cx - a.cx) ** 2 + (b.cy - a.cy) ** 2)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
            if best_idx >= 0 and best_dist < self.match_distance_px:
                matched.append((b, after_circles[best_idx]))
                used_after.add(best_idx)
        return matched

    def _measure_roi_change(
        self,
        circle: DetectedCircle,
        before_img: np.ndarray,
        after_img: np.ndarray,
    ) -> Tuple[float, float, str]:
        h, w = before_img.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(mask, (circle.cx, circle.cy), circle.radius, 255, -1)
        erode_px = max(1, int(circle.radius * 0.10))
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (erode_px * 2 + 1, erode_px * 2 + 1)
        )
        mask = cv2.erode(mask, kernel, iterations=1)
        mask_bool = mask > 0
        total = int(np.count_nonzero(mask_bool))
        if total == 0:
            return 0.0, 0.0, "Empty ROI mask"

        # 1) BGR pixel-change ratio.
        before_gray = cv2.cvtColor(before_img, cv2.COLOR_BGR2GRAY)
        after_gray = cv2.cvtColor(after_img, cv2.COLOR_BGR2GRAY)
        diff = cv2.absdiff(before_gray, after_gray)
        changed = int(np.count_nonzero((diff > self.pixel_diff_threshold) & mask_bool))
        ratio = changed / total

        # 2) Mean Lab deltaE inside the ROI (perceptual colour shift).
        before_lab = cv2.cvtColor(before_img, cv2.COLOR_BGR2LAB).astype(np.float32)
        after_lab = cv2.cvtColor(after_img, cv2.COLOR_BGR2LAB).astype(np.float32)
        b_mean = cv2.mean(before_lab, mask=mask)[:3]
        a_mean = cv2.mean(after_lab, mask=mask)[:3]
        delta_e = float(
            math.sqrt(sum((a - b) ** 2 for a, b in zip(a_mean, b_mean)))
        )

        detail = (
            f"change={ratio:.1%} pixels ({changed}/{total}), "
            f"lab_delta_e={delta_e:.1f}, "
            f"thresholds=change:{self.change_threshold:.0%} "
            f"or delta_e>={self.change_threshold*100:.0f}"
        )
        return ratio, delta_e, detail


class CircleChangeVerifier:
    """
    Verifies spray success by comparing circle snapshots before and after.

    Algorithm:
    1. Match circles between before/after by proximity (same position)
    2. Measure blue/cyan chemical response inside the target circle
    3. Measure generic pixel change as supporting evidence
    4. Pass only when the blue response is present/increased

    Rain or plain water can make the paper darker/glossier, so generic pixel
    change alone is not enough. The scoring cue is the red-cabbage/baking-soda
    shift from purple toward blue.
    """

    def __init__(
        self,
        change_threshold: float = 0.20,
        pixel_diff_threshold: int = 30,
        match_distance_px: int = 50,
        blue_after_threshold: float = 0.08,
        blue_increase_threshold: float = 0.05,
    ):
        """
        Args:
            change_threshold: Minimum fraction of changed pixels to
                consider verification passed (default 0.20 = 20%).
            pixel_diff_threshold: Minimum per-pixel intensity difference
                (0-255) to count as "changed" (default 30).
            match_distance_px: Max center distance to match before/after
                circles as the same physical target (default 50px).
            blue_after_threshold: Minimum fraction of target pixels that should
                look blue/cyan after spray.
            blue_increase_threshold: Minimum blue/cyan fraction increase from
                before to after.
        """
        self.change_threshold = change_threshold
        self.pixel_diff_threshold = pixel_diff_threshold
        self.match_distance_px = match_distance_px
        self.blue_after_threshold = blue_after_threshold
        self.blue_increase_threshold = blue_increase_threshold
        self._detector = Task2CircleDetector()

    def capture_snapshot(
        self, bgr_image: np.ndarray, image_path: Optional[str] = None
    ) -> CircleSnapshot:
        """
        Detect circles in an image and return a snapshot for later comparison.

        Args:
            bgr_image: OpenCV BGR image
            image_path: Optional path to saved image file

        Returns:
            CircleSnapshot with detected circles and image data
        """
        circles = self._detector.detect(bgr_image)
        return CircleSnapshot(
            circles=circles,
            image=bgr_image,
            image_path=image_path,
        )

    def capture_snapshot_from_file(self, image_path: str) -> CircleSnapshot:
        """Load an image file and capture a circle snapshot."""
        img = cv2.imread(image_path)
        if img is None:
            logger.warning(f"Cannot read image: {image_path}")
            return CircleSnapshot(circles=[], image_path=image_path)
        return self.capture_snapshot(img, image_path)

    def compare(
        self,
        before: CircleSnapshot,
        after: CircleSnapshot,
    ) -> CircleComparison:
        """
        Compare before and after circle snapshots to verify spray effect.

        Matches circles by proximity, then computes pixel difference ratio
        within each matched circle's ROI.

        Args:
            before: CircleSnapshot from before spray
            after: CircleSnapshot from after spray

        Returns:
            CircleComparison with change_ratio, verified flag, and details
        """
        if not before.circles or not after.circles:
            # No circles detected in one or both images
            # If after has fewer circles, target may have visually changed enough
            # to not be detected the same way - that's also a signal
            if before.circles and not after.circles:
                return CircleComparison(
                    change_ratio=1.0,
                    verified=False,
                    matched_pairs=0,
                    details="Circle disappeared after spray; chemical blue response unknown",
                )
            return CircleComparison(
                change_ratio=0.0,
                verified=False,
                matched_pairs=0,
                details="No circles detected in before image",
            )

        if before.image is None or after.image is None:
            return CircleComparison(
                change_ratio=0.0,
                verified=False,
                matched_pairs=0,
                details="Missing image data for pixel comparison",
            )

        # Match circles between before and after by proximity
        matched = self._match_circles(before.circles, after.circles)

        if not matched:
            return CircleComparison(
                change_ratio=0.0,
                verified=False,
                matched_pairs=0,
                details="No matching circles found between before/after",
            )

        # Compute change for each matched pair
        best_ratio = 0.0
        best_details = ""
        best_blue_after = 0.0
        best_blue_increase = 0.0

        for b_circle, a_circle in matched:
            ratio, blue_after, blue_increase, detail = self._compute_circle_change(
                b_circle, a_circle, before.image, after.image
            )
            score = max(ratio, blue_after + blue_increase)
            best_score = max(best_ratio, best_blue_after + best_blue_increase)
            if score > best_score:
                best_ratio = ratio
                best_blue_after = blue_after
                best_blue_increase = blue_increase
                best_details = detail

        blue_verified = (
            best_blue_after >= self.blue_after_threshold
            and best_blue_increase >= self.blue_increase_threshold
        )
        blue_with_change_verified = (
            best_ratio >= self.change_threshold
            and best_blue_after >= self.blue_after_threshold
            and best_blue_increase >= self.blue_increase_threshold * 0.5
        )
        verified = blue_verified or blue_with_change_verified

        return CircleComparison(
            change_ratio=best_ratio,
            verified=verified,
            matched_pairs=len(matched),
            details=best_details,
            blue_after_ratio=best_blue_after,
            blue_increase_ratio=best_blue_increase,
        )

    def _match_circles(
        self,
        before_circles: List[DetectedCircle],
        after_circles: List[DetectedCircle],
    ) -> List[Tuple[DetectedCircle, DetectedCircle]]:
        """Match before/after circles by center proximity."""
        matched = []
        used_after = set()

        for b in before_circles:
            best_dist = float("inf")
            best_idx = -1

            for i, a in enumerate(after_circles):
                if i in used_after:
                    continue
                dist = math.sqrt((b.cx - a.cx) ** 2 + (b.cy - a.cy) ** 2)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i

            if best_idx >= 0 and best_dist < self.match_distance_px:
                matched.append((b, after_circles[best_idx]))
                used_after.add(best_idx)

        return matched

    def _compute_circle_change(
        self,
        before_circle: DetectedCircle,
        after_circle: DetectedCircle,
        before_img: np.ndarray,
        after_img: np.ndarray,
    ) -> Tuple[float, float, float, str]:
        """
        Compute the fraction of pixels that changed within a circle ROI.

        Uses the before circle's mask (position/size), since that's where
        we know the target was before spraying.
        """
        h, w = before_img.shape[:2]

        # Use the before circle's ROI mask
        mask = before_circle.roi_mask
        if mask is None:
            # Recreate mask if not stored
            mask = np.zeros((h, w), dtype=np.uint8)
            cv2.circle(mask, (before_circle.cx, before_circle.cy), before_circle.radius, 255, -1)
        else:
            mask = mask.copy()

        # Avoid the circle boundary where white backing, tape, and glare can
        # dominate the color measurement.
        erode_px = max(1, int(before_circle.radius * 0.08))
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (erode_px * 2 + 1, erode_px * 2 + 1)
        )
        mask = cv2.erode(mask, kernel, iterations=1)

        # Ensure images are same size
        if after_img.shape[:2] != before_img.shape[:2]:
            after_img = cv2.resize(after_img, (w, h))

        # Compute absolute difference on grayscale
        before_gray = cv2.cvtColor(before_img, cv2.COLOR_BGR2GRAY)
        after_gray = cv2.cvtColor(after_img, cv2.COLOR_BGR2GRAY)

        diff = cv2.absdiff(before_gray, after_gray)

        # Count pixels exceeding threshold within circle mask
        mask_bool = mask > 0
        total_pixels = int(np.count_nonzero(mask_bool))

        if total_pixels == 0:
            return 0.0, 0.0, 0.0, "Empty circle mask"

        changed_pixels = int(np.count_nonzero((diff > self.pixel_diff_threshold) & mask_bool))
        change_ratio = changed_pixels / total_pixels
        before_blue_ratio = self._blue_response_ratio(before_img, mask_bool)
        after_blue_ratio = self._blue_response_ratio(after_img, mask_bool)
        blue_increase = max(0.0, after_blue_ratio - before_blue_ratio)

        # Also compute color shift for details
        b_mean = before_circle.mean_color
        a_mean = after_circle.mean_color
        if b_mean is not None and a_mean is not None:
            color_shift = np.linalg.norm(a_mean.astype(float) - b_mean.astype(float))
            color_detail = f"color_shift={color_shift:.1f}"
        else:
            color_detail = "no_color_data"

        detail = (
            f"change={change_ratio:.1%} pixels changed "
            f"({changed_pixels}/{total_pixels}), {color_detail}, "
            f"blue_after={after_blue_ratio:.1%}, "
            f"blue_increase={blue_increase:.1%}, "
            f"thresholds=change:{self.change_threshold:.0%} "
            f"blue:{self.blue_after_threshold:.0%}/+{self.blue_increase_threshold:.0%}"
        )

        return change_ratio, after_blue_ratio, blue_increase, detail

    @staticmethod
    def _blue_response_ratio(bgr_img: np.ndarray, mask_bool: np.ndarray) -> float:
        """Fraction of target pixels showing the blue/cyan baking-soda response."""
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0]
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]
        blue_like = (
            (hue >= 85)
            & (hue <= 130)
            & (sat >= 18)
            & (val >= 35)
            & mask_bool
        )
        total = int(np.count_nonzero(mask_bool))
        if total <= 0:
            return 0.0
        return float(np.count_nonzero(blue_like)) / total
