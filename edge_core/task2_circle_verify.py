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

        Mirrors edge_core/ros/simple_video_bridge.py::_detect_shape_circles.
        Used here on saved before/after snapshots, so the ZED depth gate is
        absent -- physical-diameter validation runs at the live overlay path
        in the bridge. All other gates (Lab+HSV colour, white-backing ring,
        ellipse fit, fused confidence) match.
        """
        if bgr_image is None or bgr_image.size == 0:
            return []

        h, w = bgr_image.shape[:2]
        blurred = cv2.GaussianBlur(
            bgr_image, (self.blur_kernel, self.blur_kernel), 0
        )
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        s_ch = hsv[:, :, 1]
        v_ch = hsv[:, :, 2]
        a_ch = lab[:, :, 1].astype(np.int16)
        b_ch = lab[:, :, 2].astype(np.int16)
        chroma = np.abs(a_ch - 128) + np.abs(b_ch - 128)

        backing_raw = (
            (s_ch <= 40) & (v_ch >= 175) & (chroma <= 22)
        ).astype(np.uint8) * 255
        k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        backing_mask = cv2.morphologyEx(backing_raw, cv2.MORPH_CLOSE, k5, iterations=2)
        backing_mask = cv2.morphologyEx(backing_mask, cv2.MORPH_OPEN, k5, iterations=1)
        n_back, back_labels, back_stats, _ = cv2.connectedComponentsWithStats(
            (backing_mask > 0).astype(np.uint8), connectivity=8
        )
        min_backing_area = max(400, int(0.0020 * w * h))
        valid_backing = np.zeros(n_back, dtype=bool)
        for bidx in range(1, n_back):
            if back_stats[bidx, cv2.CC_STAT_AREA] >= min_backing_area:
                valid_backing[bidx] = True

        mauve = (
            (a_ch > 130) & (chroma >= 6)
            & (v_ch >= 90) & (v_ch <= 240) & (s_ch <= 210)
        )
        blue = (
            (a_ch < 127) & (b_ch < 132) & (chroma >= 8)
            & (v_ch >= 70) & (v_ch <= 220) & (s_ch <= 210)
        )
        not_white = ~((s_ch <= 25) & (v_ch >= 195))
        target_mask = ((mauve | blue) & not_white).astype(np.uint8) * 255
        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        target_mask = cv2.morphologyEx(target_mask, cv2.MORPH_OPEN, k3, iterations=1)
        target_mask = cv2.morphologyEx(target_mask, cv2.MORPH_CLOSE, k5, iterations=2)

        n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            target_mask, connectivity=8
        )

        min_r = self.min_radius_px
        max_r = self.max_radius_px
        min_area = max(20, int(math.pi * min_r * min_r * 0.40))
        max_area = int(math.pi * max_r * max_r)

        candidates: List[Tuple[float, DetectedCircle]] = []
        for idx in range(1, n_labels):
            x = int(stats[idx, cv2.CC_STAT_LEFT])
            y = int(stats[idx, cv2.CC_STAT_TOP])
            bw = int(stats[idx, cv2.CC_STAT_WIDTH])
            bh = int(stats[idx, cv2.CC_STAT_HEIGHT])
            area = int(stats[idx, cv2.CC_STAT_AREA])
            if area < min_area or area > max_area or bw < 4 or bh < 4:
                continue
            aspect = bw / float(bh)
            if aspect < 0.45 or aspect > 2.20:
                continue

            pad = 2
            x0 = max(0, x - pad); y0 = max(0, y - pad)
            x1 = min(w, x + bw + pad); y1 = min(h, y + bh + pad)
            comp_mask = (labels[y0:y1, x0:x1] == idx).astype(np.uint8)
            contours, _ = cv2.findContours(
                comp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
            )
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            contour_area = float(cv2.contourArea(contour))
            if contour_area < min_area:
                continue
            perim = float(cv2.arcLength(contour, True))
            if perim < 1.0:
                continue
            circularity = (4.0 * math.pi * contour_area) / (perim * perim)
            if circularity < 0.42:
                continue
            hull = cv2.convexHull(contour)
            hull_area = float(cv2.contourArea(hull))
            if hull_area < 1.0:
                continue
            solidity = contour_area / hull_area
            if solidity < 0.68:
                continue

            if len(contour) >= 5:
                try:
                    (ecx_loc, ecy_loc), (axis_a, axis_b), _angle = cv2.fitEllipse(contour)
                except cv2.error:
                    continue
                ecx = float(x0) + float(ecx_loc)
                ecy = float(y0) + float(ecy_loc)
                major = float(max(axis_a, axis_b)) * 0.5
                minor = float(min(axis_a, axis_b)) * 0.5
                if minor < 1.0:
                    continue
                ellipticity = minor / major
                if ellipticity < 0.50:
                    continue
            else:
                (ecx_loc, ecy_loc), r_fit = cv2.minEnclosingCircle(contour)
                ecx = float(x0) + float(ecx_loc)
                ecy = float(y0) + float(ecy_loc)
                major = float(r_fit); minor = float(r_fit)
                ellipticity = 1.0

            cx_i = int(round(ecx)); cy_i = int(round(ecy))
            if not (0 <= cx_i < w and 0 <= cy_i < h):
                continue
            radius_px = major
            if radius_px < min_r or radius_px > max_r:
                continue

            outer_r = int(radius_px * 1.85) + 6
            ry0 = max(0, cy_i - outer_r); ry1 = min(h, cy_i + outer_r + 1)
            rx0 = max(0, cx_i - outer_r); rx1 = min(w, cx_i + outer_r + 1)
            if ry1 <= ry0 or rx1 <= rx0:
                continue
            yy, xx = np.ogrid[ry0:ry1, rx0:rx1]
            d2 = (xx - cx_i) ** 2 + (yy - cy_i) ** 2
            inner = max(radius_px * 1.10, radius_px + 2.0)
            ring = (d2 >= inner * inner) & (d2 <= outer_r * outer_r)
            ring_count = int(ring.sum())
            backing_ratio = 0.0
            has_backing = False
            if ring_count > 0:
                ring_white = int(np.count_nonzero(
                    backing_mask[ry0:ry1, rx0:rx1] & ring
                ))
                backing_ratio = ring_white / ring_count
                sup = back_labels[ry0:ry1, rx0:rx1][ring]
                sup = sup[sup > 0]
                support_label = 0
                if sup.size:
                    support_label = int(np.argmax(np.bincount(sup)))
                if (
                    0 < support_label < n_back
                    and valid_backing[support_label]
                ):
                    plate_area = int(
                        back_stats[support_label, cv2.CC_STAT_AREA]
                    )
                    if (
                        plate_area >= max(int(area * 1.2), 500)
                        and plate_area <= int(w * h * 0.55)
                        and backing_ratio >= 0.15
                    ):
                        has_backing = True

            inner_r = max(2.0, radius_px * 0.7)
            inner_disk = d2 <= inner_r * inner_r
            inner_count = int(inner_disk.sum())
            if inner_count <= 0:
                continue
            inside_color = int(np.count_nonzero(target_mask[ry0:ry1, rx0:rx1] & inner_disk))
            interior_color_ratio = inside_color / inner_count
            if interior_color_ratio < 0.45:
                continue

            shape_strong = (
                circularity >= 0.62
                and solidity >= 0.80
                and ellipticity >= 0.62
                and interior_color_ratio >= 0.62
            )
            if not has_backing and not shape_strong:
                continue

            backing_score = min(backing_ratio, 0.80) * 0.25 if has_backing else 0.0
            confidence = min(
                0.99,
                0.12
                + min(circularity, 0.95) * 0.28
                + min(solidity, 0.95) * 0.14
                + min(ellipticity, 0.95) * 0.12
                + backing_score
                + min(interior_color_ratio, 0.95) * 0.28,
            )

            det = self._make_detection(
                cx_i, cy_i, int(round(major)), bgr_image, "cabbage_v2"
            )
            if det is None:
                continue
            det.confidence = float(confidence)
            candidates.append((confidence, det))

        # NMS by center distance, keep highest-confidence per spatial group.
        candidates.sort(key=lambda kv: -kv[0])
        detections: List[DetectedCircle] = []
        for _, det in candidates:
            if not self._overlaps_existing(det, detections):
                detections.append(det)
            if len(detections) >= 8:
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
