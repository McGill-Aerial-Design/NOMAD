"""
task2_circle_verify.py - Task 2 Circle Detection & Change Verification

Color-agnostic circle detection for indoor fire extinguishing targets.
Detects circles regardless of color (purple, blue, or any other),
then verifies spray success by comparing before/after circle snapshots.

Competition context (AEAC 2026 Task 2):
- Targets are circles (5-30cm dia) on white plastic backing
- Before spray: purple (red cabbage juice dyed paper)
- After spray: turns blue (pH reaction with baking soda water)
- Verification: circle change > 20% indicates successful hit

Detection approach:
- Grayscale Hough Circle Transform (primary) - works on any color circle
- Contour-based circularity check (fallback) - uses edge detection + shape analysis
- Neither method depends on HSV color filtering

Verification approach:
- Before spray: detect circles, store properties (position, radius, ROI pixels)
- After spray: detect circles, match by proximity, compare
- Change metric: pixel difference ratio within matched circle ROI
- If > 20% of pixels changed significantly => verification passes

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
    verified: bool             # change_ratio > threshold
    matched_pairs: int         # number of before/after circle pairs matched
    details: str = ""          # human-readable summary


class Task2CircleDetector:
    """
    Color-agnostic circle detector for Task 2 indoor targets.

    Uses two methods:
    1. Hough Circle Transform on grayscale (primary)
    2. Contour circularity on Canny edges (fallback/supplement)

    Neither method filters by color, so circles are detected regardless
    of whether they are purple (pre-spray) or blue (post-spray).
    """

    def __init__(
        self,
        min_radius_px: int = 15,
        max_radius_px: int = 300,
        hough_dp: float = 1.5,
        hough_param1: int = 100,
        hough_param2: int = 40,
        min_circularity: float = 0.60,
        min_solidity: float = 0.70,
        blur_kernel: int = 5,
        canny_low: int = 50,
        canny_high: int = 150,
    ):
        self.min_radius_px = min_radius_px
        self.max_radius_px = max_radius_px
        self.hough_dp = hough_dp
        self.hough_param1 = hough_param1
        self.hough_param2 = hough_param2
        self.min_circularity = min_circularity
        self.min_solidity = min_solidity
        self.blur_kernel = blur_kernel
        self.canny_low = canny_low
        self.canny_high = canny_high

    def detect(self, bgr_image: np.ndarray) -> List[DetectedCircle]:
        """
        Detect circles in a BGR image (color-agnostic).

        Runs Hough Circle Transform first, then supplements with
        contour-based detection. Deduplicates overlapping results.

        Args:
            bgr_image: OpenCV BGR image (e.g., from ZED camera)

        Returns:
            List of DetectedCircle with pixel positions and properties.
        """
        if bgr_image is None or bgr_image.size == 0:
            return []

        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (self.blur_kernel, self.blur_kernel), 0)

        detections: List[DetectedCircle] = []

        # Method 1: Hough Circle Transform
        hough_circles = self._detect_hough(blurred)
        for c in hough_circles:
            det = self._make_detection(c[0], c[1], c[2], bgr_image, "hough")
            if det is not None:
                detections.append(det)

        # Method 2: Contour-based detection (catches circles Hough misses)
        contour_circles = self._detect_contours(blurred)
        for c in contour_circles:
            det = self._make_detection(c[0], c[1], c[2], bgr_image, "contour")
            if det is not None and not self._overlaps_existing(det, detections):
                detections.append(det)

        return detections

    def _detect_hough(self, gray_blurred: np.ndarray) -> List[Tuple[int, int, int]]:
        """Run Hough Circle Transform on preprocessed grayscale image."""
        try:
            circles = cv2.HoughCircles(
                gray_blurred,
                cv2.HOUGH_GRADIENT,
                dp=self.hough_dp,
                minDist=self.min_radius_px * 2,
                param1=self.hough_param1,
                param2=self.hough_param2,
                minRadius=self.min_radius_px,
                maxRadius=self.max_radius_px,
            )
        except cv2.error as e:
            logger.warning(f"HoughCircles failed: {e}")
            return []

        if circles is None:
            return []

        result = []
        for c in circles[0]:
            cx, cy, r = int(c[0]), int(c[1]), int(c[2])
            result.append((cx, cy, r))
        return result

    def _detect_contours(self, gray_blurred: np.ndarray) -> List[Tuple[int, int, int]]:
        """Detect circles via Canny edges + contour circularity analysis."""
        edges = cv2.Canny(gray_blurred, self.canny_low, self.canny_high)

        # Dilate edges to close small gaps in circle outlines
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        edges = cv2.dilate(edges, kernel, iterations=1)

        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        result = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < math.pi * self.min_radius_px ** 2:
                continue
            if area > math.pi * self.max_radius_px ** 2:
                continue

            perimeter = cv2.arcLength(contour, True)
            if perimeter < 1.0:
                continue

            circularity = (4.0 * math.pi * area) / (perimeter * perimeter)
            if circularity < self.min_circularity:
                continue

            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            if hull_area < 1.0:
                continue
            solidity = area / hull_area
            if solidity < self.min_solidity:
                continue

            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            cx, cy, radius = int(cx), int(cy), int(radius)
            if radius < self.min_radius_px or radius > self.max_radius_px:
                continue

            result.append((cx, cy, radius))
        return result

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

        # Confidence: for Hough use the accumulator value proxy,
        # for contour use circularity proxy
        confidence = 0.8 if method == "hough" else 0.6

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


class CircleChangeVerifier:
    """
    Verifies spray success by comparing circle snapshots before and after.

    Algorithm:
    1. Match circles between before/after by proximity (same position)
    2. For each matched pair, compute pixel difference ratio within the circle ROI
    3. If any matched circle shows > threshold change, verification passes

    The change ratio is the fraction of pixels in the circle that changed
    by more than a per-pixel threshold (default: 30/255 intensity difference).

    A 20% change ratio threshold means: if more than 20% of the pixels
    inside the target circle look different after spraying, the spray hit.
    """

    def __init__(
        self,
        change_threshold: float = 0.20,
        pixel_diff_threshold: int = 30,
        match_distance_px: int = 50,
    ):
        """
        Args:
            change_threshold: Minimum fraction of changed pixels to
                consider verification passed (default 0.20 = 20%).
            pixel_diff_threshold: Minimum per-pixel intensity difference
                (0-255) to count as "changed" (default 30).
            match_distance_px: Max center distance to match before/after
                circles as the same physical target (default 50px).
        """
        self.change_threshold = change_threshold
        self.pixel_diff_threshold = pixel_diff_threshold
        self.match_distance_px = match_distance_px
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
                    verified=True,
                    matched_pairs=0,
                    details="Circle disappeared after spray (likely hit)",
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

        for b_circle, a_circle in matched:
            ratio, detail = self._compute_circle_change(
                b_circle, a_circle, before.image, after.image
            )
            if ratio > best_ratio:
                best_ratio = ratio
                best_details = detail

        verified = best_ratio >= self.change_threshold

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
    ) -> Tuple[float, str]:
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
            return 0.0, "Empty circle mask"

        changed_pixels = int(np.count_nonzero((diff > self.pixel_diff_threshold) & mask_bool))
        change_ratio = changed_pixels / total_pixels

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
            f"threshold={self.change_threshold:.0%}"
        )

        return change_ratio, detail
