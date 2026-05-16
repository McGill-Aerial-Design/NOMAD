"""
detectors.py

Target detection for AEAC 2026 Task 1.

Target detection (colored circles):
  - Uses HSV color segmentation + contour analysis
  - No neural network needed; circles are solid-color on building surfaces
  - Classifies: black, white, red, yellow, blue, green (per ConOps)

The main node handles 3D back-projection using depth and TF data.
"""

import math

import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple
from enum import Enum


class TargetColor(Enum):
    BLACK = "black"
    WHITE = "white"
    RED = "red"
    YELLOW = "yellow"
    BLUE = "blue"
    GREEN = "green"
    UNKNOWN = "unknown"


@dataclass
class CircleDetection:
    """A detected colored circle target."""
    cx: int              # center x in pixels
    cy: int              # center y in pixels
    radius: int          # radius in pixels
    color: TargetColor   # classified color
    confidence: float    # detection confidence [0, 1]
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    mask: Optional[np.ndarray] = None  # binary mask of the detection


# ------------------------------------------------------------------ #
#  HSV Color Ranges for Target Classification
# ------------------------------------------------------------------ #
# These ranges are tuned for outdoor/overcast lighting typical of
# Ottawa in May. Adjust after on-site calibration.

# Each color has a list of (lower_hsv, upper_hsv) ranges.
# Red wraps around 0/180 in OpenCV HSV, so it needs two ranges.
HSV_RANGES = {
    # RED covers pure red AND orange-red (hue 0-17). Phone screens, printed
    # targets, and low-sun lighting all shift red toward orange; the old 0-10
    # cap left a dead zone at hue 11-17 that dropped targets silently.
    TargetColor.RED: [
        (np.array([0, 60, 60]), np.array([17, 255, 255])),
        (np.array([165, 60, 60]), np.array([180, 255, 255])),
    ],
    # YELLOW starts at hue 22 so it does not overlap the extended RED band.
    TargetColor.YELLOW: [
        (np.array([22, 90, 90]), np.array([38, 255, 255])),
    ],
    TargetColor.GREEN: [
        (np.array([40, 70, 60]), np.array([85, 255, 255])),
    ],
    # BLUE tightened: old S_min=50 V_min=50 grabbed dark navy shadows and
    # denim/tarp edges. 90/70 keeps saturated competition targets without
    # the false positives seen in indoor testing.
    TargetColor.BLUE: [
        (np.array([95, 90, 70]), np.array([130, 255, 255])),
    ],
    # Black and white use value channel primarily
    TargetColor.BLACK: [
        (np.array([0, 0, 0]), np.array([180, 80, 60])),
    ],
    TargetColor.WHITE: [
        (np.array([0, 0, 180]), np.array([180, 40, 255])),
    ],
}

# Priority order for classification when multiple colors match
COLOR_PRIORITY = [
    TargetColor.RED,
    TargetColor.YELLOW,
    TargetColor.GREEN,
    TargetColor.BLUE,
    TargetColor.BLACK,
    TargetColor.WHITE,
]


class CircleDetector:
    """
    Detects colored circular targets using HSV segmentation and
    contour circularity analysis.

    Pipeline:
      1. Convert to HSV
      2. For each target color, create binary mask from HSV ranges
      3. Find contours in each mask
      4. Filter contours by circularity, area, and solidity
      5. Return detected circles with classified colors
    """

    def __init__(self,
                 min_radius_px: int = 10,
                 max_radius_px: int = 300,
                 min_circularity: float = 0.65,
                 min_solidity: float = 0.75,
                 blur_kernel: int = 5,
                 morph_kernel: int = 5):
        self.min_radius_px = min_radius_px
        self.max_radius_px = max_radius_px
        self.min_circularity = min_circularity
        self.min_solidity = min_solidity
        self.blur_kernel = blur_kernel
        self.morph_kernel = morph_kernel

    def detect(self, bgr_image: np.ndarray) -> List[CircleDetection]:
        """
        Detect colored circles in a BGR image.

        Args:
            bgr_image: OpenCV BGR image (e.g., from ZED camera)

        Returns:
            List of CircleDetection with pixel coordinates and color.
        """
        # Preprocess
        blurred = cv2.GaussianBlur(bgr_image, (self.blur_kernel, self.blur_kernel), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        detections = []
        morph_k = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self.morph_kernel, self.morph_kernel)
        )

        for color in COLOR_PRIORITY:
            # Build mask from all HSV ranges for this color
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in HSV_RANGES[color]:
                mask |= cv2.inRange(hsv, lower, upper)

            # Morphological cleanup
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, morph_k)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, morph_k)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                det = self._evaluate_contour(contour, color, mask)
                if det is not None:
                    # Check for overlap with existing detections
                    if not self._overlaps_existing(det, detections):
                        detections.append(det)

        return detections

    def _evaluate_contour(self, contour: np.ndarray,
                          color: TargetColor,
                          mask: np.ndarray) -> Optional[CircleDetection]:
        """Evaluate a contour for circularity and return a detection if valid."""
        area = cv2.contourArea(contour)
        if area < math.pi * self.min_radius_px**2:
            return None
        if area > math.pi * self.max_radius_px**2:
            return None

        perimeter = cv2.arcLength(contour, True)
        if perimeter < 1.0:
            return None

        # Circularity: 4*pi*area / perimeter^2 (1.0 = perfect circle)
        circularity = (4.0 * math.pi * area) / (perimeter * perimeter)
        if circularity < self.min_circularity:
            return None

        # Solidity: area / convex_hull_area
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area < 1.0:
            return None
        solidity = area / hull_area
        if solidity < self.min_solidity:
            return None

        # Minimum enclosing circle
        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        cx, cy, radius = int(cx), int(cy), int(radius)

        if radius < self.min_radius_px or radius > self.max_radius_px:
            return None

        # Confidence based on circularity and solidity
        confidence = min(circularity, solidity)

        # Bounding box
        x, y, w, h = cv2.boundingRect(contour)
        bbox = (x, y, x + w, y + h)

        return CircleDetection(
            cx=cx, cy=cy, radius=radius,
            color=color, confidence=confidence,
            bbox=bbox
        )

    def _overlaps_existing(self, det: CircleDetection,
                           existing: List[CircleDetection],
                           iou_threshold: float = 0.3) -> bool:
        """Check if a detection overlaps significantly with existing ones."""
        for e in existing:
            # Simple center distance check
            dist = math.sqrt((det.cx - e.cx)**2 + (det.cy - e.cy)**2)
            if dist < max(det.radius, e.radius) * 0.7:
                return True
        return False


class ColorVerifier:
    """
    Independent HSV color verification used to cross-check the circle
    detector's color classification.
    """

    @staticmethod
    def classify_roi(bgr_image: np.ndarray,
                     bbox: Tuple[int, int, int, int]) -> Tuple[TargetColor, float]:
        """
        Classify the dominant color within a bounding box ROI.

        Args:
            bgr_image: Full BGR image
            bbox: (x1, y1, x2, y2) region of interest

        Returns:
            (color, ratio): Best matching color and the fraction of
            pixels in that color's HSV range.
        """
        x1, y1, x2, y2 = bbox
        roi = bgr_image[max(0, y1):y2, max(0, x1):x2]
        if roi.size == 0:
            return TargetColor.UNKNOWN, 0.0

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        total_pixels = hsv.shape[0] * hsv.shape[1]
        if total_pixels == 0:
            return TargetColor.UNKNOWN, 0.0

        best_color = TargetColor.UNKNOWN
        best_ratio = 0.0

        for color in COLOR_PRIORITY:
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in HSV_RANGES[color]:
                mask |= cv2.inRange(hsv, lower, upper)
            ratio = float(cv2.countNonZero(mask)) / total_pixels
            if ratio > best_ratio:
                best_ratio = ratio
                best_color = color

        return best_color, best_ratio
