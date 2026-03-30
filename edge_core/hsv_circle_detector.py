"""
HSV-based Circle/Ellipse Detection for NOMAD Task 1.

Detects colored circular targets that may appear as ellipses (angled view),
partially occluded, or deformed due to perspective. Uses a multi-strategy
approach:

1. Per-color HSV masking to isolate candidate blobs
2. Contour analysis with ellipse fitting (handles ovals, perspective)
3. Circularity + solidity scoring to reject random shapes
4. HoughCircles as secondary confirmation
5. Duplicate merging across both strategies

No YOLO26 dependency.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger("edge_core.hsv_circle_detector")

CIRCLE_COLORS = ["red", "blue", "green", "yellow", "white", "black"]

# HSV ranges (OpenCV: H 0-179, S 0-255, V 0-255)
# High saturation minimums to reject skin, wood, natural surfaces.
# Competition targets are vivid/saturated colors, not subtle tones.
HSV_RANGES = {
    "red":    [(0, 8, 120, 70), (172, 179, 120, 70)],
    "blue":   [(100, 128, 100, 50)],
    "green":  [(38, 82, 100, 50)],
    "yellow": [(20, 35, 130, 100)],
    "white":  [],
    "black":  [],
}

WHITE_S_MAX = 30       # very low saturation only
WHITE_V_MIN = 210      # very bright only
BLACK_V_MAX = 35       # very dark only — tighter to avoid shadows/dark objects

# Skin tone exclusion: skin in HSV is roughly H=0-20, S=40-170, V=80-255
# We reject any "red" contour where most pixels fall in this range
SKIN_H_LO, SKIN_H_HI = 0, 22
SKIN_S_LO, SKIN_S_HI = 30, 175
SKIN_V_LO = 70


@dataclass
class DetectedCircle:
    """A detected colored circle/ellipse target."""
    cx: int
    cy: int
    radius: int
    color: str
    confidence: float
    # Ellipse: half-axes in the fitEllipse (width, height) order + angle
    ellipse_half_w: int = 0       # half of fitEllipse width axis
    ellipse_half_h: int = 0       # half of fitEllipse height axis
    ellipse_angle: float = 0.0    # fitEllipse rotation (degrees)
    aspect_ratio: float = 1.0     # minor/major
    circularity: float = 1.0
    solidity: float = 1.0         # contour area / convex hull area
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    bbox_x: int = 0
    bbox_y: int = 0
    bbox_w: int = 0
    bbox_h: int = 0
    dominant_hue: float = 0.0
    dominant_sat: float = 0.0
    dominant_val: float = 0.0
    detection_method: str = ""


# ---------------------------------------------------------------------------
# Color classification
# ---------------------------------------------------------------------------

def classify_color_hsv(roi_hsv: np.ndarray) -> Tuple[str, float]:
    """Classify dominant color in an HSV region. Returns (name, confidence)."""
    pixels = roi_hsv.reshape(-1, 3)
    n = len(pixels)
    if n == 0:
        return "unknown", 0.0

    h, s, v = pixels[:, 0], pixels[:, 1], pixels[:, 2]

    votes: dict[str, int] = {}
    black_mask = v < BLACK_V_MAX
    white_mask = (s < WHITE_S_MAX) & (v > WHITE_V_MIN) & ~black_mask
    votes["black"] = int(np.sum(black_mask))
    votes["white"] = int(np.sum(white_mask))

    chromatic = ~black_mask & ~white_mask
    for cname, ranges in HSV_RANGES.items():
        if cname in ("white", "black"):
            continue
        count = 0
        for h_lo, h_hi, s_min, v_min in ranges:
            count += int(np.sum(
                chromatic & (h >= h_lo) & (h <= h_hi) & (s >= s_min) & (v >= v_min)
            ))
        votes[cname] = count

    best = max(votes, key=lambda k: votes[k])
    if votes[best] == 0:
        return "unknown", 0.0
    return best, votes[best] / n


# ---------------------------------------------------------------------------
# HSV mask generation
# ---------------------------------------------------------------------------

def _make_color_mask(hsv: np.ndarray, color: str, morph_size: int = 5) -> np.ndarray:
    """Create binary mask for a target color with morphological cleanup."""
    if color == "black":
        mask = cv2.inRange(hsv, (0, 0, 0), (179, 255, BLACK_V_MAX))
    elif color == "white":
        mask = cv2.inRange(hsv, (0, 0, WHITE_V_MIN), (179, WHITE_S_MAX, 255))
    else:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for h_lo, h_hi, s_min, v_min in HSV_RANGES.get(color, []):
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, (h_lo, s_min, v_min), (h_hi, 255, 255)))

    # Median blur kills salt-and-pepper noise in the mask before morphology
    mask = cv2.medianBlur(mask, 5)

    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_size, morph_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)
    return mask


def _adaptive_morph_size(min_radius: int, max_radius: int) -> int:
    """Scale morphology kernel from expected target radius range.

    Keeps the kernel odd and bounded so cleanup remains stable across
    near/far targets while avoiding over-smoothing.
    """
    expected_radius = max(1, int(round((min_radius + max_radius) * 0.5)))
    morph_size = int(round(expected_radius * 0.08))
    morph_size = max(3, min(11, morph_size))
    if morph_size % 2 == 0:
        morph_size += 1
    return morph_size


# ---------------------------------------------------------------------------
# Contour-based detection (primary)
# ---------------------------------------------------------------------------

def _contour_detect(
    image_hsv: np.ndarray,
    h_img: int,
    w_img: int,
    min_radius: int,
    max_radius: int,
    morph_size: int,
    min_circularity: float,
    min_solidity: float,
    min_color_confidence: float,
    min_contour_area: int,
    min_aspect_ratio: float,
) -> List[DetectedCircle]:
    """Detect colored ellipses via per-color HSV masks + contour + ellipse fit."""
    results: List[DetectedCircle] = []

    for color in CIRCLE_COLORS:
        mask = _make_color_mask(image_hsv, color, morph_size=morph_size)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Max area: a target circle won't cover more than 15% of the frame
        max_contour_area = h_img * w_img * 0.15

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_contour_area or area > max_contour_area:
                continue

            eq_radius = math.sqrt(area / math.pi)
            if eq_radius < min_radius or eq_radius > max_radius:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = (4.0 * math.pi * area) / (perimeter * perimeter)
            if circularity < min_circularity:
                continue

            # Solidity: area / convex hull area — rejects irregular junk
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            if solidity < min_solidity:
                continue

            # Fit ellipse (needs >= 5 points)
            if len(cnt) < 5:
                continue

            ellipse = cv2.fitEllipse(cnt)
            (ecx, ecy), (ew, eh), angle = ellipse
            # ew, eh are FULL axis lengths; angle rotates the ew-axis
            half_w = ew / 2.0
            half_h = eh / 2.0
            semi_major = max(half_w, half_h)
            semi_minor = min(half_w, half_h)
            aspect = semi_minor / semi_major if semi_major > 0 else 0

            # Require contour to not touch image edges (likely a clipped object,
            # not a full target — e.g. a shirt, wall, table)
            bx, by, bw, bh = cv2.boundingRect(cnt)
            touches_edge = (bx <= 2 or by <= 2 or
                            bx + bw >= w_img - 2 or by + bh >= h_img - 2)
            if touches_edge:
                continue

            if aspect < min_aspect_ratio:
                continue

            cx = int(ecx)
            cy = int(ecy)
            if cx < 0 or cy < 0 or cx >= w_img or cy >= h_img:
                continue

            radius = int((semi_major + semi_minor) / 2.0)

            # Verify color inside contour
            cmask = np.zeros((h_img, w_img), dtype=np.uint8)
            cv2.drawContours(cmask, [cnt], -1, 255, -1)
            cmask_inner = cv2.erode(cmask, np.ones((3, 3), np.uint8), iterations=1)
            if cv2.countNonZero(cmask_inner) < 5:
                cmask_inner = cmask

            masked_hsv = image_hsv[cmask_inner > 0]
            if len(masked_hsv) < 5:
                continue

            verified_color, color_conf = classify_color_hsv(masked_hsv)
            if verified_color != color or color_conf < min_color_confidence:
                continue

            # Skin-tone rejection for red detections:
            # If >40% of pixels fall in skin HSV range, skip
            if color == "red":
                ph = masked_hsv[:, 0]
                ps = masked_hsv[:, 1]
                pv = masked_hsv[:, 2]
                skin_mask = (
                    (ph >= SKIN_H_LO) & (ph <= SKIN_H_HI)
                    & (ps >= SKIN_S_LO) & (ps <= SKIN_S_HI)
                    & (pv >= SKIN_V_LO)
                )
                skin_ratio = np.sum(skin_mask) / len(masked_hsv)
                if skin_ratio > 0.40:
                    continue

            # Composite confidence
            size_factor = min(1.0, eq_radius / 25.0)
            confidence = color_conf * (0.4 + 0.3 * circularity + 0.3 * solidity) * (0.5 + 0.5 * size_factor)

            mean_hsv = np.mean(masked_hsv, axis=0)

            results.append(DetectedCircle(
                cx=cx, cy=cy, radius=radius,
                color=color, confidence=confidence,
                # Store in the same order as fitEllipse so drawing works
                ellipse_half_w=int(half_w),
                ellipse_half_h=int(half_h),
                ellipse_angle=angle,
                aspect_ratio=aspect,
                circularity=circularity,
                solidity=solidity,
                bbox_x=bx, bbox_y=by, bbox_w=bw, bbox_h=bh,
                dominant_hue=float(mean_hsv[0]),
                dominant_sat=float(mean_hsv[1]),
                dominant_val=float(mean_hsv[2]),
                detection_method="contour",
            ))

    return results


# ---------------------------------------------------------------------------
# HoughCircles (secondary)
# ---------------------------------------------------------------------------

def _hough_detect(
    image_bgr: np.ndarray,
    image_hsv: np.ndarray,
    h_img: int,
    w_img: int,
    min_radius: int,
    max_radius: int,
    dp: float,
    min_dist: int,
    param1: float,
    param2: float,
    min_color_confidence: float,
) -> List[DetectedCircle]:
    """HoughCircles for well-formed circles.

    Preprocessing pipeline:
    1. Bilateral filter — smooths texture/noise while preserving circle edges
    2. CLAHE — normalizes contrast so circles in shadow or bright areas both
       produce strong edges
    3. Gaussian blur — final softening so Hough accumulator tolerates edge
       points ±2-3 px from the true circle (positional sloppiness)
    """
    # Bilateral: preserve edges, kill texture
    denoised = cv2.bilateralFilter(image_bgr, d=9, sigmaColor=75, sigmaSpace=75)
    gray = cv2.cvtColor(denoised, cv2.COLOR_BGR2GRAY)

    # CLAHE: adaptive contrast — makes circle edges pop in uneven lighting
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)

    # Gaussian: let Hough tolerate a few px of edge sloppiness
    gray = cv2.GaussianBlur(gray, (9, 9), 2)

    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=dp, minDist=min_dist,
        param1=param1, param2=param2,
        minRadius=min_radius, maxRadius=max_radius,
    )
    if circles is None:
        return []

    results: List[DetectedCircle] = []
    for (fcx, fcy, fr) in np.round(circles[0]).astype(int):
        if fcx < 0 or fcy < 0 or fcx >= w_img or fcy >= h_img:
            continue

        inner_r = max(3, int(fr * 0.65))
        y1, y2 = max(0, fcy - inner_r), min(h_img, fcy + inner_r)
        x1, x2 = max(0, fcx - inner_r), min(w_img, fcx + inner_r)
        if x2 <= x1 or y2 <= y1:
            continue

        cmask = np.zeros((y2 - y1, x2 - x1), dtype=np.uint8)
        cv2.circle(cmask, (fcx - x1, fcy - y1), inner_r, 255, -1)
        masked_hsv = image_hsv[y1:y2, x1:x2][cmask > 0]
        if len(masked_hsv) < 5:
            continue

        cname, cconf = classify_color_hsv(masked_hsv)
        if cname == "unknown" or cconf < min_color_confidence:
            continue

        # Hough picks up too many dark shadows and bright spots as black/white.
        # Only the contour path (which requires a coherent color blob) should
        # detect black and white targets.
        if cname in ("black", "white"):
            continue

        # Skin rejection for red
        if cname == "red":
            ph = masked_hsv[:, 0]; ps = masked_hsv[:, 1]; pv = masked_hsv[:, 2]
            skin_r = np.sum((ph >= SKIN_H_LO) & (ph <= SKIN_H_HI) & (ps >= SKIN_S_LO) & (ps <= SKIN_S_HI) & (pv >= SKIN_V_LO)) / len(masked_hsv)
            if skin_r > 0.40:
                continue

        mean_hsv = np.mean(masked_hsv, axis=0)
        results.append(DetectedCircle(
            cx=fcx, cy=fcy, radius=fr,
            color=cname, confidence=cconf,
            ellipse_half_w=fr, ellipse_half_h=fr, ellipse_angle=0.0,
            aspect_ratio=1.0, circularity=1.0, solidity=1.0,
            bbox_x=max(0, fcx - fr), bbox_y=max(0, fcy - fr),
            bbox_w=min(2 * fr, w_img), bbox_h=min(2 * fr, h_img),
            dominant_hue=float(mean_hsv[0]),
            dominant_sat=float(mean_hsv[1]),
            dominant_val=float(mean_hsv[2]),
            detection_method="hough",
        ))
    return results


# ---------------------------------------------------------------------------
# Merge overlapping detections
# ---------------------------------------------------------------------------

def _merge_detections(dets: List[DetectedCircle], overlap_px: int = 30) -> List[DetectedCircle]:
    """Merge overlapping detections of the same color, keeping higher confidence."""
    if len(dets) <= 1:
        return dets

    dets.sort(key=lambda d: d.confidence, reverse=True)
    merged: List[DetectedCircle] = []

    for det in dets:
        dup = False
        for kept in merged:
            if det.color != kept.color:
                continue
            dist = math.hypot(det.cx - kept.cx, det.cy - kept.cy)
            threshold = max(overlap_px, (det.radius + kept.radius) * 0.5)
            if dist < threshold:
                dup = True
                break
        if not dup:
            merged.append(det)

    return merged


# ---------------------------------------------------------------------------
# Depth -> 3D
# ---------------------------------------------------------------------------

def _project_to_3d(det: DetectedCircle, depth_image, camera_matrix) -> None:
    if depth_image is None or camera_matrix is None:
        return
    dh, dw = depth_image.shape[:2]
    cx, cy = det.cx, det.cy
    if not (0 <= cy < dh and 0 <= cx < dw):
        return
    r = max(1, det.radius // 4)
    patch = depth_image[
        max(0, cy - r):min(dh, cy + r + 1),
        max(0, cx - r):min(dw, cx + r + 1),
    ].flatten()
    valid = patch[(patch > 0.3) & (patch < 30.0) & np.isfinite(patch)]
    if len(valid) == 0:
        return
    d = float(np.median(valid))
    det.x = float((cx - camera_matrix[0, 2]) * d / camera_matrix[0, 0])
    det.y = float((cy - camera_matrix[1, 2]) * d / camera_matrix[1, 1])
    det.z = d


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def detect_circles_hsv(
    image_bgr: np.ndarray,
    depth_image: Optional[np.ndarray] = None,
    camera_matrix: Optional[np.ndarray] = None,
    min_radius: int = 10,
    max_radius: int = 200,
    dp: float = 1.2,
    min_dist: int = 50,
    param1: float = 100,
    param2: float = 30,
    min_color_confidence: float = 0.50,
    min_circularity: float = 0.60,
    min_solidity: float = 0.80,
    min_aspect_ratio: float = 0.35,
    min_contour_area: int = 500,
) -> List[DetectedCircle]:
    """
    Detect colored circles/ellipses in a BGR image.

    Two strategies merged:
    1. Per-color HSV mask -> contour -> ellipse fit (handles ovals/occlusion)
    2. HoughCircles -> HSV verify (catches clean circles)

    Args:
        min_circularity: 0-1, how round the contour perimeter must be.
            1.0=perfect circle, 0.55=moderately deformed/oval.
        min_solidity: 0-1, contour area / convex hull area.
            Rejects irregular or concave shapes. 0.75 is fairly strict.
        min_aspect_ratio: ellipse minor/major. 0.35 = ~70° viewing angle.
        min_contour_area: minimum blob size in pixels².
    """
    if image_bgr is None or image_bgr.size == 0:
        return []

    h_img, w_img = image_bgr.shape[:2]
    morph_size = _adaptive_morph_size(min_radius=min_radius, max_radius=max_radius)

    # Contour path only: light pre-blur reduces tiny mask speckles before HSV.
    contour_src = cv2.GaussianBlur(image_bgr, (5, 5), 0)
    contour_hsv = cv2.cvtColor(contour_src, cv2.COLOR_BGR2HSV)

    # Hough color verification uses minimally processed HSV.
    verify_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

    contour_dets = _contour_detect(
        contour_hsv, h_img, w_img,
        min_radius=min_radius, max_radius=max_radius,
        morph_size=morph_size,
        min_circularity=min_circularity,
        min_solidity=min_solidity,
        min_color_confidence=min_color_confidence,
        min_contour_area=min_contour_area,
        min_aspect_ratio=min_aspect_ratio,
    )

    hough_dets = _hough_detect(
        image_bgr, verify_hsv, h_img, w_img,
        min_radius=min_radius, max_radius=max_radius,
        dp=dp, min_dist=min_dist, param1=param1, param2=param2,
        min_color_confidence=min_color_confidence,
    )

    merged = _merge_detections(contour_dets + hough_dets)

    for det in merged:
        _project_to_3d(det, depth_image, camera_matrix)

    merged.sort(key=lambda d: d.confidence, reverse=True)
    return merged


# ---------------------------------------------------------------------------
# Overlay drawing
# ---------------------------------------------------------------------------

def draw_detection_overlay(
    image_bgr: np.ndarray,
    detections: List[DetectedCircle],
    thickness: int = 2,
    font_scale: float = 0.6,
) -> np.ndarray:
    """Draw detection overlays with correctly oriented ellipses."""
    DRAW_COLORS = {
        "red": (0, 0, 255), "blue": (255, 0, 0), "green": (0, 255, 0),
        "yellow": (0, 255, 255), "white": (200, 200, 200),
        "black": (128, 128, 128), "unknown": (100, 100, 100),
    }

    for i, det in enumerate(detections):
        c = DRAW_COLORS.get(det.color, (100, 100, 100))

        # Draw the ellipse using the ORIGINAL fitEllipse convention:
        # cv2.ellipse(img, center, (half_w, half_h), angle, ...)
        if det.ellipse_half_w > 0 and det.ellipse_half_h > 0:
            cv2.ellipse(
                image_bgr,
                (det.cx, det.cy),
                (det.ellipse_half_w, det.ellipse_half_h),
                det.ellipse_angle,
                0, 360, c, thickness,
            )
        else:
            cv2.circle(image_bgr, (det.cx, det.cy), det.radius, c, thickness)

        # Crosshair
        cv2.drawMarker(image_bgr, (det.cx, det.cy), c,
                        cv2.MARKER_CROSS, 10, 1)

        # Labels
        letter = chr(ord('A') + i) if i < 26 else str(i)
        line1 = f"{letter}: {det.color} {det.confidence:.0%}"
        line2 = f"ar={det.aspect_ratio:.2f} c={det.circularity:.2f} s={det.solidity:.2f} [{det.detection_method}]"
        if det.z is not None:
            line2 += f" {det.z:.1f}m"

        for j, text in enumerate([line1, line2]):
            sc = font_scale if j == 0 else font_scale * 0.7
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, sc, 1)
            ly = max(th + 4, det.bbox_y - 8 + j * (th + 6))
            lx = max(0, det.cx - tw // 2)
            cv2.rectangle(image_bgr, (lx - 2, ly - th - 2), (lx + tw + 2, ly + 2), (0, 0, 0), -1)
            cv2.putText(image_bgr, text, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, sc, c, 1, cv2.LINE_AA)

    if detections:
        methods = {}
        for d in detections:
            methods[d.detection_method] = methods.get(d.detection_method, 0) + 1
        mstr = " ".join(f"{k}:{v}" for k, v in methods.items())
        summary = f"HSV Detection: {len(detections)} target(s) | {mstr}"
        cv2.rectangle(image_bgr, (0, 0), (len(summary) * 10 + 10, 28), (0, 0, 0), -1)
        cv2.putText(image_bgr, summary, (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)

    return image_bgr
