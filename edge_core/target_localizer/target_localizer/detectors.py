"""
detectors.py

Target and landmark detection for AEAC 2026 Task 1.

Target detection (colored circles):
  - Uses HSV color segmentation + contour analysis
  - No neural network needed; circles are solid-color on building surfaces
  - Classifies: black, white, red, yellow, blue, green (per ConOps)

Landmark detection (doors, windows):
  - Uses YOLO model via TensorRT / OpenCV DNN
  - Auto-registers detected landmarks into the building model

Both detectors output bounding boxes and class labels. The main node
handles 3D back-projection using depth and TF data.
"""

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


@dataclass
class LandmarkDetection:
    """A detected landmark (door, window, etc.)."""
    cx: int
    cy: int
    kind: str            # "door", "window"
    confidence: float
    bbox: Tuple[int, int, int, int]


# ------------------------------------------------------------------ #
#  HSV Color Ranges for Target Classification
# ------------------------------------------------------------------ #
# These ranges are tuned for outdoor/overcast lighting typical of
# Ottawa in May. Adjust after on-site calibration.

# Each color has a list of (lower_hsv, upper_hsv) ranges.
# Red wraps around 0/180 in OpenCV HSV, so it needs two ranges.
HSV_RANGES = {
    TargetColor.RED: [
        (np.array([0, 80, 60]), np.array([10, 255, 255])),
        (np.array([170, 80, 60]), np.array([180, 255, 255])),
    ],
    TargetColor.YELLOW: [
        (np.array([18, 80, 80]), np.array([38, 255, 255])),
    ],
    TargetColor.GREEN: [
        (np.array([38, 50, 50]), np.array([85, 255, 255])),
    ],
    TargetColor.BLUE: [
        (np.array([90, 50, 50]), np.array([135, 255, 255])),
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


class LandmarkDetector:
    """
    Detects building landmarks (doors, windows) using a YOLO model.

    The model should be trained or fine-tuned to detect:
      - door / doorway
      - window

    Accepts either:
      - TensorRT engine file (.engine) for Jetson deployment
      - ONNX model file (.onnx) for development/testing
    """

    # YOLO class names (adjust to match your trained model)
    CLASS_NAMES = {
        0: "door",
        1: "window",
    }

    def __init__(self,
                 model_path: str,
                 input_size: Tuple[int, int] = (640, 640),
                 conf_threshold: float = 0.4,
                 nms_threshold: float = 0.45,
                 use_tensorrt: bool = True):
        """
        Args:
            model_path: Path to ONNX or TensorRT engine file.
            input_size: Model input resolution (width, height).
            conf_threshold: Minimum detection confidence.
            nms_threshold: NMS IoU threshold.
            use_tensorrt: If True, attempt to load as TensorRT engine.
        """
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        self.net = None
        self.use_tensorrt = use_tensorrt
        self.model_path = model_path

        self._load_model()

    def _load_model(self):
        """Load the YOLO model via OpenCV DNN."""
        try:
            if self.model_path.endswith('.onnx'):
                self.net = cv2.dnn.readNetFromONNX(self.model_path)
                if self.use_tensorrt:
                    self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                    self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
                else:
                    self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                    self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            print(f"[LandmarkDetector] Model loaded from {self.model_path}")
        except Exception as e:
            print(f"[LandmarkDetector] WARNING: Could not load model: {e}")
            print("[LandmarkDetector] Landmark detection will be disabled.")
            self.net = None

    def detect(self, bgr_image: np.ndarray) -> List[LandmarkDetection]:
        """Run YOLO inference and return detected landmarks."""
        if self.net is None:
            return []

        h, w = bgr_image.shape[:2]
        blob = cv2.dnn.blobFromImage(
            bgr_image, 1/255.0, self.input_size,
            swapRB=True, crop=False
        )
        self.net.setInput(blob)

        try:
            outputs = self.net.forward()
        except Exception as e:
            print(f"[LandmarkDetector] Inference error: {e}")
            return []

        return self._postprocess(outputs, w, h)

    def _postprocess(self, outputs: np.ndarray,
                     img_w: int, img_h: int) -> List[LandmarkDetection]:
        """Parse YOLO output tensor into LandmarkDetection list."""
        detections = []
        # YOLO output shape: (1, num_classes+4, num_detections) for YOLOv8
        # or (1, num_detections, num_classes+5) for YOLOv5
        # Adjust based on your model variant

        if len(outputs.shape) == 3:
            out = outputs[0]
            # YOLOv8 format: (num_classes+4, N) -> transpose to (N, num_classes+4)
            if out.shape[0] < out.shape[1]:
                out = out.T

            boxes = []
            confidences = []
            class_ids = []

            for row in out:
                # row: [cx, cy, w, h, class_scores...]
                if len(row) < 5:
                    continue
                scores = row[4:]
                class_id = int(np.argmax(scores))
                conf = float(scores[class_id])

                if conf < self.conf_threshold:
                    continue
                if class_id not in self.CLASS_NAMES:
                    continue

                cx_norm = row[0] / self.input_size[0]
                cy_norm = row[1] / self.input_size[1]
                w_norm = row[2] / self.input_size[0]
                h_norm = row[3] / self.input_size[1]

                x1 = int((cx_norm - w_norm / 2) * img_w)
                y1 = int((cy_norm - h_norm / 2) * img_h)
                x2 = int((cx_norm + w_norm / 2) * img_w)
                y2 = int((cy_norm + h_norm / 2) * img_h)

                boxes.append([x1, y1, x2 - x1, y2 - y1])
                confidences.append(conf)
                class_ids.append(class_id)

            # NMS
            if boxes:
                indices = cv2.dnn.NMSBoxes(
                    boxes, confidences,
                    self.conf_threshold, self.nms_threshold
                )
                if len(indices) > 0:
                    for i in indices.flatten():
                        x1, y1, w, h = boxes[i]
                        cx = x1 + w // 2
                        cy = y1 + h // 2
                        detections.append(LandmarkDetection(
                            cx=cx, cy=cy,
                            kind=self.CLASS_NAMES[class_ids[i]],
                            confidence=confidences[i],
                            bbox=(x1, y1, x1 + w, y1 + h)
                        ))

        return detections


class ColorVerifier:
    """
    Independent HSV color verification for cross-checking YOLO or
    circle detector color output. Used as a sanity check.
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


# Need math import at module level
import math
