#!/usr/bin/env python3
"""
Local test for HSV circle detection.

Runs the HSV circle detector on your webcam feed or a test image.
Press 'q' to quit, 's' to save a screenshot.

Usage:
    # Webcam (default)
    python scripts/task1/test_hsv_local.py

    # From an image file
    python scripts/task1/test_hsv_local.py --image path/to/photo.jpg

    # Adjust detection params
    python scripts/task1/test_hsv_local.py --min-radius 15 --max-radius 150 --param2 25
"""

import argparse
import sys
import os
import time

# Add project root to path and import the detector directly
# (avoid edge_core/__init__.py which pulls in pydantic/fastapi)
project_root = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, project_root)

import cv2
import numpy as np
import importlib.util

_mod_name = "edge_core.hsv_circle_detector"
_spec = importlib.util.spec_from_file_location(
    _mod_name,
    os.path.join(project_root, "edge_core", "hsv_circle_detector.py"),
)
_mod = importlib.util.module_from_spec(_spec)
sys.modules[_mod_name] = _mod  # required for dataclass on Python 3.13
_spec.loader.exec_module(_mod)
detect_circles_hsv = _mod.detect_circles_hsv
draw_detection_overlay = _mod.draw_detection_overlay


def run_webcam(args):
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"ERROR: Cannot open camera {args.camera}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("HSV Circle/Ellipse Detection - Local Test")
    print("Controls:")
    print("  q = quit, s = save screenshot")
    print("  c/C = circularity threshold -/+ (lower = accept more deformed)")
    print("  a/A = aspect ratio threshold -/+ (lower = accept more elongated)")
    print("  +/- = Hough param2 sensitivity")
    print(f"  circularity: {args.min_circularity:.2f}  aspect_ratio: {args.max_aspect_ratio:.2f}")

    param2 = args.param2
    circ_thresh = args.min_circularity
    aspect_thresh = args.max_aspect_ratio
    frame_count = 0
    fps_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        detections = detect_circles_hsv(
            frame,
            min_radius=args.min_radius,
            max_radius=args.max_radius,
            param1=args.param1,
            param2=param2,
            min_color_confidence=args.min_confidence,
            min_circularity=circ_thresh,
            min_aspect_ratio=aspect_thresh,
            min_contour_area=args.min_area,
            min_solidity=args.min_solidity,
        )

        annotated = draw_detection_overlay(frame.copy(), detections)

        # FPS counter
        frame_count += 1
        elapsed = time.time() - fps_time
        if elapsed > 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            fps_time = time.time()
        else:
            fps = frame_count / max(elapsed, 0.001)

        # Info bar at bottom
        info = (f"FPS:{fps:.0f} | circ>={circ_thresh:.2f} aspect>={aspect_thresh:.2f} "
                f"p2={param2} | {len(detections)} det")
        h, w = annotated.shape[:2]
        cv2.rectangle(annotated, (0, h - 30), (w, h), (0, 0, 0), -1)
        cv2.putText(annotated, info, (5, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow("HSV Circle/Ellipse Detection", annotated)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            fname = f"hsv_capture_{int(time.time())}.jpg"
            cv2.imwrite(fname, annotated)
            print(f"Saved: {fname}")
        elif key == ord('+') or key == ord('='):
            param2 = max(5, param2 - 5)
            print(f"param2 = {param2} (more sensitive)")
        elif key == ord('-'):
            param2 = min(100, param2 + 5)
            print(f"param2 = {param2} (less sensitive)")
        elif key == ord('c'):
            circ_thresh = max(0.1, circ_thresh - 0.05)
            print(f"min_circularity = {circ_thresh:.2f} (accept more deformed)")
        elif key == ord('C'):
            circ_thresh = min(0.95, circ_thresh + 0.05)
            print(f"min_circularity = {circ_thresh:.2f} (require rounder)")
        elif key == ord('a'):
            aspect_thresh = max(0.1, aspect_thresh - 0.05)
            print(f"max_aspect_ratio = {aspect_thresh:.2f} (accept more elongated)")
        elif key == ord('A'):
            aspect_thresh = min(0.9, aspect_thresh + 0.05)
            print(f"max_aspect_ratio = {aspect_thresh:.2f} (require rounder)")

    cap.release()
    cv2.destroyAllWindows()


def run_image(args):
    image = cv2.imread(args.image)
    if image is None:
        print(f"ERROR: Cannot read image: {args.image}")
        sys.exit(1)

    print(f"Processing: {args.image} ({image.shape[1]}x{image.shape[0]})")

    detections = detect_circles_hsv(
        image,
        min_radius=args.min_radius,
        max_radius=args.max_radius,
        param1=args.param1,
        param2=args.param2,
        min_color_confidence=args.min_confidence,
        min_circularity=args.min_circularity,
        min_aspect_ratio=args.max_aspect_ratio,
        min_contour_area=args.min_area,
        min_solidity=args.min_solidity,
    )

    print(f"Detected {len(detections)} targets:")
    for i, d in enumerate(detections):
        letter = chr(ord('A') + i)
        print(f"  Target {letter}: {d.color} at ({d.cx},{d.cy}) r={d.radius} "
              f"conf={d.confidence:.0%} ar={d.aspect_ratio:.2f} circ={d.circularity:.2f} "
              f"[{d.detection_method}]")

    annotated = draw_detection_overlay(image.copy(), detections)

    cv2.imshow("HSV Circle Detection", annotated)
    print("\nPress any key to close, 's' to save")
    key = cv2.waitKey(0) & 0xFF
    if key == ord('s'):
        fname = f"hsv_result_{int(time.time())}.jpg"
        cv2.imwrite(fname, annotated)
        print(f"Saved: {fname}")
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Test HSV circle detection locally")
    parser.add_argument('--image', help='Path to image file (default: use webcam)')
    parser.add_argument('--camera', type=int, default=0, help='Camera index (default: 0)')
    parser.add_argument('--min-radius', type=int, default=10, help='Min circle radius (px)')
    parser.add_argument('--max-radius', type=int, default=200, help='Max circle radius (px)')
    parser.add_argument('--param1', type=float, default=100, help='Canny edge threshold')
    parser.add_argument('--param2', type=float, default=30, help='Accumulator threshold (lower=more detections)')
    parser.add_argument('--min-confidence', type=float, default=0.50, help='Min color confidence (0-1)')
    parser.add_argument('--min-circularity', type=float, default=0.60,
                        help='Min circularity (0-1). Lower accepts more deformed. Default 0.60')
    parser.add_argument('--max-aspect-ratio', type=float, default=0.35,
                        help='Min ellipse minor/major ratio. Lower accepts more elongated. Default 0.35')
    parser.add_argument('--min-area', type=int, default=500,
                        help='Minimum contour area in px² (default 500)')
    parser.add_argument('--min-solidity', type=float, default=0.80,
                        help='Min solidity (area/hull). Higher = reject irregular shapes. Default 0.80')
    args = parser.parse_args()

    if args.image:
        run_image(args)
    else:
        run_webcam(args)


if __name__ == '__main__':
    main()
