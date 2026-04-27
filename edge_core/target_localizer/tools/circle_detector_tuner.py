#!/usr/bin/env python3
"""
circle_detector_tuner.py

End-to-end tuner for the Task 1 CircleDetector. Unlike hsv_tuner.py (which
tunes a single HSV range), this runs the full detector pipeline — HSV masks,
contour filtering, circularity + solidity — on a sample image and visualizes
which circles are accepted vs rejected.

Usage:
    python3 circle_detector_tuner.py --image path/to/capture.jpg
    python3 circle_detector_tuner.py --folder path/to/Task1_folder
    python3 circle_detector_tuner.py --camera 0            # live webcam

Keys while the window is open:
    n / right-arrow   Next image (folder mode)
    p / left-arrow    Previous image (folder mode)
    s                 Print current params in ROS-set command form
    q / ESC           Quit

Trackbars mirror the ROS2 parameters on the Jetson:
    circle.min_radius_px
    circle.max_radius_px
    circle.min_circularity   (stored ×100)
    circle.min_solidity      (stored ×100)
    circle.blur_kernel       (odd values only)
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np

# Make the sibling target_localizer package importable when run from a checkout.
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from target_localizer.detectors import CircleDetector, HSV_RANGES, TargetColor  # noqa: E402


def _color_bgr_for(target_color: TargetColor) -> tuple[int, int, int]:
    return {
        TargetColor.RED: (0, 0, 255),
        TargetColor.YELLOW: (0, 255, 255),
        TargetColor.GREEN: (0, 200, 0),
        TargetColor.BLUE: (255, 0, 0),
        TargetColor.BLACK: (40, 40, 40),
        TargetColor.WHITE: (230, 230, 230),
    }.get(target_color, (0, 255, 0))


def _ensure_odd(n: int) -> int:
    return max(1, n if n % 2 == 1 else n + 1)


def _build_detector(min_r, max_r, circ_100, solid_100, blur_k) -> CircleDetector:
    return CircleDetector(
        min_radius_px=max(1, int(min_r)),
        max_radius_px=max(int(min_r) + 1, int(max_r)),
        min_circularity=max(0.01, float(circ_100) / 100.0),
        min_solidity=max(0.01, float(solid_100) / 100.0),
        blur_kernel=_ensure_odd(int(blur_k)),
    )


def _annotate(frame: np.ndarray, detector: CircleDetector) -> tuple[np.ndarray, list]:
    annotated = frame.copy()
    dets = detector.detect(frame)
    for det in dets:
        color = _color_bgr_for(det.color)
        x1, y1, x2, y2 = det.bbox
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
        cv2.circle(annotated, (det.cx, det.cy), det.radius, color, 1)
        label = f"{det.color.value} r={det.radius} c={det.confidence:.2f}"
        cv2.putText(
            annotated, label, (x1, max(0, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA,
        )
    return annotated, dets


def _collect_images(image: str | None, folder: str | None) -> list[Path]:
    if image:
        return [Path(image)]
    if folder:
        p = Path(folder)
        exts = {".jpg", ".jpeg", ".png", ".bmp"}
        return sorted(f for f in p.iterdir() if f.suffix.lower() in exts)
    return []


def main():
    parser = argparse.ArgumentParser(description="Task 1 CircleDetector tuner")
    parser.add_argument("--image", help="Single image path")
    parser.add_argument("--folder", help="Folder of images to scroll with n/p")
    parser.add_argument("--camera", type=int, help="Live webcam index")
    parser.add_argument("--max-width", type=int, default=1280,
                        help="Down-scale wider frames for display")
    args = parser.parse_args()

    images = _collect_images(args.image, args.folder)
    use_camera = args.camera is not None
    cap = cv2.VideoCapture(args.camera) if use_camera else None
    if use_camera and (cap is None or not cap.isOpened()):
        print(f"Could not open camera {args.camera}")
        return
    if not use_camera and not images:
        parser.print_help()
        return

    cv2.namedWindow("tuner", cv2.WINDOW_NORMAL)

    # Defaults mirror the current node defaults.
    cv2.createTrackbar("min_radius_px", "tuner", 12, 200, lambda _: None)
    cv2.createTrackbar("max_radius_px", "tuner", 400, 800, lambda _: None)
    cv2.createTrackbar("min_circ x100", "tuner", 70, 100, lambda _: None)
    cv2.createTrackbar("min_solid x100", "tuner", 80, 100, lambda _: None)
    cv2.createTrackbar("blur_kernel (odd)", "tuner", 7, 31, lambda _: None)

    idx = 0
    frozen_frame: np.ndarray | None = None

    while True:
        if use_camera:
            ok, frame = cap.read()
            if not ok:
                print("Camera read failed")
                break
        else:
            path = images[idx]
            frame = cv2.imread(str(path))
            if frame is None:
                print(f"Could not read {path}; skipping")
                idx = (idx + 1) % len(images)
                continue
            frozen_frame = frame

        # Optional down-scale for display.
        h, w = frame.shape[:2]
        if w > args.max_width:
            scale = args.max_width / w
            frame = cv2.resize(frame, (int(w * scale), int(h * scale)))

        detector = _build_detector(
            cv2.getTrackbarPos("min_radius_px", "tuner"),
            cv2.getTrackbarPos("max_radius_px", "tuner"),
            cv2.getTrackbarPos("min_circ x100", "tuner"),
            cv2.getTrackbarPos("min_solid x100", "tuner"),
            cv2.getTrackbarPos("blur_kernel (odd)", "tuner"),
        )
        annotated, dets = _annotate(frame, detector)

        header = f"{len(dets)} circle(s)"
        if not use_camera:
            header = f"[{idx + 1}/{len(images)}] {images[idx].name}  |  {header}"
        cv2.putText(annotated, header, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow("tuner", annotated)
        key = cv2.waitKey(30 if use_camera else 50) & 0xFF
        if key in (ord("q"), 27):
            break
        if key in (ord("n"), 83) and images:  # 83 = right arrow on many OS
            idx = (idx + 1) % len(images)
        if key in (ord("p"), 81) and images:  # 81 = left arrow
            idx = (idx - 1) % len(images)
        if key == ord("s"):
            min_r = cv2.getTrackbarPos("min_radius_px", "tuner")
            max_r = cv2.getTrackbarPos("max_radius_px", "tuner")
            circ = cv2.getTrackbarPos("min_circ x100", "tuner") / 100.0
            solid = cv2.getTrackbarPos("min_solid x100", "tuner") / 100.0
            blur = _ensure_odd(cv2.getTrackbarPos("blur_kernel (odd)", "tuner"))
            print("\n# Apply these live on the Jetson:")
            print(
                f'docker exec -it nomad_isaac_ros bash -lc "source /opt/ros/humble/setup.bash && \\\n'
                f'  ros2 param set /target_localizer circle.min_radius_px {min_r} && \\\n'
                f'  ros2 param set /target_localizer circle.max_radius_px {max_r} && \\\n'
                f'  ros2 param set /target_localizer circle.min_circularity {circ} && \\\n'
                f'  ros2 param set /target_localizer circle.min_solidity {solid} && \\\n'
                f'  ros2 param set /target_localizer circle.blur_kernel {blur}"'
            )
            for d in dets:
                print(
                    f"  - {d.color.value:6s}  bbox={d.bbox}  r={d.radius}  "
                    f"circ/sol={d.confidence:.2f}"
                )

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
