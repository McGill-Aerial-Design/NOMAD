#!/usr/bin/env python3
"""
Ground-station Task 2 circle detector.

Runs the heavier OpenCV circle detector on the laptop/ground station, then
posts timestamped image-space detections back to Edge Core. This keeps Jetson
motion/spray ownership onboard while moving expensive image processing off the
Orin Nano.

Example:
  python scripts/dev/groundstation_task2_detector.py --jetson 100.85.121.98
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DETECTOR_PATH = REPO_ROOT / "edge_core" / "task2_circle_verify.py"
spec = importlib.util.spec_from_file_location("task2_circle_verify", DETECTOR_PATH)
if spec is None or spec.loader is None:
    raise RuntimeError(f"Cannot load detector module: {DETECTOR_PATH}")
detector_module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = detector_module
spec.loader.exec_module(detector_module)
Task2CircleDetector = detector_module.Task2CircleDetector


def _fetch_jpeg(url: str, timeout_s: float) -> bytes | None:
    try:
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            if resp.status != 200:
                return None
            return resp.read()
    except (urllib.error.URLError, TimeoutError):
        return None


def _post_json(
    url: str,
    payload: dict,
    timeout_s: float,
    api_key: str = "",
    internal_token: str = "",
) -> bool:
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=body,
        method="POST",
        headers={
            "Content-Type": "application/json",
            "Accept": "application/json",
        },
    )
    if api_key:
        req.add_header("X-API-Key", api_key)
    if internal_token:
        req.add_header("X-NOMAD-Internal-Token", internal_token)
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            return 200 <= resp.status < 300
    except (urllib.error.URLError, TimeoutError):
        return False


def _decode_jpeg(payload: bytes) -> np.ndarray | None:
    arr = np.frombuffer(payload, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


def _circle_to_detection(circle, frame_w: int, frame_h: int, idx: int) -> dict:
    r = float(circle.radius)
    x = max(0.0, float(circle.cx) - r)
    y = max(0.0, float(circle.cy) - r)
    w = min(float(frame_w) - x, r * 2.0)
    h = min(float(frame_h) - y, r * 2.0)
    return {
        "target_id": idx,
        "label": "circle",
        "confidence": float(circle.confidence * 100.0),
        "source": "groundstation_task2",
        "image_only": True,
        "bbox_x": x,
        "bbox_y": y,
        "bbox_w": w,
        "bbox_h": h,
        "cx": float(circle.cx),
        "cy": float(circle.cy),
        "pixel_x": float(circle.cx),
        "pixel_y": float(circle.cy),
        "radius_px": r,
        "method": circle.method,
        "src_w": frame_w,
        "src_h": frame_h,
        "timestamp": time.time(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--jetson", default="100.85.121.98")
    parser.add_argument("--edge-port", type=int, default=8000)
    parser.add_argument("--bridge-port", type=int, default=9200)
    parser.add_argument("--fps", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--api-key", default=os.environ.get("NOMAD_API_KEY", ""))
    parser.add_argument(
        "--internal-token", default=os.environ.get("NOMAD_INTERNAL_TOKEN", "")
    )
    parser.add_argument("--snapshot-path", default="/snapshot_raw")
    parser.add_argument("--min-radius", type=int, default=5)
    parser.add_argument("--max-radius", type=int, default=260)
    parser.add_argument("--hough-param2", type=int, default=28)
    parser.add_argument("--blur-kernel", type=int, default=3)
    parser.add_argument("--min-confidence", type=float, default=45.0)
    parser.add_argument("--max-detections", type=int, default=6)
    parser.add_argument("--count", type=int, default=0, help="Stop after N posts; 0 runs forever.")
    args = parser.parse_args()

    snapshot_url = f"http://{args.jetson}:{args.bridge_port}{args.snapshot_path}"
    update_url = f"http://{args.jetson}:{args.edge_port}/api/detections/update"
    detector = Task2CircleDetector(
        min_radius_px=args.min_radius,
        max_radius_px=args.max_radius,
        hough_param2=args.hough_param2,
        blur_kernel=max(3, args.blur_kernel | 1),
        min_circularity=0.55,
        min_solidity=0.65,
    )

    period = 1.0 / max(args.fps, 0.1)
    print(f"Ground-station Task 2 detector")
    print(f"  snapshot: {snapshot_url}")
    print(f"  update:   {update_url}")
    print(f"  rate:     {args.fps:.1f} Hz")

    last_hash = None
    post_count = 0
    while True:
        loop_start = time.time()
        payload = _fetch_jpeg(snapshot_url, args.timeout)
        detections = []
        if payload:
            payload_hash = hash(payload)
            if payload_hash != last_hash:
                last_hash = payload_hash
                frame = _decode_jpeg(payload)
                if frame is not None:
                    h, w = frame.shape[:2]
                    circles = detector.detect(frame)
                    circles.sort(key=lambda c: c.confidence * c.radius, reverse=True)
                    for circle in circles[:args.max_detections]:
                        det = _circle_to_detection(circle, w, h, len(detections))
                        if det["confidence"] >= args.min_confidence:
                            detections.append(det)

        payload_out = {
            "source": "groundstation_task2",
            "source_timestamp": time.time(),
            "detections": detections,
        }
        ok = _post_json(
            update_url,
            payload_out,
            args.timeout,
            api_key=args.api_key,
            internal_token=args.internal_token,
        )
        print(
            f"{time.strftime('%H:%M:%S')} posted={ok} detections={len(detections)}",
            flush=True,
        )
        post_count += 1
        if args.count > 0 and post_count >= args.count:
            break
        sleep_s = period - (time.time() - loop_start)
        if sleep_s > 0:
            time.sleep(sleep_s)


if __name__ == "__main__":
    raise SystemExit(main())
