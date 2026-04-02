#!/usr/bin/env python3
"""
hsv_tuner.py

Interactive HSV range tuner for calibrating circle color detection.
Run this on a laptop with a sample image of the actual competition
targets to dial in the HSV ranges before flight.

Usage:
    python3 hsv_tuner.py --image target_sample.jpg
    python3 hsv_tuner.py --camera 0  # live webcam

Controls:
    - Adjust trackbars to set H, S, V lower and upper bounds
    - The mask preview shows which pixels fall within the range
    - Press 'p' to print current HSV range to terminal
    - Press 'q' to quit
    - Press 's' to save current range to a file
"""

import cv2
import numpy as np
import argparse
import json


def nothing(x):
    pass


def main():
    parser = argparse.ArgumentParser(description='HSV Range Tuner')
    parser.add_argument('--image', type=str, help='Path to image file')
    parser.add_argument('--camera', type=int, default=None, help='Camera index')
    args = parser.parse_args()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"Error: Could not read {args.image}")
            return
        use_camera = False
    elif args.camera is not None:
        cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            print(f"Error: Could not open camera {args.camera}")
            return
        use_camera = True
    else:
        print("Provide --image or --camera")
        return

    cv2.namedWindow('HSV Tuner', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Result', cv2.WINDOW_NORMAL)

    # Create trackbars
    cv2.createTrackbar('H Low', 'HSV Tuner', 0, 180, nothing)
    cv2.createTrackbar('H High', 'HSV Tuner', 180, 180, nothing)
    cv2.createTrackbar('S Low', 'HSV Tuner', 50, 255, nothing)
    cv2.createTrackbar('S High', 'HSV Tuner', 255, 255, nothing)
    cv2.createTrackbar('V Low', 'HSV Tuner', 50, 255, nothing)
    cv2.createTrackbar('V High', 'HSV Tuner', 255, 255, nothing)

    saved_ranges = {}

    while True:
        if use_camera:
            ret, frame = cap.read()
            if not ret:
                break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h_low = cv2.getTrackbarPos('H Low', 'HSV Tuner')
        h_high = cv2.getTrackbarPos('H High', 'HSV Tuner')
        s_low = cv2.getTrackbarPos('S Low', 'HSV Tuner')
        s_high = cv2.getTrackbarPos('S High', 'HSV Tuner')
        v_low = cv2.getTrackbarPos('V Low', 'HSV Tuner')
        v_high = cv2.getTrackbarPos('V High', 'HSV Tuner')

        lower = np.array([h_low, s_low, v_low])
        upper = np.array([h_high, s_high, v_high])

        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Show pixel count
        pixel_count = cv2.countNonZero(mask)
        total = mask.shape[0] * mask.shape[1]
        pct = pixel_count / total * 100

        info = frame.copy()
        cv2.putText(info, f"Lower: [{h_low}, {s_low}, {v_low}]", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(info, f"Upper: [{h_high}, {s_high}, {v_high}]", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(info, f"Pixels: {pixel_count} ({pct:.1f}%)", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('HSV Tuner', info)
        cv2.imshow('Mask', mask)
        cv2.imshow('Result', result)

        key = cv2.waitKey(30 if use_camera else 0) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('p'):
            print(f"\nHSV Range:")
            print(f"  Lower: np.array([{h_low}, {s_low}, {v_low}])")
            print(f"  Upper: np.array([{h_high}, {s_high}, {v_high}])")
        elif key == ord('s'):
            name = input("Color name (e.g., red, blue): ").strip()
            if name:
                saved_ranges[name] = {
                    'lower': [int(h_low), int(s_low), int(v_low)],
                    'upper': [int(h_high), int(s_high), int(v_high)]
                }
                with open('hsv_ranges.json', 'w') as f:
                    json.dump(saved_ranges, f, indent=2)
                print(f"Saved '{name}' range to hsv_ranges.json")
        elif key == ord(' ') and not use_camera:
            # Spacebar to continue in image mode
            continue

    if use_camera:
        cap.release()
    cv2.destroyAllWindows()

    if saved_ranges:
        print("\n=== All saved ranges ===")
        print(json.dumps(saved_ranges, indent=2))
        print("\nCopy these into detectors.py HSV_RANGES dict.")


if __name__ == '__main__':
    main()
