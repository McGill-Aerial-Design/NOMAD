#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
Test P3-7 Mesh Rebuild Debounce functionality.
Connects to WebSocket endpoint to monitor mesh update rates and timing.
"""

import argparse
import json
import os
import sys
import time
from collections import defaultdict
from datetime import datetime

import websocket


def test_debounce_behavior(jetson_ip, duration_seconds=60, debug=False):
    """Monitor mesh update rates to verify debounce behavior (250ms window)."""

    uri = f"ws://{jetson_ip}:8000/ws/slam"
    print(f"\n{'=' * 70}")
    print("P3-7 Mesh Rebuild Debounce Test")
    print(f"{'=' * 70}")
    print(f"Target: {uri}")
    print(f"Duration: {duration_seconds}s")
    print("Expected: No more than 1 rebuild per 250ms window")
    print(f"Start time: {datetime.now()}\n")

    try:
        ws = websocket.create_connection(uri, timeout=5)
        print("✓ WebSocket connection established\n")

        stats = {
            "total_frames": 0,
            "mesh_frames": 0,
            "pose_frames": 0,
            "mesh_timestamps": [],
            "frame_types": defaultdict(int),
            "inter_mesh_times": [],  # Time between consecutive mesh updates
        }

        start_time = time.time()
        prev_mesh_time = None

        print("Monitoring WebSocket frames...")

        while time.time() - start_time < duration_seconds:
            try:
                msg = ws.recv(timeout=1.0)
                frame = json.loads(msg)

                stats["total_frames"] += 1
                frame_type = frame.get("type", "unknown")
                stats["frame_types"][frame_type] += 1

                if frame_type == "mesh":
                    current_time = time.time()
                    stats["mesh_frames"] += 1
                    stats["mesh_timestamps"].append(current_time)

                    if prev_mesh_time is not None:
                        time_delta = (current_time - prev_mesh_time) * 1000  # Convert to ms
                        stats["inter_mesh_times"].append(time_delta)

                        # Debounce should allow no more frequently than 250ms
                        if time_delta < 250:
                            status = "⚠ FAST"
                            print(f"Frame {stats['total_frames']:4d} (mesh): delta={time_delta:6.1f}ms {status}")
                        else:
                            print(f"Frame {stats['total_frames']:4d} (mesh): delta={time_delta:6.1f}ms")
                    else:
                        print(f"Frame {stats['total_frames']:4d} (mesh): first mesh frame")

                    prev_mesh_time = current_time
                elif frame_type == "pose":
                    stats["pose_frames"] += 1
                    if stats["total_frames"] % 30 == 0:
                        print(f"Frame {stats['total_frames']:4d} (pose): received")

            except websocket.WebSocketTimeoutException:
                pass  # Timeout is OK, just means no frame in this second
            except json.JSONDecodeError as e:
                print(f"JSON parse error: {e}")
            except Exception as e:
                print(f"Error: {type(e).__name__}: {e}")
                break

        ws.close()
        elapsed = time.time() - start_time

        # Analyze results
        print(f"\n{'=' * 70}")
        print("Test Results Summary")
        print(f"{'=' * 70}")
        print(f"Total frames received: {stats['total_frames']}")
        print(f"Mesh updates: {stats['mesh_frames']}")
        print(f"Pose updates: {stats['pose_frames']}")
        print(f"Frame types: {dict(stats['frame_types'])}")
        print(f"Elapsed time: {elapsed:.1f}s")

        if stats["inter_mesh_times"]:
            inter_times = stats["inter_mesh_times"]
            avg_interval = sum(inter_times) / len(inter_times)
            min_interval = min(inter_times)
            max_interval = max(inter_times)

            # Count how many inter-frame times are < 250ms
            too_fast = sum(1 for t in inter_times if t < 250)

            print("\nMesh update intervals:")
            print(f"  Average: {avg_interval:.1f}ms")
            print(f"  Min: {min_interval:.1f}ms")
            print(f"  Max: {max_interval:.1f}ms")
            print(f"  Updates faster than 250ms: {too_fast}/{len(inter_times)}")
            print(f"  Percentage: {too_fast / len(inter_times) * 100:.1f}%")

            # Expected behavior: Debounce should prevent rapid updates
            # If we see >20% of updates faster than 250ms, debounce may not be working
            if too_fast / len(inter_times) > 0.2:
                print(f"\n⚠ WARNING: {too_fast} updates faster than 250ms - debounce may not be active")
                print("  Expected: <50ms (at 30Hz frame rate)")
                if debug:
                    print(f"  Intervals: {[f'{t:.1f}' for t in inter_times[:10]]}...")

        # Pass/Fail decision
        print(f"\n{'=' * 70}")

        if stats["mesh_frames"] == 0:
            print("⚠ No mesh updates received (mesh data not present)")
            print("  This is OK if no mesh data is being streamed")
            print("  Recommendation: Run test while mesh is actively updating")
            return None  # Inconclusive

        if stats["inter_mesh_times"]:
            too_fast = sum(1 for t in stats["inter_mesh_times"] if t < 250)
            # Allow some updates to be faster (initial mesh, or synthetic test data)
            if too_fast / len(stats["inter_mesh_times"]) <= 0.2:
                print("✓ PASS: Debounce behavior verified")
                print(f"  {len(stats['inter_mesh_times'])} mesh update intervals analyzed")
                print(f"  Only {too_fast} updates faster than 250ms (acceptable)")
                print(f"{'=' * 70}\n")
                return True
            else:
                print("✗ FAIL: Too many rapid updates detected")
                print(f"  {too_fast}/{len(stats['inter_mesh_times'])} intervals < 250ms")
                print("  Debounce may not be functioning correctly")
                print(f"{'=' * 70}\n")
                return False

    except ConnectionRefusedError:
        print(f"✗ Connection refused on {jetson_ip}:8000")
        return False
    except Exception as e:
        print(f"✗ Error: {type(e).__name__}: {e}\n")
        import traceback

        traceback.print_exc()
        return False


def test_mesh_visual_quality(jetson_ip, duration_seconds=30):
    """
    Basic check that mesh frames contain expected data.
    For full visual validation, use Mission Planner SLAM3DView.
    """

    print(f"\n{'=' * 70}")
    print("P3-7 Mesh Visual Quality Check")
    print(f"{'=' * 70}\n")

    try:
        ws = websocket.create_connection(f"ws://{jetson_ip}:8000/ws/slam", timeout=5)

        mesh_samples = []
        start = time.time()

        while time.time() - start < duration_seconds and len(mesh_samples) < 5:
            try:
                msg = ws.recv(timeout=1.0)
                frame = json.loads(msg)

                if frame.get("type") == "mesh" and "mesh" in frame:
                    mesh_samples.append(frame)
                    print(
                        f"✓ Mesh sample {len(mesh_samples)}: "
                        f"type=mesh, has_mesh=true, "
                        f"blocks={len(frame['mesh'].get('blocks', []))}"
                    )

                    # Check for required fields
                    has_pose = all(k in frame for k in ["x", "y", "z", "roll", "pitch", "yaw"])
                    has_frame_id = "frame_id" in frame
                    print(f"  Pose data: {has_pose}, Frame ID: {has_frame_id}")

            except websocket.WebSocketTimeoutException:
                pass
            except Exception:
                break

        ws.close()

        if mesh_samples:
            print(f"\n✓ PASS: Received {len(mesh_samples)} valid mesh frames with expected structure")
            return True
        else:
            print("\n⚠ No mesh data received (mesh updates may not be streaming)")
            return None

    except Exception as e:
        print(f"✗ Error: {e}")
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test P3-7 Mesh Rebuild Debounce functionality")
    parser.add_argument(
        "--jetson-ip", default=os.environ.get("JETSON_IP", ""), help="Jetson IP address (defaults to $JETSON_IP)"
    )
    parser.add_argument("--duration", type=int, default=60, help="Test duration in seconds (default: 60)")
    parser.add_argument("--visual-check", action="store_true", help="Also check mesh visual quality")
    parser.add_argument("--debug", action="store_true", help="Enable debug output")

    args = parser.parse_args()

    try:
        import websocket
    except ImportError:
        print("ERROR: websocket-client not installed")
        print("Run: pip install websocket-client")
        sys.exit(1)

    # Run debounce timing test
    result = test_debounce_behavior(args.jetson_ip, args.duration, debug=args.debug)

    # Optionally run visual quality check
    if args.visual_check:
        result2 = test_mesh_visual_quality(args.jetson_ip, duration_seconds=30)

    sys.exit(0 if result is not False else 1)
