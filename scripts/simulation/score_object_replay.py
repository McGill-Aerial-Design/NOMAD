# SPDX-License-Identifier: Apache-2.0
"""Score sampled Object RGB replay after inference; labels never enter the tracker."""

import argparse
import hashlib
import json
import math
import time
import uuid
from pathlib import Path, PureWindowsPath


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def owned(path):
    path = Path(path).resolve(strict=True)
    marker = path.name.removeprefix("run-")
    if path.name != "run-" + uuid.UUID(hex=marker).hex or (path / "run.id").read_text().strip() != marker:
        raise ValueError("Owned run marker mismatch")
    return path


def overlap(a, b):
    x, y, w, h = a
    u, v, s, t = b
    intersection = max(0, min(x + w, u + s) - max(x, u)) * max(0, min(y + h, v + t) - max(y, v))
    return intersection / (w * h + s * t - intersection)


def canonical_fingerprint(path, geometry):
    import cv2
    import numpy as np

    rgb = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if rgb is None or rgb.shape[:2] != (geometry["height"], geometry["width"]):
        raise ValueError("Decoded RGB shape mismatch")
    yy = (np.arange(48) * 10 + 5) * rgb.shape[0] // 480
    xx = (np.arange(64) * 10 + 5) * rgb.shape[1] // 640
    pixels = rgb[yy[:, None], xx].astype(np.int32)
    gray = ((77 * pixels[:, :, 2] + 150 * pixels[:, :, 1] + 29 * pixels[:, :, 0]) >> 8).astype(np.uint8)
    return gray.tobytes().hex()


def measure(image, bounds, aim, found):
    import cv2
    import numpy as np

    if image.ndim != 2 or image.dtype != np.uint8:
        raise ValueError("Expected uint8 semantic labels")
    selected = (image == 10) | (image == 40)
    yy, xx = np.nonzero(selected)
    truth = [int(xx.min()), int(yy.min()), int(np.ptp(xx)) + 1, int(np.ptp(yy)) + 1] if len(xx) else None
    result = dict(
        visible=len(xx) >= 8,
        foreground=False,
        bbox_iou=0.0,
        aim_label=None,
        distractor_iou=0.0,
        pixel_error=None,
        target_centre=[truth[0] + truth[2] / 2, truth[1] + truth[3] / 2] if truth else None,
        centre_error_x=None,
        centre_error_y=None,
        wrong_instance=False,
    )
    if not found:
        return result
    if (
        bounds is None
        or len(bounds) != 4
        or aim is None
        or len(aim) != 2
        or not all(math.isfinite(v) for v in bounds + aim)
        or bounds[2] <= 0
        or bounds[3] <= 0
    ):
        raise ValueError("Verified result lacks finite bounds/aim")
    if not (0 <= aim[0] < image.shape[1] and 0 <= aim[1] < image.shape[0]):
        raise ValueError("Verified aim outside image")
    x, y = round(aim[0]), round(aim[1])
    if 0 <= y < image.shape[0] and 0 <= x < image.shape[1]:
        result["aim_label"] = int(image[y, x])
        result["foreground"] = bool(cv2.dilate(selected.astype(np.uint8), np.ones((3, 3), np.uint8))[y, x])
        if truth:
            result["pixel_error"] = float(
                cv2.distanceTransform((~selected).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[y, x]
            )
    if truth:
        result["bbox_iou"] = overlap(bounds, truth)
    for label in (20, 30):
        yy, xx = np.nonzero(image == label)
        if len(xx):
            other = [int(xx.min()), int(yy.min()), int(np.ptp(xx)) + 1, int(np.ptp(yy)) + 1]
            result["distractor_iou"] = max(result["distractor_iou"], overlap(bounds, other))
    result["wrong_instance"] = (
        not result["visible"]
        or result["bbox_iou"] == 0
        or (result["distractor_iou"] > 0 and result["distractor_iou"] >= result["bbox_iou"])
    )
    if truth:
        result["centre_error_x"] = abs(aim[0] - result["target_centre"][0])
        result["centre_error_y"] = abs(aim[1] - result["target_centre"][1])
    return result


def score(dataset, replay):
    import cv2
    import numpy as np

    began = time.monotonic()
    cv2.setNumThreads(2)
    dataset, replay = owned(dataset), owned(replay)
    options = json.loads((replay / "replay.json").read_text(encoding="utf-8-sig"))
    named = PureWindowsPath(options["dataset"]).name
    if named != dataset.name or any(
        options[k] != v for k, v in (("folder", "decoded"), ("first", 0), ("mode", "Object"))
    ):
        raise ValueError("Scoring requires the full declared Object decoded replay")
    rows = [json.loads(line) for line in (replay / "replay-frames.jsonl").read_text().splitlines()]
    files = sorted((dataset / "decoded").glob("*.png"))
    if not 2 <= len(files) <= 2000:
        raise ValueError("Replay requires 2..2000 saved RGB frames")
    if [r["frame"] for r in rows] != [p.name for p in files[1:]]:
        raise ValueError("Replay omitted or reordered saved RGB frames")
    original = {r["sequence"]: r for r in map(json.loads, (dataset / "video-frames.jsonl").read_text().splitlines())}
    scene = json.loads((dataset / "world-manifest.json").read_text())
    whole_contract = scene["object_motion"].get("target_contract") == "whole_object_bounds"
    geometry = scene["camera"]
    metadata = sorted((dataset / "frames").glob("*.json"))
    if not 1 <= len(metadata) <= 10000:
        raise ValueError("Capture metadata count exceeds scorer budget")
    captures = [json.loads(p.read_text()) for p in metadata]
    captures = [c for c in captures if abs(c["pose_delta_ns"]) <= 2_000_000]
    fingerprints = np.asarray([list(bytes.fromhex(c["fingerprint"])) for c in captures], dtype=np.int32)
    if fingerprints.shape != (len(captures), 3072):
        raise ValueError("Invalid capture fingerprints")
    previous = original[int(files[0].stem.split("_")[-1])]["decoded_received_ticks"]
    evaluated, bindings = [], {str(p.relative_to(dataset)): sha(p) for p in metadata}
    if canonical_fingerprint(files[0], geometry) != original[int(files[0].stem.split("_")[-1])]["fingerprint"]:
        raise ValueError("Template RGB does not match original receipt fingerprint")
    unavailable = 0
    for row, path in zip(rows, files[1:]):
        if time.monotonic() - began >= 90:
            raise TimeoutError("Object replay scoring exceeded its 90-second deadline")
        source = original[int(path.stem.split("_")[-1])]
        received, frequency = source["decoded_received_ticks"], source["frequency"]
        gap = received <= previous or (received - previous) / frequency >= 0.5
        previous = received
        if (
            row["sha256"] != sha(path)
            or row["source_received_ticks"] != received
            or row["frequency"] != frequency
            or row["discontinuous"] != gap
        ):
            raise ValueError("RGB or original receipt/gap binding changed")
        if canonical_fingerprint(path, geometry) != source["fingerprint"]:
            raise ValueError("Saved RGB does not match its original receipt fingerprint")
        if source["stage"] != 40:
            continue
        fingerprint = np.frombuffer(bytes.fromhex(source["fingerprint"]), dtype=np.uint8)
        errors = np.mean((fingerprints - fingerprint) ** 2, axis=1)
        best = min(errors)
        if best > 100:
            unavailable += 1
            continue
        plausible = [c for c, error in zip(captures, errors) if error <= best + 1]
        plausible = [
            c
            for c in plausible
            if 0 <= c.get("motion_elapsed_sim_ns", -1) <= scene["object_motion"]["duration_sim_seconds"] * 1e9
        ]
        if not plausible:
            continue
        measures = []
        for capture in plausible:
            if abs(capture["mask_delta_ns"]) > 2_000_000 or (capture["width"], capture["height"]) != (
                geometry["width"],
                geometry["height"],
            ):
                raise ValueError("Mask clock or geometry mismatch")
            if abs(capture["hfov_radians"] - geometry["hfov_radians"]) > 1e-9:
                raise ValueError("Capture HFOV changed")
            mask_path = (dataset / "frames" / capture["mask"]).resolve(strict=True)
            mask_path.relative_to((dataset / "frames").resolve())
            image = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
            if image is None or image.shape != (geometry["height"], geometry["width"]):
                raise ValueError("Mask shape mismatch")
            bindings[str(mask_path.relative_to(dataset))] = sha(mask_path)
            found = row["state"] == "Verified"
            measures.append(measure(image, row["bounds"], [row["aim_x"], row["aim_y"]] if found else None, found))
        if any(m["visible"] for m in measures) != all(m["visible"] for m in measures):
            unavailable += 1
            continue
        evaluated.append(
            dict(
                frame=path.name,
                verified=found,
                foreground=all(m["foreground"] for m in measures),
                bbox_iou=min(m["bbox_iou"] for m in measures),
                whole_centre=found
                and all(
                    m["visible"]
                    and not m["wrong_instance"]
                    and m["bbox_iou"] >= 0.5
                    and m["centre_error_x"] <= geometry["width"] * 0.02
                    and m["centre_error_y"] <= geometry["height"] * 0.02
                    for m in measures
                ),
                wrong_instance=found and any(m["wrong_instance"] for m in measures),
                aim_labels=sorted({m["aim_label"] for m in measures if m["aim_label"] is not None}),
                distractor_iou=max(m["distractor_iou"] for m in measures),
                pixel_error=max((m["pixel_error"] for m in measures if m["pixel_error"] is not None), default=None),
                source_received_ticks=received,
                frequency=frequency,
                discontinuous=gap,
                simulation_capture_ns=[c["capture_sim_ns"] for c in plausible],
            )
        )
    total = len(evaluated)
    correct = sum(r["foreground"] for r in evaluated)
    wrong = sum(r["verified"] and not r["foreground"] for r in evaluated)
    boxes = sum(r["verified"] and r["bbox_iou"] >= 0.5 for r in evaluated)
    foreground = total >= 40 and unavailable == 0 and correct >= 0.95 * total and wrong == 0
    bbox = total >= 40 and unavailable == 0 and boxes >= 0.95 * total
    whole_correct = sum(r["whole_centre"] for r in evaluated)
    wrong_instances = sum(r["wrong_instance"] for r in evaluated)
    whole = total >= 40 and unavailable == 0 and whole_correct >= 0.95 * total and wrong_instances == 0
    errors = [r["pixel_error"] for r in evaluated if r["verified"] and r["pixel_error"] is not None]
    error_p95 = float(np.quantile(errors, 0.95)) if errors else None
    focal = geometry["width"] / (2 * math.tan(geometry["hfov_radians"] / 2))
    result = dict(
        passed=(whole if whole_contract else foreground) and bbox,
        target_contract="whole_object_bounds" if whole_contract else "legacy_surface_point",
        whole_object_passed=whole,
        whole_object_correct=whole_correct,
        wrong_instance_accepts=wrong_instances,
        whole_object_centre_band="per-axis2%image, same as calibrated controller deadband; bboxIoU>=.5",
        foreground_passed=foreground,
        bbox_passed=bbox,
        frames=total,
        correct_aims=correct,
        wrong_aims=wrong,
        boxes_iou_at_least_half=boxes,
        unavailable_truth=unavailable,
        positive_gate=0.95,
        false_accept_gate=0,
        boundary_tolerance_px=1,
        all_admitted_pixel_error_p95=error_p95,
        all_admitted_angular_error_bound_p95_deg=math.degrees(math.atan2(error_p95, focal)) if errors else None,
        error_scope="all admitted aims including failures; nearest silhouette pixel; pinhole angular upper bound",
        scope="sampled saved RGB regression; not exact live history, gimbal or real-time qualification",
    )
    (replay / "object-replay-frames.json").write_text(json.dumps(evaluated, indent=2) + "\n")
    (replay / "object-replay-result.json").write_text(json.dumps(result, indent=2) + "\n")
    bindings.update(
        {
            "template_png": sha(files[0]),
            "replay_options": sha(replay / "replay.json"),
            "replay-frames.jsonl": sha(replay / "replay-frames.jsonl"),
            "source_video_frames": sha(dataset / "video-frames.jsonl"),
            "source_world_manifest": sha(dataset / "world-manifest.json"),
            "scorer": sha(Path(__file__)),
        }
    )
    (replay / "object-replay-binding.json").write_text(json.dumps(bindings, indent=2) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--replay", required=True, type=Path)
    args = parser.parse_args()
    replay = owned(args.replay)
    try:
        result = score(args.dataset, replay)
    except Exception as error:
        (replay / "object-replay-error.json").write_text(
            json.dumps(dict(error=type(error).__name__, reason=str(error))) + "\n"
        )
        raise
    print(json.dumps(result))
    raise SystemExit(0 if result["passed"] else 1)
