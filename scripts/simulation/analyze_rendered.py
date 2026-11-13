# SPDX-License-Identifier: Apache-2.0
"""Independent fixed-plane geometry oracle. Its results never enter NOMAD's pixel/control path."""

import argparse
import csv
import json
import math
from pathlib import Path


def rotate(q, vector):
    w, x, y, z = q
    a, b, c = vector
    tx, ty, tz = 2 * (y * c - z * b), 2 * (z * a - x * c), 2 * (x * b - y * a)
    return (a + w * tx + y * tz - z * ty, b + w * ty + z * tx - x * tz, c + w * tz + x * ty - y * tx)


def anchor(camera, pixel, plane_normal=(0, 0, 1), plane_point=(0, 0, 0)):
    ray = rotate(camera["quaternion"], (1, -(pixel[0] - 320) / 320, -(pixel[1] - 240) / 320))
    origin = camera["position"]
    denominator = sum(a * b for a, b in zip(ray, plane_normal))
    if abs(denominator) < 0.05:
        raise ValueError("Oracle ray is too near the plane horizon")
    distance = sum((a - b) * n for a, b, n in zip(plane_point, origin, plane_normal)) / denominator
    if distance <= 0:
        raise ValueError("Oracle intersection is behind camera")
    return tuple(a + distance * b for a, b in zip(origin, ray))


def project(camera, world):
    q = camera["quaternion"]
    local = rotate((q[0], -q[1], -q[2], -q[3]), tuple(a - b for a, b in zip(world, camera["position"])))
    if local[0] <= 0.05:
        return None
    return 320 - 320 * local[1] / local[0], 240 - 320 * local[2] / local[0]


def match(row, captures):
    if "capture_sequence" in row:
        return (
            [c for c in captures if c.get("sequence") == row["capture_sequence"]]
            if row["capture_sequence"] is not None
            else []
        )
    fingerprint = bytes.fromhex(row["fingerprint"])
    if len(fingerprint) != 3072:
        raise ValueError("Decoded fingerprint size mismatch")
    scores = [
        (sum((a - b) ** 2 for a, b in zip(fingerprint, capture["pixels"])) / len(fingerprint), capture)
        for capture in captures
    ]
    best = min(score for score, _ in scores)
    if best > 100:
        return []
    return [capture for score, capture in scores if score <= best + 1]


def percentile(values, fraction):
    return sorted(values)[math.ceil((len(values) - 1) * fraction)] if values else None


def angular_error(left, right):
    a = (1, (left[0] - 320) / 320, (left[1] - 240) / 320)
    b = (1, (right[0] - 320) / 320, (right[1] - 240) / 320)
    cosine = sum(x * y for x, y in zip(a, b)) / math.sqrt(sum(x * x for x in a) * sum(x * x for x in b))
    return math.degrees(math.acos(max(-1, min(1, cosine))))


def ray_box(camera, world, box):
    q = box["quaternion"]
    inverse = (q[0], -q[1], -q[2], -q[3])
    origin = rotate(inverse, tuple(a - b for a, b in zip(camera["position"], box["position"])))
    target = rotate(inverse, tuple(a - b for a, b in zip(world, box["position"])))
    start, end = 0.0, 1.0
    for axis in range(3):
        direction = target[axis] - origin[axis]
        if abs(direction) < 1e-12:
            if not box["min"][axis] <= origin[axis] <= box["max"][axis]:
                return False
            continue
        a = (box["min"][axis] - origin[axis]) / direction
        b = (box["max"][axis] - origin[axis]) / direction
        start, end = max(start, min(a, b)), min(end, max(a, b))
        if start > end:
            return False
    return start < 1 - 1e-6 and end > 1e-6


def box_visibility(camera, targets):
    box = camera["occluder"]
    blocked = [ray_box(camera, point, box) for point in targets]
    if all(blocked):
        return "full"
    if any(blocked):
        return "partial"
    corners = [
        tuple(box["max"][axis] if mask & (1 << axis) else box["min"][axis] for axis in range(3)) for mask in range(8)
    ]
    world = [tuple(a + b for a, b in zip(rotate(box["quaternion"], corner), box["position"])) for corner in corners]
    pyramid = [camera["position"], *targets]
    for axis in range(3):
        if max(point[axis] for point in world) < min(point[axis] for point in pyramid) or min(
            point[axis] for point in world
        ) > max(point[axis] for point in pyramid):
            return "clear"
    return "partial"  # Enclosing volumes overlap; full visibility has not been established.


def analyze_chart(run, scene, captures, rows):
    chart = scene["detector_chart"]
    x, y, width, height = chart["primary_box"]
    worlds = []
    for u, v in ((x, y), (x + width, y), (x + width, y + height), (x, y + height)):
        horizontal = (u / chart["width"] - 0.5) * 24
        vertical = (0.5 - v / chart["height"]) * 13.5
        worlds.append(
            tuple(
                a + b
                for a, b in zip(scene["plane_point"], (vertical / math.sqrt(2), -horizontal, vertical / math.sqrt(2)))
            )
        )
    models = {name: {"frames": 0, "positives": 0, "filtered_errors": 0} for name in "ABCD"}
    errors, overlaps, unavailable = [], [], 0
    for row in rows:
        if row["stage"] != 10 or not row["DetectorIdentity"]:
            continue
        name = row["DetectorIdentity"].split()[1]
        if name not in models:
            raise ValueError("Unknown declared detector chart profile")
        models[name]["frames"] += 1
        if name == "D":
            models[name]["filtered_errors"] += len(row["detections"])
            continue
        candidates = match(row, captures)
        truth = [[project(camera, point) for point in worlds] for camera in candidates]
        if not truth or any(point is None for points in truth for point in points):
            unavailable += 1
            continue
        matches = []
        for detection in row["detections"]:
            if detection["ClassId"] != chart["class_id"]:
                continue
            left, top = detection["x"], detection["y"]
            right, bottom = left + detection["width"], top + detection["height"]
            ious, offsets = [], []
            for points in truth:
                a, b = min(point[0] for point in points), min(point[1] for point in points)
                c, d = max(point[0] for point in points), max(point[1] for point in points)
                intersection = max(0, min(c, right) - max(a, left)) * max(0, min(d, bottom) - max(b, top))
                ious.append(intersection / ((c - a) * (d - b) + (right - left) * (bottom - top) - intersection))
                offsets.append(math.dist(((a + c) / 2, (b + d) / 2), ((left + right) / 2, (top + bottom) / 2)))
            matches.append((min(ious), max(offsets)))
        accepted = [value for value in matches if value[0] >= 0.5 and value[1] <= 5]
        if accepted:
            best = max(accepted)
            models[name]["positives"] += 1
            overlaps.append(best[0])
            errors.append(best[1])
    passed = (
        unavailable == 0
        and all(
            models[name]["frames"] >= 3 and models[name]["positives"] >= models[name]["frames"] * 0.95 for name in "ABC"
        )
        and models["D"]["frames"] >= 3
        and models["D"]["filtered_errors"] == 0
    )
    result = {
        "schema": 1,
        "scope": "synthetic calibration chart; one manually annotated car; other cars unscored",
        "models": models,
        "unavailable_truth_frames": unavailable,
        "center_error_p95_px": percentile(errors, 0.95),
        "minimum_iou": min(overlaps) if overlaps else None,
        "passed": passed,
    }
    (run / "oracle-result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def analyze(run, output=None):
    output = run if output is None else output
    scene = json.loads((run / "world-manifest.json").read_text())
    occlusion = scene.get("occlusion", False)
    paths = sorted((run / "frames").glob("*.json"))
    if scene.get("endurance"):
        prefix = json.loads((run / "oracle-prefix.json").read_text())
        paths = [path for path in paths if int(path.stem.removeprefix("capture_")) <= prefix["last_sequence"]]
    captures = [json.loads(path.read_text()) for path in paths]
    captures = [capture for capture in captures if abs(capture["pose_delta_ns"]) <= 2_000_000]
    if occlusion:
        captures = [capture for capture in captures if "occluder" in capture]
    for capture in captures:
        capture["pixels"] = bytes.fromhex(capture["fingerprint"])
        if len(capture["pixels"]) != 3072:
            raise ValueError("Oracle fingerprint size mismatch")
        if (
            len(capture["position"]) != 3
            or len(capture["quaternion"]) != 4
            or not all(math.isfinite(value) for value in capture["position"] + capture["quaternion"])
            or abs(sum(value * value for value in capture["quaternion"]) - 1) > 1e-6
        ):
            raise ValueError("Invalid independent camera pose")
        if occlusion:
            box = capture["occluder"]
            if (
                len(box["position"]) != 3
                or len(box["quaternion"]) != 4
                or len(box["min"]) != 3
                or len(box["max"]) != 3
                or not all(
                    math.isfinite(value) for value in box["position"] + box["quaternion"] + box["min"] + box["max"]
                )
                or abs(sum(value * value for value in box["quaternion"]) - 1) > 1e-6
                or any(a >= b for a, b in zip(box["min"], box["max"]))
            ):
                raise ValueError("Invalid independent occluder pose/bounds")
    rows = [json.loads(line) for line in (run / "video-frames.jsonl").read_text().splitlines()]
    if not rows or not captures:
        raise ValueError("No aligned rendered/decoded records")
    if scene.get("detector_chart"):
        return analyze_chart(output, scene, captures, rows)
    if scene.get("object_motion"):
        from object_motion import analyze as analyze_object

        return analyze_object(run, output, scene, captures, rows, match)
    reference = match(rows[0], captures)
    if not reference:
        raise ValueError("Reference capture correspondence unavailable")
    plane_normal = scene.get("plane_normal", (0, 0, 1))
    plane_point = scene.get("plane_point", (0, 0, 0))
    anchors = [anchor(camera, (432, 252), plane_normal, plane_point) for camera in reference]
    low = tuple(min(point[axis] for point in anchors) for axis in range(3))
    high = tuple(max(point[axis] for point in anchors) for axis in range(3))
    if math.dist(low, high) > 0.05:
        raise ValueError("Reference world anchor is ambiguous beyond 5 cm")
    # Positive-depth pinhole coordinate extrema lie at these enclosing box vertices.
    worlds = set(tuple(high[axis] if mask & (1 << axis) else low[axis] for axis in range(3)) for mask in range(8))
    targets = set(worlds)
    if occlusion:
        for pixel in ((360, 180), (504, 180), (504, 324), (360, 324)):
            samples = [anchor(camera, pixel, plane_normal, plane_point) for camera in reference]
            low_corner = tuple(min(point[axis] for point in samples) for axis in range(3))
            high_corner = tuple(max(point[axis] for point in samples) for axis in range(3))
            targets.update(
                tuple(high_corner[axis] if mask & (1 << axis) else low_corner[axis] for axis in range(3))
                for mask in range(8)
            )
    evaluated, errors, angles, center_x, center_y = [], [], [], [], []
    visible = verified = wrong = unavailable = 0
    full = partial = clear_streak = recovery_grace = 0
    recovery = False
    for row in rows:
        if row["stage"] not in ((2, 20) if occlusion else (2,)):
            continue
        candidates = match(row, captures)
        if not candidates:
            unavailable += 1
            evaluated.append({"sequence": row["sequence"], "truth": "capture correspondence unavailable"})
            continue
        visibility = {box_visibility(camera, targets) for camera in candidates} if occlusion else {"clear"}
        if visibility == {"full"}:
            full += 1
            recovery = True
            clear_streak = 0
            if row["verified"]:
                wrong += 1
            evaluated.append({"sequence": row["sequence"], "truth": "full occlusion", "verified": row["verified"]})
            continue
        is_partial = occlusion and visibility != {"clear"}
        if is_partial:
            partial += 1
            recovery = True
            clear_streak = 0
            if "full" in visibility and row["verified"]:
                wrong += 1
            if not row["verified"]:
                evaluated.append(
                    {"sequence": row["sequence"], "truth": "partial/uncertain visibility", "verified": False}
                )
                continue
        is_confirmation = False
        if not is_partial and recovery:
            clear_streak += 1
            if clear_streak <= 2:
                recovery_grace += 1
                is_confirmation = True
        points = [project(camera, world) for camera in candidates for world in worlds]
        points = [point for point in points if point is not None]
        x_bounds = (min(point[0] for point in points), max(point[0] for point in points)) if points else (0, math.inf)
        y_bounds = (min(point[1] for point in points), max(point[1] for point in points)) if points else (0, math.inf)
        spread = math.hypot(x_bounds[1] - x_bounds[0], y_bounds[1] - y_bounds[0])
        if not points or spread > 1:
            unavailable += 1
            evaluated.append({"sequence": row["sequence"], "truth": "unavailable"})
            continue
        truth = points[0]
        bounds = [(x, y) for x in x_bounds for y in y_bounds]
        eligible = all(0 <= x < 640 and 0 <= y < 480 for x, y in bounds)
        if not eligible and any(0 <= x < 640 and 0 <= y < 480 for x, y in bounds):
            unavailable += 1
            evaluated.append({"sequence": row["sequence"], "truth": "visibility boundary ambiguous"})
            continue
        error = None
        angle = None
        if eligible:
            if not is_partial and not is_confirmation:
                visible += 1
            center_x.append(max(abs(point[0] - 320) for point in bounds))
            center_y.append(max(abs(point[1] - 240) for point in bounds))
            if row["verified"]:
                if not is_partial and not is_confirmation:
                    verified += 1
                error = max(math.dist(point, (row["aim_x"], row["aim_y"])) for point in bounds)
                errors.append(error)
                angle = max(angular_error(point, (row["aim_x"], row["aim_y"])) for point in bounds)
                angles.append(angle)
                if error > 5:
                    wrong += 1
        elif row["verified"]:
            wrong += 1
        evaluated.append(
            {
                "sequence": row["sequence"],
                "truth": "clear recovery confirmation" if is_confirmation else "equivalent-pose-set",
                "source_candidates": len(points),
                "eligible": eligible,
                "verified": row["verified"],
                "truth_x": truth[0],
                "truth_y": truth[1],
                "accepted_error_px": error,
                "accepted_error_deg": angle,
            }
        )
    result = {
        "schema": 1,
        "scope": "fixed plane and opaque occluder; conservative equivalent-pose/reference-anchor bounds"
        if occlusion
        else "unoccluded fixed plane; conservative equivalent-pose/reference-anchor bounds",
        "visible_frames": visible,
        "full_occlusion_frames": full,
        "partial_frames": partial,
        "recovery_confirmation_frames": recovery_grace,
        "verified_frames": verified,
        "wrong_verified_frames": wrong,
        "unavailable_truth_frames": unavailable,
        "verified_coverage": verified / visible if visible else 0,
        "accepted_error_p95_px": percentile(errors, 0.95),
        "accepted_error_p95_deg": percentile(angles, 0.95),
        "center_error_x_p95_px": percentile(center_x[-20:], 0.95),
        "center_error_y_p95_px": percentile(center_y[-20:], 0.95),
        "capture_latency": "unavailable: product does not expose capture identity/time",
        "passed": visible >= 40
        and verified >= 0.95 * visible
        and wrong == 0
        and unavailable == 0
        and (not occlusion or (full >= 5 and clear_streak >= 3)),
    }
    if scene.get("endurance"):
        result["scope"] = "initial five-second slice only; later stability/resources are independently reported"
    (output / "oracle-result.json").write_text(json.dumps(result, indent=2) + "\n")
    with (output / "oracle-frames.csv").open("w", newline="") as report:
        writer = csv.DictWriter(
            report,
            fieldnames=[
                "sequence",
                "truth",
                "source_candidates",
                "eligible",
                "verified",
                "truth_x",
                "truth_y",
                "accepted_error_px",
                "accepted_error_deg",
            ],
        )
        writer.writeheader()
        writer.writerows(evaluated)
    return result


def self_check():
    box = {"position": [2, 0, 0], "quaternion": [1, 0, 0, 0], "min": [-0.5, -0.5, -0.5], "max": [0.5, 0.5, 0.5]}
    assert ray_box({"position": [0, 0, 0]}, (4, 0, 0), box)
    assert not ray_box({"position": [0, 0, 0]}, (4, 4, 0), box)
    assert not ray_box({"position": [0, 0, 0]}, (1, 0, 0), box)
    camera = {"position": [0, 0, 0], "occluder": box}
    assert box_visibility(camera, [(4, 0.1, 0.1), (4, -0.1, -0.1)]) == "full"
    assert box_visibility(camera, [(4, 4, 0.1), (4, 5, -0.1)]) == "partial"
    assert abs(angular_error((320, 240), (640, 240)) - 45) < 1e-8
    camera = {"position": [0, 0, 1], "quaternion": [math.sqrt(0.5), 0, math.sqrt(0.5), 0]}
    point = anchor(camera, (432, 252))
    assert math.dist(project(camera, point), (432, 252)) < 1e-9
    assert math.dist(project({"position": [0, 0, 0], "quaternion": [1, 0, 0, 0]}, (1, -0.5, 0)), (480, 240)) < 1e-9
    q = (math.sqrt(0.5), 0, 0, math.sqrt(0.5))
    assert math.dist(rotate(q, (1, 0, 0)), (0, 1, 0)) < 1e-9


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path, nargs="?")
    parser.add_argument("--output", type=Path, help="Separate report directory when preserving an earlier attempt")
    args = parser.parse_args()
    self_check()
    if args.run:
        output = args.output.resolve(strict=True) if args.output else None
        result = analyze(args.run.resolve(strict=True), output)
        print(json.dumps(result))
        raise SystemExit(0 if result["passed"] else 1)
    print("PASS independent camera-ray/plane oracle self-check")
