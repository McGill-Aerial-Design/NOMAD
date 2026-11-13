# SPDX-License-Identifier: Apache-2.0
"""Small deer/body-disturbance fixture and its independent semantic-pixel oracle."""

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def disable_gravity(model):
    for link in model.findall(".//link"):
        gravity = link.find("gravity")
        if gravity is None:
            gravity = ET.SubElement(link, "gravity")
        gravity.text = "false"


def configure(world, aircraft, camera, run, motion):
    from prepare_assets import assets_for, digest
    from task1_world import mesh_visual, tag

    kind, altitude, speed = motion["kind"], motion["altitude_m"], motion["speed_m_s"]
    target_case = motion.get("target_case", "static")
    if target_case not in ("static", "moving", "occlusion", "lookalike"):
        raise ValueError("Unknown target motion case")
    if kind not in ("orbit", "flyby", "direct", "approach") or altitude not in (5, 15) or speed not in (5, 10):
        raise ValueError("Object motion needs orbit/flyby/direct/approach, altitude 5/15 and speed 5/10")
    inputs = Path(__file__).resolve().parents[2] / "local/simulation/assets"
    site = {"deer": assets_for("deer")[0]}
    asset = inputs / site["deer"]["name"]
    if not asset.is_file() or asset.stat().st_size != site["deer"]["bytes"] or digest(asset) != site["deer"]["sha256"]:
        raise ValueError("Pinned deer fixture missing/changed; run python3 scripts/simulation/prepare_assets.py deer")
    scale = site["deer"]["proxy_length_m"] / site["deer"]["source_length_m"]

    def label(element, value):
        plugin = ET.SubElement(element, "plugin", filename="gz-sim-label-system", name="gz::sim::systems::Label")
        ET.SubElement(plugin, "label").text = str(value)

    for name, position, identity in (("selected_deer", "0 0 .015 0 0 0", 10), ("other_deer", "6 8 .015 0 0 1", 20)):
        model = ET.SubElement(world, "model", name=name)
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = position
        link = ET.SubElement(model, "link", name="decoy")
        visual = mesh_visual(
            link, "deer", (inputs / site["deer"]["name"]).resolve().as_posix(), scale, f"0 0 0 {math.pi / 2} 0 0"
        )
        label(visual, identity)
        if identity == 10:
            tag(link, "A1", scale, run)
            for panel in link.findall("visual"):
                if panel.get("name", "").startswith("tag_"):
                    label(panel, 40)
    blocker = ET.SubElement(world, "model", name="object_occluder")
    ET.SubElement(blocker, "static").text = "true"
    ET.SubElement(blocker, "pose").text = "30 30 2 0 0 0"
    body = ET.SubElement(blocker, "link", name="body")
    visual = ET.SubElement(body, "visual", name="opaque")
    ET.SubElement(ET.SubElement(ET.SubElement(visual, "geometry"), "box"), "size").text = "2 2 4"
    label(visual, 30)
    aircraft.remove(aircraft.find("joint[@name='body_fixed_to_world']"))
    # Velocity is prescribed by the test plant; gravity otherwise adds a timestep-dependent altitude drift.
    for model in (aircraft, camera):
        disable_gravity(model)
    plant = aircraft.find("plugin[@name='NomadTestSystem']")
    for key, value in (("object_motion", kind), ("motion_altitude", altitude), ("motion_speed", speed)):
        ET.SubElement(plant, key).text = str(value)
    ET.SubElement(plant, "target_case").text = target_case
    ET.SubElement(plant, "bounded_pngs").text = "true"
    start = (
        [0, -35, altitude]
        if kind == "approach"
        else [0, -20, altitude]
        if kind == "orbit"
        else [-30, -20 if kind == "flyby" else 0, altitude]
    )
    body_yaw = math.pi / 2 if kind == "approach" else 0
    world.find("include[name='iris']/pose").text = " ".join(map(str, [*start, 0, 0, body_yaw]))
    sensor = camera.find(".//sensor[@name='camera']")
    sensor.find("camera/horizontal_fov").text = "2.0"
    width, height = 1920, 1080
    sensor.find("camera/image/width").text = str(width)
    sensor.find("camera/image/height").text = str(height)
    sensor.find("update_rate").text = "10" if motion.get("cpu_only") else "30"
    encoder = sensor.find("plugin[@name='GstCameraPlugin']")
    cuda = encoder.find("use_cuda")
    if cuda is None:
        cuda = ET.SubElement(encoder, "use_cuda")
    cuda.text = "false" if motion.get("cpu_only") else "true"
    semantic = ET.fromstring(ET.tostring(sensor))
    semantic.set("name", "oracle_semantic")
    semantic.set("type", "segmentation")
    semantic.find("topic").text = "/nomad/oracle/semantic"
    ET.SubElement(semantic.find("camera"), "segmentation_type").text = "semantic"
    for plugin in list(semantic.findall("plugin")):
        semantic.remove(plugin)
    camera.find(".//link[@name='pitch_link']").append(semantic)
    radius = math.hypot(*start[:2])
    pitch = -math.atan2(altitude - 0.65, radius)
    focal = width / (2 * math.tan(1))
    # This pinned proxy spans about 1.52 m horizontally and 1.48 m vertically after its fixture transform.
    selection_height, selection_extent = 0.75, 1.8
    depth = radius * math.cos(pitch) + (selection_height - altitude) * math.sin(pitch)
    up = -radius * math.sin(pitch) + (selection_height - altitude) * math.cos(pitch)
    size = math.ceil(focal * selection_extent / depth)
    roi = [round(width / 2 - size / 2), round(height / 2 - focal * up / depth - size / 2), size, size]
    return dict(
        motion,
        radius_or_offset_m=0 if kind in ("direct", "approach") else 20,
        duration_sim_seconds=2 * math.pi * 20 / speed
        if kind == "orbit"
        else (27 if kind == "approach" else 60) / speed,
        initial_pitch_deg=math.degrees(pitch),
        initial_yaw_deg=math.degrees(body_yaw - math.atan2(-start[1], -start[0])),
        initial_body_pose=[*start, 0, 0, body_yaw],
        selection_roi=roi,
        selection_torso_height_m=selection_height,
        selection_extent_m=selection_extent,
        target_contract="whole_object_bounds",
        target_case=target_case,
        labels={"selected": 10, "other": 20, "occluder": 30, "tag": 40},
        body_yaw_disturbance="orbit: 10deg, approach: 5deg; sin(2pi*t/8simsec); otherwise none",
        asset_sha256=hashlib.sha256((inputs / site["deer"]["name"]).read_bytes()).hexdigest(),
        scope="kinematic aircraft/target disturbance; camera geometry/rate declared separately; not flight dynamics",
    )


def analyze(run, output, scene, captures, rows, match=None):
    import cv2
    import numpy as np

    motion = scene["object_motion"]
    target_case = motion.get("target_case", "static")
    whole_object = motion.get("target_contract") == "whole_object_bounds"
    if whole_object:
        from score_object_replay import measure
    duration = motion["duration_sim_seconds"]
    camera = scene.get("camera", dict(width=640, height=480, hfov_radians=math.pi / 2))
    width, height = camera["width"], camera["height"]
    if match is None:
        fingerprints = np.asarray([list(c["pixels"]) for c in captures], dtype=np.int32)

        def match(row, _):
            fingerprint = np.frombuffer(bytes.fromhex(row["fingerprint"]), dtype=np.uint8)
            if fingerprint.shape != (3072,):
                raise ValueError("Decoded fingerprint size mismatch")
            scores = np.mean((fingerprints - fingerprint) ** 2, axis=1)
            best = min(scores)
            return [] if best > 100 else [c for c, score in zip(captures, scores) if score <= best + 1]

    evaluated, offsets, accepted_errors, body_positions = [], [], [], {}
    video_captures = set()
    # Travel is a property of the full observed motion, not the UI's sampling of its endpoints.
    for capture in captures:
        if not 0 <= capture.get("motion_elapsed_sim_ns", -1) <= duration * 1e9:
            continue
        if abs(capture.get("pose_delta_ns", 0)) > 2_000_000:
            continue
        position = capture.get("body_position", [])
        if len(position) != 3 or not all(math.isfinite(value) for value in position):
            raise ValueError("Invalid independent aircraft body pose")
        body_positions[capture["capture_sim_ns"]] = position
    total = verified = wrong = incorrect = unavailable = visible = prefix = prefix_verified = infeasible = 0
    occluded = recovery_grace = clear_streak = 0
    was_occluded = False
    selected_positions, lookalike_positions = [], []
    tag_pixels_max = 0
    for row in rows:
        if row["stage"] != 40:
            continue
        correspondences = match(row, captures)
        if not correspondences:
            unavailable += 1
            continue
        candidates = [c for c in correspondences if 0 <= c.get("motion_elapsed_sim_ns", -1) <= duration * 1e9]
        if not candidates:
            continue  # Aligned pre-trigger/drain pixels are outside the commanded motion interval.
        video_captures.update(capture["capture_sim_ns"] for capture in candidates)
        total += 1
        masks, object_metrics, blockers = [], [], []
        if whole_object and row["verified"] and not row.get("region_bounds"):
            raise ValueError(
                "Whole-object oracle requires frame-owned region_bounds; historical point runs keep their old contract"
            )
        for capture in candidates:
            if (
                capture.get("width", width) != width
                or capture.get("height", height) != height
                or abs(capture.get("hfov_radians", camera["hfov_radians"]) - camera["hfov_radians"]) > 1e-9
            ):
                raise ValueError("Capture intrinsics differ from the declared camera")
            if abs(capture["mask_delta_ns"]) > 2_000_000:
                raise ValueError("Object mask/capture alignment exceeds 2 ms")
            if len(capture["body_position"]) != 3 or not all(math.isfinite(v) for v in capture["body_position"]):
                raise ValueError("Invalid independent aircraft body pose")
            path = (run / "frames" / capture["mask"]).resolve(strict=True)
            path.relative_to((run / "frames").resolve())
            image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
            if image is not None and image.ndim == 3:
                if (
                    image.shape != (height, width, 3)
                    or not np.array_equal(image[:, :, 0], image[:, :, 1])
                    or not np.array_equal(image[:, :, 0], image[:, :, 2])
                ):
                    raise ValueError("Invalid semantic oracle mask")
                image = image[:, :, 0]
            if image is None or image.shape != (height, width) or image.dtype != np.uint8:
                raise ValueError("Invalid semantic oracle mask")
            masks.append((image == 10) | (image == 40))
            blockers.append(bool(np.any(image == 30)))
            if whole_object:
                object_metrics.append(
                    measure(
                        image,
                        row.get("region_bounds"),
                        [row["aim_x"], row["aim_y"]] if row["verified"] else None,
                        bool(row["verified"]),
                    )
                )
            tag_pixels_max = max(tag_pixels_max, int(np.count_nonzero(image == 40)))
            if target_case != "static":
                for key, positions_list in (
                    ("selected_position", selected_positions),
                    ("lookalike_position", lookalike_positions),
                ):
                    point = capture.get(key)
                    if point is None or len(point) != 3 or not all(math.isfinite(v) for v in point):
                        raise ValueError("Missing observed target pose: " + key)
                    positions_list.append(point)
        visibility = [np.count_nonzero(mask) >= 8 for mask in masks]
        if any(visibility) != all(visibility):
            unavailable += 1
            continue
        visible += int(all(visibility))
        found = bool(row["verified"])
        t = min(c["motion_elapsed_sim_ns"] for c in candidates) / 1e9
        full_occlusion = (
            target_case == "occlusion"
            and not any(visibility)
            and all(blockers)
            and duration * 0.3 <= t <= duration * 0.55 + 0.2
        )
        if full_occlusion:
            occluded += 1
            was_occluded = True
            clear_streak = 0
        elif was_occluded and all(visibility):
            clear_streak += 1
            if clear_streak <= 2:
                # Display can skip frames already processed by inference; only exempt unverified confirmation frames.
                recovery_grace += int(not found)
        accepted = False
        distance = None
        if found:
            point = (int(round(row["aim_y"])), int(round(row["aim_x"])))
            if whole_object:
                # Match the controller's per-axis2% band; historical surface trials keep their point contract.
                accepted = all(
                    m["visible"]
                    and not m["wrong_instance"]
                    and m["bbox_iou"] >= 0.5
                    and m["centre_error_x"] <= width * 0.02
                    and m["centre_error_y"] <= height * 0.02
                    for m in object_metrics
                )
                wrong += int(any(m["wrong_instance"] for m in object_metrics))
            else:
                accepted = all(
                    0 <= point[0] < height
                    and 0 <= point[1] < width
                    and cv2.dilate(mask.astype(np.uint8), np.ones((3, 3), np.uint8))[point]
                    for mask in masks
                )
                wrong += int(not accepted)
            if accepted:
                distances = (
                    [metric["pixel_error"] for metric in object_metrics]
                    if whole_object
                    else [cv2.distanceTransform((~mask).astype(np.uint8), cv2.DIST_L2, 3)[point] for mask in masks]
                )
                distance = float(max(distances))
                accepted_errors.append(distance)
                if whole_object:
                    offsets.append(
                        (
                            max(abs(m["target_centre"][0] - width / 2) / width for m in object_metrics),
                            max(abs(m["target_centre"][1] - height / 2) / height for m in object_metrics),
                        )
                    )
                else:
                    offsets.append((abs(row["aim_x"] - width / 2) / width, abs(row["aim_y"] - height / 2) / height))
        verified += int(accepted)
        incorrect += int(found and not accepted)
        if t < 2:
            prefix += 1
            prefix_verified += int(accepted)
        if motion["kind"] == "direct":
            infeasible += int(
                any(
                    c["body_position"][0] >= 0
                    or math.degrees(math.atan2(c["body_position"][2] - 0.65, abs(c["body_position"][0]))) > 85
                    for c in candidates
                )
            )
        evaluated.append(
            dict(
                sequence=row["sequence"],
                visible=all(visibility),
                accepted=accepted,
                accepted_mask_error_px=distance,
                motion_sim_seconds=t,
            )
        )
    if not body_positions:
        raise ValueError("No observed object-motion body poses")
    positions = np.asarray([body_positions[key] for key in sorted(body_positions)])
    video_time_coverage = float(np.ptp([row["motion_sim_seconds"] for row in evaluated])) / duration if evaluated else 0
    altitude_error = float(np.max(np.abs(positions[:, 2] - motion.get("altitude_m", 5))))
    path_coverage = (
        float(np.ptp(np.unwrap(np.arctan2(positions[:, 1], positions[:, 0]))))
        if motion["kind"] == "orbit"
        else float(np.ptp(positions[:, 1 if motion["kind"] == "approach" else 0]))
    )
    joints = np.genfromtxt(run / "joints.csv", delimiter=",", names=True)
    start = json.loads((run / "object-motion-ack.json").read_text())["start_sim_ns"]
    joints = joints[(joints["simulation_ns"] >= start) & (joints["simulation_ns"] <= start + duration * 1e9)]
    if len(joints) < 10:
        raise ValueError("Missing measured gimbal motion during body trajectory")
    actual_motion_deg = math.degrees(max(float(np.ptp(joints["pitch"])), float(np.ptp(joints["yaw"]))))
    compliant = True
    for axis, low, high, rate in (("roll", -30, 30, 20), ("pitch", -45, 135, 30), ("yaw", -160, 160, 30)):
        angle, velocity = np.degrees(joints[axis]), np.degrees(joints[axis + "_velocity"])
        compliant &= bool(
            np.all(np.isfinite(angle))
            and np.all(np.isfinite(velocity))
            and min(angle) >= low - 0.2
            and max(angle) <= high + 0.2
            and max(abs(velocity)) <= rate * 1.05
        )
    commands = [row for row in rows if row["stage"] == 40]
    native_motion_pwm = (
        max(np.ptp([r["native_pitch_pwm"] for r in commands]), np.ptp([r["native_yaw_pwm"] for r in commands]))
        if commands
        else 0
    )
    centered = bool(offsets) and max(np.quantile(np.asarray(offsets), 0.95, axis=0)) <= 0.05
    positive = motion["kind"] != "direct"
    eligible = total - occluded - recovery_grace
    selected_travel = float(np.max(np.ptp(np.asarray(selected_positions), axis=0))) if selected_positions else 0
    lookalike_travel = float(np.max(np.ptp(np.asarray(lookalike_positions), axis=0))) if lookalike_positions else 0
    target_exercised = target_case == "static" or selected_travel >= 1
    target_exercised &= target_case != "lookalike" or lookalike_travel >= 4
    target_exercised &= target_case != "occlusion" or (occluded >= 5 and clear_streak >= 3)
    passed = (
        total >= 40
        and len(video_captures) >= 40
        and wrong == 0
        and incorrect == 0
        and target_exercised
        and unavailable == 0
        and actual_motion_deg >= 4
        and native_motion_pwm >= 10
        and compliant
        and video_time_coverage >= 0.95
        and altitude_error <= 0.1
        and path_coverage + 1e-6
        >= (2 * math.pi - 0.1 if motion["kind"] == "orbit" else 26.5 if motion["kind"] == "approach" else 59.5)
        and (
            verified >= 0.95 * eligible and centered
            if positive
            else prefix >= 5 and prefix_verified >= 0.95 * prefix and infeasible >= 3
        )
    )
    result = dict(
        schema=1,
        scope=motion["scope"],
        motion=motion,
        motion_frames=total,
        visible_frames=visible,
        verified_frames=verified,
        verified_coverage=verified / total if total else 0,
        eligible_frames=eligible,
        eligible_coverage=verified / eligible if eligible else 0,
        full_occlusion_frames=occluded,
        recovery_confirmation_frames=recovery_grace,
        incorrect_verified_frames=incorrect,
        observed_target_travel_m=selected_travel,
        observed_lookalike_travel_m=lookalike_travel,
        wrong_verified_frames=wrong,
        unavailable_truth_frames=unavailable,
        accepted_mask_error_p95_px=float(np.quantile(accepted_errors, 0.95)) if accepted_errors else None,
        centered_p95=bool(centered),
        observed_path_coverage=path_coverage,
        scored_video_time_coverage=video_time_coverage,
        distinct_scored_captures=len(video_captures),
        maximum_altitude_error_m=altitude_error,
        actual_joint_motion_deg=actual_motion_deg,
        joint_rate_and_travel_compliant=bool(compliant),
        native_motion_pwm=float(native_motion_pwm),
        tag_visible_pixels_max=tag_pixels_max,
        tag_readability="not qualified: selected-deer retention does not establish 3cm glyph reading",
        capture_latency="unavailable: product carries no capture identity/time",
        target_contract="whole_object_bounds" if whole_object else "legacy_surface_point",
        passed=bool(passed),
    )
    (output / "oracle-result.json").write_text(json.dumps(result, indent=2) + "\n")
    (output / "oracle-object-frames.json").write_text(json.dumps(evaluated, indent=2) + "\n")
    return result
