# SPDX-License-Identifier: Apache-2.0
"""Small deer/body-disturbance fixture and its independent semantic-pixel oracle."""

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def configure(world, aircraft, camera, run, motion):
    from task1_assets import verify
    from task1_world import mesh_visual, tag

    kind, altitude, speed = motion["kind"], motion["altitude_m"], motion["speed_m_s"]
    if kind not in ("orbit", "flyby", "direct", "approach") or altitude not in (5, 15) or speed not in (5, 10):
        raise ValueError("Object motion needs orbit/flyby/direct/approach, altitude 5/15 and speed 5/10")
    inputs = Path(__file__).resolve().parents[2] / "local/simulation/task1/inputs"
    frozen = verify(inputs)
    site = frozen["site"]
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
    plant = aircraft.find("plugin[@name='NomadTestSystem']")
    for key, value in (("object_motion", kind), ("motion_altitude", altitude), ("motion_speed", speed)):
        ET.SubElement(plant, key).text = str(value)
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
    sensor.find("camera/image/width").text = "1920"
    sensor.find("camera/image/height").text = "1080"
    sensor.find("update_rate").text = "30"
    encoder = sensor.find("plugin[@name='GstCameraPlugin']")
    cuda = encoder.find("use_cuda")
    if cuda is None:
        cuda = ET.SubElement(encoder, "use_cuda")
    cuda.text = "true"
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
    focal = 1920 / (2 * math.tan(1))
    depth = radius * math.cos(pitch) + (1.2 - altitude) * math.sin(pitch)
    up = -radius * math.sin(pitch) + (1.2 - altitude) * math.cos(pitch)
    size = math.ceil(focal * 2.5 / depth)
    roi = [round(960 - size / 2), round(540 - focal * up / depth - size / 2), size, size]
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
        selection_torso_height_m=1.2,
        selection_extent_m=2.5,
        target_contract="whole_object_bounds",
        labels={"selected": 10, "other": 20, "occluder": 30, "tag": 40},
        body_yaw_disturbance="orbit: 10deg, approach: 5deg; sin(2pi*t/8simsec); otherwise none",
        asset_sha256=hashlib.sha256((inputs / site["deer"]["name"]).read_bytes()).hexdigest(),
        scope=(
            "kinematic aircraft disturbance,1920x1080/30fps/2rad,NVENC at camera.encoder_kbps;"
            " not flight dynamics or tag reading"
        ),
    )


def analyze(run, output, scene, captures, rows, match=None):
    import cv2
    import numpy as np

    motion = scene["object_motion"]
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
    total = verified = wrong = unavailable = visible = prefix = prefix_verified = infeasible = 0
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
        total += 1
        masks, object_metrics = [], []
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
            body_positions[capture["capture_sim_ns"]] = capture["body_position"]
        visibility = [np.count_nonzero(mask) >= 8 for mask in masks]
        if any(visibility) != all(visibility):
            unavailable += 1
            continue
        visible += int(all(visibility))
        found = bool(row["verified"])
        accepted = False
        distance = None
        if found:
            point = (int(round(row["aim_y"])), int(round(row["aim_x"])))
            accepted = all(
                0 <= point[0] < height
                and 0 <= point[1] < width
                and cv2.dilate(mask.astype(np.uint8), np.ones((3, 3), np.uint8))[point]
                for mask in masks
            )
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
                wrong += int(not accepted)
            if accepted:
                distances = [cv2.distanceTransform((~mask).astype(np.uint8), cv2.DIST_L2, 3)[point] for mask in masks]
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
        t = min(c["motion_elapsed_sim_ns"] for c in candidates) / 1e9
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
    passed = (
        total >= 40
        and wrong == 0
        and unavailable == 0
        and actual_motion_deg >= 4
        and native_motion_pwm >= 10
        and compliant
        and path_coverage
        >= (2 * math.pi - 0.1 if motion["kind"] == "orbit" else 26.5 if motion["kind"] == "approach" else 59.5)
        and (
            verified >= 0.95 * total and centered
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
        wrong_verified_frames=wrong,
        unavailable_truth_frames=unavailable,
        accepted_mask_error_p95_px=float(np.quantile(accepted_errors, 0.95)) if accepted_errors else None,
        centered_p95=bool(centered),
        observed_path_coverage=path_coverage,
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
