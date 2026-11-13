# SPDX-License-Identifier: Apache-2.0
import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


@pytest.fixture
def motion(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts/simulation"))
    import object_motion

    return object_motion


@pytest.mark.parametrize("kind", ["orbit", "flyby", "direct", "approach"])
@pytest.mark.parametrize("altitude,speed", [(5, 5), (15, 10)])
@pytest.mark.parametrize("cpu_only", [False, True])
@pytest.mark.parametrize("target_case", ["static", "moving", "occlusion", "lookalike"])
def test_deer_fixture_keeps_native_plant_and_coincident_oracle(
    motion, monkeypatch, tmp_path, kind, altitude, speed, cpu_only, target_case
):
    import prepare_assets
    import task1_world

    monkeypatch.setattr(motion, "__file__", str(tmp_path / "scripts/simulation/object_motion.py"))
    inputs = tmp_path / "local/simulation/assets"
    inputs.mkdir(parents=True)
    (inputs / "deer.glb").write_bytes(b"frozen deer")
    monkeypatch.setattr(
        prepare_assets,
        "assets_for",
        lambda _: [
            {
                "proxy_length_m": 1.524,
                "source_length_m": 2.132851,
                "name": "deer.glb",
                "bytes": 11,
                "sha256": hashlib.sha256(b"frozen deer").hexdigest(),
            }
        ],
    )
    monkeypatch.setattr(task1_world, "tag", lambda link, *_: ET.SubElement(link, "visual", name="tag_A1_0"))
    world = ET.fromstring("<world><include><name>iris</name><pose>0 0 12 0 0 0</pose></include></world>")
    aircraft = ET.fromstring(
        '<model><link name="base"><gravity>true</gravity></link>'
        '<joint name="body_fixed_to_world"/><plugin name="NomadTestSystem"/> '
        '<plugin name="ArduPilotPlugin"/></model>'
    )
    camera = ET.fromstring("""<sdf><model><link name="pitch_link"><sensor name="camera" type="camera">
        <pose>0 0 0 -1.57 -1.57 0</pose><topic>/nomad/rgb</topic><update_rate>10</update_rate><camera>
        <horizontal_fov>1.5707963267948966</horizontal_fov><image><width>640</width><height>480</height></image>
        <clip><near>.05</near><far>100</far></clip></camera><plugin name="GstCameraPlugin"/></sensor>
        </link></model></sdf>""")
    info = motion.configure(
        world,
        aircraft,
        camera,
        tmp_path,
        dict(kind=kind, altitude_m=altitude, speed_m_s=speed, cpu_only=cpu_only, target_case=target_case),
    )
    rgb, mask = camera.findall(".//sensor")
    assert all(link.findtext("gravity") == "false" for model in (aircraft, camera) for link in model.findall(".//link"))
    assert [rgb.findtext(k) for k in ("pose", "update_rate", "camera/horizontal_fov", "camera/image/width")] == [
        mask.findtext(k) for k in ("pose", "update_rate", "camera/horizontal_fov", "camera/image/width")
    ]
    assert mask.get("type") == "segmentation" and not mask.findall("plugin")
    assert rgb.findtext("camera/image/width") == "1920"
    assert rgb.findtext("camera/image/height") == "1080"
    assert rgb.findtext("update_rate") == ("10" if cpu_only else "30")
    assert rgb.findtext("camera/horizontal_fov") == "2.0"
    assert rgb.findtext("plugin/use_cuda") == ("false" if cpu_only else "true")
    assert aircraft.findtext("plugin/target_case") == info["target_case"] == target_case
    assert not aircraft.findall("joint") and aircraft.find("plugin[@name='ArduPilotPlugin']") is not None
    assert [n.text for n in world.findall(".//plugin/label")] == ["10", "40", "20", "30"]
    assert info["altitude_m"] == altitude and info["speed_m_s"] == speed
    assert info["duration_sim_seconds"] == pytest.approx(
        40 * math.pi / speed if kind == "orbit" else (27 if kind == "approach" else 60) / speed
    )
    roi = info["selection_roi"]
    assert abs(roi[0] + roi[2] / 2 - 960) <= 0.5
    assert roi[1] + roi[3] / 2 < 541
    assert info["selection_torso_height_m"] == 0.75
    if kind == "approach":
        assert info["initial_body_pose"][:3] == [0, -35, altitude]
        assert info["initial_body_pose"][-1] == pytest.approx(math.pi / 2)
        assert info["initial_yaw_deg"] == pytest.approx(0)


@pytest.mark.parametrize(
    "fault",
    [
        None,
        "wrong_instance",
        "searching",
        "background",
        "no_body",
        "no_native",
        "no_joint",
        "overspeed",
        "stale_mask",
        "outside_native_tolerance",
        "edge_native_tolerance",
        "whole_hollow",
        "whole_wrong_instance",
        "whole_centre_error",
        "whole_missing_bbox",
        "moving",
        "lookalike",
        "missing_target_motion",
        "missing_lookalike_motion",
        "missing_occlusion",
        "occlusion",
        "occlusion_sparse_display",
        "occlusion_false_accept",
        "altitude_drift",
        "sampled_endpoints",
        "prefix_only_video",
        "frozen_video",
    ],
)
@pytest.mark.parametrize("size", [(640, 480), (1920, 1080)])
def test_object_oracle_requires_correct_pixels_positive_coverage_and_observed_motion(motion, tmp_path, fault, size):
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    (tmp_path / "frames").mkdir()
    width, height = size
    center_x, center_y = width // 2, height // 2
    image = np.zeros((height, width), np.uint8)
    image[center_y - 10 : center_y + 10, center_x - 10 : center_x + 10] = 10
    if fault == "whole_hollow":
        image[center_y - 4 : center_y + 4, center_x - 4 : center_x + 4] = 0
    assert cv2.imwrite(str(tmp_path / "frames/target.png"), image)
    image[center_y - 10 : center_y + 10, center_x - 10 : center_x + 10] = 20
    assert cv2.imwrite(str(tmp_path / "frames/other.png"), image)
    image[image == 20] = 30
    assert cv2.imwrite(str(tmp_path / "frames/occluder.png"), image)
    duration = 8 * math.pi
    captures, rows = [], []
    for index in range(50):
        t = duration * index / 49
        angle = t / 4 - math.pi / 2 if fault != "no_body" else 0
        captures.append(
            dict(
                capture_sim_ns=1e9 + t * 1e9,
                motion_elapsed_sim_ns=t * 1e9,
                mask_delta_ns=3_000_000 if fault == "stale_mask" else 0,
                body_position=[20 * math.cos(angle), 20 * math.sin(angle), 5.3 if fault == "altitude_drift" else 5],
                mask="other.png"
                if fault in ("wrong_instance", "whole_wrong_instance") and index == 20
                else "target.png",
            )
        )
        rows.append(
            dict(
                sequence=index,
                stage=40,
                verified=fault != "searching",
                aim_x=50
                if fault == "background"
                else center_x
                + (
                    width * 0.03
                    if fault == "whole_centre_error"
                    else 11
                    if fault == "outside_native_tolerance"
                    else 10
                    if fault == "edge_native_tolerance"
                    else 0
                ),
                aim_y=center_y,
                native_pitch_pwm=1700,
                native_yaw_pwm=1500 if fault == "no_native" else 1500 + index,
                region_bounds=None if fault == "whole_missing_bbox" else [center_x - 10, center_y - 10, 20, 20],
            )
        )
    (tmp_path / "object-motion-ack.json").write_text('{"start_sim_ns":1000000000}')
    (tmp_path / "joints.csv").write_text(
        "simulation_ns,roll,pitch,yaw,roll_velocity,pitch_velocity,yaw_velocity\n"
        + "\n".join(
            f"{1e9 + i / 49 * duration * 1e9},0,0,{0 if fault == 'no_joint' else 0.1 * math.sin(i)},0,0,"
            f"{1 if fault == 'overspeed' else 0}"
            for i in range(50)
        )
    )
    scene = {
        "object_motion": dict(kind="orbit", duration_sim_seconds=duration, scope="known oracle fixture"),
        "camera": dict(width=width, height=height, hfov_radians=2.0),
    }
    target_cases = {
        "moving": "moving",
        "lookalike": "lookalike",
        "missing_target_motion": "moving",
        "missing_lookalike_motion": "lookalike",
        "missing_occlusion": "occlusion",
        "occlusion": "occlusion",
        "occlusion_sparse_display": "occlusion",
        "occlusion_false_accept": "occlusion",
    }
    if fault in target_cases:
        scene["object_motion"]["target_case"] = target_cases[fault]
        for index, capture in enumerate(captures):
            capture["selected_position"] = [0 if fault == "missing_target_motion" else index / 10, 0, 0]
            capture["lookalike_position"] = [0 if fault == "missing_lookalike_motion" else index / 10, 8, 0]
            if fault.startswith("occlusion") and 15 <= index <= 26:
                capture["mask"] = "occluder.png"
                rows[index]["verified"] = fault == "occlusion_false_accept"
            elif fault == "occlusion" and index in (27, 28):
                rows[index]["verified"] = False
    if fault and fault.startswith("whole_"):
        scene["object_motion"]["target_contract"] = "whole_object_bounds"
    if fault == "sampled_endpoints":
        rows = rows[1:-1]
    elif fault == "prefix_only_video":
        rows = rows[:40]
    if fault == "whole_missing_bbox":
        with pytest.raises(ValueError, match="frame-owned"):
            motion.analyze(tmp_path, tmp_path, scene, captures, rows, lambda row, _: [captures[row["sequence"]]])
        return
    if fault == "stale_mask":
        with pytest.raises(ValueError, match="alignment"):
            motion.analyze(tmp_path, tmp_path, scene, captures, rows, lambda row, _: [captures[row["sequence"]]])
        return

    def matching(row, _):
        index = row["sequence"]
        if fault == "frozen_video" and index != 49:
            index = 0
        return [captures[index]]

    result = motion.analyze(tmp_path, tmp_path, scene, captures, rows, matching)
    assert result["passed"] is (
        fault
        in (
            None,
            "edge_native_tolerance",
            "whole_hollow",
            "moving",
            "lookalike",
            "occlusion",
            "occlusion_sparse_display",
            "sampled_endpoints",
        )
    )
    assert result["eligible_coverage"] <= 1
    assert (
        result["wrong_verified_frames"] > 0
        if fault in ("wrong_instance", "background", "outside_native_tolerance", "whole_wrong_instance")
        else True
    )
