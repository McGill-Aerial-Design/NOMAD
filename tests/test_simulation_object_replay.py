# SPDX-License-Identifier: Apache-2.0
import importlib.util
import json
import uuid
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    "object_replay", Path(__file__).parents[1] / "scripts/simulation/score_object_replay.py"
)
scorer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scorer)


def test_bbox_center_in_hole_is_not_foreground_success():
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    image = np.zeros((30, 30), dtype=np.uint8)
    image[5:25, 5:25] = 10
    image[10:20, 10:20] = 0
    result = scorer.measure(image, [5, 5, 20, 20], [15, 15], True)
    assert result["bbox_iou"] == 1 and not result["foreground"] and result["aim_label"] == 0
    assert not result["wrong_instance"] and result["centre_error_x"] == result["centre_error_y"] == 0
    assert scorer.measure(image, [5, 5, 20, 20], [7, 15], True)["foreground"]


def test_one_native_pixel_border_and_wrong_instance():
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    image = np.zeros((30, 40), dtype=np.uint8)
    image[10:20, 5:15], image[10:20, 25:35] = 10, 20
    assert scorer.measure(image, [5, 10, 10, 10], [15, 15], True)["foreground"]
    assert not scorer.measure(image, [5, 10, 10, 10], [16, 15], True)["foreground"]
    other = scorer.measure(image, [25, 10, 10, 10], [30, 15], True)
    assert not other["foreground"] and other["aim_label"] == 20 and other["distractor_iou"] == 1
    with pytest.raises(ValueError, match="outside image"):
        scorer.measure(image, [5, 10, 10, 10], [-0.1, 15], True)


def test_searching_has_no_positive_credit():
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    image = np.full((20, 20), 10, dtype=np.uint8)
    result = scorer.measure(image, None, None, False)
    assert result["visible"] and not result["foreground"] and result["bbox_iou"] == 0


def test_owned_marker_is_required(tmp_path):
    path = tmp_path / "run-0123456789abcdef0123456789abcdef"
    path.mkdir()
    (path / "run.id").write_text("different")
    with pytest.raises(ValueError, match="marker"):
        scorer.owned(path)


@pytest.fixture
def sampled_replay(tmp_path):
    np = pytest.importorskip("numpy")
    cv2 = pytest.importorskip("cv2")
    dataset, replay = [tmp_path / ("run-" + uuid.uuid4().hex) for _ in range(2)]
    for path in (dataset, replay):
        path.mkdir()
        (path / "run.id").write_text(path.name[4:])
    (dataset / "decoded").mkdir()
    (dataset / "frames").mkdir()
    geometry = dict(width=20, height=20, hfov_radians=1.0)
    (dataset / "world-manifest.json").write_text(
        json.dumps(dict(camera=geometry, object_motion=dict(duration_sim_seconds=2)))
    )
    capture = dict(
        **geometry,
        pose_delta_ns=0,
        mask_delta_ns=0,
        capture_sim_ns=1_000_000_000,
        motion_elapsed_sim_ns=1_000_000_000,
        mask="mask.png",
        fingerprint="00" * 3072,
    )
    (dataset / "frames/capture_1.json").write_text(json.dumps(capture))
    mask = np.zeros((20, 20), dtype=np.uint8)
    mask[5:15, 5:15] = 10
    assert cv2.imwrite(str(dataset / "frames/mask.png"), mask)
    original, rows = [], []
    for index in range(42):
        image = dataset / "decoded" / f"decoded_{index:08}.png"
        assert cv2.imwrite(str(image), np.zeros((20, 20, 3), dtype=np.uint8))
        original.append(
            dict(
                sequence=index,
                decoded_received_ticks=100 + index,
                frequency=100,
                stage=40 if index else 0,
                fingerprint="00" * 3072,
            )
        )
        if index:
            rows.append(
                dict(
                    frame=image.name,
                    sha256=scorer.sha(image),
                    source_received_ticks=100 + index,
                    frequency=100,
                    discontinuous=False,
                    state="Searching",
                    bounds=None,
                    aim_x=None,
                    aim_y=None,
                )
            )
    (dataset / "video-frames.jsonl").write_text("\n".join(map(json.dumps, original)))
    (replay / "replay.json").write_text(
        json.dumps(dict(dataset=str(dataset), folder="decoded", first=0, mode="Object"))
    )
    return dataset, replay, rows


def write_rows(replay, rows):
    (replay / "replay-frames.jsonl").write_text("\n".join(map(json.dumps, rows)))


def test_always_searching_keeps_full_denominator(sampled_replay):
    dataset, replay, rows = sampled_replay
    write_rows(replay, rows)
    result = scorer.score(dataset, replay)
    assert result["frames"] == 41 and result["correct_aims"] == 0 and not result["passed"]
    for row in rows:
        row.update(state="Verified", bounds=[5, 5, 10, 10], aim_x=10, aim_y=10)
    write_rows(replay, rows)
    result = scorer.score(dataset, replay)
    assert result["passed"] and result["correct_aims"] == 41
    for row in rows[-10:]:
        row["aim_x"] = 19
    write_rows(replay, rows)
    result = scorer.score(dataset, replay)
    assert not result["passed"] and result["wrong_aims"] == 10 and result["all_admitted_pixel_error_p95"] >= 5


@pytest.mark.parametrize("fault", ["omitted", "clock", "hfov", "pixels"])
def test_replay_rejects_incomplete_or_unbound_inputs(sampled_replay, fault):
    dataset, replay, rows = sampled_replay
    message = {"omitted": "omitted", "clock": "binding", "hfov": "HFOV", "pixels": "fingerprint"}[fault]
    if fault == "omitted":
        rows.pop(0)
    elif fault == "clock":
        rows[0]["source_received_ticks"] += 1
    elif fault == "hfov":
        path = dataset / "frames/capture_1.json"
        capture = json.loads(path.read_text())
        capture["hfov_radians"] = 1.01
        path.write_text(json.dumps(capture))
    else:
        import cv2
        import numpy as np

        path = dataset / "decoded" / rows[0]["frame"]
        assert cv2.imwrite(str(path), np.full((20, 20, 3), 255, dtype=np.uint8))
        rows[0]["sha256"] = scorer.sha(path)
    write_rows(replay, rows)
    with pytest.raises(ValueError, match=message):
        scorer.score(dataset, replay)


def test_scoring_deadline_stops_owned_analysis(sampled_replay, monkeypatch):
    dataset, replay, rows = sampled_replay
    write_rows(replay, rows)
    clock = iter([0, 91])
    monkeypatch.setattr(scorer.time, "monotonic", lambda: next(clock))
    with pytest.raises(TimeoutError, match="90-second"):
        scorer.score(dataset, replay)


def test_whole_object_centre_may_be_in_a_hole_without_changing_legacy_surface_gate(sampled_replay):
    import cv2

    dataset, replay, rows = sampled_replay
    path = dataset / "world-manifest.json"
    scene = json.loads(path.read_text())
    scene["object_motion"]["target_contract"] = "whole_object_bounds"
    path.write_text(json.dumps(scene))
    mask_path = dataset / "frames/mask.png"
    labels = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
    labels[8:13, 8:13] = 0
    assert cv2.imwrite(str(mask_path), labels)
    for row in rows:
        row.update(state="Verified", bounds=[5, 5, 10, 10], aim_x=10, aim_y=10)
    write_rows(replay, rows)
    result = scorer.score(dataset, replay)
    assert result["passed"] and result["whole_object_passed"] and result["wrong_instance_accepts"] == 0
    assert not result["foreground_passed"] and result["wrong_aims"] == 41
