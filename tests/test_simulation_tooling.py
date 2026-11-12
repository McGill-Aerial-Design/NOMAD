# SPDX-License-Identifier: Apache-2.0
import hashlib
import importlib
import json
import math
import os
import socket
import struct
import subprocess
import sys
import time
import types
import uuid
import zipfile
from pathlib import Path

import pytest


@pytest.mark.parametrize("rate", [True, 0, 500, 400.0])
def test_rendered_fixture_rejects_unsupported_physics_rate(tooling, tmp_path, rate):
    with pytest.raises(ValueError, match="physics_hz"):
        tooling("world").create(tmp_path, tmp_path, "127.0.0.1", physics_hz=rate)


@pytest.fixture
def tooling(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts/simulation"))
    return importlib.import_module


def test_independent_camera_oracle(tooling):
    oracle = tooling("analyze_rendered")
    oracle.self_check()
    with pytest.raises(ValueError, match="horizon"):
        oracle.anchor({"position": [0, 0, 1], "quaternion": [1, 0, 0, 0]}, (320, 240))


def test_panorama_cardinals_projection_and_wrap(tooling):
    panorama = tooling("panorama")
    assert math.dist(panorama.world_ray((320, 240), (90, 0, 0)), (0, 1, 0)) < 1e-12
    assert math.dist(panorama.world_ray((320, 240), (0, 90, 0)), (0, 0, 1)) < 1e-12
    assert panorama.project((1, 0, 0), (180, 0, 0)) is None
    for pose in ((0, 0, 0), (179, -20, 15), (-181, -20, 15), (360, 10, -10)):
        ray = panorama.world_ray((432, 252), pose)
        assert math.dist(panorama.project(ray, pose), (432, 252)) < 1e-9
    ray = panorama.world_ray((432, 252), (179, -20, 15))
    assert math.dist(panorama.project(ray, (-181, -20, 15)), (432, 252)) < 1e-9
    outlined = panorama.object_projection((75, -12, 0))
    assert all(math.dist(a, b) < 1e-9 for a, b in zip(outlined, panorama.OBJECT_OUTLINE))
    assert all(math.dist(a, b) < 1e-9 for a, b in zip(outlined, panorama.object_projection((435, -12, 0))))
    assert all(point is None for point in panorama.object_projection((255, -12, 0)))


def test_recovery_grace_never_exempts_accepted_point(tooling, monkeypatch, tmp_path):
    oracle = tooling("analyze_rendered")
    (tmp_path / "frames").mkdir()
    (tmp_path / "world-manifest.json").write_text(json.dumps({"occlusion": True}))
    capture = {
        "position": [0, 0, 1],
        "quaternion": [math.sqrt(0.5), 0, math.sqrt(0.5), 0],
        "pose_delta_ns": 0,
        "fingerprint": bytes(3072).hex(),
        "occluder": {
            "position": [0, 0, 0.5],
            "quaternion": [1, 0, 0, 0],
            "min": [-0.1, -0.1, -0.1],
            "max": [0.1, 0.1, 0.1],
        },
    }
    (tmp_path / "frames/capture_1.json").write_text(json.dumps(capture))
    monkeypatch.setattr(oracle, "box_visibility", lambda camera, _: "full" if camera["blocked"] else "clear")

    def matching(row, captures):
        return [dict(captures[0], blocked=row["sequence"] < 5)]

    monkeypatch.setattr(oracle, "match", matching)
    rows = [{"sequence": n, "stage": 20, "verified": n >= 5, "aim_x": 432, "aim_y": 252} for n in range(50)]
    path = tmp_path / "video-frames.jsonl"
    path.write_text("\n".join(json.dumps(row) for row in rows))
    assert oracle.analyze(tmp_path)["passed"]
    rows[5]["aim_x"] += 100
    path.write_text("\n".join(json.dumps(row) for row in rows))
    result = oracle.analyze(tmp_path)
    assert not result["passed"] and result["wrong_verified_frames"] == 1


def test_endurance_detects_growth_and_missing_observations(tooling):
    summarize = tooling("resources").summarize_endurance
    rows = [
        {
            "tracking_seconds": t,
            "private_bytes": 200 * 1048576,
            "process_cpu_ms": t * 1000,
            "handle_count": 400,
            "gdi_objects": 80,
            "user_objects": 90,
        }
        for t in range(601)
    ]
    assert summarize(rows, 600)["passed"]
    for row in rows:
        row["private_bytes"] += row["tracking_seconds"] / 60 * 3 * 1048576
    result = summarize(rows, 600)
    assert not result["passed"] and abs(result["private_slope_mib_per_minute"] - 3) < 1e-8
    assert not summarize(rows[:400], 600)["passed"]


def test_recording_budget_excludes_copied_runtimes(tooling, tmp_path):
    resources = tooling("resources")
    for name in ("frames", "decoded", "host"):
        (tmp_path / name).mkdir()
    (tmp_path / "frames/capture.json").write_bytes(b"123")
    (tmp_path / "decoded/image.png").write_bytes(b"12345")
    (tmp_path / "video-health.jsonl").write_bytes(b"1234567")
    (tmp_path / "host/runtime.dll").write_bytes(b"excluded reference binary")
    assert resources.recording_bytes(tmp_path) == 15


def test_renderer_health_rejects_late_capture_failure(tooling, tmp_path):
    check = tooling("supervise").check_renderer
    process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    try:
        check(process, tmp_path)
        (tmp_path / "capture-error.txt").write_text("late recorder failure")
        with pytest.raises(RuntimeError, match="recording failed"):
            check(process, tmp_path)
    finally:
        process.terminate()
        process.wait(timeout=2)


@pytest.mark.parametrize("home_workspace", [False, True])
def test_visible_readiness_failure_still_cleans_owned_processes(tooling, monkeypatch, tmp_path, home_workspace):
    supervisor = tooling("supervise")
    workspace = tmp_path / "workspace"
    workspace.mkdir()
    (workspace / ".nomad-simulation-owned").write_text("NOMAD simulation sources v1")
    run_id = uuid.uuid4().hex
    run = tmp_path / ("run-" + run_id)
    run.mkdir()
    (run / "run.id").write_text(run_id)
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("USERPROFILE", str(tmp_path))
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "supervise",
            "--workspace",
            "~/workspace" if home_workspace else str(workspace),
            "--run",
            str(run),
            "--rendered",
            "--visible",
        ],
    )
    monkeypatch.setattr(supervisor, "verify", lambda *_: {"sha256": "fixture"})
    monkeypatch.setattr(supervisor, "available", lambda *_: None)
    monkeypatch.setattr(supervisor, "record_resources", lambda *_: None)
    monkeypatch.setattr(supervisor.signal, "signal", lambda *_: None)
    monkeypatch.setattr(supervisor.signal, "SIGHUP", 1, raising=False)
    monkeypatch.setattr(supervisor, "create_world", lambda *_: run / "world.sdf")
    monkeypatch.setattr(supervisor, "RtpRelay", lambda: types.SimpleNamespace(stats={}, close=lambda: None))
    original_read = Path.read_text

    def read(path, *args, **kwargs):
        if str(path).replace("\\", "/").endswith("/gz-sim8/gui/gui.config"):
            return "<gui/>"
        return original_read(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read)
    processes, stopped = [], []

    def start(*_, **__):
        process = types.SimpleNamespace(pid=100 + len(processes), returncode=None)
        process.poll = lambda: process.returncode
        processes.append(process)
        return process

    def stop(process, owner):
        assert owner == run.name
        stopped.append(process.pid)
        process.returncode = -15

    def command(args, **_):
        if args[0] == "git":
            return supervisor.AP_REVISION + "\n"
        raise RuntimeError("Injected camera readiness failure")

    monkeypatch.setattr(supervisor.subprocess, "Popen", start)
    monkeypatch.setattr(supervisor.subprocess, "check_output", command)
    monkeypatch.setattr(supervisor, "stop_owned", stop)
    with pytest.raises(RuntimeError, match="Injected camera readiness failure"):
        supervisor.main()
    result = json.loads((run / "linux.json").read_text())
    assert stopped == [101, 100] and not result["passed"]
    assert result["owned_process_exited"] and result["renderer_exited"]


def test_detector_chart_scores_manual_geometry_not_confidence_alone(tooling, tmp_path):
    oracle = tooling("analyze_rendered")
    scene = {
        "detector_chart": {"primary_box": [347, 211, 43, 28], "width": 640, "height": 360, "class_id": 2},
        "plane_point": [12 / math.sqrt(2), 0, 12 - 12 / math.sqrt(2)],
    }
    captures = [
        {
            "position": [0, 0, 12],
            "quaternion": [math.cos(math.pi / 8), 0, math.sin(math.pi / 8), 0],
            "pixels": bytes(3072),
        }
    ]
    rows = [
        {
            "stage": 10,
            "DetectorIdentity": "Owned " + name + " fixture",
            "fingerprint": bytes(3072).hex(),
            "detections": []
            if name == "D"
            else [{"ClassId": 2, "Confidence": 0.9, "x": 347, "y": 271, "width": 43, "height": 28}],
        }
        for name in "ABCD"
        for _ in range(3)
    ]
    assert oracle.analyze_chart(tmp_path, scene, captures, rows)["passed"]
    rows[0]["detections"][0]["x"] += 100
    assert not oracle.analyze_chart(tmp_path, scene, captures, rows)["passed"]


def test_asset_cache_refuses_changed_input(tooling, tmp_path):
    assets = tooling("prepare_assets")
    path = tmp_path / "fixture.bin"
    path.write_bytes(b"preserve modified fixture")
    with pytest.raises(ValueError, match="preserved"):
        assets.acquire(tmp_path, {"name": path.name, "bytes": 3, "sha256": "wrong"})
    assert path.read_bytes() == b"preserve modified fixture"


def test_model_archive_cannot_escape_or_create_symlink(tooling, tmp_path):
    assets = tooling("prepare_assets")
    archive = tmp_path / "asset.zip"
    target = tmp_path / "models"
    with zipfile.ZipFile(archive, "w") as bundle:
        bundle.writestr("../escape.txt", "outside")
    with pytest.raises(ValueError):
        assets.extract_model(archive, target)
    assert not (tmp_path / "escape.txt").exists()
    with zipfile.ZipFile(archive, "w") as bundle:
        entry = zipfile.ZipInfo("outside")
        entry.external_attr = 0o120777 << 16
        bundle.writestr(entry, "../escape.txt")
    with pytest.raises(ValueError, match="symbolic"):
        assets.extract_model(archive, target)
    assert list(target.iterdir()) == []


def test_build_manifest_rejects_changed_shared_library(tooling, monkeypatch, tmp_path):
    manifest = tooling("build_manifest")
    (tmp_path / "sources/example").mkdir(parents=True)
    (tmp_path / "logs").mkdir()
    binary = tmp_path / "bridge"
    library = tmp_path / "libflight.so"
    binary.write_bytes(b"owned bridge")
    library.write_bytes(b"owned dynamics")
    monkeypatch.setattr(
        manifest.subprocess, "check_output", lambda command, **_: "pin\n" if "rev-parse" in command else ""
    )
    manifest.record(tmp_path, "example", binary)
    manifest.verify(tmp_path, "example", "pin", binary)
    library.write_bytes(b"changed dynamics")
    with pytest.raises(ValueError, match="checksum"):
        manifest.verify(tmp_path, "example", "pin", binary)


def test_build_manifest_rejects_outside_artifact(tooling, monkeypatch, tmp_path):
    manifest = tooling("build_manifest")
    (tmp_path / "sources/example").mkdir(parents=True)
    (tmp_path / "logs").mkdir()
    binary = tmp_path / "bridge"
    binary.write_bytes(b"owned")
    monkeypatch.setattr(
        manifest.subprocess, "check_output", lambda command, **_: "pin\n" if "rev-parse" in command else ""
    )
    data = manifest.record(tmp_path, "example", binary)
    data["artifacts"]["../outside"] = "not a hash"
    (tmp_path / "logs/example-build.json").write_text(json.dumps(data))
    with pytest.raises((ValueError, FileNotFoundError)):
        manifest.verify(tmp_path, "example", "pin", binary)


def test_build_manifest_rejects_changed_runtime_model(tooling, monkeypatch, tmp_path):
    manifest = tooling("build_manifest")
    (tmp_path / "sources/example").mkdir(parents=True)
    (tmp_path / "logs").mkdir()
    binary = tmp_path / "bridge"
    binary.write_bytes(b"owned")
    dirty = False

    def git(command, **_):
        if "rev-parse" in command:
            return "pin\n"
        return " M models/recipe.yaml\n" if dirty and "status" in command and "--porcelain" in command else ""

    monkeypatch.setattr(manifest.subprocess, "check_output", git)
    manifest.record(tmp_path, "example", binary)
    dirty = True
    with pytest.raises(ValueError, match="source/model changed"):
        manifest.verify(tmp_path, "example", "pin", binary)


@pytest.mark.parametrize(
    "executable, state",
    [
        (None, "recorded"),
        ("arduplane", "recorded"),
        ("other", "unrecorded"),
        ("arduplane", "modified"),
        ("arduplane", "missing"),
    ],
)
def test_component_selects_only_verified_build_artifacts(tooling, monkeypatch, tmp_path, executable, state):
    manifest = tooling("build_manifest")
    builder = tooling("interactive_world")
    (tmp_path / "sources/ardupilot").mkdir(parents=True)
    (tmp_path / "logs").mkdir()
    binaries = tmp_path / "outputs"
    binaries.mkdir()
    copter, plane = binaries / "arducopter", binaries / "arduplane"
    copter.write_bytes(b"copter")
    plane.write_bytes(b"plane")
    monkeypatch.setattr(
        manifest.subprocess, "check_output", lambda command, **_: "pin\n" if "rev-parse" in command else ""
    )
    manifest.record(tmp_path, "ardupilot", copter, [plane])
    if state == "unrecorded":
        (binaries / executable).write_bytes(b"unrecorded")
    elif state == "modified":
        plane.write_bytes(b"changed")
    elif state == "missing":
        plane.unlink()
    if state == "recorded":
        selected = builder.component(tmp_path, "ardupilot", executable)
        binary = binaries / (executable or "arducopter")
        assert selected["binary"] == str(binary)
        assert selected["sha256"] == hashlib.sha256(binary.read_bytes()).hexdigest()
    else:
        with pytest.raises((ValueError, FileNotFoundError)):
            builder.component(tmp_path, "ardupilot", executable)


def test_rtp_bridge_preserves_datagram_and_stops(tooling):
    relay_type = tooling("rtp_relay").RtpRelay
    relay = relay_type(udp_port=0, tcp_port=0)
    try:
        with (
            socket.create_connection(relay.server.getsockname(), timeout=1) as client,
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender,
        ):
            data = bytes([0x80, 96]) + bytes(10) + b"owned encoded pixels"
            sender.sendto(data, relay.udp.getsockname())
            client.settimeout(1)
            header = client.recv(2)
            assert struct.unpack("!H", header)[0] == len(data)
            received = bytearray()
            while len(received) < len(data):
                received.extend(client.recv(len(data) - len(received)))
            assert received == data
    finally:
        relay.close()
    assert not relay.thread.is_alive()
    assert relay.stats["forwarded"] == 1
    # 208 KB (the Linux default) overflowed on 8 Mbit/s keyframe bursts and left the decoder grey.
    assert relay.stats["udp_receive_buffer"] > 212992


def test_rtp_bridge_rejects_non_rtp(tooling):
    relay = tooling("rtp_relay").RtpRelay(udp_port=0, tcp_port=0)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(b"not RTP data", relay.udp.getsockname())
        deadline = time.monotonic() + 1
        while relay.stats["error"] is None and time.monotonic() < deadline:
            time.sleep(0.005)
        assert relay.stats["error"] == "Unexpected RTP packet format"
        assert relay.stats["forwarded"] == 0
    finally:
        relay.close()


def test_rtp_viewer_disconnect_permits_reconnect_without_stopping_source(tooling):
    relay = tooling("rtp_relay").RtpRelay(udp_port=0, tcp_port=0)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    packet = bytes([0x80, 96]) + bytes(10) + b"owned pixels"
    try:
        first = socket.create_connection(relay.server.getsockname(), timeout=1)
        sender.sendto(packet, relay.udp.getsockname())
        first.settimeout(1)
        assert first.recv(2) == struct.pack("!H", len(packet))
        assert first.recv(len(packet)) == packet
        first.shutdown(socket.SHUT_RDWR)
        first.close()
        deadline = time.monotonic() + 1
        while relay.client is not None and relay.stats["error"] is None and time.monotonic() < deadline:
            sender.sendto(packet, relay.udp.getsockname())
            time.sleep(0.005)
        assert relay.stats["error"] is None and relay.client is None and relay.thread.is_alive()
        with socket.create_connection(relay.server.getsockname(), timeout=1) as second:
            second.settimeout(1)
            sender.sendto(packet, relay.udp.getsockname())
            assert second.recv(2) == struct.pack("!H", len(packet))
            assert second.recv(len(packet)) == packet
    finally:
        sender.close()
        relay.close()


def test_slow_rtp_viewer_does_not_block_source_or_corrupt_tcp_framing(tooling):
    relay = tooling("rtp_relay").RtpRelay(udp_port=0, tcp_port=0)
    packet = bytes([0x80, 96]) + bytes(60000)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
            with client:
                client.connect(relay.server.getsockname())
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline and relay.stats["error"] is None:
                    sender.sendto(packet, relay.udp.getsockname())
                    if relay.stats.get("dropped_backpressure", 0) > 0:
                        break
                    time.sleep(0.002)
                assert relay.stats["error"] is None and relay.thread.is_alive(), relay.stats
                assert relay.stats["dropped_backpressure"] > 0
                client.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
                client.settimeout(1)

                def exact(count):
                    data = bytearray()
                    while len(data) < count:
                        chunk = client.recv(count - len(data))
                        assert chunk, "Slow viewer lost its framed connection"
                        data.extend(chunk)
                    return bytes(data)

                assert struct.unpack("!H", exact(2))[0] == len(packet)
                assert exact(len(packet)) == packet
    finally:
        relay.close()


@pytest.mark.skipif(sys.platform != "linux", reason="Linux process-group ownership")
def test_owned_session_includes_child_process_groups(tooling):
    owned = tooling("owned_processes")
    resources = tooling("resources")
    marker = "test-owned-child-group"
    child_code = "import os,time; os.setpgrp(); time.sleep(30)"
    parent_code = (
        "import subprocess,sys,time; p=subprocess.Popen([sys.executable,'-c',sys.argv[1]]); "
        "print(p.pid,flush=True); time.sleep(30)"
    )
    parent = subprocess.Popen(
        [sys.executable, "-c", parent_code, child_code],
        stdout=subprocess.PIPE,
        text=True,
        start_new_session=True,
        env=dict(os.environ, NOMAD_SIMULATION_RUN_ID=marker),
    )
    child = int(parent.stdout.readline())
    try:
        deadline = time.monotonic() + 2
        while os.getpgid(child) != child and time.monotonic() < deadline:
            time.sleep(0.01)
        assert os.getpgid(child) == child and os.getsid(child) == parent.pid
        assert child in [pid for pid, _ in owned.members(parent.pid)]
        assert child in resources.snapshot({parent.pid})["members"]
        owned.stop(parent, marker)
        assert owned.members(parent.pid) == []
    finally:
        owned.stop(parent, marker)
        try:
            os.kill(child, 9)
        except ProcessLookupError:
            pass


@pytest.mark.skipif(sys.platform != "linux", reason="Linux process-group ownership")
def test_owned_group_cleans_child_after_leader_exit(tooling):
    owned = tooling("owned_processes")
    marker = "test-owned-descendant"
    child_code = "import time; time.sleep(30)"
    parent_code = (
        "import subprocess,sys; p=subprocess.Popen([sys.executable,'-c',sys.argv[1]]); print(p.pid,flush=True)"
    )
    parent = subprocess.Popen(
        [sys.executable, "-c", parent_code, child_code],
        stdout=subprocess.PIPE,
        text=True,
        start_new_session=True,
        env=dict(os.environ, NOMAD_SIMULATION_RUN_ID=marker),
    )
    try:
        child = int(parent.stdout.readline())
        parent.wait(timeout=2)
        assert any(pid == child for pid, _ in owned.members(parent.pid))
        owned.stop(parent, marker)
        assert owned.members(parent.pid) == []
    finally:
        owned.stop(parent, marker)


@pytest.mark.skipif(sys.platform != "linux", reason="Linux process-group ownership")
def test_owned_group_refuses_another_run(tooling):
    owned = tooling("owned_processes")
    child = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(30)"],
        start_new_session=True,
        env=dict(os.environ, NOMAD_SIMULATION_RUN_ID="other-test-run"),
    )
    try:
        with pytest.raises(RuntimeError, match="unowned"):
            owned.stop(child, "this-test-run")
        assert child.poll() is None
    finally:
        owned.stop(child, "other-test-run")
