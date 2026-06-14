# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for edge_core.services.video_stream_manager.

The manager drives the in-container video bridge through ``docker`` (subprocess)
and the bridge's localhost HTTP API (urllib). Both boundaries are stubbed:
``subprocess.run`` via a command-classifying handler, and ``urlopen`` via a
URL->payload map. The watchdog loop is driven one iteration at a time by
stubbing the stop event's ``wait``.
"""

from __future__ import annotations

import subprocess
from types import SimpleNamespace
from urllib.error import URLError

from edge_core.services import video_stream_manager as vsm
from edge_core.services.video_stream_manager import StreamStatus, VideoStreamManager


class FakeResp:
    def __init__(self, payload):
        self._payload = payload

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def read(self):
        return self._payload.encode() if isinstance(self._payload, str) else self._payload


def _urlopen_map(monkeypatch, mapping):
    """Patch urlopen to dispatch on URL substring; values are JSON str or Exception."""

    def _open(url_or_req, timeout=None):
        url = url_or_req.full_url if hasattr(url_or_req, "full_url") else url_or_req
        for key, val in mapping.items():
            if key in url:
                if isinstance(val, Exception):
                    raise val
                return FakeResp(val)
        raise URLError(f"unmapped url: {url}")

    monkeypatch.setattr(vsm, "urlopen", _open)


def _run_handler(**outcomes):
    """Build a subprocess.run stub classifying docker commands by tag."""

    def classify(cmd):
        if cmd[:2] == ["docker", "ps"]:
            return "ps"
        if cmd[:2] == ["docker", "cp"]:
            return "cp"
        if "-d" in cmd[:3]:
            return "exec"
        if "pkill" in cmd:
            return "pkill"
        if "pgrep" in cmd:
            return "pgrep"
        if "tail" in cmd:
            return "tail"
        return "other"

    def run(cmd, **kwargs):
        out = outcomes.get(classify(cmd))
        if isinstance(out, Exception):
            raise out
        if out is None:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        return out

    return run


def _ok(stdout="", returncode=0, stderr=""):
    return SimpleNamespace(returncode=returncode, stdout=stdout, stderr=stderr)


# --------------------------------------------------------------------------- #
# StreamStatus + url helper
# --------------------------------------------------------------------------- #


def test_stream_status_to_dict():
    s = StreamStatus(True, "/t", "rtsp://x", 15.0, 10, 1, 2, 3.0, 640, 360, 800)
    d = s.to_dict()
    assert d["streaming"] is True and d["current_topic"] == "/t" and d["bitrate_kbps"] == 800


def test_local_rtsp_url_rewrites_docker_host():
    m = VideoStreamManager(rtsp_url="rtsp://172.17.0.1:8554/stream")
    assert m._local_rtsp_url() == "rtsp://localhost:8554/stream"


def test_default_rtsp_url_is_single_stream():
    # The Jetson exposes exactly one stream — no /primary or /secondary.
    assert vsm.DEFAULT_RTSP_URL.endswith(":8554/stream")
    assert "/primary" not in vsm.DEFAULT_RTSP_URL
    assert "/secondary" not in vsm.DEFAULT_RTSP_URL


# --------------------------------------------------------------------------- #
# container / relay status
# --------------------------------------------------------------------------- #


def test_is_container_running(monkeypatch):
    m = VideoStreamManager(container_name="nomad_isaac_ros")
    monkeypatch.setattr(vsm.subprocess, "run", lambda *a, **k: _ok(stdout="nomad_isaac_ros\n"))
    assert m.is_container_running() is True
    monkeypatch.setattr(vsm.subprocess, "run", lambda *a, **k: _ok(stdout="something_else\n"))
    assert m.is_container_running() is False


def test_is_container_running_handles_error(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(vsm.subprocess, "run", lambda *a, **k: (_ for _ in ()).throw(OSError("docker gone")))
    assert m.is_container_running() is False


def test_get_relay_status_data(monkeypatch):
    m = VideoStreamManager()
    _urlopen_map(monkeypatch, {"/status": '{"last_frame_age_s": 1.0}'})
    assert m._get_relay_status_data() == {"last_frame_age_s": 1.0}
    _urlopen_map(monkeypatch, {"/status": URLError("down")})
    assert m._get_relay_status_data() is None


def test_is_relay_running_healthy(monkeypatch):
    m = VideoStreamManager()
    _urlopen_map(monkeypatch, {"/health": '{"healthy": true, "pipeline_playing": true}'})
    assert m.is_relay_running() is True


def test_is_relay_running_unhealthy(monkeypatch):
    m = VideoStreamManager()
    _urlopen_map(monkeypatch, {"/health": '{"healthy": false, "pipeline_playing": false}'})
    assert m.is_relay_running() is False


def test_is_relay_running_requires_recent_frames(monkeypatch):
    m = VideoStreamManager()
    _urlopen_map(
        monkeypatch,
        {"/health": '{"healthy": true, "pipeline_playing": true}', "/status": '{"last_frame_age_s": 2.0}'},
    )
    assert m.is_relay_running(require_recent_frames=True) is True
    _urlopen_map(
        monkeypatch,
        {"/health": '{"healthy": true, "pipeline_playing": true}', "/status": '{"last_frame_age_s": 99.0}'},
    )
    assert m.is_relay_running(require_recent_frames=True) is False


def test_is_relay_running_handles_error(monkeypatch):
    m = VideoStreamManager()
    _urlopen_map(monkeypatch, {"/health": URLError("refused")})
    assert m.is_relay_running() is False


# --------------------------------------------------------------------------- #
# start_with_reason
# --------------------------------------------------------------------------- #


def test_start_adopts_existing_running_bridge(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: True)
    assert m.start_with_reason() == (True, "Already running")


def test_start_fails_when_container_down(monkeypatch):
    m = VideoStreamManager(container_name="nomad_isaac_ros")
    # alive-but-stale path: recent=True False, recent=False True -> logs warning, continues
    seq = iter([False, True])
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: next(seq))
    monkeypatch.setattr(m, "is_container_running", lambda: False)
    ok, msg = m.start_with_reason()
    assert ok is False and "is not running" in msg


def test_start_fails_when_script_missing(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: False)
    monkeypatch.setattr(m, "is_container_running", lambda: True)
    monkeypatch.setattr(vsm.os.path, "exists", lambda p: False)
    ok, msg = m.start_with_reason()
    assert ok is False and "script not found" in msg


def test_start_fails_when_docker_cp_errors(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: False)
    monkeypatch.setattr(m, "is_container_running", lambda: True)
    monkeypatch.setattr(vsm.os.path, "exists", lambda p: True)
    err = subprocess.CalledProcessError(1, "docker cp", stderr="permission denied")
    monkeypatch.setattr(vsm.subprocess, "run", _run_handler(cp=err))
    ok, msg = m.start_with_reason()
    assert ok is False and "Failed to copy" in msg


def test_start_fails_when_exec_returns_nonzero(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: False)
    monkeypatch.setattr(m, "is_container_running", lambda: True)
    monkeypatch.setattr(vsm.os.path, "exists", lambda p: True)
    monkeypatch.setattr(vsm.subprocess, "run", _run_handler(exec=_ok(returncode=1, stderr="boom")))
    ok, msg = m.start_with_reason()
    assert ok is False and "Docker exec failed: boom" in msg


def test_start_fails_when_exec_times_out(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: False)
    monkeypatch.setattr(m, "is_container_running", lambda: True)
    monkeypatch.setattr(vsm.os.path, "exists", lambda p: True)
    monkeypatch.setattr(vsm.subprocess, "run", _run_handler(exec=subprocess.TimeoutExpired("docker exec", 10)))
    ok, msg = m.start_with_reason()
    assert ok is False and "timed out" in msg


def test_start_succeeds_when_relay_comes_up(monkeypatch):
    m = VideoStreamManager()
    m._edge_core_api_key = "key"  # exercise the credential cmd.extend branches
    m._edge_core_internal_token = "tok"
    monkeypatch.setenv("NOMAD_TASK2_DEBUG", "1")  # exercise the detector-env passthrough
    calls = {"n": 0}

    def relay(require_recent_frames=False):
        calls["n"] += 1
        return calls["n"] >= 3  # top checks fail; loop's first probe succeeds

    monkeypatch.setattr(m, "is_relay_running", relay)
    monkeypatch.setattr(m, "is_container_running", lambda: True)
    monkeypatch.setattr(vsm.os.path, "exists", lambda p: True)
    monkeypatch.setattr(vsm.time, "sleep", lambda *_: None)
    monkeypatch.setattr(vsm.subprocess, "run", _run_handler(exec=_ok(returncode=0)))
    assert m.start_with_reason() == (True, "Started successfully")


def _start_loop_setup(monkeypatch, m):
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: False)
    monkeypatch.setattr(m, "is_container_running", lambda: True)
    monkeypatch.setattr(vsm.os.path, "exists", lambda p: True)
    monkeypatch.setattr(vsm.time, "sleep", lambda *_: None)


def test_start_reports_crash_when_process_gone(monkeypatch):
    m = VideoStreamManager()
    _start_loop_setup(monkeypatch, m)
    monkeypatch.setattr(
        vsm.subprocess,
        "run",
        # pkill raises (swallowed); process later found gone -> crash log via tail.
        _run_handler(
            pkill=OSError("no pkill"),
            exec=_ok(returncode=0),
            pgrep=_ok(returncode=1),
            tail=_ok(stdout="Traceback: boom"),
        ),
    )
    ok, msg = m.start_with_reason()
    assert ok is False and "crashed" in msg


def test_start_reports_no_frames_when_process_alive(monkeypatch):
    m = VideoStreamManager()
    _start_loop_setup(monkeypatch, m)
    monkeypatch.setattr(vsm.subprocess, "run", _run_handler(exec=_ok(returncode=0), pgrep=_ok(returncode=0)))
    ok, msg = m.start_with_reason()
    assert ok is False and "no fresh frames" in msg


def test_start_reports_when_status_check_fails(monkeypatch):
    m = VideoStreamManager()
    _start_loop_setup(monkeypatch, m)
    monkeypatch.setattr(
        vsm.subprocess, "run", _run_handler(exec=_ok(returncode=0), pgrep=OSError("docker exec failed"))
    )
    ok, msg = m.start_with_reason()
    assert ok is False and "could not check process status" in msg


def test_start_handles_generic_exec_error(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: False)
    monkeypatch.setattr(m, "is_container_running", lambda: True)
    monkeypatch.setattr(vsm.os.path, "exists", lambda p: True)
    monkeypatch.setattr(vsm.subprocess, "run", _run_handler(exec=RuntimeError("kaboom")))
    ok, msg = m.start_with_reason()
    assert ok is False and "Error starting bridge" in msg


# --------------------------------------------------------------------------- #
# watchdog
# --------------------------------------------------------------------------- #


def _drive_watchdog_once(monkeypatch, m):
    """Run a single watchdog iteration by stopping after the second wait()."""
    waits = {"n": 0}

    def fake_wait(timeout=None):
        waits["n"] += 1
        if waits["n"] >= 2:  # 1 = initial delay, 2 = end-of-iteration
            m._watchdog_stop.set()
        return True

    monkeypatch.setattr(m._watchdog_stop, "wait", fake_wait)


def test_watchdog_restarts_dead_bridge(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_container_running", lambda: True)
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: False)
    restarted = []
    monkeypatch.setattr(m, "start_with_reason", lambda: (restarted.append(1), (True, "ok"))[1])
    _drive_watchdog_once(monkeypatch, m)
    m._watchdog_loop(0.0)
    assert restarted == [1]


def test_watchdog_logs_failed_restart(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_container_running", lambda: True)
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: False)
    monkeypatch.setattr(m, "start_with_reason", lambda: (False, "still broken"))
    _drive_watchdog_once(monkeypatch, m)
    m._watchdog_loop(0.0)  # restart fails -> error logged, no raise


def test_watchdog_restarts_stalled_pipeline(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_container_running", lambda: True)
    # process alive (recent=False True) but no fresh frames (recent=True False)
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: not require_recent_frames)
    posted = []
    _urlopen_map(monkeypatch, {"/restart": '{"ok": true}'})
    monkeypatch.setattr(vsm, "urlopen", lambda req, timeout=None: posted.append(req) or FakeResp("{}"))
    _drive_watchdog_once(monkeypatch, m)
    m._watchdog_loop(0.0)
    assert len(posted) == 1


def test_watchdog_relaunches_when_restart_request_fails(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_container_running", lambda: True)
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: not require_recent_frames)
    monkeypatch.setattr(vsm, "urlopen", lambda *a, **k: (_ for _ in ()).throw(URLError("no restart")))
    relaunched = []
    monkeypatch.setattr(m, "start_with_reason", lambda: (relaunched.append(1), (True, "ok"))[1])
    _drive_watchdog_once(monkeypatch, m)
    m._watchdog_loop(0.0)
    assert relaunched == [1]


def test_watchdog_logs_failed_relaunch_after_restart_request_fails(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_container_running", lambda: True)
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: not require_recent_frames)
    monkeypatch.setattr(vsm, "urlopen", lambda *a, **k: (_ for _ in ()).throw(URLError("no restart")))
    monkeypatch.setattr(m, "start_with_reason", lambda: (False, "relaunch broke"))
    _drive_watchdog_once(monkeypatch, m)
    m._watchdog_loop(0.0)  # both pipeline restart and relaunch fail -> error logged, no raise


def test_watchdog_passes_when_container_down(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_container_running", lambda: False)
    monkeypatch.setattr(m, "start_with_reason", lambda: (_ for _ in ()).throw(AssertionError("must not restart")))
    _drive_watchdog_once(monkeypatch, m)
    m._watchdog_loop(0.0)  # container down -> nothing to do


def test_watchdog_swallows_unexpected_error(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_container_running", lambda: (_ for _ in ()).throw(RuntimeError("boom")))
    _drive_watchdog_once(monkeypatch, m)
    m._watchdog_loop(0.0)  # error inside the loop body is caught


def test_watchdog_start_stop_lifecycle(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "_watchdog_loop", lambda initial_delay_s: m._watchdog_stop.wait())
    m.start_watchdog(initial_delay_s=0.0)
    assert m.start_watchdog(initial_delay_s=0.0) is None  # idempotent
    m.stop_watchdog()
    assert not (m._watchdog_thread and m._watchdog_thread.is_alive())


# --------------------------------------------------------------------------- #
# bridge HTTP API
# --------------------------------------------------------------------------- #


def test_switch_topic_not_running(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: False)
    assert m.switch_topic("/foo") is False


def test_switch_topic_success_and_failure(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: True)
    _urlopen_map(monkeypatch, {"/switch": '{"success": true}'})
    assert m.switch_topic("/zed/topic") is True
    _urlopen_map(monkeypatch, {"/switch": '{"success": false, "message": "bad topic"}'})
    assert m.switch_topic("/zed/topic") is False


def test_switch_topic_url_and_generic_errors(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: True)
    _urlopen_map(monkeypatch, {"/switch": URLError("conn refused")})
    assert m.switch_topic("/t") is False
    monkeypatch.setattr(vsm, "urlopen", lambda *a, **k: (_ for _ in ()).throw(ValueError("weird")))
    assert m.switch_topic("/t") is False


def test_set_overlay(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: False)
    assert m.set_overlay(True) is False

    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: True)
    _urlopen_map(monkeypatch, {"/overlay/enable": '{"success": true}'})
    assert m.set_overlay(True) is True
    _urlopen_map(monkeypatch, {"/overlay/disable": '{"success": false}'})
    assert m.set_overlay(False) is False
    monkeypatch.setattr(vsm, "urlopen", lambda *a, **k: (_ for _ in ()).throw(OSError("boom")))
    assert m.set_overlay(True) is False


def test_get_status_default_when_not_running(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: False)
    status = m.get_status()
    assert status.streaming is False and status.current_topic == ""


def test_get_status_default_when_no_data(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: True)
    monkeypatch.setattr(m, "_get_relay_status_data", lambda timeout_s=5.0: None)
    assert m.get_status().streaming is False


def test_get_status_populated_from_bridge(monkeypatch):
    m = VideoStreamManager()
    monkeypatch.setattr(m, "is_relay_running", lambda require_recent_frames=False: True)
    monkeypatch.setattr(
        m,
        "_get_relay_status_data",
        lambda timeout_s=5.0: {
            "streaming": True,
            "source_topic": "/zed/x",
            "fps": 14.9,
            "frame_count": 100,
            "error_count": 1,
            "dropped_count": 2,
        },
    )
    s = m.get_status()
    assert s.streaming is True and s.current_topic == "/zed/x" and s.fps == 14.9 and s.frame_count == 100
