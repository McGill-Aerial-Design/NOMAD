# SPDX-License-Identifier: Apache-2.0
"""Record snapshots wait for live replacements but never repair crash evidence."""

import ctypes
import json
import os
import subprocess
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest

from scripts.release import lifecycle, storage


def load_kernel(win_dll):
    from ctypes import wintypes

    kernel = win_dll("kernel32", use_last_error=True)
    kernel.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    kernel.WaitForSingleObject.restype = wintypes.DWORD
    kernel.ReplaceFileW.argtypes = [
        wintypes.LPCWSTR,
        wintypes.LPCWSTR,
        wintypes.LPCWSTR,
        wintypes.DWORD,
        ctypes.c_void_p,
        ctypes.c_void_p,
    ]
    kernel.ReplaceFileW.restype = wintypes.BOOL
    return kernel


class KernelProxy:
    def __init__(self, kernel, **overrides):
        self.kernel = kernel
        self.overrides = overrides

    def __getattr__(self, name):
        return self.overrides.get(name, getattr(self.kernel, name))


def observe_reader_wait(kernel, observed, blocked):
    def wait(handle, timeout):
        if threading.current_thread().name.startswith("record-reader"):
            result = kernel.WaitForSingleObject(handle, 0)
            if result == 258:
                blocked.set()
                observed.set()
            else:
                return result
        return kernel.WaitForSingleObject(handle, timeout)

    return wait


def pause_replacement(kernel, gap, finish):
    def replace(primary, temporary, backup, *arguments):
        # Control the documented rename gap; publication still uses native ReplaceFileW.
        os.replace(primary, backup)
        gap.set()
        try:
            assert finish.wait(10), "reader did not release the controlled replacement"
        finally:
            os.replace(backup, primary)
        return kernel.ReplaceFileW(primary, temporary, backup, *arguments)

    return replace


def assert_reader_waits(reader, observed, blocked):
    reader.add_done_callback(lambda _: observed.set())
    assert observed.wait(10), "reader neither acquired nor waited for the record guard"
    if reader.done():
        reader.result()  # Propagate false recovery classification from the unfixed tree.
    assert blocked.is_set(), "reader must wait while the primary is transiently absent"


def read_record(operation, deployment, path):
    read = {
        "read": lambda: storage.read_json(path),
        "exists": lambda: storage.record_exists(path),
        "status": deployment.status,
        "preflight": lambda: storage.require_record_recovery(path),
    }[operation]
    return read()


@pytest.mark.skipif(os.name != "nt", reason="Windows native replacement and kernel ownership")
@pytest.mark.parametrize("operation", ["read", "exists", "status", "preflight"])
def test_reader_waits_for_live_windows_replacement(tmp_path, monkeypatch, operation):
    deployment = lifecycle.Deployment(tmp_path, "router")
    deployment.root.mkdir()
    path = deployment.record
    a = {"schema_version": 1, "component": "router", "version": "A"}
    b = {**a, "version": "B"}
    storage.write_json(path, a)
    gap, finish, observed, blocked = (threading.Event() for _ in range(4))
    win_dll = ctypes.WinDLL
    kernel = load_kernel(win_dll)
    proxy = KernelProxy(
        kernel,
        ReplaceFileW=pause_replacement(kernel, gap, finish),
        WaitForSingleObject=observe_reader_wait(kernel, observed, blocked),
    )
    monkeypatch.setattr(
        storage.ctypes,
        "WinDLL",
        lambda library, **kwargs: proxy if library == "kernel32" else win_dll(library, **kwargs),
    )
    with storage.open_record(path) as snapshot, ThreadPoolExecutor(1) as writer:
        with ThreadPoolExecutor(1, thread_name_prefix="record-reader") as readers:
            writing = writer.submit(storage.write_json, path, b)
            try:
                assert gap.wait(10), "writer did not reach the controlled rename gap"
                assert not path.exists()
                assert json.loads(path.with_name(".deployment.json.previous").read_text()) == a
                reading = readers.submit(read_record, operation, deployment, path)
                assert_reader_waits(reading, observed, blocked)
            finally:
                finish.set()
            writing.result(timeout=10)
            assert reading.result(timeout=10) == {"read": b, "exists": True, "status": b, "preflight": None}[operation]
        assert json.loads(snapshot.read()) == a
    assert storage.read_json(path) == b
    assert not path.with_name(".deployment.json.previous").exists()


def start_interrupted_writer(path):
    code = """
import os, sys
from pathlib import Path
from scripts.release import storage
path = Path(sys.argv[1])
with storage.record_guard(path):
    os.replace(path, path.with_name('.' + path.name + '.previous'))
    print('gap', flush=True)
    sys.stdin.buffer.read(1)
    os._exit(17)
"""
    return subprocess.Popen(
        [sys.executable, "-c", code, str(path)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        cwd=Path(__file__).resolve().parents[1],
    )


def wait_writer_gap(child):
    with ThreadPoolExecutor(1) as handshake:
        reading = handshake.submit(child.stdout.readline)
        try:
            message = reading.result(timeout=10)
        except TimeoutError:
            child.kill()
            child.wait(timeout=10)
            raise AssertionError("writer did not reach the interrupted record boundary") from None
    assert message.strip() == "gap", "child did not hold the interrupted record guard"


@pytest.mark.skipif(os.name != "nt", reason="Windows releases abandoned interprocess mutex ownership")
def test_dead_windows_writer_requires_explicit_recovery(tmp_path, monkeypatch):
    path = tmp_path / "process.json"
    storage.write_json(path, {"version": "A", "pending": True})
    original = path.read_bytes()
    observed, blocked = threading.Event(), threading.Event()
    win_dll = ctypes.WinDLL
    kernel = load_kernel(win_dll)
    proxy = KernelProxy(kernel, WaitForSingleObject=observe_reader_wait(kernel, observed, blocked))
    child = start_interrupted_writer(path)
    try:
        wait_writer_gap(child)
        monkeypatch.setattr(
            storage.ctypes,
            "WinDLL",
            lambda library, **kwargs: proxy if library == "kernel32" else win_dll(library, **kwargs),
        )
        with ThreadPoolExecutor(1, thread_name_prefix="record-reader") as readers:
            reading = readers.submit(storage.read_json, path)
            try:
                assert_reader_waits(reading, observed, blocked)
            finally:
                child.stdin.write("x")
                child.stdin.flush()
            assert child.wait(timeout=10) == 17
            with pytest.raises(storage.RecordRecoveryRequired, match="operator repair"):
                reading.result(timeout=10)
        backup = path.with_name(".process.json.previous")
        assert not path.exists()
        assert backup.read_bytes() == original
        with pytest.raises(storage.RecordRecoveryRequired, match="operator repair"):
            storage.write_json(path, {"pending": False})
        assert backup.read_bytes() == original
    finally:
        if child.poll() is None:
            child.kill()
        child.communicate(timeout=10)


@pytest.mark.parametrize("primary", [None, '{"pending":true}', "{", "[]", '{"pending":true,"pending":false}'])
def test_leftover_backup_preserves_recovery_invariant(tmp_path, primary):
    path = tmp_path / "deployment.json"
    backup = path.with_name(".deployment.json.previous")
    backup.write_text('{"pending":true,"version":"A"}')
    if primary is not None:
        path.write_text(primary)
    before = backup.read_bytes()
    if primary == '{"pending":true}':
        assert storage.read_json(path) == {"pending": True}
    else:
        with pytest.raises((storage.RecordRecoveryRequired, ValueError)):
            storage.read_json(path)
    with pytest.raises(storage.RecordRecoveryRequired, match="operator repair"):
        storage.write_json(path, {"pending": False})
    assert backup.read_bytes() == before
    assert (path.read_text() if path.exists() else None) == primary


@pytest.mark.parametrize("primary", [None, "{", "[]", '{"a":1,"a":2}'])
def test_invalid_primary_without_backup_fails_closed(tmp_path, primary):
    path = tmp_path / "process.json"
    if primary is not None:
        path.write_text(primary)
    with pytest.raises((OSError, ValueError)):
        storage.read_json(path)
    assert (path.read_text() if path.exists() else None) == primary
