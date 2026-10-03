# SPDX-License-Identifier: Apache-2.0
"""Qualify versioned release transactions with real runtime/router child processes.

Fixtures deliberately use synthetic development identities, never official releases.
Two payload directories must contain independently built, distinguishable executables.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import tempfile
import zipfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ground_router_smoke import config_for
from mavsdk_peer import VehiclePeer
from release_process_fixture import RouterProcess, RuntimeProcess
from release_supervisor_fixture import FixtureSupervisor
from runtime_ipc_smoke import free_port, require
from runtime_lifecycle_fixture import deployment
from runtime_lifecycle_qualification import (
    admit,
    execute_servo,
    reject_old_context,
    status,
    verify_history,
    wait_for_vehicle,
)
from verify_core_package import REQUIRED_FILES

from scripts.release.lifecycle import Deployment
from scripts.release.manifest import EXPECTED, digest_file


def create_component(directory: Path, payload: Path | None, key: tuple, shared: dict, label: str) -> dict:
    name, target, architecture = key
    source = shared["source_sha"]
    entry = {
        "name": name,
        "platform": target,
        "architecture": architecture,
        "version": "0.0.0-dev." + source,
        "filename": f"{name}-{target}-{label}.zip",
        "protocol_versions": {"router_management": 1} if name == "router" else {"runtime_ipc": 1},
    }
    if name == "plugin":
        entry["mission_planner_target"] = "1.3.83"
    package = directory / entry["filename"]
    files = (
        sorted(path for path in payload.rglob("*") if path.is_file() and path.name != "package-identity.json")
        if payload
        else []
    )
    essentials = {
        "plugin": ["NOMADPlugin.dll"],
        "router": ["nomad-link-router.exe", "Nomad.LinkRouter.dll"],
        "core": ["bin/nomad.exe", "bin/nomad-runtime.exe"]
        if target == "windows"
        else ["bin/nomad", "bin/nomad-runtime"],
    }
    essentials["core"].extend(path.as_posix() for path in REQUIRED_FILES)
    entry["required_files"] = ["package-identity.json"] + (
        [path.relative_to(payload).as_posix() for path in files] if files else essentials[name]
    )
    with zipfile.ZipFile(package, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("package-identity.json", json.dumps({**shared, **entry}))
        for path in files:
            archive.write(path, path.relative_to(payload).as_posix())
        if not files:
            for placeholder in essentials[name]:
                archive.writestr(placeholder, "nonselected fixture component; never executed")
    entry["sha256"] = digest_file(package)
    return entry


def create_fixture(directory: Path, payload: Path, component: str, label: str) -> tuple[Path, Path]:
    """Wrap actual test binaries in complete synthetic release metadata."""
    source = ("a" if label == "A" else "b") * 40
    shared = {
        "schema_version": 1,
        "release_version": "dev-" + source,
        "source_sha": source,
        "mavsdk_sha": "c" * 40,
        "official": False,
    }
    platform = "windows" if os.name == "nt" else "linux"
    entries, selected = [], None
    for key in sorted(EXPECTED):
        selected_payload = payload if key[:2] == (component, platform) else None
        entry = create_component(directory, selected_payload, key, shared, label)
        entries.append(entry)
        if selected_payload is not None:
            selected = directory / entry["filename"]
    manifest = directory / f"manifest-{component}-{label}.json"
    manifest.write_text(json.dumps({**shared, "components": entries}), encoding="utf-8")
    require(selected is not None, "fixture selects a supported component platform")
    return manifest, selected


def stage_release(engine: Deployment, directory: Path, payload: Path, component: str, label: str) -> dict:
    platform = "windows" if os.name == "nt" else "linux"
    manifest, package = create_fixture(directory, payload, component, label)
    return engine.stage(manifest, package, platform, "x86_64")


def verify_transition(engine: Deployment, adapter, record: dict) -> None:
    engine.activate(record["release_version"], adapter)
    require(
        engine.status()["active"]["artifact_digest"] == record["artifact_digest"], "active package is exact candidate"
    )
    adapter.health(record)


def verify_failed_candidate(engine: Deployment, adapter, records: list[dict]) -> None:
    adapter.fail_version = records[1]["release_version"]
    try:
        engine.activate(records[1]["release_version"], adapter)
    except RuntimeError:
        pass
    else:
        raise AssertionError("intentionally unhealthy B was accepted")
    adapter.fail_version = None
    require(engine.status()["active"]["artifact_digest"] == records[0]["artifact_digest"], "failed B restores exact A")
    adapter.health(records[0])


def verify_runtime_start_failure(engine, adapter, records) -> None:
    original = adapter.config.read_bytes()
    adapter.fail_start = True
    try:
        engine.activate(records[1]["release_version"], adapter)
    except RuntimeError:
        pass
    else:
        raise AssertionError("runtime candidate that exited during startup was committed")
    require(engine.status()["active"] == records[0], "startup failure automatically restores exact runtime A")
    require(engine.status()["pending"] is None, "runtime startup recovery completes its journal")
    require(adapter.config.read_bytes() == original, "runtime startup fault preserves protected configuration")
    adapter.health(records[0])


def verify_corrupt_candidate(engine: Deployment, adapter, root: Path, payload: Path, component: str) -> None:
    manifest, package = create_fixture(root, payload, component, "B")
    with package.open("ab") as stream:
        stream.write(b"intentional-corruption-after-manifest")
    active = engine.status()["active"]
    active_pid = adapter.child.pid
    platform = "windows" if os.name == "nt" else "linux"
    try:
        engine.stage(manifest, package, platform, "x86_64")
    except ValueError:
        pass
    else:
        raise AssertionError("corrupt candidate was staged")
    require(engine.status()["active"] == active, "corrupt candidate leaves active release record untouched")
    require(
        adapter.child.pid == active_pid and adapter.child.poll() is None, "corrupt package leaves active process alive"
    )


def exercise_core(
    engine: Deployment,
    adapter: RuntimeProcess,
    peer: VehiclePeer,
    ipc: int,
    root: Path,
    payloads: list[Path],
    records: list[dict],
) -> list[str]:
    incarnations = []
    verify_transition(engine, adapter, records[0])
    wait_for_vehicle(ipc)
    incarnations.append(status(ipc)["runtime_incarnation"])
    admit(ipc)
    old = execute_servo(ipc, peer, 1, "release-A-authority")
    records.append(stage_release(engine, root, payloads[1], "core", "B"))
    require(status(ipc)["runtime_incarnation"] == incarnations[0], "staging B leaves running A undisturbed")
    require(status(ipc)["authority_owner"] == "runtime-smoke", "staging B does not revoke A admission")
    require(
        digest_file(adapter.executable(records[0])) != digest_file(adapter.executable(records[1])),
        "fixture runtime binaries A and B have distinguishable bytes",
    )
    verify_transition(engine, adapter, records[1])
    wait_for_vehicle(ipc)
    incarnations.append(status(ipc)["runtime_incarnation"])
    require(incarnations[-1] != incarnations[0], "B has a new runtime incarnation")
    reject_old_context(ipc, old, peer, 1)
    admit(ipc)
    old_b = execute_servo(ipc, peer, 2, "release-B-authority")
    engine.rollback(adapter)
    wait_for_vehicle(ipc)
    incarnations.append(status(ipc)["runtime_incarnation"])
    require(len(set(incarnations)) == 3, "rollback starts another fresh incarnation")
    reject_old_context(ipc, old_b, peer, 2)
    require(engine.status()["active"]["artifact_digest"] == records[0]["artifact_digest"], "rollback restores exact A")
    verify_failed_candidate(engine, adapter, records)
    verify_runtime_start_failure(engine, adapter, records)
    verify_corrupt_candidate(engine, adapter, root, payloads[1], "core")
    return incarnations


def verify_core(root: Path, payloads: list[Path]) -> None:
    root.mkdir()
    udp, ipc = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM)
    config, _ = deployment(root, udp, ipc)
    preserved = {path: path.read_bytes() for path in (config, root / "credentials.json")}
    engine = Deployment(root / "versions", "core")
    peer = VehiclePeer(udp, 1)
    records = [stage_release(engine, root, payloads[0], "core", "A")]
    adapter = RuntimeProcess(config, ipc, root)
    peer.start()
    try:
        incarnations = exercise_core(engine, adapter, peer, ipc, root, payloads, records)
        for path, before in preserved.items():
            require(path.read_bytes() == before, "operator configuration and credential map remain byte-identical")
    finally:
        adapter.close()
        peer.stop()
    verify_history(root, incarnations, clean=True)


def verify_router(root: Path, payloads: list[Path]) -> None:
    root.mkdir()
    ports = [free_port(socket.SOCK_DGRAM) for _ in range(5)]
    management = free_port(socket.SOCK_STREAM)
    config = root / "router.json"
    settings = config_for(ports, [free_port(socket.SOCK_DGRAM) for _ in range(2)], management)
    for link in settings["Links"]:
        link["BindAddress"] = "127.0.0.1"
    config.write_text(json.dumps(settings))
    before = config.read_bytes()
    engine = Deployment(root / "versions", "router")
    records = [stage_release(engine, root, payloads[0], "router", "A")]
    adapter = RouterProcess(config, management, root)
    try:
        verify_transition(engine, adapter, records[0])
        active_pid = adapter.child.pid
        records.append(stage_release(engine, root, payloads[1], "router", "B"))
        require(adapter.child.pid == active_pid and adapter.child.poll() is None, "staging router B leaves A alive")
        require(
            digest_file(adapter.executable(records[0])) != digest_file(adapter.executable(records[1])),
            "fixture router executables A and B have distinguishable bytes",
        )
        verify_transition(engine, adapter, records[1])
        engine.rollback(adapter)
        adapter.health(records[0])
        require(
            engine.status()["active"]["artifact_digest"] == records[0]["artifact_digest"], "router restores exact A"
        )
        verify_failed_candidate(engine, adapter, records)
        verify_corrupt_candidate(engine, adapter, root, payloads[1], "router")
        require(config.read_bytes() == before, "router external topology remains byte-identical")
    finally:
        adapter.close()


def verify_router_supervisor(root: Path, payloads: list[Path]) -> None:
    root.mkdir()
    management = free_port(socket.SOCK_STREAM)
    config = root / "router.json"
    settings = config_for(
        [free_port(socket.SOCK_DGRAM) for _ in range(5)], [free_port(socket.SOCK_DGRAM) for _ in range(2)], management
    )
    for link in settings["Links"]:
        link["BindAddress"] = "127.0.0.1"
    config.write_text(json.dumps(settings))
    deployment_root = root / "versions"
    launcher = Path(__file__).with_name("release_router_launcher.py")
    command = [sys.executable, str(launcher), "--root", str(deployment_root), "--config", str(config)]
    adapter = FixtureSupervisor(deployment_root, config, management, command)
    engine = Deployment(deployment_root, "router")
    records = [stage_release(engine, root, payloads[0], "router", "A")]
    try:
        verify_transition(engine, adapter, records[0])
        records.append(stage_release(engine, root, payloads[1], "router", "B"))
        adapter.health(records[0])
        verify_transition(engine, adapter, records[1])
        engine.rollback(adapter)
        adapter.health(records[0])
        verify_failed_candidate(engine, adapter, records)
        verify_supervisor_start_failure(engine, adapter, records)
        require(adapter.current_matches(engine.status()["active"]), "production router pointer matches recovered A")
    finally:
        adapter.stop()
    require(json.loads(adapter.process.read_text())["status"] == "stopped", "supervisor verified graceful child exit")


def verify_supervisor_start_failure(engine, adapter, records) -> None:
    original = adapter.config.read_bytes()
    adapter.fail_start = True
    try:
        engine.activate(records[1]["release_version"], adapter)
    except (TimeoutError, RuntimeError):
        pass
    else:
        raise AssertionError("router candidate that exited during startup was committed")
    require(engine.status()["pending"] is None, "startup failure restoration completes its journal")
    require(adapter.config.read_bytes() == original, "startup failure preserves external operator topology")
    adapter.health(records[0])
    require(engine.status()["active"] == records[0], "startup failure restores verified live router A")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--core-a", type=Path)
    parser.add_argument("--core-b", type=Path)
    parser.add_argument("--router-a", type=Path)
    parser.add_argument("--router-b", type=Path)
    args = parser.parse_args()
    if bool(args.router_a) != bool(args.router_b):
        parser.error("router A and B must both be supplied")
    if bool(args.core_a) != bool(args.core_b) or not (args.core_a or args.router_a):
        parser.error("supply both core fixtures, both router fixtures, or both pairs")
    with tempfile.TemporaryDirectory(prefix="nomad-release-process-") as temporary:
        root = Path(temporary)
        if args.core_a:
            verify_core(root / "core", [args.core_a, args.core_b])
        if args.router_a:
            verify_router(root / "router", [args.router_a, args.router_b])
            verify_router_supervisor(root / "router-supervised", [args.router_a, args.router_b])
    print("software-only release process qualification passed; no privileged service registration was exercised")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
