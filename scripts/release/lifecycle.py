# SPDX-License-Identifier: Apache-2.0
"""NOMAD component transactions. Operator state never enters release directories."""

from __future__ import annotations

import os
import re
import shutil
import tempfile
from datetime import datetime, timezone
from pathlib import Path

from scripts.release import manifest, storage


def timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


class Deployment:
    def __init__(self, root: Path, component: str):
        if component not in {"core", "router", "plugin"}:
            raise ValueError("unsupported NOMAD component")
        storage.reject_links(root.absolute())
        self.root = root.absolute() / component
        self.component = component
        self.releases = self.root / "releases"
        self.record = self.root / "deployment.json"

    def status(self) -> dict:
        if not storage.record_exists(self.record):
            return {
                "schema_version": 1,
                "component": self.component,
                "status": "staged",
                "active": None,
                "previous": None,
                "pending": None,
            }
        value = storage.read_json(self.record)
        if value.get("schema_version") != 1 or value.get("component") != self.component:
            raise ValueError("incompatible deployment record")
        return value

    def release_path(self, version: str) -> Path:
        if not re.fullmatch(r"v\d+\.\d+\.\d+|dev-[0-9a-f]{40}", version):
            raise ValueError("invalid release directory identity")
        return self.releases / version

    def get_release(self, version: str) -> dict:
        directory = self.release_path(version)
        entry = storage.read_json(directory / "record.json")
        if entry.get("release_version") != version or entry.get("name") != self.component:
            raise ValueError("staged identity mismatch")
        if Path(entry.get("install_path", "")) != directory / "payload":
            raise ValueError("staged payload path mismatch")
        storage.reject_links(directory / "package")
        if storage.digest(directory / "package") != entry["artifact_digest"]:
            raise ValueError("staged package digest mismatch")
        if storage.payload_hashes(directory / "payload") != entry["payload_sha256"]:
            raise ValueError("staged payload has changed")
        return entry

    def stage(self, manifest_path: Path, package: Path, platform: str, architecture: str = "x86_64") -> dict:
        document = manifest.load_manifest(manifest_path)
        entry = manifest.select_component(document, self.component, platform, architecture)
        manifest.verify_package(entry, package)
        with storage.lock(self.root):
            storage.require_record_recovery(self.record, writing=True)
            self.releases.mkdir(exist_ok=True, mode=0o755)
            final = self.release_path(document["release_version"])
            if final.exists():
                existing = self.get_release(document["release_version"])
                if existing["artifact_digest"] != entry["sha256"]:
                    raise ValueError("release already exists with different bytes")
                return existing
            return self.stage_verified(document, entry, package, final)

    def stage_verified(self, document: dict, entry: dict, package: Path, final: Path) -> dict:
        temporary = Path(tempfile.mkdtemp(prefix=".stage-", dir=self.releases))
        try:
            shutil.copyfile(package, temporary / "package")
            if storage.digest(temporary / "package") != entry["sha256"]:
                raise ValueError("package changed while staging")
            unpacked = temporary / "unpacked"
            unpacked.mkdir()
            storage.extract(temporary / "package", unpacked)
            payload = find_payload(unpacked)
            validate_payload(document, entry, payload)
            payload.rename(temporary / "payload")
            if unpacked.exists():
                unpacked.rmdir()
            result = {
                **entry,
                "release_version": document["release_version"],
                "source_sha": document["source_sha"],
                "mavsdk_sha": document["mavsdk_sha"],
                "official": document["official"],
                "source_dirty": document.get("source_dirty", False),
                "artifact_digest": entry["sha256"],
                "install_path": str(final / "payload"),
                "payload_sha256": storage.payload_hashes(temporary / "payload"),
                "staged_at": timestamp(),
                "status": "staged",
            }
            storage.write_json(temporary / "record.json", result)
            storage.sync_tree(temporary)
            protect_payload(temporary)
            os.rename(temporary, final)
            storage.sync_directory(self.releases)
            return result
        finally:
            if temporary.exists():
                remove_release(temporary)

    def adopt(self, version: str, adapter) -> dict:
        with storage.lock(self.root):
            storage.require_record_recovery(self.record, writing=True)
            state = self.status()
            if state.get("active") or state.get("pending"):
                raise ValueError("adoption requires an unmanaged, idle deployment")
            candidate = self.get_release(version)
            adapter.preflight(candidate)
            if not adapter.current_matches(candidate):
                raise ValueError("existing deployment does not match verified staged payload")
            adapter.health(candidate)
            state.update(active=candidate, status="active", activated_at=timestamp())
            storage.write_json(self.record, state)
            return state

    def activate(self, version: str, adapter) -> dict:
        with storage.lock(self.root):
            storage.require_record_recovery(self.record, writing=True)
            state = self.status()
            candidate = self.get_release(version)
            return self.transition(state, candidate, adapter, "active")

    def rollback(self, adapter) -> dict:
        with storage.lock(self.root):
            storage.require_record_recovery(self.record, writing=True)
            state = self.status()
            if not state.get("previous"):
                raise ValueError("previous release is unavailable")
            candidate = self.get_release(state["previous"]["release_version"])
            return self.transition(state, candidate, adapter, "rolled_back")

    def transition(self, state: dict, candidate: dict, adapter, success: str) -> dict:
        if state.get("pending"):
            raise ValueError("interrupted transition requires recover first")
        previous = state.get("active")
        if previous:
            self.get_release(previous["release_version"])
            if not adapter.current_matches(previous):
                raise ValueError("actual deployment differs from the recorded active release; reconcile it first")
        elif getattr(adapter, "requires_adoption", False) or adapter.has_unmanaged():
            raise ValueError("adopt the verified existing deployment before activation")
        adapter.preflight(candidate)
        state.update(status="activating", pending={"candidate": candidate, "restore": previous}, updated_at=timestamp())
        storage.write_json(self.record, state)
        try:
            adapter.stop()
            adapter.switch(candidate)
            adapter.start()
            adapter.health(candidate)
            committed = {
                **state,
                "active": candidate,
                "previous": previous,
                "status": success,
                "pending": None,
                "activated_at": timestamp(),
            }
            storage.write_json(self.record, committed)
            return committed
        except Exception:
            self.restore(state, previous, adapter)
            raise

    def restore(self, state: dict, previous: dict | None, adapter) -> dict:
        state.update(status="rollback_pending", updated_at=timestamp())
        # Persist the intent before any restoration. Failure leaves the original pending journal intact.
        storage.write_json(self.record, state)
        try:
            if previous:
                previous = self.get_release(previous["release_version"])
                adapter.preflight(previous)
            adapter.stop()
            adapter.switch(previous)
            if previous:
                adapter.start()
                adapter.health(previous)
            committed = {
                **state,
                "active": previous,
                "status": "rolled_back" if previous else "failed",
                "pending": None,
                "recovered_at": timestamp(),
            }
            storage.write_json(self.record, committed)
            return committed
        except Exception:
            # Do not clear pending or claim restoration when any restoration step failed.
            state.update(status="failed", updated_at=timestamp())
            storage.write_json(self.record, state)
            raise

    def recover(self, adapter) -> dict:
        with storage.lock(self.root):
            storage.require_record_recovery(self.record, writing=True)
            state = self.status()
            pending = state.get("pending")
            if not pending:
                return state
            return self.restore(state, pending["restore"], adapter)

    def cleanup(self, version: str) -> None:
        with storage.lock(self.root):
            storage.require_record_recovery(self.record, writing=True)
            state = self.status()
            protected = [state.get("active"), state.get("previous")]
            if state.get("pending"):
                protected.extend(state["pending"].values())
            if any(item and item["release_version"] == version for item in protected):
                raise ValueError("cannot delete active, previous or pending release")
            directory = self.release_path(version)
            self.get_release(version)
            remove_release(directory)


def find_payload(unpacked: Path) -> Path:
    if (unpacked / "package-identity.json").is_file():
        return unpacked
    children = list(unpacked.iterdir())
    if len(children) == 1 and children[0].is_dir() and (children[0] / "package-identity.json").is_file():
        return children[0]
    raise ValueError("package requires a single identified payload root")


def validate_payload(document: dict, entry: dict, payload: Path) -> None:
    identity = storage.read_json(payload / "package-identity.json")
    for key in ("release_version", "source_sha", "mavsdk_sha", "official", "source_dirty"):
        if identity.get(key) != document.get(key):
            raise ValueError("embedded release identity mismatch")
    for key in ("name", "version", "platform", "architecture", "protocol_versions", "mission_planner_target"):
        if identity.get(key) != entry.get(key):
            raise ValueError("embedded component identity mismatch")
    for name in entry["required_files"]:
        if not (payload / storage.member_path(name)).is_file():
            raise ValueError(f"missing required payload: {name}")
    forbidden = {"credentials.json", "runtime.json", "router.json", "nomad.env", "config.xml"}
    for path in payload.rglob("*"):
        if path.name.casefold() in forbidden or path.name.startswith("nomad-qualification"):
            raise ValueError("package contains operator state or qualification executable")


def protect_payload(directory: Path) -> None:
    if os.name == "nt":
        return  # Protected parent ACL is inherited; CLI checks operator ownership before mutation.
    for path in directory.rglob("*"):
        if path.is_file():
            path.chmod(0o555 if path.stat().st_mode & 0o111 else 0o444)
        elif path.is_dir():
            path.chmod(0o555)
    directory.chmod(0o555)


def remove_release(directory: Path) -> None:
    storage.reject_links(directory)
    for path in directory.rglob("*"):
        storage.reject_links(path)
        if os.name != "nt":
            path.chmod(0o700 if path.is_dir() else 0o600)
    if os.name != "nt":
        directory.chmod(0o700)
    shutil.rmtree(directory)
