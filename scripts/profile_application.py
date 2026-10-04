# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Stage the two profile targets and restore the environment on MP failure."""

from __future__ import annotations

import os
import shutil
import stat
import tempfile
from datetime import datetime
from pathlib import Path


def remove_temporary(path: Path) -> None:
    try:
        path.unlink(missing_ok=True)
    except PermissionError:
        # Windows forbids deleting a staged copy with the target's read-only bit.
        path.chmod(stat.S_IRUSR | stat.S_IWUSR)
        path.unlink(missing_ok=True)


def stage_file(path: Path, content: bytes) -> Path:
    """Create a private sibling file, keeping existing permission bits."""
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists():
            shutil.copymode(path, temporary)
        return temporary
    except BaseException:
        remove_temporary(temporary)
        raise


def _backup_env(path: Path) -> None:
    if not path.exists():
        return
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    descriptor, name = tempfile.mkstemp(prefix=f"nomad.env.bak.{timestamp}.", dir=path.parent)
    os.close(descriptor)
    backup = Path(name)
    try:
        shutil.copy2(path, backup)
    except OSError:
        remove_temporary(backup)
        raise
    print(f"[INFO] Backed up current config to {backup.name}")


def _restore_env(path: Path, original: Path | None) -> str:
    try:
        if original is None:
            path.unlink()
        else:
            original.replace(path)
    except OSError:
        return "[FAILED] env: changed; rollback failed; restore the env backup or remove newly created env before use"
    return "[SKIPPED] env: rolled back after Mission Planner commit failed"


def _commit_targets(env_path: Path, staged: list[Path], mp_path: Path | None, original: Path | None) -> bool:
    _backup_env(env_path)
    try:
        staged[0].replace(env_path)
    except OSError:
        print("[FAILED] env: atomic replacement failed; unchanged")
        print("[SKIPPED] mission_planner: env commit failed; unchanged")
        return False
    if mp_path is not None:
        try:
            staged[-1].replace(mp_path)
        except OSError:
            print(_restore_env(env_path, original))
            print("[FAILED] mission_planner: atomic replacement failed; unchanged")
            return False
    print("[APPLIED] env: profile settings loaded (existing credentials preserved)")
    if mp_path is None:
        print("[SKIPPED] mission_planner: config path unavailable; set NOMAD_MP_CONFIG to sync")
    else:
        print("[APPLIED] mission_planner: profile settings synced")
    return True


def apply_targets(env_path: Path, content: bytes, mp_config: tuple[Path, bytes] | None) -> bool:
    """Preflight both writes; rollback env if MP's final replacement fails."""
    print(f"[INFO] env target: {env_path}")
    if mp_config is not None:
        print(f"[INFO] mission_planner target: {mp_config[0]}")
    staged: list[Path] = []
    target = "env"
    try:
        original = stage_file(env_path, env_path.read_bytes()) if env_path.exists() else None
        if original is not None:
            staged.append(original)
        env_staged = stage_file(env_path, content)
        staged.insert(0, env_staged)
        mp_path = None
        if mp_config is not None:
            target = "mission_planner"
            mp_path, mp_content = mp_config
            if mp_path.resolve() == env_path.resolve():
                raise ValueError("Mission Planner and env targets must differ")
            staged.append(stage_file(mp_path, mp_content))
        target = "env"
        return _commit_targets(env_path, staged, mp_path, original)
    except (OSError, ValueError):
        print(f"[FAILED] {target}: preflight or backup failed; unchanged")
        other = "env" if target == "mission_planner" else "mission_planner"
        print(f"[SKIPPED] {other}: application aborted; unchanged")
        return False
    finally:
        for temporary in staged:
            remove_temporary(temporary)
