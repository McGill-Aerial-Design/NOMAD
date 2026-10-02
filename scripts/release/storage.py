# SPDX-License-Identifier: Apache-2.0
"""Protected deployment records, exclusive host mutation, and safe archive extraction."""

from __future__ import annotations

import contextlib
import hashlib
import json
import os
import stat
import tarfile
import tempfile
import zipfile
from pathlib import Path, PurePosixPath

from scripts.release.manifest import parse_json


def digest(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def reject_links(path: Path) -> None:
    for item in (path, *path.parents):
        if item.is_symlink() or (item.exists() and getattr(item.lstat(), "st_file_attributes", 0) & 1024):
            raise ValueError("deployment paths must not contain links/reparse points")


def sync_directory(path: Path) -> None:
    if os.name != "nt":
        descriptor = os.open(path, os.O_RDONLY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def sync_tree(root: Path) -> None:
    """Make extracted payload bytes durable before publishing their directory."""
    directories = [root]
    for path in root.rglob("*"):
        reject_links(path)
        if path.is_dir():
            directories.append(path)
        elif path.is_file():
            with path.open("r+b") as stream:
                stream.flush()
                os.fsync(stream.fileno())
    for directory in sorted(directories, key=lambda path: len(path.parts), reverse=True):
        sync_directory(directory)


def write_json(path: Path, value: dict) -> None:
    reject_links(path)
    descriptor, temporary = tempfile.mkstemp(prefix=".record-", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, sort_keys=True, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        sync_directory(path.parent)
    finally:
        Path(temporary).unlink(missing_ok=True)


def read_json(path: Path) -> dict:
    reject_links(path)
    value = parse_json(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("deployment record must be an object")
    return value


@contextlib.contextmanager
def lock(root: Path):
    reject_links(root)
    root.mkdir(parents=True, exist_ok=True, mode=0o755)
    path = root / ".lock"
    reject_links(path)
    descriptor = os.open(path, os.O_RDWR | os.O_CREAT, 0o600)
    try:
        if os.name == "nt":
            import msvcrt

            if os.fstat(descriptor).st_size == 0:
                os.write(descriptor, b"0")
            os.lseek(descriptor, 0, os.SEEK_SET)
            msvcrt.locking(descriptor, msvcrt.LK_NBLCK, 1)
        else:
            import fcntl

            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield
    finally:
        os.close(descriptor)


def member_path(name: str) -> Path:
    member = PurePosixPath(name)
    if not name or "\\" in name or ":" in name or member.is_absolute() or ".." in member.parts:
        raise ValueError("unsafe archive member")
    if any(part.rstrip(". ") != part for part in member.parts):
        raise ValueError("ambiguous archive member")
    if any(
        part.upper().split(".")[0]
        in {"CON", "PRN", "AUX", "NUL", *(f"COM{i}" for i in range(10)), *(f"LPT{i}" for i in range(10))}
        for part in member.parts
    ):
        raise ValueError("reserved archive member")
    return Path(*member.parts)


def extract(package: Path, destination: Path) -> None:
    """Extract regular files only; bounded size and case collisions are rejected."""
    seen = set()
    total = 0
    if zipfile.is_zipfile(package):
        with zipfile.ZipFile(package) as archive:
            for member in archive.infolist():
                mode = member.external_attr >> 16
                if stat.S_IFMT(mode) not in (0, stat.S_IFREG, stat.S_IFDIR):
                    raise ValueError("archive contains special files or links")
                total = extract_member(member.filename, member.file_size, total, seen)
                path = destination / member_path(member.filename)
                if member.is_dir():
                    path.mkdir(parents=True, exist_ok=True)
                else:
                    path.parent.mkdir(parents=True, exist_ok=True)
                    with archive.open(member) as source, path.open("xb") as target:
                        copy_stream(source, target)
                    path.chmod(0o755 if mode & 0o111 else 0o644)
        return
    with tarfile.open(package, "r:gz") as archive:
        for member in archive:
            if not (member.isfile() or member.isdir()):
                raise ValueError("archive contains special files or links")
            total = extract_member(member.name, member.size, total, seen)
            path = destination / member_path(member.name)
            if member.isdir():
                path.mkdir(parents=True, exist_ok=True)
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                with archive.extractfile(member) as source, path.open("xb") as target:
                    copy_stream(source, target)
                path.chmod(0o755 if member.mode & 0o111 else 0o644)


def extract_member(name: str, size: int, total: int, seen: set) -> int:
    key = member_path(name).as_posix().casefold()
    if key in seen or len(seen) >= 10000 or total + size > 1024**3:
        raise ValueError("duplicate archive path or archive limit exceeded")
    seen.add(key)
    return total + size


def copy_stream(source, target) -> None:
    while block := source.read(1024 * 1024):
        target.write(block)


def payload_hashes(path: Path) -> dict[str, str]:
    reject_links(path)
    result = {}
    for item in sorted(path.rglob("*")):
        reject_links(item)
        if item.is_file():
            result[item.relative_to(path).as_posix()] = digest(item)
    return result
