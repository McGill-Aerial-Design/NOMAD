# SPDX-License-Identifier: Apache-2.0
"""Protected deployment records, exclusive host mutation, and safe archive extraction."""

from __future__ import annotations

import contextlib
import ctypes
import hashlib
import json
import os
import stat
import tarfile
import tempfile
import zipfile
from pathlib import Path, PurePosixPath

from scripts.release.manifest import parse_json


class RecordRecoveryRequired(RuntimeError):
    """A retained prior record requires explicit storage repair."""


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


def create_record_mutex(kernel, name: str):
    """Share synchronization rights across authorized service/operator accounts."""
    from ctypes import wintypes

    class SecurityAttributes(ctypes.Structure):
        _fields_ = [("length", wintypes.DWORD), ("descriptor", ctypes.c_void_p), ("inherit", wintypes.BOOL)]

    security = ctypes.WinDLL("advapi32", use_last_error=True)
    convert = security.ConvertStringSecurityDescriptorToSecurityDescriptorW
    convert.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, ctypes.POINTER(ctypes.c_void_p), ctypes.c_void_p]
    convert.restype = wintypes.BOOL
    descriptor = ctypes.c_void_p()
    # Mutex access does not grant record access; allow only wait/release, not DACL changes.
    if not convert("D:(A;;0x00100001;;;AU)(A;;0x00100001;;;SY)", 1, ctypes.byref(descriptor), None):
        raise ctypes.WinError(ctypes.get_last_error())
    attributes = SecurityAttributes(ctypes.sizeof(SecurityAttributes), descriptor, False)
    kernel.CreateMutexExW.argtypes = [
        ctypes.POINTER(SecurityAttributes),
        wintypes.LPCWSTR,
        wintypes.DWORD,
        wintypes.DWORD,
    ]
    kernel.CreateMutexExW.restype = wintypes.HANDLE
    kernel.LocalFree.argtypes = [ctypes.c_void_p]
    kernel.LocalFree.restype = ctypes.c_void_p
    try:
        handle = kernel.CreateMutexExW(ctypes.byref(attributes), name, 0, 0x00100001)
        if not handle:
            raise ctypes.WinError(ctypes.get_last_error())
        return handle
    finally:
        kernel.LocalFree(descriptor)


@contextlib.contextmanager
def record_guard(path: Path):
    """Exclude live Windows replacement; kernel ownership ends if the writer dies."""
    if os.name != "nt":
        yield
        return
    from ctypes import wintypes

    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    kernel.WaitForSingleObject.restype = wintypes.DWORD
    kernel.ReleaseMutex.argtypes = [wintypes.HANDLE]
    kernel.ReleaseMutex.restype = wintypes.BOOL
    kernel.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel.CloseHandle.restype = wintypes.BOOL
    identity = os.path.normcase(str(path.resolve())).encode("utf-8")
    name = "Global\\NOMAD-record-" + hashlib.sha256(identity).hexdigest()
    handle = create_record_mutex(kernel, name)
    try:
        result = kernel.WaitForSingleObject(handle, 30000)
        if result == 258:
            raise TimeoutError("deployment record replacement did not finish")
        if result not in (0, 128):
            raise ctypes.WinError(ctypes.get_last_error())
        try:
            # Abandoned ownership still requires inspecting the records left on disk.
            yield
        finally:
            kernel.ReleaseMutex(handle)
    finally:
        kernel.CloseHandle(handle)


def write_json(path: Path, value: dict) -> None:
    with record_guard(path):
        reject_links(path)
        require_record_recovery(path, writing=True)
        descriptor, temporary = tempfile.mkstemp(prefix=".record-", dir=path.parent)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(value, stream, sort_keys=True, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            replace_record(Path(temporary), path)
            sync_directory(path.parent)
        finally:
            Path(temporary).unlink(missing_ok=True)


def replace_record(temporary: Path, path: Path) -> None:
    if os.name != "nt" or not path.exists():
        os.replace(temporary, path)
        return
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    replace = kernel.ReplaceFileW
    replace.argtypes = [
        ctypes.c_wchar_p,
        ctypes.c_wchar_p,
        ctypes.c_wchar_p,
        ctypes.c_uint,
        ctypes.c_void_p,
        ctypes.c_void_p,
    ]
    replace.restype = ctypes.c_int
    backup = path.with_name("." + path.name + ".previous")
    try:
        if not replace(str(path), str(temporary), str(backup), 0, None, None):
            error = ctypes.WinError(ctypes.get_last_error())
            if not path.exists() and backup.exists():
                os.replace(backup, path)
            raise error
    finally:
        if path.exists():
            backup.unlink(missing_ok=True)


def read_json(path: Path) -> dict:
    with record_guard(path):
        reject_links(path)
        require_record_recovery(path)
        stream = open_record(path)
    with stream:
        value = parse_json(stream.read())
    if not isinstance(value, dict):
        raise ValueError("deployment record must be an object")
    return value


def record_exists(path: Path) -> bool:
    """Probe only stable record state, including when absence means first deployment."""
    with record_guard(path):
        reject_links(path)
        require_record_recovery(path)
        return path.exists()


def require_record_recovery(path: Path, writing: bool = False) -> None:
    with record_guard(path):
        backup = path.with_name("." + path.name + ".previous")
        reject_links(backup)
        if backup.exists() and (writing or not path.exists()):
            raise RecordRecoveryRequired(f"interrupted record replacement requires operator repair: {backup.name}")


def open_record(path: Path):
    """Readers retain one complete snapshot without blocking Windows atomic replacement."""
    if os.name != "nt":
        return path.open("r", encoding="utf-8")
    import msvcrt
    from ctypes import wintypes

    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    create = kernel.CreateFileW
    create.argtypes = [
        wintypes.LPCWSTR,
        wintypes.DWORD,
        wintypes.DWORD,
        ctypes.c_void_p,
        wintypes.DWORD,
        wintypes.DWORD,
        wintypes.HANDLE,
    ]
    create.restype = wintypes.HANDLE
    handle = create(str(path), 0x80000000, 7, None, 3, 0x80, None)
    if handle == ctypes.c_void_p(-1).value:
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        descriptor = msvcrt.open_osfhandle(handle, os.O_RDONLY | os.O_BINARY)
    except Exception:
        kernel.CloseHandle.argtypes = [wintypes.HANDLE]
        kernel.CloseHandle(handle)
        raise
    return os.fdopen(descriptor, "r", encoding="utf-8")


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
