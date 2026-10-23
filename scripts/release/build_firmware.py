#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Build and qualify the pinned native firmware on Linux; never install or flash it."""

import argparse
import base64
import ctypes
import gzip
import hashlib
import importlib.metadata
import json
import os
import re
import shutil
import signal
import stat
import subprocess
import sys
import tarfile
import time
import zlib
from pathlib import Path, PurePosixPath

REVISION = "fb773567896014da007bca58358704d4a0d8516f"
REPOSITORY = "https://github.com/ArduPilot/ardupilot.git"
COHORTS = (
    "test.Copter.Terminate",
    "test.QuadPlane.Terminate",
    "test.Plane.AdvancedFailsafe",
    "test.Plane.MAV_CMD_DO_FLIGHTTERMINATION",
    "test.Copter.MAV_CMD_DO_FLIGHTTERMINATION",
    "test.Copter.RCOverridesNoRCReceiver",
    "test.Copter.RCProtocolFailsafe",
)
GUARDS = {
    "no-afs": (["--disable-ADVANCED_FAILSAFE", "--disable-COPTER_ADVANCED_FAILSAFE"], ["copter", "plane"]),
    "no-copter-afs": (["--enable-ADVANCED_FAILSAFE", "--disable-COPTER_ADVANCED_FAILSAFE"], ["copter"]),
    "no-quadplane": (["--enable-ADVANCED_FAILSAFE", "--disable-QUADPLANE"], ["plane"]),
}
FEATURE_FLAGS = ["--enable-ADVANCED_FAILSAFE", "--enable-COPTER_ADVANCED_FAILSAFE", "--enable-QUADPLANE"]
MANDATORY_GATES = (
    "inputs",
    "checkout",
    "source",
    "compiler",
    "sitl",
    *COHORTS,
    *GUARDS,
    "hardware",
    "stable",
    "package",
)
NEW_MODES = {"ArduCopter/mode_terminate.cpp", "ArduPlane/mode_terminate.cpp"}
SHA = re.compile(r"[0-9a-f]{40}")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def git(path, *args, input=None):
    return subprocess.check_output(["git", "-C", str(path), *args], input=input).decode().rstrip()


def load_identity(path):
    value = json.loads(path.read_text(encoding="utf-8"))
    keys = {
        "repository",
        "revision",
        "version",
        "board",
        "apj_board_id",
        "patch",
        "patch_sha256",
        "license",
        "private_mode_id",
        "controlled_action",
        "private_rc_option",
        "scripting_enabled",
        "submodules",
    }
    require(set(value) == keys, "source.json schema differs from reviewed schema")
    expected = {
        "repository": REPOSITORY,
        "revision": REVISION,
        "version": "4.7.2",
        "board": "CubeOrangePlus",
        "apj_board_id": 1063,
        "patch": "controlled-termination.patch",
        "license": "GPL-3.0-or-later",
        "private_mode_id": 29,
        "controlled_action": 44,
        "private_rc_option": 187,
        "scripting_enabled": False,
    }
    require(
        all(type(value[k]) is type(v) and value[k] == v for k, v in expected.items()),
        "source identity differs from the reviewed firmware target",
    )
    require(re.fullmatch(r"[0-9a-f]{64}", value["patch_sha256"]) is not None, "invalid patch SHA256")
    pins = value["submodules"]
    require(isinstance(pins, list) and len(pins) == 21, "expected all 21 recursive submodule pins")
    paths = []
    for pin in pins:
        require(set(pin) == {"path", "revision"}, "invalid submodule schema")
        path = PurePosixPath(pin["path"])
        require(
            not path.is_absolute()
            and ".." not in path.parts
            and path.parts[0] == "modules"
            and "\\" not in pin["path"]
            and str(path) == pin["path"],
            "unsafe submodule path",
        )
        require(SHA.fullmatch(pin["revision"]) is not None, "invalid submodule revision")
        paths.append(pin["path"])
    require(len(set(paths)) == 21, "duplicate submodule path")
    return value


def verify_revisions(source, identity):
    require(git(source, "rev-parse", "HEAD") == identity["revision"], "upstream revision mismatch")
    actual = {}
    for line in git(source, "submodule", "status", "--recursive").splitlines():
        require(line.startswith(" "), "submodule is missing or differs from gitlink")
        fields = line.split()
        actual[fields[1]] = fields[0]
    expected = {pin["path"]: pin["revision"] for pin in identity["submodules"]}
    require(actual == expected, "recursive submodule pins differ from source.json")
    for path, revision in expected.items():
        require(git(source / path, "rev-parse", "HEAD") == revision, "submodule HEAD mismatch: " + path)


def content_manifest(root):
    entries = {}
    for path in sorted(root.rglob("*")):
        relative = path.relative_to(root).as_posix()
        if path.is_symlink():
            target = os.readlink(path)
            require(
                not Path(target).is_absolute() and (path.parent / target).resolve().is_relative_to(root.resolve()),
                "source symlink escapes archive: " + relative,
            )
            entries[relative] = {"link": target}
        elif path.is_file():
            entries[relative] = {"sha256": digest(path), "mode": stat.S_IMODE(path.stat().st_mode)}
    return entries


def copy_tracked(source, destination):
    destination.mkdir(parents=True, exist_ok=True)
    records = subprocess.check_output(["git", "-C", str(source), "ls-files", "--stage", "-z"])
    for record in records.split(b"\0"):
        if not record:
            continue
        metadata, name = record.split(b"\t", 1)
        if metadata.startswith(b"160000"):
            continue
        relative = os.fsdecode(name)
        original, target = source / relative, destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if original.is_symlink():
            target.symlink_to(os.readlink(original))
        else:
            shutil.copy2(original, target)
            target.chmod(0o755 if metadata.startswith(b"100755") else 0o644)


def copy_revision_metadata(source, destination):
    """Keep only public reachable objects and the minimal index Waf needs."""
    metadata = destination / ".git"
    (metadata / "objects" / "pack").mkdir(parents=True)
    (metadata / "refs" / "heads").mkdir(parents=True)
    revision = git(source, "rev-parse", "HEAD")
    (metadata / "HEAD").write_text(revision + "\n", encoding="ascii")
    (metadata / "config").write_text(
        "[core]\nrepositoryformatversion = 0\nfilemode = true\nbare = false\n[submodule]\nactive = .\n",
        encoding="ascii",
    )
    index = Path(git(source, "rev-parse", "--path-format=absolute", "--git-path", "index"))
    shutil.copy2(index, metadata / "index")
    shallow = Path(git(source, "rev-parse", "--path-format=absolute", "--git-path", "shallow"))
    if shallow.exists():
        shutil.copy2(shallow, metadata / "shallow")
    pack = metadata / "objects" / "pack" / "source.pack"
    with pack.open("wb") as stream:
        subprocess.run(
            ["git", "-C", str(source), "pack-objects", "--stdout", "--revs"],
            input=(revision + "\n").encode(),
            stdout=stream,
            check=True,
        )
    pack_hash = git(destination, "index-pack", str(pack))
    pack.rename(pack.with_name("pack-" + pack_hash + ".pack"))
    pack.with_suffix(".idx").rename(pack.with_name("pack-" + pack_hash + ".idx"))


def freeze_source(source, stage, identity, inputs):
    stage.mkdir()
    exported = stage / "ardupilot"
    repositories = ["", *(pin["path"] for pin in identity["submodules"])]
    for relative in repositories:
        copy_tracked(source / relative, exported / relative)
        copy_revision_metadata(source / relative, exported / relative)
    for relative in NEW_MODES:
        require((source / relative).is_file(), "patch omitted new mode: " + relative)
        (exported / relative).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / relative, exported / relative)
        (exported / relative).chmod(0o644)
    for original, name in inputs:
        shutil.copy2(original, stage / name)
        (stage / name).chmod(0o644)
    (stage / "REPRODUCE.txt").write_text(
        "Complete corresponding source for the enclosed pinned ArduPilot extension.\n"
        "The patch is already applied, including both new mode files. Preserve upstream license notices.\n"
        "ArduPilot and its patch retain GPL notices; NOMAD build helpers use NOMAD-build-support-LICENSE.txt.\n"
        "GNU Arm Embedded 10.2.1; Linux native prerequisites; Python dependencies in firmware-requirements.txt.\n"
        "From the matching NOMAD checkout: python3 scripts/release/build_firmware.py --work /new/empty/path\n"
        "To build this extraction directly, cd ardupilot and run:\n"
        "./waf configure --board sitl --out build --no-submodule-update " + " ".join(FEATURE_FLAGS) + "\n"
        "./waf --out build copter plane -j4\n"
        "python3 Tools/autotest/autotest.py --speedup 10 --timeout 600 test.Copter.Terminate\n"
        "python3 Tools/autotest/autotest.py --speedup 10 --timeout 600 test.QuadPlane.Terminate\n"
        "./waf configure --board CubeOrangePlus --out build-plus --toolchain arm-none-eabi "
        "--no-submodule-update " + " ".join(FEATURE_FLAGS) + " --disable-scripting\n"
        "./waf --out build-plus copter plane -j4\n"
        "Full mandatory stock regressions, compile guards, packaging and toolchain pin: "
        "build_firmware.py and firmware.yml.\n"
        "Software qualification does not establish physical aircraft outcomes. No flashing is performed.\n",
        encoding="utf-8",
    )
    # tar's safe data filter normalizes file permissions; freeze that same portable representation.
    for path in stage.rglob("*"):
        if path.is_file() and not path.is_symlink():
            path.chmod(0o755 if path.stat().st_mode & 0o111 else 0o644)
    manifest = content_manifest(stage)
    write_json(stage / "source-content.json", manifest)
    return manifest


def archive_header(info):
    info.uid = info.gid = 0
    info.uname = info.gname = ""
    info.mtime = 0
    info.pax_headers = {}
    return info


def extract_verified(archive, destination, expected_hash):
    require(digest(archive) == expected_hash, "source archive SHA256 mismatch")
    destination.mkdir()
    with tarfile.open(archive, "r:gz") as stream:
        for entry in stream.getmembers():
            path = PurePosixPath(entry.name)
            require(
                not path.is_absolute() and ".." not in path.parts and "\\" not in entry.name, "unsafe archive member"
            )
            require(entry.isdir() or entry.isfile() or entry.issym(), "unsupported archive member")
            if entry.issym():
                link = PurePosixPath(entry.linkname)
                require(
                    not link.is_absolute()
                    and (destination / entry.name)
                    .parent.joinpath(entry.linkname)
                    .resolve()
                    .is_relative_to(destination.resolve()),
                    "unsafe archive symlink",
                )
        stream.extractall(destination, filter="data")
    manifest_path = destination / "source-content.json"
    expected = json.loads(manifest_path.read_text(encoding="utf-8"))
    actual = content_manifest(destination)
    actual.pop("source-content.json")
    require(actual == expected, "extracted source content or permissions mismatch")
    return expected


def validate_apj(apj, binary, board=1063):
    value = json.loads(apj.read_text(encoding="utf-8"))
    require(type(value.get("board_id")) is int and value["board_id"] == board, "APJ board mismatch")
    image = zlib.decompress(base64.b64decode(value["image"], validate=True))
    require(type(value.get("image_size")) is int and len(image) == value["image_size"] > 0, "APJ image size mismatch")
    require(hashlib.sha256(image).hexdigest() == digest(binary), "APJ image differs from .bin")
    return {"board_id": board, "image_size": len(image), "image_sha256": digest(binary), "apj_sha256": digest(apj)}


def validate_features(text, vehicle):
    required = {"AP_ADVANCEDFAILSAFE_ENABLED", "!AP_SCRIPTING_ENABLED"}
    required.add("AP_COPTER_ADVANCED_FAILSAFE_ENABLED" if vehicle == "arducopter" else "HAL_QUADPLANE_ENABLED")
    features = set(text.splitlines())
    require(required <= features, "required firmware feature missing: " + ", ".join(sorted(required - features)))
    return sorted(required)


def overall_pass(gates):
    return set(gates) == set(MANDATORY_GATES) and all(gates[name].get("status") == "PASS" for name in MANDATORY_GATES)


def owned_children():
    processes = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            processes[int(entry.name)] = (int(fields[1]), fields[19])
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    parents, owned = {os.getpid()}, {}
    while parents:
        children = {pid for pid, (parent, _) in processes.items() if parent in parents}
        owned.update({pid: processes[pid][1] for pid in children})
        parents = children
    return owned


def cleanup_children():
    # Subreaper adoption retains ownership of forkpty children even after their autotest parent exits.
    for sig, grace in ((signal.SIGINT, 5), (signal.SIGTERM, 1), (signal.SIGKILL, 1)):
        children = owned_children()
        if not children:
            return
        for pid, started in children.items():
            try:
                current = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
                if current[19] == started:
                    os.kill(pid, sig)
            except (FileNotFoundError, ProcessLookupError):
                continue
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            for pid in owned_children():
                try:
                    os.waitpid(pid, os.WNOHANG)
                except ChildProcessError:
                    pass
            if not owned_children():
                return
            time.sleep(0.05)
    require(not owned_children(), "owned command descendants survived bounded cleanup")


class Pipeline:
    def __init__(self, repo, work, jobs):
        require(not work.exists(), "work directory must not already exist")
        work.mkdir(parents=True)
        self.repo, self.work, self.jobs = repo, work, jobs
        self.diagnostics = work / "diagnostics"
        self.diagnostics.mkdir()
        self.package = work / "release"
        self.gates, self.details = {}, {}
        self.command_number = 0
        self.source = work / "checkout"
        if sys.platform == "linux":
            require(
                ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0) == 0,
                "Linux child-subreaper support is required for bounded SITL cleanup",
            )

    def command(self, args, cwd=None, env=None, timeout=3600, token=None):
        self.command_number += 1
        log = self.diagnostics / (f"command-{self.command_number:03d}.log")
        started = time.monotonic()
        process = None
        record = {"command": [str(arg) for arg in args], "timeout_seconds": timeout, "log": log.name}
        try:
            with log.open("wb") as output:
                process = subprocess.Popen(
                    [str(arg) for arg in args],
                    cwd=cwd,
                    env=env,
                    stdout=output,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                try:
                    record["exit_code"] = process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    record["timed_out"] = True
                    raise
                finally:
                    cleanup_children()
                    process.wait()
            require(record["exit_code"] == 0, "command failed; see " + log.name)
            text = log.read_text(encoding="utf-8", errors="replace")
            if token:
                require(
                    re.search(r"^>>>> PASSED STEP: " + re.escape(token) + r" at ", text, re.MULTILINE),
                    "native PASSED STEP marker missing; see " + log.name,
                )
            return text
        finally:
            record["duration_seconds"] = round(time.monotonic() - started, 3)
            write_json(log.with_suffix(".json"), record)

    def gate(self, name, operation):
        self.gates[name] = {"status": "RUNNING"}
        self.save()
        print("Running " + name, flush=True)
        try:
            operation()
            self.gates[name] = {"status": "PASS"}
        except BaseException as error:
            self.gates[name] = {"status": "FAIL", "error_type": type(error).__name__}
            raise
        finally:
            self.save()

    def save(self):
        write_json(
            self.work / "results.json",
            {"status": "PASS" if overall_pass(self.gates) else "FAIL", "gates": self.gates, "details": self.details},
        )

    def inputs(self):
        carrier = self.repo / "firmware" / "ardupilot"
        self.identity = load_identity(carrier / "source.json")
        self.patch = carrier / self.identity["patch"]
        require(digest(self.patch) == self.identity["patch_sha256"], "patch SHA256 mismatch")
        new_files = set(re.findall(r"^diff --git a/(\S+) b/\S+\nnew file mode ", self.patch.read_text(), re.MULTILINE))
        require(new_files == NEW_MODES, "patch must include both new mode files")
        self.input_files = [
            (carrier / "source.json", "source.json"),
            (self.patch, self.patch.name),
            (carrier / "COPYING.txt", "COPYING.txt"),
            (self.repo / "docs/native-termination.md", "native-termination.md"),
            (self.repo / "scripts/release/build_firmware.py", "build_firmware.py"),
            (self.repo / "scripts/release/firmware-requirements.txt", "firmware-requirements.txt"),
            (self.repo / "LICENSE", "NOMAD-build-support-LICENSE.txt"),
            (self.repo / ".github/workflows/firmware.yml", "firmware.yml"),
        ]
        self.input_hashes = {name: digest(path) for path, name in self.input_files}
        tracked_dirty = git(self.repo, "diff", "HEAD", "--name-only").splitlines()
        # Record public input names/hashes, never raw diffs, remote URLs or environment dumps.
        self.details["nomad"] = {
            "base_revision": git(self.repo, "rev-parse", "HEAD"),
            "dirty_tracked_paths": tracked_dirty,
            "inputs_sha256": self.input_hashes,
        }
        self.details["source"] = self.identity
        requirements = self.repo / "scripts/release/firmware-requirements.txt"
        self.details["python_packages"] = {}
        for line in requirements.read_text().splitlines():
            if not line or line.startswith("#"):
                continue
            name, version = line.split("==")
            actual = importlib.metadata.version(name)
            require(actual == version, "Python dependency version mismatch: " + name)
            self.details["python_packages"][name] = actual
        self.details["installed_python_package_versions"] = {
            distribution.metadata["Name"]: distribution.version for distribution in importlib.metadata.distributions()
        }

    def checkout(self):
        self.command(["git", "init", self.source])
        self.command(["git", "-C", self.source, "fetch", "--depth=1", REPOSITORY, self.identity["revision"]])
        self.command(["git", "-C", self.source, "checkout", "--detach", "FETCH_HEAD"])
        self.command(
            ["git", "-C", self.source, "submodule", "update", "--init", "--recursive", "--depth=1"], timeout=7200
        )
        verify_revisions(self.source, self.identity)
        self.command(["git", "apply", "--check", self.patch], cwd=self.source)
        self.command(["git", "apply", self.patch], cwd=self.source)

    def source_archive(self):
        stage = self.work / "corresponding-source"
        freeze_source(self.source, stage, self.identity, self.input_files)
        self.archive = self.work / "complete-corresponding-source.tar.gz"
        with self.archive.open("wb") as raw:
            with gzip.GzipFile(fileobj=raw, mode="wb", filename="", mtime=0) as compressed:
                with tarfile.open(fileobj=compressed, mode="w", dereference=False) as stream:
                    for path in sorted(stage.iterdir()):
                        stream.add(path, arcname=path.name, filter=archive_header)
        self.details["source_archive_sha256"] = digest(self.archive)
        extracted = self.work / "source-verified"
        self.frozen_manifest = extract_verified(self.archive, extracted, self.details["source_archive_sha256"])
        self.source = extracted / "ardupilot"
        verify_revisions(self.source, self.identity)
        self.command(["git", "apply", "--reverse", "--check", extracted / self.patch.name], cwd=self.source)
        shutil.copy2(extracted / "source-content.json", self.work / "source-content.json")

    def compiler(self):
        version = self.command(["arm-none-eabi-gcc", "--version"])
        require("10.2.1" in version and "2020-q4-major" in version, "reviewed GNU ARM 10.2.1 toolchain required")
        compiler = Path(shutil.which("arm-none-eabi-gcc")).resolve()
        self.details["compiler"] = {"version": version.strip(), "sha256": digest(compiler)}
        self.details["host_compiler"] = self.command(["g++", "--version"]).strip()

    def build(self, board, out, flags, targets):
        self.command(
            [
                sys.executable,
                self.source / "waf",
                "configure",
                "--board",
                board,
                "--out",
                out,
                "--no-submodule-update",
                *flags,
            ],
            cwd=self.source,
        )
        self.command(
            [sys.executable, self.source / "waf", "--out", out, *targets, f"-j{self.jobs}"],
            cwd=self.source,
            timeout=7200,
        )

    def sitl(self):
        self.build("sitl", "build", FEATURE_FLAGS, ["copter", "plane"])
        self.sitl_hashes = {
            vehicle: digest(self.source / "build/sitl/bin" / vehicle) for vehicle in ("arducopter", "arduplane")
        }
        self.details["sitl_binaries_sha256"] = self.sitl_hashes

    def cohort(self, name):
        directory = self.diagnostics / name
        directory.mkdir()
        runtime, logs = directory / "runtime", directory / "buildlogs"
        runtime.mkdir()
        logs.mkdir()
        environment = dict(os.environ, BUILDLOGS=str(logs))
        before = {vehicle: digest(self.source / "build/sitl/bin" / vehicle) for vehicle in self.sitl_hashes}
        require(before == self.sitl_hashes, "SITL ELF changed before cohort")
        try:
            self.command(
                [
                    sys.executable,
                    self.source / "Tools/autotest/autotest.py",
                    "--speedup",
                    "10",
                    "--timeout",
                    "600",
                    name,
                ],
                cwd=runtime,
                env=environment,
                timeout=1800,
                token=name,
            )
            files = [path for path in directory.rglob("*") if path.is_file()]
            require(any(path.suffix.lower() == ".bin" for path in files), "cohort lacks native BIN evidence")
            require(any(path.suffix.lower() == ".tlog" for path in files), "cohort lacks native TLOG evidence")
        finally:
            # Each fresh runtime and BUILDLOGS stays frozen before another cohort can run.
            after = {vehicle: digest(self.source / "build/sitl/bin" / vehicle) for vehicle in self.sitl_hashes}
            write_json(
                directory / "evidence.json",
                {
                    "cohort": name,
                    "source_archive_sha256": self.details["source_archive_sha256"],
                    "fixtures_sha256": {
                        path: value
                        for path, value in self.frozen_manifest.items()
                        if path.startswith("ardupilot/Tools/autotest/")
                    },
                    "elf_before": before,
                    "elf_after": after,
                    "files": content_manifest(directory),
                },
            )
            require(after == before, "SITL ELF changed during cohort")

    def hardware(self):
        self.build(
            "CubeOrangePlus",
            "build-plus",
            ["--toolchain", "arm-none-eabi", *FEATURE_FLAGS, "--disable-scripting"],
            ["copter", "plane"],
        )
        self.hardware_files = []
        self.details["hardware"] = {}
        for vehicle in ("arducopter", "arduplane"):
            directory = self.source / "build-plus/CubeOrangePlus/bin"
            apj, binary, elf = [directory / (vehicle + extension) for extension in (".apj", ".bin", "")]
            result = validate_apj(apj, binary)
            features = self.command(
                [sys.executable, self.source / "Tools/scripts/extract_features.py", elf], cwd=self.source
            )
            result["required_features"] = validate_features(features, vehicle)
            result["elf_sha256"] = digest(elf)
            self.details["hardware"][vehicle] = result
            self.hardware_files.extend([(apj, vehicle + ".apj"), (binary, vehicle + ".bin"), (elf, vehicle + ".elf")])
        self.hardware_hashes = {name: digest(path) for path, name in self.hardware_files}

    def stable(self):
        require(self.input_hashes == {name: digest(path) for path, name in self.input_files}, "NOMAD inputs changed")
        require(digest(self.archive) == self.details["source_archive_sha256"], "source archive changed")
        root = self.work / "source-verified"
        for relative, expected in self.frozen_manifest.items():
            # Git can refresh index stat fields; source bytes and archive metadata remain independently bound.
            if relative.endswith("/.git/index"):
                continue
            path = root / relative
            actual = (
                {"link": os.readlink(path)}
                if path.is_symlink()
                else {"sha256": digest(path), "mode": stat.S_IMODE(path.stat().st_mode)}
            )
            require(actual == expected, "source changed during build/test: " + relative)
        verify_revisions(self.source, self.identity)
        require(
            self.hardware_hashes == {name: digest(path) for path, name in self.hardware_files},
            "hardware binaries changed",
        )
        require(
            self.sitl_hashes == {name: digest(self.source / "build/sitl/bin" / name) for name in self.sitl_hashes},
            "qualified SITL binaries changed",
        )

    def package_assets(self):
        require(
            all(self.gates.get(name, {}).get("status") == "PASS" for name in MANDATORY_GATES if name != "package"),
            "release assets require every qualification gate",
        )
        pending = self.work / "release-pending"
        pending.mkdir()
        assets = [
            *self.hardware_files,
            *self.input_files,
            (self.archive, self.archive.name),
            (self.work / "source-content.json", "source-content.json"),
        ]
        expected = {
            **self.hardware_hashes,
            **self.input_hashes,
            self.archive.name: self.details["source_archive_sha256"],
            "source-content.json": digest(self.work / "source-content.json"),
        }
        for original, name in assets:
            shutil.copy2(original, pending / name)
            require(digest(pending / name) == expected[name], "asset changed after qualification: " + name)
        manifest = {
            "source": self.identity,
            "source_archive_sha256": self.details["source_archive_sha256"],
            "provenance": self.details,
            "gates": self.gates,
            "assets": {name: digest(pending / name) for _, name in assets},
        }
        manifest["gates"] = {**self.gates, "package": {"status": "PASS"}}
        write_json(pending / "manifest.json", manifest)
        (pending / "release-notes.md").write_text(
            "Custom CubeOrangePlus termination firmware (APJ board 1063).\n\n"
            "All mandatory software build, SITL, stock regression and optional-feature compile gates passed.\n"
            "See manifest.json for source, toolchain, feature and binary identities. Matching complete corresponding "
            "source and GPL notices are included. Follow native-termination.md for setup and limitations.\n\n"
            "Software/SITL evidence does not qualify physical descent, actuator calibration, transmitter mixing, "
            "touchdown or organizer acceptance. No aircraft was flashed or configured.\n",
            encoding="utf-8",
        )
        names = sorted(path.name for path in pending.iterdir())
        (pending / "SHA256SUMS").write_text(
            "".join(digest(pending / name) + "  " + name + "\n" for name in names), encoding="ascii"
        )
        write_json(self.work / "release-assets.json", [*names, "SHA256SUMS"])
        pending.rename(self.package)

    def run(self):
        try:
            for name, operation in (
                ("inputs", self.inputs),
                ("checkout", self.checkout),
                ("source", self.source_archive),
                ("compiler", self.compiler),
                ("sitl", self.sitl),
            ):
                self.gate(name, operation)
            for name in COHORTS:
                self.gate(name, lambda name=name: self.cohort(name))
            for name, (flags, targets) in GUARDS.items():
                self.gate(
                    name,
                    lambda name=name, flags=flags, targets=targets: self.build("sitl", "build-" + name, flags, targets),
                )
            self.gate("hardware", self.hardware)
            self.gate("stable", self.stable)
            self.gate("package", self.package_assets)
        finally:
            self.save()
        require(overall_pass(self.gates), "mandatory qualification incomplete")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--work", type=Path, required=True, help="fresh output directory; existing paths are refused")
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args()
    require(sys.platform == "linux", "firmware qualification runs on Linux")
    require(args.jobs > 0, "jobs must be positive")
    repo = Path(__file__).resolve().parents[2]
    Pipeline(repo, args.work.resolve(), args.jobs).run()


if __name__ == "__main__":
    main()
