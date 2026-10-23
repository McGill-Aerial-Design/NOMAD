#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Build only the pinned Windows conference Copter; never install or flash it."""

import argparse
import ast
import json
import os
import re
import shutil
import subprocess
import sys
import urllib.request
from pathlib import Path, PureWindowsPath

SOURCE_URL = (
    "https://github.com/McGill-Aerial-Design/NOMAD/releases/download/"
    "ardupilot-4.7.2-nomad.1/complete-corresponding-source.tar.gz"
)
SOURCE_SHA256 = "f728fb7fc24260b3f85445e2a3f8ed2786ec642c256374a41db703fbb8b374ab"
FEATURES = {"AP_ADVANCEDFAILSAFE_ENABLED": "1", "AP_COPTER_ADVANCED_FAILSAFE_ENABLED": "1", "AP_SCRIPTING_ENABLED": "0"}
FLAGS = ["--enable-ADVANCED_FAILSAFE", "--enable-COPTER_ADVANCED_FAILSAFE", "--disable-scripting"]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def verify_features(cache, symbols):
    configuration = {}
    for statement in ast.parse(cache).body:
        require(
            isinstance(statement, ast.Assign)
            and len(statement.targets) == 1
            and isinstance(statement.targets[0], ast.Name),
            "Waf cache must contain literal assignments only",
        )
        require(
            not any(isinstance(node, ast.Call) for node in ast.walk(statement.value)),
            "Waf cache values must not call functions",
        )
        value = ast.literal_eval(statement.value)
        name = statement.targets[0].id
        if name in ("CXXFLAGS", "DEFINES"):
            require(name not in configuration, "Duplicate Waf configuration assignment")
            require(
                isinstance(value, list) and all(isinstance(item, str) for item in value),
                "Waf compiler definitions must be string lists",
            )
            configuration[name] = value
    require(set(configuration) == {"CXXFLAGS", "DEFINES"}, "Waf compiler definition lists missing")
    entries = configuration["DEFINES"] + [flag[2:] for flag in configuration["CXXFLAGS"] if flag.startswith("-D")]
    definitions = {}
    for entry in entries:
        name, separator, value = entry.partition("=")
        if name not in FEATURES:
            continue
        require(separator and value in ("0", "1"), "Invalid required feature definition")
        require(name not in definitions or definitions[name] == value, "Conflicting required feature definitions")
        definitions[name] = value
    require(
        all(definitions.get(name) == value for name, value in FEATURES.items()),
        "The built configuration must enable both AFS features and disable Lua",
    )
    require(
        re.search(r"\bModeTerminate::run\(\)", symbols), "The Windows binary has no native TERMINATE implementation"
    )
    return FEATURES.copy()


def cygwin_dependencies(text):
    require(
        not re.search(r"could not find|cannot find|not found", text, re.IGNORECASE),
        "cygcheck reported an unresolved dependency",
    )
    paths = {}
    for line in text.splitlines():
        candidate = line.strip()
        name = PureWindowsPath(candidate).name
        if not re.fullmatch(r"cyg[A-Za-z0-9_.+-]*\.dll", name, re.IGNORECASE):
            continue
        require(PureWindowsPath(candidate).is_absolute(), "cygcheck returned a relative runtime path")
        key = name.lower()
        require(key not in paths or paths[key].lower() == candidate.lower(), "Conflicting Cygwin runtime DLLs")
        paths[key] = candidate
    require("cygwin1.dll" in paths, "The Windows executable did not resolve cygwin1.dll")
    return paths


def run(command, cwd=None, timeout=600):
    print("+ " + " ".join(str(value) for value in command), flush=True)
    try:
        result = subprocess.run(
            [str(value) for value in command],
            cwd=cwd,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        print(output.decode("utf-8", errors="replace") if isinstance(output, bytes) else output, flush=True)
        raise
    if result.returncode:
        print(result.stdout, flush=True)
        result.check_returncode()
    return result.stdout


def build(work):
    require(sys.platform == "cygwin" and sys.version_info[:2] == (3, 12), "Use Cygwin Python 3.12")
    require(not work.exists(), "The build directory must be fresh")
    repo = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(repo))
    from scripts.release.build_firmware import digest, extract_verified, load_identity, verify_revisions

    identity = load_identity(repo / "firmware/ardupilot/source.json")
    patch = repo / "firmware/ardupilot" / identity["patch"]
    require(digest(patch) == identity["patch_sha256"], "Reviewed patch hash mismatch")
    compiler = shutil.which("x86_64-pc-cygwin-g++")
    require(compiler, "Cygwin x86_64 compiler missing")
    compiler_version = run([compiler, "--version"])
    compiler_package = run(["cygcheck", "-c", "gcc-g++"])
    require("13.4.0" in compiler_version and "13.4.0-1" in compiler_package, "Use reviewed Cygwin GCC 13.4.0-1")
    require(run([compiler, "-dumpmachine"]).strip() == "x86_64-pc-cygwin", "Wrong compiler target")
    work.mkdir(parents=True)
    archive = work / "complete-corresponding-source.tar.gz"
    with urllib.request.urlopen(SOURCE_URL, timeout=60) as response, archive.open("xb") as target:
        shutil.copyfileobj(response, target, length=1024 * 1024)
    frozen = extract_verified(archive, work / "source", SOURCE_SHA256)
    source = work / "source/ardupilot"
    require(
        load_identity(work / "source/source.json") == identity, "Published source identity differs from this checkout"
    )
    require(digest(work / "source" / identity["patch"]) == identity["patch_sha256"], "Published patch identity differs")
    verify_revisions(source, identity)
    run(["git", "apply", "--reverse", "--check", patch], cwd=source)
    output = source / "build-conference"
    configure = [
        sys.executable,
        source / "waf",
        "configure",
        "--board",
        "sitl",
        "--toolchain",
        "x86_64-pc-cygwin",
        "--out",
        output,
        "--no-submodule-update",
        *FLAGS,
    ]
    compile_command = [sys.executable, source / "waf", "--out", output, "copter", "-j4"]
    for command in (configure, compile_command):
        print(run(command, cwd=source, timeout=3600), flush=True)
    executable = output / "sitl/bin/arducopter"
    require(executable.is_file(), "Copter executable missing")
    with executable.open("rb") as binary:
        require(binary.read(2) == b"MZ", "Copter output is not a Windows executable")
    cache = (output / "c4che/sitl_cache.py").read_text()
    symbols = run(["nm", "--demangle", executable])
    features = verify_features(cache, symbols)
    dependencies = cygwin_dependencies(run(["cygcheck", executable]))
    # The frozen source includes its own manifest; every original source file must remain unchanged.
    for relative, record in frozen.items():
        if relative.endswith("/.git/index"):
            continue  # Git refreshes index stat fields; source bytes and pinned revisions remain checked.
        path = work / "source" / relative
        if "sha256" in record:
            require(path.is_file() and digest(path) == record["sha256"], "Build changed source: " + relative)
        else:
            require(path.is_symlink() and os.readlink(path) == record["link"], "Build changed source link: " + relative)
    verify_revisions(source, identity)
    runtime = work / "runtime"
    runtime.mkdir()
    shutil.copy2(executable, runtime / "ArduCopter.exe")
    for name, windows_path in dependencies.items():
        native = Path(run(["cygpath", "-u", windows_path]).strip())
        require(native.is_file(), "Missing runtime dependency: " + name)
        shutil.copy2(native, runtime / name)
    shutil.copy2(source / "Tools/autotest/default_params/copter.parm", runtime / "copter.parm")
    for filename in ("source.json", identity["patch"], "COPYING.txt"):
        original = work / "source" / filename
        require(original.is_file(), "Corresponding-source notice missing: " + filename)
        shutil.copy2(original, runtime / filename)
    nomad_sha = run(["git", "-C", repo, "rev-parse", "HEAD"]).strip()
    (runtime / "SOURCE.txt").write_text(
        "Windows SITL only; not an aircraft firmware image or physical qualification.\n"
        "ArduPilot and the native termination patch are GPL-3.0-or-later; see COPYING.txt.\n"
        "Complete corresponding source (patch already applied, including pinned submodules):\n"
        f"{SOURCE_URL}\nSHA256 {SOURCE_SHA256}\n"
        f"NOMAD build instructions/helper at commit {nomad_sha}:\n"
        f"https://github.com/McGill-Aerial-Design/NOMAD/blob/{nomad_sha}/conference/build_sitl.py\n"
        "From that checkout, Cygwin Python 3.12: python3.12 conference/build_sitl.py --work /fresh/path\n"
        "Cygwin gcc-g++ 13.4.0-1; build arguments and file hashes are in manifest.json.\n"
        "Runtime capability, ArduPilot commissioning and Project AirSim behavior require separate checks.\n",
        encoding="utf-8",
    )
    files = {path.name: digest(path) for path in sorted(runtime.iterdir())}
    manifest = {
        "target": "Windows x86_64 Copter SITL",
        "behavior_qualified": False,
        "nomad_sha": nomad_sha,
        "source_url": SOURCE_URL,
        "source_sha256": SOURCE_SHA256,
        "source_identity": identity,
        "compiler_version": compiler_version.strip(),
        "compiler_package": compiler_package.strip(),
        "compiler_sha256": digest(Path(compiler).resolve()),
        "python_version": sys.version,
        "configure": [str(x) for x in configure],
        "build": [str(x) for x in compile_command],
        "verified_features": features,
        "files": files,
    }
    (runtime / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print("Built Windows simulator only: " + str(runtime), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--work", type=Path, required=True)
    args = parser.parse_args()
    build(args.work.resolve())
