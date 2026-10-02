# SPDX-License-Identifier: Apache-2.0
"""Unprivileged Windows adapters: exact DLL replacement and SCM boundaries."""

import hashlib
import importlib.util
import json
import os
import shutil
import struct
import subprocess
import zipfile
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.release import deploy, lifecycle, manifest, storage


def load_module(name):
    path = Path(__file__).resolve().parents[1] / "scripts" / "release" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


plugin = load_module("plugin")
windows = load_module("windows")


def candidate(directory, version, source_sha="a" * 40):
    directory.mkdir()
    header = bytearray(88)
    header[:2] = b"MZ"
    struct.pack_into("<I", header, 60, 64)
    header[64:68] = b"PE\0\0"
    struct.pack_into("<H", header, 86, 0x2000)
    component_version = version[1:] if version.startswith("v") else version
    payload = bytes(header) + (component_version + "+" + source_sha).encode()
    (directory / "NOMADPlugin.dll").write_bytes(payload)
    return {
        "install_path": str(directory),
        "release_version": version,
        "version": component_version,
        "source_sha": source_sha,
        "payload_sha256": {"NOMADPlugin.dll": hashlib.sha256(payload).hexdigest()},
    }


def fake_install(directory):
    directory.mkdir()
    (directory / "MissionPlanner.exe").write_bytes(b"fake Mission Planner")
    plugins = directory / "plugins"
    plugins.mkdir()
    (plugins / "ThirdParty.dll").write_bytes(b"unrelated")
    (directory / "settings.json").write_bytes(b'{"CoreClientCredential":"fixture-only"}')
    return plugin.PluginAdapter(
        directory,
        process_check=lambda: None,
        version_reader=lambda _: "1.3.83.0",
        payload_version_reader=lambda path: path.read_bytes()[88:].decode(),
    )


def test_plugin_exact_a_b_a_preserves_operator_state(tmp_path):
    adapter = fake_install(tmp_path / "Program Files (x86) Mission Planner")
    a, b = candidate(tmp_path / "A", "A"), candidate(tmp_path / "B", "B")
    preserved = {path: path.read_bytes() for path in adapter.directory.rglob("*") if path.is_file()}
    for release in (a, b, a):
        adapter.preflight(release)
        adapter.stop()
        adapter.switch(release)
        adapter.start()
        adapter.health(release)
    assert adapter.target.read_bytes() == (tmp_path / "A" / "NOMADPlugin.dll").read_bytes()
    assert all(path.read_bytes() == value for path, value in preserved.items())
    assert (tmp_path / "B" / "NOMADPlugin.dll").is_file()


def test_plugin_embedded_identity_must_match_manifest(tmp_path):
    adapter = fake_install(tmp_path / "MP")
    release = candidate(tmp_path / "A", "A")
    release["source_sha"] = "b" * 40
    with pytest.raises(ValueError, match="version/source"):
        adapter.preflight(release)
    assert not adapter.target.exists()


def test_plugin_refuses_loaded_app_and_unsupported_target(tmp_path):
    adapter = fake_install(tmp_path / "MP")
    a = candidate(tmp_path / "A", "A")
    adapter.version_reader = lambda _: "1.3.84.0"
    with pytest.raises(ValueError, match="Unsupported Mission Planner"):
        adapter.preflight(a)
    adapter.process_check = lambda: (_ for _ in ()).throw(RuntimeError("Close Mission Planner"))
    with pytest.raises(RuntimeError, match="Close Mission Planner"):
        adapter.switch(a)
    assert not adapter.target.exists()


def test_plugin_refuses_non_dll_and_failed_health(tmp_path):
    adapter = fake_install(tmp_path / "MP")
    a = candidate(tmp_path / "A", "A")
    (tmp_path / "A" / "NOMADPlugin.dll").write_bytes(b"not a DLL")
    with pytest.raises(ValueError, match="header"):
        adapter.preflight(a)
    with pytest.raises(RuntimeError, match="does not match"):
        adapter.health(a)


def test_scm_switch_uses_quoted_version_path_and_keeps_configuration(tmp_path):
    executable = tmp_path / "Program Files (x86)" / "A" / "bin" / "nomad-runtime.exe"
    executable.parent.mkdir(parents=True)
    executable.write_bytes(b"fixture")
    config = tmp_path / "operator config.json"
    config.write_text("{}")
    calls = []

    def run(arguments, **_):
        calls.append(arguments)
        return SimpleNamespace(returncode=0, stdout="STATE : 1 STOPPED")

    adapter = windows.ScmAdapter(config, lambda _: None, runner=run)
    release = {"install_path": str(executable.parents[1])}
    adapter.preflight(release)
    adapter.stop()
    adapter.switch(release)
    assert calls[-1][1:4] == ["config", "nomad-runtime", "binPath="]
    assert calls[-1][4] == f'"{executable}" --service --config "{config}"'
    assert config.read_text() == "{}"


def test_scm_never_switches_running_service(tmp_path):
    def run(_arguments, **_):
        return SimpleNamespace(returncode=0, stdout="STATE : 4 RUNNING")

    adapter = windows.ScmAdapter(tmp_path / "config", lambda _: None, runner=run)
    with pytest.raises(RuntimeError, match="Stop"):
        adapter.switch({"install_path": str(tmp_path)})


def fixture_entry(name, platform, architecture, version):
    required = {
        "plugin": ["NOMADPlugin.dll"],
        "router": ["nomad-link-router.exe", "Nomad.LinkRouter.dll"],
        "core": ["bin/nomad.exe", "bin/nomad-runtime.exe"]
        if platform == "windows"
        else ["bin/nomad", "bin/nomad-runtime"],
    }[name]
    if name == "core":
        required.extend(manifest.CORE_REQUIRED_FILES)
    entry = {
        "name": name,
        "platform": platform,
        "architecture": architecture,
        "version": version[1:],
        "filename": f"{name}-{platform}.zip",
        "sha256": "b" * 64,
        "protocol_versions": {"router_management": 1} if name == "router" else {"runtime_ipc": 1},
        "required_files": ["package-identity.json", *required],
    }
    if name == "plugin":
        entry["mission_planner_target"] = "1.3.83"
    return entry


def release_bundle(directory, number):
    directory.mkdir()
    version = f"v0.0.{number}"
    document = {
        "schema_version": 1,
        "release_version": version,
        "source_sha": str(number) * 40,
        "mavsdk_sha": "a" * 40,
        "official": True,
        "components": [fixture_entry(*key, version) for key in sorted(manifest.EXPECTED)],
    }
    entry = next(item for item in document["components"] if item["name"] == "plugin")
    identity = {
        **{key: value for key, value in document.items() if key != "components"},
        **{key: value for key, value in entry.items() if key not in {"sha256", "filename"}},
    }
    payload = candidate(directory / "payload", version, document["source_sha"])
    package = directory / entry["filename"]
    with zipfile.ZipFile(package, "w") as archive:
        archive.writestr("package-identity.json", json.dumps(identity))
        archive.write(Path(payload["install_path"]) / "NOMADPlugin.dll", "NOMADPlugin.dll")
    entry["sha256"] = storage.digest(package)
    path = directory / "release-manifest.json"
    path.write_text(json.dumps(document))
    return path, package, version


def stage_plugin(deployment, release):
    path, package, _ = release
    return deployment.stage(path, package, "windows", "any")


def test_plugin_transaction_staging_rollback_corruption_and_preservation(tmp_path):
    adapter = fake_install(tmp_path / "Program Files (x86) Mission Planner")
    deployment = lifecycle.Deployment(tmp_path / "deployment", "plugin")
    a, b = release_bundle(tmp_path / "A", 1), release_bundle(tmp_path / "B", 2)
    preserved = {path: path.read_bytes() for path in adapter.directory.rglob("*") if path.is_file()}
    stage_plugin(deployment, a)
    deployment.activate(a[2], adapter)
    exact_a = adapter.target.read_bytes()
    stage_plugin(deployment, b)
    assert adapter.target.read_bytes() == exact_a, "Staging B must leave A active"
    deployment.activate(b[2], adapter)
    assert adapter.target.read_bytes() != exact_a
    deployment.rollback(adapter)
    assert deployment.status()["active"]["release_version"] == a[2]
    assert adapter.target.read_bytes() == exact_a
    corrupt = release_bundle(tmp_path / "corrupt", 3)
    with corrupt[1].open("ab") as stream:
        stream.write(b"modified-after-manifest")
    with pytest.raises(ValueError, match="mismatch"):
        stage_plugin(deployment, corrupt)
    assert adapter.target.read_bytes() == exact_a
    assert all(path.read_bytes() == value for path, value in preserved.items())


def test_plugin_failed_b_health_restores_exact_a(tmp_path):
    adapter = fake_install(tmp_path / "MP")
    deployment = lifecycle.Deployment(tmp_path / "deployment", "plugin")
    a, b = release_bundle(tmp_path / "A", 1), release_bundle(tmp_path / "B", 2)
    for release in (a, b):
        stage_plugin(deployment, release)
    deployment.activate(a[2], adapter)
    exact_a = adapter.target.read_bytes()
    normal_health = adapter.health

    def health(record):
        if record["release_version"] == b[2]:
            raise RuntimeError("injected B health failure")
        normal_health(record)

    adapter.health = health
    with pytest.raises(RuntimeError, match="injected B"):
        deployment.activate(b[2], adapter)
    assert deployment.status()["status"] == "rolled_back"
    assert deployment.status()["active"]["release_version"] == a[2]
    assert adapter.target.read_bytes() == exact_a
    assert (adapter.directory / "settings.json").read_bytes() == b'{"CoreClientCredential":"fixture-only"}'


def test_plugin_recovery_restores_journalled_a_after_interrupted_switch(tmp_path):
    adapter = fake_install(tmp_path / "MP")
    deployment = lifecycle.Deployment(tmp_path / "deployment", "plugin")
    a, b = release_bundle(tmp_path / "A", 1), release_bundle(tmp_path / "B", 2)
    for release in (a, b):
        stage_plugin(deployment, release)
    deployment.activate(a[2], adapter)
    exact_a = adapter.target.read_bytes()
    state = deployment.status()
    candidate_b = deployment.get_release(b[2])
    state.update(status="activating", pending={"candidate": candidate_b, "restore": state["active"]})
    storage.write_json(deployment.record, state)
    adapter.switch(candidate_b)
    assert adapter.target.read_bytes() != exact_a
    with pytest.raises(ValueError, match="recover"):
        deployment.activate(b[2], adapter)
    recovered = deployment.recover(adapter)
    assert recovered["status"] == "rolled_back"
    assert recovered["pending"] is None
    assert adapter.target.read_bytes() == exact_a


@pytest.mark.skipif(os.name != "nt", reason="Windows DLL replacement sharing semantics")
def test_plugin_locked_dll_fails_without_changing_active_bytes(tmp_path):
    adapter = fake_install(tmp_path / "MP")
    a, b = candidate(tmp_path / "A", "A"), candidate(tmp_path / "B", "B")
    adapter.switch(a)
    before = adapter.target.read_bytes()
    with adapter.target.open("rb") as held_dll:
        with pytest.raises(RuntimeError, match="locked or access denied"):
            adapter.switch(b)
        assert held_dll.read() == before
    assert adapter.target.read_bytes() == before
    assert not list(adapter.target.parent.glob(".nomad-*"))


def test_scm_command_rejects_quotes_and_line_breaks(tmp_path):
    for name in ('quote".exe', "newline\n.exe"):
        with pytest.raises(ValueError, match="quotes or newlines"):
            windows.service_command(tmp_path / name, tmp_path / "config.json")


@pytest.mark.skipif(os.name != "nt", reason="PowerShell installer argument boundaries")
def test_installer_passes_paths_with_spaces_as_single_arguments(tmp_path):
    shell = shutil.which("pwsh") or shutil.which("powershell.exe")
    assert shell is not None, "Windows qualification requires PowerShell"
    output = tmp_path / "captured-arguments.json"
    launcher = tmp_path / "fake Python.ps1"
    launcher.write_text(
        "$args | ConvertTo-Json | Set-Content -LiteralPath $env:NOMAD_RELEASE_ARGUMENT_FILE\n$global:LASTEXITCODE = 0\n"
    )
    tool = tmp_path / "deployment tools" / "deploy.py"
    tool.parent.mkdir()
    tool.write_text("# fixture")
    installer = Path(__file__).resolve().parents[1] / "mission_planner" / "packaging" / "INSTALL.ps1"
    root, mp = tmp_path / "deployment root", tmp_path / "Program Files (x86)" / "Mission Planner"
    subprocess.run(
        [
            shell,
            "-NoProfile",
            "-File",
            str(installer),
            "-Action",
            "status",
            "-Root",
            str(root),
            "-MissionPlanner",
            str(mp),
            "-DeploymentTool",
            str(tool),
            "-Python",
            str(launcher),
        ],
        check=True,
        capture_output=True,
        text=True,
        env=dict(os.environ, NOMAD_RELEASE_ARGUMENT_FILE=str(output)),
    )
    arguments = json.loads(output.read_text(encoding="utf-8-sig"))
    assert arguments[0] == str(tool)
    assert arguments[arguments.index("--root") + 1] == str(root)
    assert arguments[arguments.index("--mission-planner") + 1] == str(mp)
    assert arguments[arguments.index("--architecture") + 1] == "any"


@pytest.mark.skipif(os.name != "nt", reason="Windows deployment ACL qualification")
def test_protected_root_rejects_broad_write_access(tmp_path):
    root = tmp_path / "protected deployment"
    deploy.protect_root(root)
    (root / "component").mkdir()
    (root / "component" / "record.json").write_text("{}")
    deploy.protect_root(root)
    subprocess.run(
        ["icacls.exe", str(root), "/grant", "*S-1-1-0:(OI)(CI)M"],
        check=True,
        capture_output=True,
    )
    with pytest.raises(ValueError, match="ACL must allow writes only"):
        deploy.protect_root(root)
