# SPDX-License-Identifier: Apache-2.0
"""Explicit NOMAD release verification, staging, activation and rollback."""

from __future__ import annotations

import argparse
import json
import os
import platform
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.release import health, manifest, storage  # noqa: E402
from scripts.release.lifecycle import Deployment, find_payload, validate_payload  # noqa: E402


def protect_root(root: Path) -> None:
    storage.reject_links(root)
    if not root.exists():
        root.mkdir(parents=True, mode=0o755)
        if os.name == "nt":
            environment = dict(os.environ, NOMAD_DEPLOYMENT_ROOT=str(root))
            environment = {key: value for key, value in environment.items() if key.casefold() != "psmodulepath"}
            script = (
                "$acl = [Security.AccessControl.DirectorySecurity]::new(); "
                "$sid = [Security.Principal.WindowsIdentity]::GetCurrent().User; $acl.SetOwner($sid); "
                "$acl.SetAccessRuleProtection($true,$false); "
                "$inherit = [Security.AccessControl.InheritanceFlags]'ContainerInherit,ObjectInherit'; "
                "foreach ($id in @($sid.Value,'S-1-5-18','S-1-5-32-544')) { "
                "$rule = [Security.AccessControl.FileSystemAccessRule]::new("
                "[Security.Principal.SecurityIdentifier]::new($id),'FullControl',$inherit,"
                "[Security.AccessControl.PropagationFlags]::None,'Allow'); $acl.AddAccessRule($rule) }; "
                "$rule = [Security.AccessControl.FileSystemAccessRule]::new("
                "[Security.Principal.SecurityIdentifier]::new('S-1-5-19'),'ReadAndExecute',$inherit,"
                "[Security.AccessControl.PropagationFlags]::None,'Allow'); $acl.AddAccessRule($rule); "
                "Set-Acl -LiteralPath $env:NOMAD_DEPLOYMENT_ROOT -AclObject $acl"
            )
            subprocess.run(
                ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", script],
                env=environment,
                check=True,
                capture_output=True,
                timeout=30,
            )
    if os.name != "nt":
        from scripts.release.security import validate_posix_permissions

        validate_posix_permissions(root)
    if os.name == "nt":
        from scripts.release.security import validate_windows_acl

        validate_windows_acl(root)


def external_config(root: Path, path: Path) -> dict:
    if path is None or not path.is_absolute():
        raise ValueError("provide absolute external --config")
    storage.reject_links(path)
    if path.is_relative_to(root):
        raise ValueError("operator configuration must be outside deployment root")
    settings = storage.read_json(path)
    for key in ("NOMAD_CLIENT_CREDENTIALS_FILE", "NOMAD_AUDIT_DIRECTORY"):
        if key in settings and Path(settings[key]).is_relative_to(root):
            raise ValueError("credentials and audit history must be outside deployment root")
    return settings


def create_adapter(arguments):
    if arguments.component == "plugin":
        from scripts.release.plugin import PluginAdapter

        if arguments.mission_planner is None:
            raise ValueError("provide --mission-planner installation directory")
        return PluginAdapter(arguments.mission_planner)
    settings = external_config(arguments.root, arguments.config)
    if arguments.component == "router":
        from scripts.release.router import RouterAdapter

        if arguments.router_start_command is None:
            raise ValueError("provide --router-start-command operator-owned JSON argv file")
        command = json.loads(arguments.router_start_command.read_text(encoding="utf-8"))
        return RouterAdapter(arguments.root, arguments.config, settings["ManagementPort"], command)
    port = int(settings["NOMAD_RUNTIME_IPC_PORT"])
    if arguments.adapter == "systemd" and os.name != "nt":
        from scripts.release.linux import SystemdAdapter

        return SystemdAdapter(arguments.root, arguments.config, port)
    if arguments.adapter == "scm" and os.name == "nt":
        from scripts.release.windows import ScmAdapter

        return ScmAdapter(arguments.config, lambda candidate: health.core(port, candidate))
    raise ValueError("select the native --adapter systemd or scm for core activation")


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action", choices=("verify", "stage", "activate", "status", "rollback", "recover", "cleanup", "adopt")
    )
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--component", choices=("core", "router", "plugin"), required=True)
    parser.add_argument("--platform", choices=("linux", "windows"))
    parser.add_argument("--architecture", choices=("x86_64", "any"))
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--package", type=Path)
    parser.add_argument("--release")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--adapter", choices=("systemd", "scm", "plugin", "router"))
    parser.add_argument("--mission-planner", type=Path)
    parser.add_argument("--router-start-command", type=Path)
    arguments = parser.parse_args()
    arguments.root = arguments.root.absolute()
    expected_platform = "windows" if os.name == "nt" else "linux"
    arguments.platform = arguments.platform or expected_platform
    arguments.architecture = arguments.architecture or ("any" if arguments.component == "plugin" else "x86_64")
    if arguments.platform != expected_platform or platform.machine().lower() not in {"amd64", "x86_64"}:
        parser.error("deployment requires the native supported x86-64 platform")
    return arguments


def verify(arguments) -> dict:
    if arguments.manifest is None or arguments.package is None:
        raise ValueError("provide --manifest and --package")
    document = manifest.load_manifest(arguments.manifest)
    entry = manifest.select_component(document, arguments.component, arguments.platform, arguments.architecture)
    manifest.verify_package(entry, arguments.package)
    with tempfile.TemporaryDirectory(prefix="nomad-verify-") as temporary:
        destination = Path(temporary)
        storage.extract(arguments.package, destination)
        validate_payload(document, entry, find_payload(destination))
    return entry


def perform(arguments) -> dict:
    engine = Deployment(arguments.root, arguments.component)
    if arguments.action == "verify":
        return verify(arguments)
    if arguments.action == "status":
        return engine.status()
    protect_root(arguments.root)
    if arguments.action == "stage":
        verify(arguments)
        return engine.stage(arguments.manifest, arguments.package, arguments.platform, arguments.architecture)
    if arguments.action == "cleanup":
        engine.cleanup(arguments.release)
        return engine.status()
    adapter = create_adapter(arguments)
    if arguments.action in {"activate", "adopt"}:
        candidate = engine.get_release(arguments.release)
        if (candidate["platform"], candidate["architecture"]) != (arguments.platform, arguments.architecture):
            raise ValueError("staged candidate platform does not match this host")
        return getattr(engine, arguments.action)(arguments.release, adapter)
    return getattr(engine, arguments.action)(adapter)


def main() -> int:
    try:
        print(json.dumps(perform(parse_arguments()), indent=2, sort_keys=True))
        return 0
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"NOMAD deployment failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
