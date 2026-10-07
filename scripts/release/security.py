# SPDX-License-Identifier: Apache-2.0
"""Reject Windows deployment state writable by accounts outside its operator boundary."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

from scripts.release import storage


def validate_posix_permissions(root: Path) -> None:
    """Every mutable deployment descendant stays inside the operator write boundary."""
    for path in (root, *root.rglob("*")):
        if path.is_symlink():
            if path not in {root / "core/current", root / "core/.current-new"}:
                raise ValueError("unexpected deployment link")
            if not path.resolve().is_relative_to(root / "core/releases") or path.resolve().name != "payload":
                raise ValueError("core pointer must target an immutable staged payload")
            continue
        storage.reject_links(path)
        metadata = path.stat()
        if metadata.st_mode & 0o022 or metadata.st_uid not in {0, os.getuid()}:
            raise ValueError("deployment tree must be operator-owned and not group/world writable")


def validate_windows_acl(root: Path) -> None:
    environment = dict(os.environ, NOMAD_DEPLOYMENT_ROOT=str(root))
    environment = {key: value for key, value in environment.items() if key.casefold() != "psmodulepath"}
    script = (
        "$ErrorActionPreference='Stop'; "
        "$user=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value; "
        "$allowed=@($user,'S-1-5-18','S-1-5-32-544'); "
        "$write=[Security.AccessControl.FileSystemRights]'Write,Delete,DeleteSubdirectoriesAndFiles,"
        "ChangePermissions,TakeOwnership'; "
        "$root=Get-Item -LiteralPath $env:NOMAD_DEPLOYMENT_ROOT; "
        "$items=@($root)+@(Get-ChildItem -LiteralPath $root.FullName -Recurse -Force); "
        "foreach($item in $items) { "
        "if($item.Attributes -band [IO.FileAttributes]::ReparsePoint) { exit 7 }; "
        "$acl=Get-Acl -LiteralPath $item.FullName; "
        "if($item.FullName -eq $root.FullName -and -not $acl.AreAccessRulesProtected) { exit 7 }; "
        "if($allowed -notcontains $acl.GetOwner([Security.Principal.SecurityIdentifier]).Value) { exit 7 }; "
        "foreach($rule in $acl.GetAccessRules($true,$true,[Security.Principal.SecurityIdentifier])) { "
        "if($rule.AccessControlType -eq 'Allow' -and ($rule.FileSystemRights -band $write) -and "
        "$allowed -notcontains $rule.IdentityReference.Value) { exit 7 } } }; exit 0"
    )
    result = subprocess.run(
        ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", script],
        env=environment,
        capture_output=True,
        timeout=30,
    )
    if result.returncode:
        raise ValueError("deployment ACL must allow writes only to the operator, SYSTEM and administrators")
