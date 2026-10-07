# SPDX-License-Identifier: Apache-2.0
"""Read actual configured target sources and linker inputs from CMake's file API."""

from __future__ import annotations

import json
import re
from pathlib import Path


def read_targets(build: Path) -> dict[str, dict]:
    reply = build / ".cmake/api/v1/reply"
    indices = sorted(reply.glob("index-*.json"))
    if not indices:
        raise ValueError("CMake file-API evidence is missing; run the resource configure/build")
    index = json.loads(indices[-1].read_text(encoding="utf-8"))
    model = json.loads((reply / index["reply"]["codemodel-v2"]["jsonFile"]).read_text(encoding="utf-8"))
    configuration = next(item for item in model["configurations"] if item["name"] == "Release")
    return {
        item["name"]: json.loads((reply / item["jsonFile"]).read_text(encoding="utf-8"))
        for item in configuration["targets"]
        if item["name"] in {"mavsdk", "nomad-runtime", "nomad"}
    }


def linked_archive_names(target: dict) -> list[str]:
    names = []
    for item in target.get("link", {}).get("commandFragments", []):
        if item["role"] != "libraries":
            continue
        for match in re.finditer(r'[^\s";]+\.(?:a|lib)\b', item["fragment"]):
            names.append(match[0].replace("\\", "/").split("/")[-1])
    return sorted(set(names))


def collect_composition(build: Path, inventory: dict) -> dict:
    targets = read_targets(build)
    plugins = set()
    for source in targets["mavsdk"]["sources"]:
        match = re.search(r"/plugins/([^/]+)/", source["path"].replace("\\", "/"))
        if match and "compileGroupIndex" in source:
            plugins.add(match[1])
    runtime_archives = linked_archive_names(targets["nomad-runtime"])
    cli_archives = linked_archive_names(targets["nomad"])
    if any("mavsdk" in name.lower() for name in cli_archives):
        raise ValueError("installed CLI unexpectedly links the vehicle transport")
    return {
        "compiled_plugins": sorted(plugins),
        "compiled_plugin_count": len(plugins),
        "runtime_link_archives": runtime_archives,
        "cli_link_archives": cli_archives,
        "static_libraries": inventory,
        "linkage": "static; linked input size is not embedded object-code size",
        "built_but_not_runtime_link_inputs": [name for name in inventory if Path(name).name not in runtime_archives],
    }
