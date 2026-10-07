# SPDX-License-Identifier: Apache-2.0
"""Prove protected service configuration loads, replaces and reloads actuator state."""

from __future__ import annotations

import json
import os
import socket
from pathlib import Path

from mavsdk_peer import VehiclePeer
from runtime_fixture_support import ROOT, authority_fields, free_port, request, require
from runtime_lifecycle_fixture import ProcessSupervisor, deployment, wait_for, write_private_json
from runtime_qualification_support import admit, status, wait_for_vehicle


def send_mutation(ipc: int, request_id: str, kind: str, **fields: object) -> dict:
    """Bind a fresh authenticated request to the current admitted owner."""
    hello = request(ipc, request_id + "-hello", "hello", client_id="runtime-smoke")
    return request(
        ipc, request_id, kind, client_id="runtime-smoke", **authority_fields(hello, "runtime-smoke"), **fields
    )


def mutate(ipc: int, request_id: str, kind: str, **fields: object) -> dict:
    response = send_mutation(ipc, request_id, kind, **fields)
    require(response["ok"], f"actuator deployment {kind} succeeded: {response.get('error')}")
    return response


def verify_unwritable_directory(ipc: int, path: Path, definitions: list[dict]) -> None:
    """A definite staging failure preserves state and is reported, rather than acknowledged as saved."""
    if os.name == "nt":
        return
    original = path.read_bytes()
    path.parent.chmod(0o500)
    try:
        response = send_mutation(ipc, "deployment-unwritable", "configure_actuators", actuator_configs=definitions)
        require(
            response["error"]["code"] == "actuator_configuration_not_saved", "unwritable parent rejects replacement"
        )
        require(path.read_bytes() == original, "failed replacement preserves the original protected configuration")
    finally:
        path.parent.chmod(0o700)


def replace_configuration(ipc: int, path: Path, definitions: list[dict]) -> None:
    """Assert actual persisted contents independently from the IPC response."""
    mutate(ipc, "deployment-configure", "configure_actuators", actuator_configs=definitions)
    require(json.loads(path.read_text())["actuator_configs"] == definitions, "replacement persisted exact definitions")
    require(not path.with_name(path.name + ".tmp").exists(), "atomic replacement left no staging file")


def run_deployment_cycle(binary: Path, config: Path, directory: Path, ipc: int, definition: dict, clear: bool) -> None:
    path = directory / "actuators" / "actuators.json"
    supervisor = ProcessSupervisor([str(binary), "--config", str(config)], directory)
    supervisor.start()
    try:
        wait_for(lambda: status(ipc)["runtime_ready"], "actuator runtime did not start")
        wait_for_vehicle(ipc)
        admit(ipc)
        catalog = request(ipc, "deployment-discover", "get_actuators", client_id="runtime-smoke")
        require(catalog["actuators"][0]["name"] == definition["name"], "startup loaded protected definitions")
        safe = mutate(
            ipc,
            "deployment-safe",
            "actuator_action",
            actuator_id=definition["id"],
            operation="safe",
            input_source="ui",
        )
        require(safe["command_result"]["success"], "explicit safe command succeeded before configuration")
        if not clear:
            definition["name"] = "Reviewed replacement"
            verify_unwritable_directory(ipc, path, [definition])
            replace_configuration(ipc, path, [definition])
        else:
            replace_configuration(ipc, path, [])
    finally:
        supervisor.close()


def verify_actuator_deployment(binary: Path, directory: Path) -> None:
    """Use the managed --config entrypoint and the same writable layout on both operating systems."""
    udp, ipc = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM)
    config, settings = deployment(directory, udp, ipc)
    parent = directory / "actuators"
    parent.mkdir(mode=0o700)
    path = parent / "actuators.json"
    definition = json.loads((ROOT / "config/actuators.example.json").read_text())["actuator_configs"][0]
    write_private_json(path, {"actuator_configs": [definition]})
    settings["NOMAD_ACTUATORS_FILE"] = str(path)
    config.write_text(json.dumps(settings))
    peer = VehiclePeer(udp, 1)
    peer.start()
    try:
        for cycle in range(2):
            run_deployment_cycle(binary, config, directory, ipc, definition, clear=cycle == 1)
    finally:
        peer.stop()
