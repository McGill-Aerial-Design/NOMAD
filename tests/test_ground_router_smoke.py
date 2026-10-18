# SPDX-License-Identifier: Apache-2.0
"""Router fixture reservations follow the protocol of each actual endpoint."""

import contextlib
import importlib
import json
import socket
from pathlib import Path

import pytest


@pytest.fixture
def smoke(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts/dev"))
    return importlib.import_module("ground_router_smoke")


def test_management_reservation_avoids_tcp_ports_occupied_beside_udp(smoke, monkeypatch):
    class ReachedHost(Exception):
        pass

    with contextlib.ExitStack() as held:

        def reserve(kind=socket.SOCK_DGRAM):
            if kind == socket.SOCK_DGRAM:
                listener = held.enter_context(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
                listener.bind(("127.0.0.1", 0))
                listener.listen()
                address = listener.getsockname()
            else:
                address = ("127.0.0.1", 0)
            endpoint = socket.socket(socket.AF_INET, kind)
            endpoint.bind(address)
            endpoint.settimeout(0.05)
            return endpoint

        def probe_start(config_path, output):
            config = json.loads(config_path.read_text(encoding="utf-8"))
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as management:
                management.bind(("127.0.0.1", config["ManagementPort"]))
            raise ReachedHost

        monkeypatch.setattr(smoke, "peer", reserve)
        monkeypatch.setattr(smoke, "check_invalid_configs", lambda *args: None)
        monkeypatch.setattr(smoke, "start_host", probe_start)
        with pytest.raises(ReachedHost):
            smoke.main()


def test_release_check_does_not_require_unused_management_udp_port(smoke):
    with contextlib.ExitStack() as held:
        management = held.enter_context(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
        management.bind(("127.0.0.1", 0))
        port = management.getsockname()[1]
        unrelated_udp = held.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        unrelated_udp.bind(("127.0.0.1", port))
        reserved = [smoke.peer() for _ in range(5)]
        ports = [endpoint.getsockname()[1] for endpoint in reserved] + [port]
        for endpoint in reserved:
            endpoint.close()
        management.close()
        smoke.assert_ports_released(ports, port)
