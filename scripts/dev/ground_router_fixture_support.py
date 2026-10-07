# SPDX-License-Identifier: Apache-2.0
"""Private router configuration and management requests for process fixtures."""

import json
import socket


def management_request(port, message):
    message = dict(message)
    message.setdefault("protocol", "nomad-link-router")
    message.setdefault("version", 1)
    with socket.create_connection(("127.0.0.1", port), timeout=2) as client:
        client.settimeout(2)
        client.sendall((json.dumps(message) + "\n").encode("utf-8"))
        response = b""
        while not response.endswith(b"\n"):
            chunk = client.recv(4096)
            if not chunk:
                raise AssertionError("management endpoint closed before its response")
            response += chunk
        return json.loads(response.decode("utf-8"))


def config_for(ports, consumer_ports, management_port):
    return {
        "PreferredLink": "a",
        "ManagementPort": management_port,
        "HeartbeatTimeoutSec": 0.3,
        "StatsTickMs": 20,
        "Links": [
            {"Id": name, "Port": port, "Priority": 100 - i}
            for i, (name, port) in enumerate(zip(("a", "b", "c"), ports[:3], strict=True))
        ],
        "Consumers": [
            {
                "Id": name,
                "RouterPort": port,
                "ClientPort": consumer_ports[i],
                "AllowOutbound": i != 0,
            }
            for i, (name, port) in enumerate(zip(("mission_planner", "nomad_core"), ports[3:5], strict=True))
        ],
    }
