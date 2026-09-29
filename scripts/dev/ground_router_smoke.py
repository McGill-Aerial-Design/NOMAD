# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Hardware-free standalone process smoke test; build-ground-router must run first."""

import contextlib
import json
import queue
import socket
import subprocess
import tempfile
import threading
import time
from pathlib import Path


def peer():
    result = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    result.bind(("127.0.0.1", 0))
    result.settimeout(0.05)
    return result


def marker(seq, value):
    return bytes([0xFD, 4, 0, 0, seq, 1, 7, 200, 0, 0]) + value.to_bytes(4, "little") + b"\0\0"


def receive(sock, expected, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            packet = sock.recv(4096)
        except TimeoutError:
            continue
        if packet == expected:
            return
    raise AssertionError(f"Expected frame was not received within {timeout}s")


def drain(sock):
    packets = []
    while True:
        try:
            packets.append(sock.recv(4096))
        except TimeoutError:
            return packets


def pump(sock, port, stop):
    seq = 0
    while not stop.wait(0.05):
        frame = bytes([0xFD, 9, 0, 0, seq, 1, 1, 0, 0, 0]) + bytes(11)
        sock.sendto(frame, ("127.0.0.1", port))
        seq = (seq + 1) % 256


def start_host(config_path, output):
    executable = Path("build/ground-router/nomad-link-router.exe").resolve()
    process = subprocess.Popen(
        [str(executable), str(config_path)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    threading.Thread(target=lambda: [output.put(line.strip()) for line in process.stdout], daemon=True).start()
    return process


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


def wait_status(port, predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        response = management_request(port, {"id": 90, "type": "get_status"})
        if response.get("ok") and predicate(response["status"]):
            return response["status"]
        time.sleep(0.05)
    raise AssertionError("management status did not reach the expected state")


def wait_output(output, expected):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        try:
            if output.get(timeout=0.1) == expected:
                return
        except queue.Empty:
            pass
    raise AssertionError(f"Standalone host did not report {expected}")


def config_for(ports, consumer_ports, management_port):
    return {
        "PreferredLink": "a",
        "ManagementBindAddress": "127.0.0.1",
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


def start_pumps(ports, physical, stops, workers):
    for sock, port, stop in zip(physical, ports[:3], stops, strict=True):
        worker = threading.Thread(target=pump, args=(sock, port, stop), daemon=True)
        worker.start()
        workers.append(worker)


def exercise_selection(ports, physical, consumers, management_port, stops, workers):
    hello = management_request(management_port, {"id": 1, "type": "hello"})
    assert hello["ok"] and hello["protocol"] == "nomad-link-router"
    initial = management_request(management_port, {"id": 2, "type": "get_status"})
    assert len(initial["status"]["links"]) == 3
    selected = management_request(
        management_port,
        {"id": 3, "type": "select_link", "link": "a"},
    )
    assert selected["ok"] and selected["status"]["manualOverrideId"] == "a"
    frame = marker(1, 701)
    physical[0].sendto(frame, ("127.0.0.1", ports[0]))
    for consumer in consumers:
        receive(consumer, frame)
    assert_receive_only(ports, physical, consumers[0])
    assert_single_outbound(ports, physical, consumers[1], 0, 702)
    stops[0].set()
    workers[0].join()
    automatic = management_request(management_port, {"id": 4, "type": "set_auto"})
    assert automatic["ok"] and automatic["status"]["manualOverrideId"] == ""
    wait_status(management_port, lambda status: status["activeLinkId"] == "b")
    frame = marker(2, 703)
    physical[1].sendto(frame, ("127.0.0.1", ports[1]))
    for consumer in consumers:
        receive(consumer, frame)
    assert_single_outbound(ports, physical, consumers[1], 1, 704)


def exercise_disconnect(ports, physical, consumers, management_port, stops, workers):
    stops[1].set()
    workers[1].join()
    wait_status(management_port, lambda status: status["activeLinkId"] == "c")
    frame = marker(3, 705)
    physical[2].sendto(frame, ("127.0.0.1", ports[2]))
    for consumer in consumers:
        receive(consumer, frame)

    # A disconnected management client must not disturb the MAVLink data plane.
    with socket.create_connection(("127.0.0.1", management_port), timeout=2) as client:
        client.sendall(
            (
                json.dumps(
                    {
                        "id": 5,
                        "type": "subscribe",
                        "protocol": "nomad-link-router",
                        "version": 1,
                    }
                )
                + "\n"
            ).encode("utf-8")
        )
        client.recv(4096)
    frame = marker(4, 706)
    physical[2].sendto(frame, ("127.0.0.1", ports[2]))
    for consumer in consumers:
        receive(consumer, frame)
    restored = management_request(management_port, {"id": 6, "type": "get_status"})
    assert restored["ok"] and restored["status"]["activeLinkId"] == "c"


def exercise(ports, physical, consumers, management_port):
    stops = [threading.Event() for _ in physical]
    workers = []
    try:
        start_pumps(ports, physical, stops, workers)
        exercise_selection(ports, physical, consumers, management_port, stops, workers)
        exercise_disconnect(ports, physical, consumers, management_port, stops, workers)
    finally:
        for stop in stops:
            stop.set()
        for worker in workers:
            worker.join()


def assert_single_outbound(ports, physical, consumer, selected, value):
    for sock in physical:
        drain(sock)
    frame = marker(3, value)
    consumer.sendto(frame, ("127.0.0.1", ports[4]))
    receive(physical[selected], frame)
    time.sleep(0.15)
    for sock in physical:
        assert frame not in drain(sock), "Outbound command copied or sent to a standby transport"


def assert_receive_only(ports, physical, consumer):
    for sock in physical:
        drain(sock)
    frame = marker(4, 707)
    consumer.sendto(frame, ("127.0.0.1", ports[3]))
    time.sleep(0.15)
    for sock in physical:
        assert frame not in drain(sock), "Receive-only Mission Planner consumer reached a physical link"


def assert_ports_released(ports, management_port):
    for port in ports:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rebound:
            rebound.bind(("127.0.0.1", port))
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as rebound:
        rebound.bind(("127.0.0.1", management_port))


def main():
    with contextlib.ExitStack() as stack:
        physical = [stack.enter_context(peer()) for _ in range(3)]
        consumers = [stack.enter_context(peer()) for _ in range(2)]
        reserved = [peer() for _ in range(6)]
        ports = [sock.getsockname()[1] for sock in reserved]
        management_port = ports[5]
        config = config_for(ports, [sock.getsockname()[1] for sock in consumers], management_port)
        for sock in reserved:
            sock.close()
        directory = stack.enter_context(tempfile.TemporaryDirectory())
        config_path = Path(directory) / "router.json"
        config_path.write_text(json.dumps(config), encoding="utf-8")
        output = queue.Queue()
        process = start_host(config_path, output)
        try:
            wait_output(output, "READY")
            exercise(ports, physical, consumers, management_port)
            process.kill()
            assert process.wait(timeout=5) != 0, "Unexpected host termination was not observed"
            assert_ports_released(ports, management_port)

            output = queue.Queue()
            process = start_host(config_path, output)
            wait_output(output, "READY")
            exercise(ports, physical, consumers, management_port)
            process.stdin.write("stop\n")
            process.stdin.flush()
            assert process.wait(timeout=5) == 0, "Router shutdown failed"
            assert_ports_released(ports, management_port)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5)
    print("Standalone smoke passed: receive-only MP, core egress, failover, and process restart")


if __name__ == "__main__":
    main()
