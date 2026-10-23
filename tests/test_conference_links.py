# SPDX-License-Identifier: Apache-2.0
import socket
import threading
import time

import pytest

from conference import links


def connect(port):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        client = socket.socket()
        client.settimeout(0.5)
        try:
            client.connect(("127.0.0.1", port))
            return client
        except (TimeoutError, ConnectionRefusedError):
            client.close()
            time.sleep(0.01)
    pytest.fail("Owned loopback gate did not listen within two seconds")


def receive(client, size):
    data = b""
    while len(data) < size:
        part = client.recv(size - len(data))
        assert part, "Loopback route closed before the complete response"
        data += part
    return data


class EchoPeer:
    def __init__(self):
        self.stop = threading.Event()
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(2)
        self.listener.settimeout(0.1)
        self.port = self.listener.getsockname()[1]
        self.worker = threading.Thread(target=self.run)
        self.worker.start()

    def run(self):
        while not self.stop.is_set():
            try:
                client, _ = self.listener.accept()
            except TimeoutError:
                continue
            with client:
                client.settimeout(0.1)
                while not self.stop.is_set():
                    try:
                        data = client.recv(65536)
                        if not data:
                            break
                        client.sendall(data)
                    except TimeoutError:
                        continue
                    except OSError:
                        break
        self.listener.close()


@pytest.fixture
def gates():
    existing_threads = set(threading.enumerate())
    stop = threading.Event()
    peers, workers, failures = [], [], []

    def start():
        peer = EchoPeer()
        peers.append(peer)
        enabled = threading.Event()
        enabled.set()
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            port = reservation.getsockname()[1]

        def serve():
            try:
                links.gate(port, peer.port, enabled, stop)
            except Exception as error:
                failures.append(error)

        worker = threading.Thread(target=serve)
        worker.start()
        workers.append(worker)
        return port, enabled

    yield start, stop
    stop.set()
    for peer in peers:
        peer.stop.set()
    for worker in workers + [peer.worker for peer in peers]:
        worker.join(timeout=2)
    assert not failures, "Owned gate worker failed: " + repr(failures)
    assert all(not worker.is_alive() for worker in workers + [peer.worker for peer in peers]), (
        "Non-daemon loopback workers did not stop within two seconds each"
    )
    assert not [worker for worker in threading.enumerate() if worker not in existing_threads and not worker.daemon], (
        "A return-path worker survived gate shutdown"
    )


def test_disconnect_leaves_standby_healthy_and_accepts_reconnect(gates):
    start, stop = gates
    radio_port, _ = start()
    network_port, _ = start()
    with connect(network_port) as network:
        with connect(radio_port) as radio:
            radio.sendall(b"radio-first")
            assert receive(radio, 11) == b"radio-first"
        network.sendall(b"standby-after-radio-disconnect")
        assert receive(network, 30) == b"standby-after-radio-disconnect"
        with connect(radio_port) as reconnected:
            reconnected.sendall(b"radio-reconnected")
            assert receive(reconnected, 17) == b"radio-reconnected"
        assert not stop.is_set(), "A single route disconnect stopped the whole link service"


def test_disabled_route_drops_data_and_restore_does_not_replay(gates):
    start, _ = gates
    port, enabled = start()
    with connect(port) as client:
        client.sendall(b"ready")
        assert receive(client, 5) == b"ready"
        enabled.clear()
        client.sendall(b"must-not-replay")
        with pytest.raises(socket.timeout):
            client.recv(1)
        enabled.set()
        client.sendall(b"restored")
        assert receive(client, 8) == b"restored", "Disabled data was replayed after route restoration"


def test_partial_send_timeout_closes_connection_without_continuing_stream(gates, monkeypatch):
    start, stop = gates
    original_connect = socket.create_connection

    class PartialSendTimeout:
        def __init__(self, wrapped):
            self.wrapped = wrapped

        def __getattr__(self, name):
            return getattr(self.wrapped, name)

        def __enter__(self):
            return self

        def __exit__(self, *args):
            self.wrapped.close()

        def sendall(self, data):
            self.wrapped.sendall(data[:4])
            raise TimeoutError("Injected timeout after a real partial loopback write")

    monkeypatch.setattr(
        links.socket, "create_connection", lambda *args, **kwargs: PartialSendTimeout(original_connect(*args, **kwargs))
    )
    port, _ = start()
    with connect(port) as client:
        client.sendall(b"prefix-and-unsent-tail")
        data = b""
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            try:
                chunk = client.recv(1024)
            except TimeoutError:
                continue
            if not chunk:
                break
            data += chunk
        else:
            pytest.fail("Partial send timeout left the corrupted TCP stream open")
        assert data in (b"", b"pref"), "The failed connection continued forwarding after its partial send"
        assert not stop.is_set(), "A failed connection stopped the entire service"
    monkeypatch.setattr(links.socket, "create_connection", original_connect)
    with connect(port) as recovered:
        recovered.sendall(b"clean-reconnect")
        assert receive(recovered, 15) == b"clean-reconnect"
