# SPDX-License-Identifier: Apache-2.0
"""Owned test-only UDP/RTP to length-framed TCP bridge; never parses aircraft telemetry."""

import select
import socket
import struct
import threading


class RtpRelay:
    def __init__(self, udp_port=15601, tcp_port=15761, tcp_host="127.0.0.1"):
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client = None
        self.pending = None
        self.offset = 0
        self.stopping = threading.Event()
        self.stats = {
            "received": 0,
            "forwarded": 0,
            "dropped_before_client": 0,
            "dropped_backpressure": 0,
            "error": None,
        }
        try:
            # Multi-Mbit/s keyframes arrive as bursts of hundreds of datagrams; the 208 KB default overflowed.
            # The kernel caps this at twice net.core.rmem_max; the effective size is recorded, not assumed.
            self.udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
            self.stats["udp_receive_buffer"] = self.udp.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
            self.udp.bind(("127.0.0.1", udp_port))
            self.server.bind((tcp_host, tcp_port))
            self.server.listen(1)
        except BaseException:
            self.udp.close()
            self.server.close()
            raise
        self.thread = threading.Thread(target=self.run, name="NOMAD RTP relay", daemon=True)
        self.thread.start()

    def run(self):
        try:
            while not self.stopping.is_set():
                current = self.client
                sockets = [self.udp, self.server] + ([current] if current is not None else [])
                writable = [current] if current is not None and self.pending is not None else []
                ready, writable, _ = select.select(sockets, writable, [], 0.1)
                if current is not None and current in ready:
                    try:
                        data = current.recv(1)
                    except BlockingIOError:
                        data = None
                    except ConnectionError:
                        data = b""
                    if data:
                        raise ValueError("Unexpected data from RTP relay viewer")
                    if data == b"":
                        self.close_client()
                if self.server in ready:
                    client, _ = self.server.accept()
                    if self.client is not None:
                        client.close()
                        raise ValueError("Unexpected second RTP relay client")
                    self.client = client
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    client.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
                    client.setblocking(False)
                if current in writable and current is self.client:
                    self.flush()
                if self.udp in ready:
                    data, _ = self.udp.recvfrom(65535)
                    if len(data) < 12 or data[0] >> 6 != 2 or data[1] & 127 != 96:
                        raise ValueError("Unexpected RTP packet format")
                    self.stats["received"] += 1
                    client = self.client
                    if client is None:
                        self.stats["dropped_before_client"] += 1
                    elif self.pending is not None:
                        # Keep the in-flight framed packet intact without blocking the simulator's UDP input.
                        self.stats["dropped_backpressure"] += 1
                    else:
                        self.pending = struct.pack("!H", len(data)) + data
                        self.offset = 0
                        self.flush()
        except (OSError, ValueError) as error:
            if not self.stopping.is_set():
                self.stats["error"] = str(error)
        finally:
            self.close_client()

    def flush(self):
        client, pending = self.client, self.pending
        if client is None or pending is None:
            return
        try:
            sent = client.send(memoryview(pending)[self.offset :])
        except BlockingIOError:
            return
        except ConnectionError:
            self.close_client()
            return
        if sent == 0:
            self.close_client()
            return
        self.offset += sent
        if self.offset == len(pending):
            self.pending = None
            self.offset = 0
            self.stats["forwarded"] += 1

    def close_client(self):
        client, self.client = self.client, None
        self.pending = None
        self.offset = 0
        if client is not None:
            client.close()

    def close(self):
        self.stopping.set()
        self.close_client()
        self.udp.close()
        self.server.close()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise TimeoutError("RTP relay thread did not stop")
