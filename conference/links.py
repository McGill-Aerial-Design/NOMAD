# SPDX-License-Identifier: Apache-2.0
"""Two byte-transparent loopback gates for audience-selected telemetry outages."""

import argparse
import json
import socket
import threading
from pathlib import Path


def forward(source, destination, enabled, stop, disconnected):
    source.settimeout(0.2)
    while not stop.is_set() and not disconnected.is_set():
        try:
            data = source.recv(65536)
            if not data:
                disconnected.set()
                return
        except TimeoutError:
            continue
        except OSError:
            disconnected.set()
            return
        if enabled.is_set():
            try:
                destination.sendall(data)
            except OSError:
                disconnected.set()
                return


def gate(listen_port, sitl_port, enabled, stop):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", listen_port))
        listener.listen(1)
        listener.settimeout(0.2)
        while not stop.is_set():
            try:
                downstream, _ = listener.accept()
            except TimeoutError:
                continue
            with downstream:
                try:
                    upstream = socket.create_connection(("127.0.0.1", sitl_port), timeout=0.5)
                except OSError:
                    continue
                with upstream:
                    disconnected = threading.Event()
                    reverse = threading.Thread(target=forward, args=(upstream, downstream, enabled, stop, disconnected))
                    reverse.start()
                    try:
                        forward(downstream, upstream, enabled, stop, disconnected)
                    finally:
                        disconnected.set()
                        reverse.join(timeout=2)


def run(directory):
    stop = threading.Event()
    enabled = {name: threading.Event() for name in ("radio", "network")}
    workers = []
    for name, listen, source in (("radio", 6410, 6310), ("network", 6412, 6312)):
        enabled[name].set()
        worker = threading.Thread(target=gate, args=(listen, source, enabled[name], stop))
        worker.start()
        workers.append(worker)
    try:
        while not stop.wait(0.1):
            if (directory / "links.stop").exists():
                break
            try:
                state = json.loads((directory / "links.json").read_text(encoding="utf-8-sig"))
            except (FileNotFoundError, PermissionError, json.JSONDecodeError):
                continue
            for name in enabled:
                if state.get(name) is True:
                    enabled[name].set()
                elif state.get(name) is False:
                    enabled[name].clear()
    finally:
        stop.set()
        for worker in workers:
            worker.join(timeout=3)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    run(parser.parse_args().directory.resolve())
