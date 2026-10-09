# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Observe runtime command delivery on the isolated authority SITL UDP path."""

from __future__ import annotations

import socket
import threading
import time
from typing import Any

from authority_sitl_observer import RELAY_PORT, ProbeError
from pymavlink import mavutil

CHANNEL = 5
SET_SERVO = mavutil.mavlink.MAV_CMD_DO_SET_SERVO
NAV_LAND = mavutil.mavlink.MAV_CMD_NAV_LAND


def is_servo_ack(messages: list[Any]) -> bool:
    return any(
        message.get_type() == "COMMAND_ACK"
        and message.command == SET_SERVO
        and message.get_srcSystem() == 1
        and message.get_srcComponent() == 1
        for message in messages
    )


class AuthorityRelay:
    """Bidirectional SITL relay with servo ACK filtering and command observations."""

    def __init__(self, client_port: int) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind(("0.0.0.0", RELAY_PORT))
            self.socket.settimeout(0.1)
        except OSError:
            self.socket.close()
            raise ProbeError("authority_relay_bind_failed") from None
        self.client = ("127.0.0.1", client_port)
        self.sitl: tuple[str, int] | None = None
        self.parser = mavutil.mavlink.MAVLink(None, srcSystem=251, srcComponent=190)
        self.parser.robust_parsing = True
        self.paused, self.drop_acks, self.dropped_acks = False, False, 0
        self.frames: list[tuple[float, int, int, int]] = []
        self.land_commands: list[dict[str, Any]] = []
        self.land_acknowledgements: list[tuple[float, int]] = []
        self.heartbeats: list[tuple[float, int]] = []
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        while self.running:
            try:
                data, sender = self.socket.recvfrom(4096)
            except TimeoutError:
                continue
            except OSError as error:
                if getattr(error, "winerror", None) == 10054:
                    continue
                return
            try:
                messages = self.parser.parse_buffer(data) or []
            except Exception:
                messages = []
            if sender == self.client:
                self.record_commands(messages)
                if self.sitl is not None and not self.paused:
                    self.socket.sendto(data, self.sitl)
                continue
            self.sitl = sender
            if self.paused:
                continue
            if self.drop_acks and is_servo_ack(messages):
                self.dropped_acks += 1
                continue
            self.record_observations(messages)
            self.socket.sendto(data, self.client)

    def record_commands(self, messages: list[Any]) -> None:
        for message in messages:
            if message.get_type() != "COMMAND_LONG":
                continue
            observed_at = time.monotonic()
            system, component = message.get_srcSystem(), message.get_srcComponent()
            if message.command == SET_SERVO and int(message.param1) == CHANNEL:
                self.frames.append((observed_at, int(message.param2), system, component))
            elif message.command == NAV_LAND:
                self.land_commands.append(
                    {
                        "observed_at": observed_at,
                        "system": system,
                        "component": component,
                        "target_system": int(message.target_system),
                        "target_component": int(message.target_component),
                        "parameters": [float(getattr(message, f"param{index}")) for index in range(1, 8)],
                    }
                )

    def record_observations(self, messages: list[Any]) -> None:
        for message in messages:
            if message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
                continue
            if message.get_type() == "COMMAND_ACK" and message.command == NAV_LAND:
                self.land_acknowledgements.append((time.monotonic(), int(message.result)))
            elif message.get_type() == "HEARTBEAT":
                self.heartbeats.append((time.monotonic(), int(message.custom_mode)))

    def pause(self) -> None:
        self.paused = True

    def resume(self) -> None:
        self.paused = False

    def close(self) -> None:
        self.running = False
        self.socket.close()
        self.thread.join(timeout=1.0)
        if self.thread.is_alive():
            raise ProbeError("relay_shutdown_failed")


def runtime_source_id(relay: AuthorityRelay) -> dict[str, int]:
    sources = {(frame[2], frame[3]) for frame in relay.frames}
    if len(sources) != 1:
        raise ProbeError("runtime_command_source_unverified")
    system, component = sources.pop()
    return {"system": system, "component": component}
