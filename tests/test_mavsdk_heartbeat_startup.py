# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Independent UDP observation of a heartbeat queued before connection publication."""

import json
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts" / "dev"))

from mavsdk_fixture_harness import find_binary  # noqa: E402

PROBE = find_binary("nomad_mavsdk_heartbeat_startup_probe")


@pytest.mark.skipif(PROBE is None, reason="build the MAVSDK heartbeat startup probe first")
@pytest.mark.parametrize("hold_ms,add_second_connection", [(0, False), (1200, False), (0, True)])
def test_queued_startup_heartbeat_preserves_wire_cadence(hold_ms: int, add_second_connection: bool) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(0.1)
        parser = mavlink.MAVLink(None)
        frames = []
        process = subprocess.Popen(
            [str(PROBE), str(receiver.getsockname()[1]), str(hold_ms), str(int(add_second_connection))],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            deadline = time.monotonic() + 12
            while process.poll() is None and time.monotonic() < deadline:
                try:
                    data, _ = receiver.recvfrom(4096)
                except TimeoutError:
                    continue
                observed = time.monotonic()
                for message in parser.parse_buffer(data) or []:
                    assert message.get_type() == "HEARTBEAT", "read-only probe emitted a mutation"
                    frames.append((observed, message))
            assert process.poll() is not None, "startup probe exceeded its deadline"
            output, error = process.communicate(timeout=1)
            assert process.returncode == 0, (output, error)
            emission = json.loads(next(line for line in output.splitlines() if line.startswith("{")))
            assert emission["connected"], emission
            assert not emission["interception_timed_out"], emission
            assert len(frames) >= 4, (frames, emission, error)
            for previous, current in zip(frames, frames[1:]):
                interval = current[0] - previous[0]
                assert 0.9 <= interval <= 1.3, (interval, emission)
                assert current[1].get_seq() != previous[1].get_seq(), "duplicate heartbeat sequence"
            for _, message in frames:
                assert (message.get_srcSystem(), message.get_srcComponent()) == (245, 190)
                assert message.type == mavlink.MAV_TYPE_GCS
                assert message.autopilot == mavlink.MAV_AUTOPILOT_INVALID
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate(timeout=2)
