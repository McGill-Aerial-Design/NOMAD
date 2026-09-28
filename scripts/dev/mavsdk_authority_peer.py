# SPDX-License-Identifier: Apache-2.0
"""Controllable vehicle peer for authority wire qualification."""

from mavsdk_peer import VehiclePeer


class AuthorityPeer(VehiclePeer):
    """Change ACK and telemetry behavior between test operations."""

    def set_ack_result(self, result: int | None) -> None:
        self._ack_result = result

    def set_streaming(self, enabled: bool) -> None:
        self._streaming = enabled
