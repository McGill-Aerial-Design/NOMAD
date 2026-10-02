# SPDX-License-Identifier: Apache-2.0
"""Qualify the production supervisor wire-up using synthetic fixture version metadata."""

from __future__ import annotations

import json

from scripts.release import storage
from scripts.release.router import RouterAdapter, wait


class FixtureSupervisor(RouterAdapter):
    """Only translate synthetic fixture identity; OS pointer/marker/start logic stays production."""

    fail_version: str | None = None
    fail_start = False

    def start(self) -> None:
        if not self.fail_start:
            super().start()
            return
        self.fail_start = False
        original = self.config.read_bytes()
        try:
            self.config.write_text(json.dumps({"Links": [], "Consumers": []}), encoding="utf-8")
            super().start()
            wait(
                lambda: self.process.exists() and storage.read_json(self.process).get("status") == "failed",
                "candidate startup did not exit as intended",
            )
        finally:
            self.config.write_bytes(original)

    def expected_version(self, candidate: dict) -> dict:
        label = "A" if candidate["source_sha"] == "a" * 40 else "B"
        return {**candidate, "version": "0.0.0-fixture." + label}

    def preflight(self, candidate: dict) -> None:
        super().preflight(self.expected_version(candidate))

    def health(self, candidate: dict) -> None:
        super().health(self.expected_version(candidate))
        if candidate["release_version"] == self.fail_version:
            raise RuntimeError("intentional supervisor candidate health failure")
