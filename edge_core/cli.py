# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Command-line parsing for the Edge Core process."""

from __future__ import annotations

import argparse
import os
from collections.abc import Sequence
from dataclasses import dataclass


@dataclass(frozen=True)
class CliArgs:
    host: str
    port: int
    log_level: str
    sim: bool


def parse_args(argv: Sequence[str] | None = None) -> CliArgs:
    parser = argparse.ArgumentParser(
        description="NOMAD Edge Core - Drone-side processing system",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host address to bind to")
    parser.add_argument("--port", type=int, default=8000, help="Port number for REST API")
    parser.add_argument(
        "--log-level",
        type=str,
        default="info",
        choices=["debug", "info", "warning", "error"],
        help="Logging level",
    )
    parser.add_argument("--sim", action="store_true", help="Enable simulation mode")

    args = parser.parse_args(argv)
    return CliArgs(
        host=args.host,
        port=args.port,
        log_level=args.log_level,
        sim=args.sim,
    )


def apply_cli_environment(args: CliArgs) -> None:
    # CLI flags are translated into environment variables here, which is the
    # single contract by which modules/services discover their configuration.
    if args.sim:
        os.environ["NOMAD_SIM_MODE"] = "true"
