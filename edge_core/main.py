# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Edge Core - Main Entry Point.

Initializes and runs the drone-side services under the NomadModule framework.
"""

from __future__ import annotations

from .platform.jetson import preload_local_libraries

preload_local_libraries()

import logging

from .api import create_app
from .cli import apply_cli_environment, parse_args
from .runtime import run
from .services.state import StateManager

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("edge_core")

# Thread-safe global state manager singleton
state_manager = StateManager.instance()


def get_app():
    """Get or create the FastAPI application."""
    return create_app(state_manager)


# Module-level singleton created at import time. uvicorn's reload/worker modes
# import the app by string ("edge_core.main:app"), so this must exist at module
# scope — do not move it into run()/main() or uvicorn cannot find it.
app = get_app()


def main() -> None:
    """CLI entry point with argument parsing."""
    args = parse_args()
    apply_cli_environment(args)
    run(app, state_manager, logger, host=args.host, port=args.port, log_level=args.log_level)


if __name__ == "__main__":
    main()
