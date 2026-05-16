import asyncio
import json
import os
import re
import shlex
import shutil
import subprocess
import time
from datetime import datetime, timezone
from typing import Any, Optional

from fastapi import HTTPException, Query, Request, WebSocket
from fastapi.encoders import jsonable_encoder
from fastapi.responses import FileResponse
from fastapi.websockets import WebSocketDisconnect
from pydantic import BaseModel
from starlette.responses import JSONResponse

from ..api_models import (
    COMMAND_WHITELIST,
    MSGPACK_AVAILABLE,
    Task1CapturesList,
    Task2HitRequest,
    TerminalCommandRequest,
    TerminalCommandResponse,
    TerminalExecRequest,
    VIOAreaLoadRequest,
    VIOAreaSaveRequest,
    VIOUpdateRequest,
    NavPositionRequest,
    NavVelocityRequest,
)

try:
    import msgpack
except ImportError:  # pragma: no cover - optional Jetson dependency
    msgpack = None

def register_terminal_routes(app, ctx) -> None:

    def _require_terminal_api_key() -> None:
        """
        Guard for terminal/admin routes.

        When NOMAD_API_KEY is configured, APIKeyMiddleware enforces X-API-Key.
        When NOMAD_API_KEY is not configured, treat this as development mode and
        allow execution so Mission Planner operations (e.g., Git Update) continue
        to work.
        """
        return

    # ==================== Terminal Endpoints ======================================

    @app.post(
        "/api/terminal/run", tags=["Terminal"], response_model=TerminalCommandResponse
    )
    async def execute_terminal_command(request: TerminalCommandRequest):
        """
        Execute a whitelisted shell command on the Jetson.

        Only commands in the whitelist are allowed. To see available commands,
        use GET /api/terminal/commands.

        Common uses:
        - System diagnostics
        - Network troubleshooting
        - Service management
        """
        _require_terminal_api_key()

        # Validate command_name is in whitelist
        if request.command_name not in COMMAND_WHITELIST:
            available = list(COMMAND_WHITELIST.keys())
            raise HTTPException(
                status_code=400,
                detail=f"Command '{request.command_name}' not allowed. Available: {available}",
            )

        command_str = COMMAND_WHITELIST[request.command_name]

        try:
            # For commands with pipes or redirects, use shell=True
            # (safe because the command itself is whitelisted)
            if "|" in command_str or ">" in command_str or "<" in command_str:
                result = await asyncio.to_thread(
                    subprocess.run,
                    command_str,
                    shell=True,
                    capture_output=True,
                    text=True,
                    timeout=request.timeout,
                )
            else:
                # For simple commands, use shell=False with list
                cmd_parts = shlex.split(command_str)
                result = await asyncio.to_thread(
                    subprocess.run,
                    cmd_parts,
                    shell=False,
                    capture_output=True,
                    text=True,
                    timeout=request.timeout,
                )

            return TerminalCommandResponse(
                success=result.returncode == 0,
                stdout=result.stdout,
                stderr=result.stderr,
                return_code=result.returncode,
                command_executed=command_str,
            )

        except subprocess.TimeoutExpired:
            return TerminalCommandResponse(
                success=False,
                stdout="",
                stderr=f"Command timed out after {request.timeout}s",
                return_code=-1,
                command_executed=command_str,
            )
        except Exception as e:
            return TerminalCommandResponse(
                success=False,
                stdout="",
                stderr=str(e),
                return_code=-1,
                command_executed=command_str,
            )

    @app.post(
        "/api/terminal/exec", tags=["Terminal"], response_model=TerminalCommandResponse
    )
    async def exec_terminal_command(request: TerminalExecRequest):
        """
        Execute an arbitrary shell command on the Jetson.

        Intended for the Mission Planner built-in terminal.
        Commands are executed via ``bash -c`` so pipes, redirects, and
        compound statements work as expected.

        Supports persistent working directory via the ``cwd`` field.
        The response includes the resolved ``cwd`` after execution so
        the client can track directory changes across commands.
        """
        _require_terminal_api_key()

        import os

        command_str = request.command.strip()
        if not command_str:
            raise HTTPException(status_code=400, detail="Empty command")

        # Resolve working directory
        work_dir = request.cwd if request.cwd else os.path.expanduser("~")
        if not os.path.isdir(work_dir):
            work_dir = os.path.expanduser("~")

        try:
            # Append pwd to capture the cwd after execution
            # This handles cd commands naturally since bash runs them in sequence
            wrapped_cmd = f'{command_str}\necho "__NOMAD_CWD__$(pwd)"'

            result = await asyncio.to_thread(
                subprocess.run,
                ["bash", "-c", wrapped_cmd],
                capture_output=True,
                text=True,
                timeout=request.timeout,
                cwd=work_dir,
            )

            # Extract cwd from stdout
            stdout_lines = result.stdout.split("\n")
            new_cwd = work_dir
            clean_stdout_lines = []
            for line in stdout_lines:
                if line.startswith("__NOMAD_CWD__"):
                    new_cwd = line[len("__NOMAD_CWD__") :]
                else:
                    clean_stdout_lines.append(line)
            clean_stdout = "\n".join(clean_stdout_lines)

            return TerminalCommandResponse(
                success=result.returncode == 0,
                stdout=clean_stdout,
                stderr=result.stderr,
                return_code=result.returncode,
                command_executed=command_str,
                cwd=new_cwd,
            )

        except subprocess.TimeoutExpired:
            return TerminalCommandResponse(
                success=False,
                stdout="",
                stderr=f"Command timed out after {request.timeout}s",
                return_code=-1,
                command_executed=command_str,
                cwd=work_dir,
            )
        except Exception as e:
            return TerminalCommandResponse(
                success=False,
                stdout="",
                stderr=str(e),
                return_code=-1,
                command_executed=command_str,
                cwd=work_dir,
            )

    @app.get("/api/terminal/commands", tags=["Terminal"])
    async def list_terminal_commands():
        """
        List all available whitelisted terminal commands.

        Returns a dictionary mapping command names to their actual shell commands.
        """
        return {"commands": COMMAND_WHITELIST}

    @app.get("/api/terminal/logs", tags=["Terminal"])
    async def get_service_logs(
        service: str = Query("edge_core", description="Service name"),
        lines: int = Query(50, description="Number of lines"),
    ):
        """Get recent logs for a service."""
        _require_terminal_api_key()

        try:
            result = await asyncio.to_thread(
                subprocess.run,
                ["journalctl", "-u", service, "-n", str(lines), "--no-pager"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            return {
                "service": service,
                "logs": result.stdout,
                "lines": lines,
            }
        except Exception as e:
            return {"error": str(e)}

