# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
import asyncio
import os
import shlex
import signal
import subprocess

from fastapi import HTTPException, Query

from ..api_models import (
    COMMAND_WHITELIST,
    TerminalCommandRequest,
    TerminalCommandResponse,
    TerminalExecRequest,
)

try:
    import msgpack
except ImportError:  # pragma: no cover - optional Jetson dependency
    msgpack = None


def _run_subprocess_group(
    cmd,
    *,
    shell: bool,
    timeout: float,
    cwd: str | None = None,
) -> subprocess.CompletedProcess:
    """Run a subprocess with timeout, killing the whole process group on hang.

    subprocess.run() with timeout sends SIGKILL only to the immediate shell.
    Children spawned by the shell (ping, curl, ssh, ...) survive as zombies
    re-parented to init. This helper puts the child in its own session via
    start_new_session=True so we can os.killpg() the entire tree.

    On non-POSIX hosts, falls back to subprocess.run() unchanged.
    """
    if os.name != "posix":
        return subprocess.run(
            cmd,
            shell=shell,
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd=cwd,
        )

    proc = subprocess.Popen(
        cmd,
        shell=shell,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        cwd=cwd,
        start_new_session=True,
    )
    try:
        stdout, stderr = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)  # type: ignore[attr-defined]
        except (ProcessLookupError, PermissionError):
            pass
        # Drain pipes so they're not left half-open in the FD table.
        try:
            stdout, stderr = proc.communicate(timeout=1.0)
        except Exception:
            stdout, stderr = "", ""
        raise subprocess.TimeoutExpired(cmd, timeout, output=stdout, stderr=stderr)
    return subprocess.CompletedProcess(cmd, proc.returncode, stdout, stderr)


def register_terminal_routes(app, ctx) -> None:
    _require_terminal_api_key = ctx.require_terminal_api_key

    def _terminal_exec_enabled() -> bool:
        return (os.environ.get("NOMAD_ENABLE_TERMINAL_EXEC") or "").strip().lower() in {"1", "true", "yes", "on"}

    # ==================== Terminal Endpoints ======================================

    @app.post("/api/terminal/run", tags=["Terminal"], response_model=TerminalCommandResponse)
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
                    _run_subprocess_group,
                    command_str,
                    shell=True,
                    timeout=request.timeout,
                )
            else:
                # For simple commands, use shell=False with list
                cmd_parts = shlex.split(command_str)
                result = await asyncio.to_thread(
                    _run_subprocess_group,
                    cmd_parts,
                    shell=False,
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

    @app.post("/api/terminal/exec", tags=["Terminal"], response_model=TerminalCommandResponse)
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
        if not _terminal_exec_enabled():
            raise HTTPException(
                status_code=403,
                detail=(
                    "Arbitrary terminal execution is disabled. Set "
                    "NOMAD_ENABLE_TERMINAL_EXEC=true only for trusted maintenance windows."
                ),
            )

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
                _run_subprocess_group,
                ["bash", "-c", wrapped_cmd],
                shell=False,
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
