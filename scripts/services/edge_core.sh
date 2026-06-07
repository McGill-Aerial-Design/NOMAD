#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# nomad-edge-core service
# Owns: edge_core.main FastAPI/uvicorn process on the host.
# =============================================================================
set -u
SERVICE="edge-core"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
. "$SCRIPT_DIR/../lib/common.sh"

PATTERN='edge_core\.main'
LOG_FILE_DEFAULT="${NOMAD_LOG_DIR:-/tmp}/edge_core.log"

svc_start() {
    if curl -sS --max-time 2 "$NOMAD_API_URL/health" >/dev/null 2>&1; then
        log_ok "already running and healthy"
        return 0
    fi
    kill_pattern_host "$PATTERN" 5

    local python_bin="python3"
    [ -x "$NOMAD_VENV/bin/python3" ] && python_bin="$NOMAD_VENV/bin/python3"

    export PYTHONPATH="$NOMAD_REPO_ROOT"
    export PYTHONUNBUFFERED=1
    export LD_LIBRARY_PATH="${HOME}/.local/lib:/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    mkdir -p "$NOMAD_MISSION_LOG_DIR" "$NOMAD_LOG_DIR"

    log_info "starting on $NOMAD_API_HOST:$NOMAD_API_PORT (python: $python_bin)"
    cd "$NOMAD_REPO_ROOT" || exit 1
    nohup "$python_bin" -m edge_core.main \
        --host "$NOMAD_API_HOST" --port "$NOMAD_API_PORT" --log-level info \
        > "$LOG_FILE_DEFAULT" 2>&1 &

    if wait_for "curl -sS --max-time 1 $NOMAD_API_URL/health" 30; then
        log_ok "ready at $NOMAD_API_URL"
        return 0
    fi
    log_fail "did not become healthy in 30s; see $LOG_FILE_DEFAULT"
    tail -n 20 "$LOG_FILE_DEFAULT" 2>/dev/null || true
    return 1
}

svc_stop() {
    log_info "stopping"
    kill_pattern_host "$PATTERN" 10
    log_ok "stopped"
}

svc_status() { status_pattern_host "$PATTERN"; }

svc_logs() {
    [ -f "$LOG_FILE_DEFAULT" ] && tail -n 50 -F "$LOG_FILE_DEFAULT" || \
        log_warn "no log file at $LOG_FILE_DEFAULT (running under systemd? use: journalctl -u nomad-edge-core -f)"
}

dispatch_service "$@"
