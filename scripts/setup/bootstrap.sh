#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# NOMAD development environment bootstrap (macOS / Linux)
#
# Usage:
#   bash scripts/setup/bootstrap.sh
#
# This script:
#   1. Checks prerequisites (git, pixi)
#   2. Runs `pixi install` to set up the reproducible dev environment
#   3. Installs pre-commit hooks
#   4. Smoke-tests that Edge Core imports work
# =============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ---- Step 1: prerequisites ----
info "Step 1/5 — Checking prerequisites"

if ! command -v git &>/dev/null; then
    error "Git is not installed."
    exit 1
fi
ok "Git found"

if ! command -v pixi &>/dev/null; then
    warn "Pixi not found — installing..."
    curl -fsSL https://pixi.sh/install.sh | bash
    export PATH="$HOME/.pixi/bin:$PATH"
    ok "Pixi installed"
else
    ok "Pixi found at $(command -v pixi)"
fi

# ---- Step 2: pixi install ----
info "Step 2/5 — Installing pixi environment"
pixi install
ok "Pixi environment installed"

# ---- Step 3: pre-commit hooks ----
info "Step 3/5 — Installing pre-commit hooks"
pixi run precommit || warn "pre-commit install had issues (may need 'git init' first)"
ok "Pre-commit hooks installed"

# ---- Step 4: Python import smoke-test ----
info "Step 4/5 — Smoke-testing Edge Core imports"
if pixi run python -c "from edge_core.api import create_app; print('create_app OK')" 2>&1; then
    ok "Edge Core imports OK"
else
    warn "Edge Core import test failed (may need dependencies)"
fi

# ---- Step 5: summary ----
info "Step 5/5 — Done"
echo ""
echo -e "${CYAN}======================================${NC}"
echo -e "${GREEN} NOMAD dev environment ready!${NC}"
echo -e "${CYAN}======================================${NC}"
echo ""
echo -e "${YELLOW}Quick commands:${NC}"
echo "  pixi run dev         Start Edge Core sim on :8000"
echo "  pixi run test        Run pytest"
echo "  pixi run lint        Run ruff check"
echo "  pixi run fmt         Auto-format all Python"
echo "  pixi run docs        Serve MkDocs site"
echo ""
echo -e "${YELLOW}Open the project in VS Code:${NC}"
echo "  code $REPO_ROOT"
echo ""
