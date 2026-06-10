# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Partition enforcement (rearchitecture plan Phase 6).

Makes the partition rule from docs/safety/partition.md a failing test instead
of a stated intention:

1. The SC core (``edge_core/safety/``) imports **stdlib and itself only** — no
   FastAPI, ROS, pymavlink, or numpy can ever leak into the decision core.
2. The rest of ``edge_core/`` reaches the core only through its public API
   (``from edge_core.safety import X``) — never submodules — so the package
   ``__init__`` stays the single reviewed doorway into SC code.
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SAFETY_DIR = REPO_ROOT / "edge_core" / "safety"
EDGE_CORE = REPO_ROOT / "edge_core"

SAFETY_SUBMODULES = {p.stem for p in SAFETY_DIR.glob("*.py") if p.stem != "__init__"}


def _imports(py_file: Path) -> list[ast.Import | ast.ImportFrom]:
    tree = ast.parse(py_file.read_text(encoding="utf-8"), filename=str(py_file))
    return [n for n in ast.walk(tree) if isinstance(n, (ast.Import, ast.ImportFrom))]


def test_safety_core_imports_stdlib_and_itself_only():
    violations: list[str] = []
    for py_file in sorted(SAFETY_DIR.glob("*.py")):
        for node in _imports(py_file):
            if isinstance(node, ast.ImportFrom):
                if node.level > 0:  # relative import = within the core
                    continue
                tops = [(node.module or "").split(".")[0]]
            else:
                tops = [alias.name.split(".")[0] for alias in node.names]
            for top in tops:
                if top and top not in sys.stdlib_module_names:
                    violations.append(f"{py_file.name}: imports {top!r}")
    assert not violations, (
        f"edge_core/safety/ must import stdlib and itself only: {violations}. "
        "If the core needs data, pass it in as an argument from the adapter."
    )


def test_safety_is_used_only_via_its_public_api():
    violations: list[str] = []
    for py_file in sorted(EDGE_CORE.rglob("*.py")):
        if SAFETY_DIR in py_file.parents:
            continue
        rel = py_file.relative_to(REPO_ROOT).as_posix()
        for node in _imports(py_file):
            if isinstance(node, ast.ImportFrom):
                # Covers absolute ("edge_core.safety[.x]") and relative
                # ("..safety[.x]") forms: anything after the "safety" segment
                # is a submodule reach-in, as is importing a submodule name
                # through the package.
                parts = (node.module or "").split(".")
                if "safety" not in parts:
                    continue
                after_safety = parts[parts.index("safety") + 1 :]
                if after_safety:
                    violations.append(f"{rel}: from {node.module} import ...")
                    continue
                submodule_imports = [a.name for a in node.names if a.name in SAFETY_SUBMODULES]
                if submodule_imports:
                    violations.append(f"{rel}: imports submodule(s) {submodule_imports} via the package")
            else:
                for alias in node.names:
                    if alias.name.startswith("edge_core.safety."):
                        violations.append(f"{rel}: import {alias.name}")
    assert not violations, (
        f"edge_core code must use the SC core only via 'from edge_core.safety import X': {violations}"
    )
