# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Keep the Windows qualification filters aligned with first-party Python inputs."""

import ast
import fnmatch
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
WORKFLOW = ROOT / ".github/workflows/csharp.yml"


def resolve_modules(path: Path, node: ast.AST) -> set[Path]:
    if isinstance(node, ast.Import):
        modules = [alias.name for alias in node.names]
        bases = [path.parent, ROOT]
    elif isinstance(node, ast.ImportFrom):
        module = node.module or ""
        modules = [module, *[module + "." + alias.name for alias in node.names]]
        base = path.parent
        for _ in range(max(0, node.level - 1)):
            base = base.parent
        bases = [base] if node.level else [path.parent, ROOT]
    else:
        return set()
    candidates = {base / (name.replace(".", "/").lstrip("/") + ".py") for base in bases for name in modules}
    return {candidate for candidate in candidates if candidate.is_file()}


def get_python_dependencies(seeds: set[Path]) -> set[Path]:
    pending, visited = set(seeds), set()
    while pending:
        path = pending.pop()
        if path in visited:
            continue
        visited.add(path)
        tree = ast.parse(path.read_text(encoding="utf-8"))
        # Router qualification passes binaries explicitly; it never uses runtime/CLI discovery.
        discovery = {"find_runtime", "find_cli"}
        references = [node.id for node in ast.walk(tree) if isinstance(node, ast.Name) and node.id in discovery]
        assert not references, f"{path.name} now references discovery helpers; extend the workflow filters"
        deferred = [
            node
            for node in ast.walk(tree)
            if path.name == "runtime_fixture_support.py"
            and isinstance(node, ast.FunctionDef)
            and node.name in discovery
        ]
        for node in ast.walk(tree):
            if not any(function.lineno <= getattr(node, "lineno", 0) <= function.end_lineno for function in deferred):
                pending.update(resolve_modules(path, node) - visited)
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                continue
            if node.func.attr not in {"with_name", "Popen", "run", "check_output", "check_call"}:
                continue
            for value in (item.value for item in ast.walk(node) if isinstance(item, ast.Constant)):
                if not isinstance(value, str) or not value.endswith(".py"):
                    continue
                for candidate in (path.parent / value, ROOT / value):
                    if candidate.is_file():
                        pending.add(candidate)
    return visited


def get_event_paths(workflow: str, event: str) -> list[str]:
    events = workflow.split("\non:\n", maxsplit=1)[1].split("\npermissions:", maxsplit=1)[0]
    block = re.search(rf"^  {event}:\n(.*?)(?=^  \w+:|\Z)", events, re.MULTILINE | re.DOTALL)
    assert block is not None, f"Missing {event} workflow event"
    return re.findall(r'^\s+- "([^"]+)"', block.group(1), re.MULTILINE)


@pytest.mark.parametrize("event", ["push", "pull_request"])
def test_csharp_filters_cover_transitive_qualification_dependencies(event):
    workflow = WORKFLOW.read_text(encoding="utf-8")
    paths = get_event_paths(workflow, event)
    seeds = {ROOT / path for path in re.findall(r"\bpython\s+(scripts/[^\s]+\.py)", workflow)}
    for script in ("build_ground_router.ps1", "build_plugin_windows.ps1"):
        assert script in workflow, f"Hosted workflow bypasses {script}"
        content = (ROOT / "scripts/build" / script).read_text(encoding="utf-8")
        assert "scripts/release/identity.py" in content
    seeds.add(ROOT / "scripts/release/identity.py")
    dependencies = {path.relative_to(ROOT).as_posix() for path in get_python_dependencies(seeds)}
    expected = {
        "scripts/dev/mavsdk_peer.py",
        "scripts/dev/mavsdk_peer_route.py",
        "scripts/dev/mavsdk_peer_transition.py",
        "scripts/dev/release_router_launcher.py",
        "scripts/dev/release_supervisor_fixture.py",
        "scripts/dev/verify_core_package.py",
        "scripts/release/router_host.py",
    }
    assert expected <= dependencies, f"Dependency scan missed {sorted(expected - dependencies)}"
    missing = sorted(path for path in dependencies if not any(fnmatch.fnmatchcase(path, pattern) for pattern in paths))
    assert not missing, f"{event} filters omit qualification inputs: {missing}"


def test_plugin_settings_stop_old_joystick_before_replacing_output_transport():
    plugin = (ROOT / "mission_planner/src/Plugin/NOMADPlugin.cs").read_text(encoding="utf-8")
    settings = plugin.split("private void ShowSettings()", maxsplit=1)[1]
    assert settings.index("_joystickService?.Stop()") < settings.index("OutputController.Initialize(_config)")
    assert settings.index("OutputController.Initialize(_config)") < settings.index(
        "_joystickService?.UpdateConfig(_config)"
    )
