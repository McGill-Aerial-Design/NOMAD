# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Report lexical C++ function metrics for tracked first-party source files."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import asdict
from pathlib import Path, PurePosixPath
from typing import Any

from scripts.dev.cpp_complexity_scanner import FileMetric, FunctionMetric, _function_rows

CPP_EXTENSIONS = {".c", ".cc", ".cpp", ".cxx", ".h", ".hpp", ".hxx"}
EXCLUDED_PARTS = {
    ".git",
    ".pixi",
    ".venv",
    "build",
    "build-core",
    "dist",
    "external",
    "generated",
    "local",
    "node_modules",
    "obj",
    "site",
    "third_party",
    "vendor",
    "vendored",
}
EXCLUDED_PATHS = {("mission_planner", "packaging"), ("mission_planner", "third_party")}
METRIC_DEFINITIONS = {
    "span_lines": "Physical lines from the detected declaration or lambda capture through its closing brace.",
    "decision_counts": (
        "Occurrences of if/for/while/case/catch/&&/||/? after comments, literals and "
        "preprocessor directives are blanked."
    ),
    "lexical_complexity_proxy": "1 plus decision-token counts; not compiler CFG cyclomatic complexity.",
    "maximum_brace_depth": "Maximum lexical brace depth including initializer/lambda braces; not control nesting.",
}
KNOWN_LIMITS = [
    "Macros are not expanded; macro definition lines are listed per file.",
    "Preprocessor branches are not evaluated; conditional directive counts are reported per file.",
    "Constructors with braced member initializers are listed per file but their body is not measured.",
    "Operator-function definitions and function-pointer declarators are not modeled.",
    "C++ grammar is not fully parsed; review coverage and unsupported constructs before any ratchet.",
    "Decisions in nested lambdas also contribute to the enclosing lexical body and appear in a lambda row.",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    output = parser.add_mutually_exclusive_group()
    output.add_argument("--json", action="store_true", help="Write the full versioned report to stdout")
    output.add_argument("--output", type=Path, help="Write the full versioned report to this JSON file")
    return parser.parse_args()


def _run_git(root: Path, *args: str) -> bytes:
    result = subprocess.run(["git", "-C", str(root), *args], check=False, capture_output=True, timeout=60)
    if result.returncode:
        message = result.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"git {' '.join(args)} failed: {message or 'unknown error'}")
    return result.stdout


def tracked_cpp_paths(root: Path) -> list[Path]:
    """Select source files from the Git index, excluding vendor and generated paths."""
    output = _run_git(root, "ls-files", "--cached", "-z")
    names = (item.decode("utf-8", errors="surrogateescape") for item in output.split(b"\0") if item)
    return sorted(root / name for name in names if _is_reportable_path(name) and (root / name).is_file())


def _is_reportable_path(name: str) -> bool:
    path = PurePosixPath(name)
    parts = set(path.parts)
    if path.suffix.lower() not in CPP_EXTENSIONS or parts & EXCLUDED_PARTS:
        return False
    return not any(_contains_parts(path.parts, excluded) for excluded in EXCLUDED_PATHS)


def _contains_parts(parts: tuple[str, ...], excluded: tuple[str, ...]) -> bool:
    return any(parts[index : index + len(excluded)] == excluded for index in range(len(parts) - len(excluded) + 1))


def _summary(files: list[FileMetric], function_count: int) -> dict[str, int]:
    return {
        "tracked_cpp_files": len(files),
        "detected_functions_and_lambdas": function_count,
        "files_with_macros": sum(bool(item.macro_definition_lines) for item in files),
        "macro_definitions": sum(len(item.macro_definition_lines) for item in files),
        "files_with_preprocessor_conditionals": sum(bool(item.preprocessor_conditional_count) for item in files),
        "preprocessor_conditional_directives": sum(item.preprocessor_conditional_count for item in files),
        "unsupported_braced_constructor_initializers": sum(
            len(item.braced_constructor_initializer_lines) for item in files
        ),
    }


def _report_scope(tracked_worktree_clean: bool) -> dict[str, Any]:
    return {
        "tracked_files_only": True,
        "tracked_worktree_content": "Current checkout contents for tracked paths.",
        "tracked_worktree_clean": tracked_worktree_clean,
        "extensions": sorted(CPP_EXTENSIONS),
        "excluded_path_parts": sorted(EXCLUDED_PARTS),
        "excluded_paths": sorted("/".join(parts) for parts in EXCLUDED_PATHS),
    }


def build_report(root: Path) -> dict[str, Any]:
    head = _run_git(root, "rev-parse", "HEAD").decode("ascii").strip()
    dirty = bool(_run_git(root, "status", "--porcelain", "--untracked-files=no").strip())
    files: list[FileMetric] = []
    functions: list[FunctionMetric] = []
    for path in tracked_cpp_paths(root):
        relative = path.relative_to(root).as_posix()
        rows, file_metric = _function_rows(relative, path.read_text(encoding="utf-8", errors="replace"))
        files.append(file_metric)
        functions.extend(rows)
    functions.sort(key=lambda item: (item.path, item.start_line, item.symbol))
    return {
        "schema_version": 1,
        "analyzer": "nomad-cpp-lexical-v1",
        "language": "C++",
        "head_sha": head,
        "scope": _report_scope(not dirty),
        "metric_definitions": METRIC_DEFINITIONS,
        "known_limits": KNOWN_LIMITS,
        "summary": _summary(files, len(functions)),
        "files": [asdict(item) for item in files],
        "functions": [asdict(item) for item in functions],
    }


def print_summary(report: dict[str, Any]) -> None:
    print(
        "C++ lexical report: "
        f"{report['summary']['tracked_cpp_files']} tracked files, "
        f"{report['summary']['detected_functions_and_lambdas']} functions/lambdas, "
        f"analyzer {report['analyzer']} (report-only)"
    )
    rows = sorted(
        report["functions"], key=lambda item: (-item["lexical_complexity_proxy"], item["path"], item["start_line"])
    )[:20]
    for row in rows:
        print(
            f"{row['path']}:{row['start_line']} {row['symbol']} "
            f"span={row['span_lines']} decisions={row['lexical_complexity_proxy']} "
            f"brace_depth={row['maximum_brace_depth']}"
        )
    summary = report["summary"]
    print(
        "Preprocessor inventory: "
        f"{summary['files_with_macros']} files / {summary['macro_definitions']} macro definitions; "
        f"{summary['files_with_preprocessor_conditionals']} files / "
        f"{summary['preprocessor_conditional_directives']} conditional directives"
    )
    print(
        f"Unsupported braced constructor initializers omitted: {summary['unsupported_braced_constructor_initializers']}"
    )


def main() -> int:
    args = parse_args()
    try:
        report = build_report(args.root.resolve())
        if args.json:
            print(json.dumps(report, indent=2, sort_keys=True))
        else:
            print_summary(report)
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
                print("Full JSON report written to the requested output path.")
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        print(f"Report unavailable: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
