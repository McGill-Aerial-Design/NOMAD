# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Report and enforce NOMAD source-size rules.

Existing oversized files are recorded in a baseline during the migration. New
or modified source files must stay under the project limits.
"""

from __future__ import annotations

import argparse
import ast
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

from scripts.dev import source_size_caps

SOURCE_EXTENSIONS = {
    ".c",
    ".cc",
    ".cpp",
    ".cs",
    ".cxx",
    ".h",
    ".hpp",
    ".ps1",
    ".py",
    ".sh",
}

# Ruff enforces the 120-column limit for Python (E501), but nothing checked
# C/C++ until now. ``.clang-format`` carries the same ColumnLimit, yet clang-format
# is not a pinned dependency and would reformat as well as wrap. Reusing this
# reporter keeps the rule on the existing toolchain and needs no new package.
CPP_EXTENSIONS = {".c", ".cc", ".cpp", ".cxx", ".h", ".hpp"}

EXCLUDED_DIRS = {
    ".claude",
    ".freebuff",
    ".git",
    ".kilo",
    ".mcp",
    ".mypy_cache",
    ".pixi",
    ".pytest_cache",
    ".ruff_cache",
    ".vscode",
    ".venv",
    "bin",
    "build",
    "build-core",
    "dist",
    "external",
    "generated",
    "local",
    "node_modules",
    "obj",
    "site",
    "vendor",
    "vendored",
    # Vendored submodules: CI checks them out as submodules and pre-commit only
    # ever sees the gitlinks, so repo size policy never applied to them.
    "third_party",
}

EXCLUDED_PATH_PARTS = {
    ("mission_planner", "packaging"),
    ("mission_planner", "third_party"),
}

EXCLUDED_FILES = {"pixi.lock"}


@dataclass(frozen=True)
class FileSize:
    path: Path
    lines: int


@dataclass(frozen=True)
class LongLine:
    path: Path
    number: int
    length: int
    text: str


@dataclass(frozen=True)
class FunctionSize:
    path: Path
    name: str
    line: int
    lines: int


@dataclass(frozen=True)
class Report:
    file_sizes: list[FileSize]
    long_lines: list[LongLine]
    functions: list[FunctionSize]


class GitCommandError(RuntimeError):
    """Raised when a required Git query cannot define the report scope."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", type=Path, help="Specific source files to inspect")
    parser.add_argument("--root", type=Path, default=Path.cwd(), help="Repository root to scan")
    parser.add_argument("--top", type=int, default=20, help="Number of rows to show per report")
    parser.add_argument("--max-file-lines", type=int, default=500)
    parser.add_argument("--max-function-lines", type=int, default=40)
    parser.add_argument("--max-line-length", type=int, default=120)
    parser.add_argument("--baseline-file", type=Path, default=Path("config/file_size_baseline.txt"))
    parser.add_argument("--baseline-function-file", type=Path, default=Path("config/function_size_baseline.txt"))
    parser.add_argument("--baseline-line-file", type=Path, default=Path("config/line_length_baseline.txt"))
    parser.add_argument("--size-caps-file", type=Path, default=Path("config/source_size_caps.json"))
    parser.add_argument("--changed-only", action="store_true")
    parser.add_argument("--fail-over-file-limit", action="store_true")
    parser.add_argument("--fail-over-function-limit", action="store_true")
    parser.add_argument(
        "--fail-line-length",
        action="store_true",
        help="Fail when a scanned C/C++ source has an unbudgeted line over --max-line-length",
    )
    parser.add_argument(
        "--fail-stale-baseline",
        action="store_true",
        help="Fail when a baseline entry no longer names an oversized file, function or tolerated line",
    )
    return parser.parse_args()


def contains_parts(parts: tuple[str, ...], needle: tuple[str, ...]) -> bool:
    if len(needle) > len(parts):
        return False
    return any(parts[index : index + len(needle)] == needle for index in range(len(parts) - len(needle) + 1))


def should_scan(path: Path, root: Path) -> bool:
    relative = path.relative_to(root)
    if path.name in EXCLUDED_FILES or path.suffix.lower() not in SOURCE_EXTENSIONS:
        return False
    if any(part in EXCLUDED_DIRS for part in relative.parts):
        return False
    return not any(contains_parts(relative.parts, excluded) for excluded in EXCLUDED_PATH_PARTS)


def git_paths(root: Path, args: list[str]) -> list[str]:
    result = subprocess.run(
        ["git", "-C", str(root), *args],
        check=False,
        capture_output=True,
    )
    if result.returncode != 0:
        message = result.stderr.decode("utf-8", errors="replace").strip()
        raise GitCommandError(f"git {' '.join(args)} failed: {message or 'unknown error'}")
    return [path.decode("utf-8", errors="surrogateescape") for path in result.stdout.split(b"\0") if path]


def tracked_source_paths(root: Path) -> list[Path]:
    """Return eligible working-tree files whose paths are in the Git index."""
    return [
        root / name
        for name in git_paths(root, ["ls-files", "--cached", "-z"])
        if (root / name).is_file() and should_scan(root / name, root)
    ]


def changed_paths(root: Path) -> list[Path]:
    base_ref = os.environ.get("NOMAD_COMPLEXITY_BASE")
    diff_args = ["diff", "--name-only", f"{base_ref}...HEAD"] if base_ref else ["diff", "--name-only", "HEAD"]
    names = set(git_paths(root, [*diff_args, "-z"]))
    if base_ref:
        names.update(git_paths(root, ["diff", "--name-only", "HEAD", "-z"]))
    names.update(git_paths(root, ["ls-files", "--others", "--exclude-standard", "-z"]))
    return [root / name for name in sorted(names) if (root / name).is_file()]


def iter_source_files(root: Path, paths: list[Path], changed_only: bool) -> list[Path]:
    if paths:
        candidates = [path if path.is_absolute() else root / path for path in paths]
    elif changed_only:
        candidates = changed_paths(root)
    else:
        candidates = tracked_source_paths(root)
    return sorted(path for path in candidates if path.is_file() and should_scan(path, root))


def scan_file(path: Path, root: Path, max_line_length: int) -> tuple[FileSize, list[LongLine]]:
    line_count = 0
    long_lines: list[LongLine] = []
    relative = path.relative_to(root)
    with path.open(encoding="utf-8", errors="replace") as source:
        for number, line in enumerate(source, start=1):
            line_count = number
            text = line.rstrip("\n")
            if len(text) > max_line_length:
                long_lines.append(LongLine(relative, number, len(text), text.strip()))
    return FileSize(relative, line_count), long_lines


def count_lines(path: Path) -> int:
    with path.open(encoding="utf-8", errors="replace") as source:
        return sum(1 for _ in source)


def count_over_length_lines(path: Path, max_line_length: int) -> int:
    with path.open(encoding="utf-8", errors="replace") as source:
        return sum(1 for line in source if len(line.rstrip("\n")) > max_line_length)


def scan_python_functions(path: Path, root: Path) -> list[FunctionSize]:
    if path.suffix != ".py":
        return []
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except SyntaxError:
        return []
    functions: list[FunctionSize] = []
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) or node.end_lineno is None:
            continue
        size = node.end_lineno - node.lineno + 1
        functions.append(FunctionSize(path.relative_to(root), node.name, node.lineno, size))
    return functions


def read_baseline(path: Path, root: Path) -> set[str]:
    baseline = path if path.is_absolute() else root / path
    if not baseline.is_file():
        return set()
    return {
        line.strip()
        for line in baseline.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.startswith("#")
    }


def find_function_size(path: Path, name: str, root: Path) -> int | None:
    """Return the line count of one function, or None when the name is gone."""
    for item in scan_python_functions(path, root):
        if item.name == name:
            return item.lines
    return None


def find_stale_file_entries(args: argparse.Namespace, root: Path) -> list[str]:
    """List baselined files that no longer exceed the source-file limit."""
    messages: list[str] = []
    for entry in sorted(read_baseline(args.baseline_file, root)):
        path = root / entry
        if not path.is_file():
            messages.append(f"{entry}: no such file")
        elif count_lines(path) <= args.max_file_lines:
            messages.append(f"{entry}: at or under the {args.max_file_lines}-line limit")
    return messages


def find_stale_function_entries(args: argparse.Namespace, root: Path) -> list[str]:
    """List baselined functions that shrank under the limit or no longer exist."""
    messages: list[str] = []
    for entry in sorted(read_baseline(args.baseline_function_file, root)):
        source, separator, name = entry.partition(":")
        path = root / source
        if not separator:
            messages.append(f"{entry}: expected a path:function entry")
        elif not path.is_file():
            messages.append(f"{entry}: no such file")
        else:
            size = find_function_size(path, name, root)
            if size is None:
                messages.append(f"{entry}: no such function")
            elif size <= args.max_function_lines:
                messages.append(f"{entry}: at or under the {args.max_function_lines}-line limit")
    return messages


def find_stale_line_length_entries(args: argparse.Namespace, root: Path) -> list[str]:
    """List baselined files with no over-length line left to tolerate."""
    messages: list[str] = []
    for entry in sorted(read_baseline(args.baseline_line_file, root)):
        path = root / entry
        if not path.is_file():
            messages.append(f"{entry}: no such file")
        elif count_over_length_lines(path, args.max_line_length) == 0:
            messages.append(f"{entry}: no line over {args.max_line_length} characters left to tolerate")
    return messages


def find_stale_size_caps(args: argparse.Namespace, root: Path) -> list[str]:
    """Reject caps that do not match the existing name-only baseline."""
    if getattr(args, "size_caps_file", None) is None:
        return []
    caps = source_size_caps.read_size_caps(args.size_caps_file, root)
    return source_size_caps.find_coverage_errors(
        read_baseline(args.baseline_file, root),
        read_baseline(args.baseline_function_file, root),
        caps,
    )


def find_stale_baseline_entries(args: argparse.Namespace, root: Path) -> list[str]:
    """List baseline entries that no longer point at an oversized file, function or line.

    A baselined name that shrank (or a file that was deleted) silently shrinks the
    gate, so each entry must still justify itself. This scans only the baselined
    files, so it is cheap even in --changed-only runs.
    """
    return (
        find_stale_file_entries(args, root)
        + find_stale_function_entries(args, root)
        + find_stale_line_length_entries(args, root)
        + find_stale_size_caps(args, root)
    )


def collect_report(files: list[Path], root: Path, max_line_length: int) -> Report:
    file_sizes: list[FileSize] = []
    long_lines: list[LongLine] = []
    functions: list[FunctionSize] = []
    for path in files:
        file_size, file_long_lines = scan_file(path, root, max_line_length)
        file_sizes.append(file_size)
        long_lines.extend(file_long_lines)
        functions.extend(scan_python_functions(path, root))
    return Report(file_sizes, long_lines, functions)


def print_table(title: str, headers: tuple[str, ...], rows: list[tuple[object, ...]]) -> None:
    print(title)
    if not rows:
        print("  none")
        return
    widths = [max(len(str(header)), *(len(str(row[index])) for row in rows)) for index, header in enumerate(headers)]
    print("  " + "  ".join(str(header).ljust(widths[index]) for index, header in enumerate(headers)))
    print("  " + "  ".join("-" * width for width in widths))
    for row in rows:
        print("  " + "  ".join(str(value).ljust(widths[index]) for index, value in enumerate(row)))


def print_report(report: Report, args: argparse.Namespace) -> None:
    largest = sorted(report.file_sizes, key=lambda item: item.lines, reverse=True)[: args.top]
    longest = sorted(report.long_lines, key=lambda item: item.length, reverse=True)[: args.top]
    oversized = [item for item in report.file_sizes if item.lines > args.max_file_lines]
    oversized_functions = [item for item in report.functions if item.lines > args.max_function_lines]
    print(f"Scanned {len(report.file_sizes)} selected source files (reported paths are root-relative)")
    print_table("\nLargest files", ("lines", "path"), [(item.lines, item.path.as_posix()) for item in largest])
    print_table(
        f"\nLongest lines over {args.max_line_length} characters",
        ("length", "line", "path"),
        [(item.length, item.number, item.path.as_posix()) for item in longest],
    )
    print_table(
        f"\nFiles over {args.max_file_lines} lines",
        ("lines", "path"),
        [(item.lines, item.path.as_posix()) for item in oversized],
    )
    print_table(
        f"\nPython functions over {args.max_function_lines} lines",
        ("lines", "line", "function", "path"),
        [(item.lines, item.line, item.name, item.path.as_posix()) for item in oversized_functions[: args.top]],
    )


def find_unbudgeted_long_lines(report: Report, args: argparse.Namespace, root: Path) -> list[LongLine]:
    """Return C/C++ over-length lines that no line-length baseline entry tolerates.

    Files recorded in the baseline carry known, pre-existing over-length lines
    that the migration deletes rather than reformats.
    """
    tolerated = read_baseline(args.baseline_line_file, root)
    return [
        item
        for item in report.long_lines
        if item.path.suffix.lower() in CPP_EXTENSIONS and item.path.as_posix() not in tolerated
    ]


def find_numeric_cap_violations(report: Report, args: argparse.Namespace, root: Path) -> list[str]:
    if not args.fail_over_file_limit and not args.fail_over_function_limit:
        return []
    return source_size_caps.check_report_caps(
        args.size_caps_file,
        root,
        read_baseline(args.baseline_file, root),
        read_baseline(args.baseline_function_file, root),
        [
            (item.path.as_posix(), item.lines)
            for item in report.file_sizes
            if args.fail_over_file_limit and item.lines > args.max_file_lines
        ],
        [
            (f"{item.path.as_posix()}:{item.name}", item.lines)
            for item in report.functions
            if args.fail_over_function_limit and item.lines > args.max_function_lines
        ],
    )


def enforce_size_limits(report: Report, args: argparse.Namespace, root: Path) -> int:
    oversized = [item for item in report.file_sizes if item.lines > args.max_file_lines]
    baseline = read_baseline(args.baseline_file, root)
    new_oversized = [item for item in oversized if item.path.as_posix() not in baseline]
    functions = [item for item in report.functions if item.lines > args.max_function_lines]
    # Legacy transitional functions are baselined by "path:function" entries so
    # all-files runs stay green while the migration deletes them. New offenders
    # (any function not in the baseline) still fail.
    baseline_functions = read_baseline(args.baseline_function_file, root)
    target_functions = [item for item in functions if f"{item.path.as_posix()}:{item.name}" not in baseline_functions]
    if args.fail_over_file_limit and new_oversized:
        print("\nNew or selected files exceed the source-file limit:")
        for item in new_oversized:
            print(f"  {item.path.as_posix()} ({item.lines} lines)")
        return 1
    if args.fail_over_function_limit and target_functions:
        print("\nTarget Python functions exceed the function limit:")
        for item in target_functions:
            print(f"  {item.path.as_posix()}:{item.line} {item.name} ({item.lines} lines)")
        return 1
    return 0


def enforce_report(report: Report, args: argparse.Namespace, root: Path) -> int:
    if args.fail_stale_baseline:
        stale = find_stale_baseline_entries(args, root)
        if stale:
            print("\nStale baseline entries (no longer oversized):")
            for message in stale:
                print(f"  {message}")
            return 1
    cap_errors = find_numeric_cap_violations(report, args, root)
    if cap_errors:
        print("\nNumerical size debt caps:")
        for message in cap_errors:
            print(f"  {message}")
        return 1
    if args.fail_line_length:
        cpp_long_lines = find_unbudgeted_long_lines(report, args, root)
        if cpp_long_lines:
            print(f"\nC/C++ lines over {args.max_line_length} characters:")
            for item in sorted(cpp_long_lines, key=lambda item: (item.path.as_posix(), item.number)):
                print(f"  {item.path.as_posix()}:{item.number} ({item.length} characters)")
            return 1
    return enforce_size_limits(report, args, root)


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    try:
        files = iter_source_files(root, args.paths, args.changed_only)
        report = collect_report(files, root, args.max_line_length)
        print_report(report, args)
        if not files:
            print("No source files were selected.")
        return enforce_report(report, args, root)
    except (GitCommandError, OSError, ValueError) as error:
        print(f"Report unavailable: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
