# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Report the largest source files and longest source lines.

This is a visibility tool for refactors. Ruff remains the Python line-length
gate; this script keeps C#, shell, and docs hotspots visible without requiring
every existing long non-Python line to be fixed at once.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

SOURCE_EXTENSIONS = {
    ".cs",
    ".md",
    ".ps1",
    ".py",
    ".sh",
    ".toml",
    ".yaml",
    ".yml",
}

EXCLUDED_DIRS = {
    ".claude",
    ".git",
    ".kilo",
    ".mcp",
    ".mypy_cache",
    ".pixi",
    ".pytest_cache",
    ".ruff_cache",
    ".vscode",
    ".venv",
    "build",
    "dist",
    "node_modules",
    "nomad_edge.egg-info",
    "site",
}

EXCLUDED_PATH_PARTS = {
    ("mission_planner", "third_party"),
}

EXCLUDED_FILES = {
    "pixi.lock",
}


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path.cwd(), help="Repository root to scan")
    parser.add_argument("--top", type=int, default=20, help="Number of rows to show per report")
    parser.add_argument("--max-file-lines", type=int, default=800, help="Hard source-file line cap")
    parser.add_argument("--max-line-length", type=int, default=120, help="Long-line reporting threshold")
    parser.add_argument(
        "--fail-over-file-limit",
        action="store_true",
        help="Exit non-zero when a source file exceeds --max-file-lines",
    )
    return parser.parse_args()


def should_scan(path: Path, root: Path) -> bool:
    relative = path.relative_to(root)
    if path.name in EXCLUDED_FILES:
        return False
    if path.suffix.lower() not in SOURCE_EXTENSIONS:
        return False
    if any(part in EXCLUDED_DIRS for part in relative.parts):
        return False
    return not any(_contains_parts(relative.parts, excluded) for excluded in EXCLUDED_PATH_PARTS)


def _contains_parts(parts: tuple[str, ...], needle: tuple[str, ...]) -> bool:
    if len(needle) > len(parts):
        return False
    return any(parts[index : index + len(needle)] == needle for index in range(len(parts) - len(needle) + 1))


def iter_source_files(root: Path) -> list[Path]:
    return sorted(path for path in root.rglob("*") if path.is_file() and should_scan(path, root))


def scan_file(path: Path, root: Path) -> tuple[FileSize, LongLine | None]:
    line_count = 0
    longest: LongLine | None = None
    relative = path.relative_to(root)

    with path.open(encoding="utf-8", errors="replace") as source:
        for number, line in enumerate(source, start=1):
            line_count = number
            text = line.rstrip("\n")
            line_length = len(text)
            if longest is None or line_length > longest.length:
                longest = LongLine(relative, number, line_length, text.strip())

    return FileSize(relative, line_count), longest


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


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    file_sizes: list[FileSize] = []
    long_lines: list[LongLine] = []

    for path in iter_source_files(root):
        file_size, longest = scan_file(path, root)
        file_sizes.append(file_size)
        if longest is not None and longest.length > args.max_line_length:
            long_lines.append(longest)

    largest_files = sorted(file_sizes, key=lambda item: item.lines, reverse=True)[: args.top]
    longest_lines = sorted(long_lines, key=lambda item: item.length, reverse=True)[: args.top]

    print(f"Scanned {len(file_sizes)} source files under {root}")
    print_table(
        "\nLargest files",
        ("lines", "path"),
        [(item.lines, item.path.as_posix()) for item in largest_files],
    )
    print_table(
        f"\nLongest lines over {args.max_line_length} characters",
        ("length", "line", "path"),
        [(item.length, item.number, item.path.as_posix()) for item in longest_lines],
    )

    oversized = [item for item in file_sizes if item.lines > args.max_file_lines]
    if oversized:
        print_table(
            f"\nFiles over {args.max_file_lines} lines",
            ("lines", "path"),
            [(item.lines, item.path.as_posix()) for item in oversized],
        )
        if args.fail_over_file_limit:
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
