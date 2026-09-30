# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Keep repository Markdown links aimed at files and sections that exist."""

from __future__ import annotations

import html
import os
import re
import unicodedata
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
SKIP_DIRECTORIES = {
    ".git",
    ".pixi",
    ".pytest_cache",
    ".venv",
    "build",
    "dist",
    "node_modules",
    "site",
    "third_party",
}
LINK_PATTERN = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
FENCE_PATTERN = re.compile(r"(?ms)^(`{3,}|~{3,})[^\n]*\n.*?^\1\s*$")
HEADING_PATTERN = re.compile(r"(?m)^ {0,3}#{1,6}\s+(.+?)\s*#*\s*$")
EXPLICIT_ID_PATTERN = re.compile(r"\{\s*#([\w-]+)\s*\}")
HTML_ID_PATTERN = re.compile(r"\bid=[\"']([^\"']+)[\"']")


def markdown_files() -> list[Path]:
    paths: list[Path] = []
    for directory, child_directories, filenames in os.walk(ROOT):
        child_directories[:] = [name for name in child_directories if name not in SKIP_DIRECTORIES]
        paths.extend(Path(directory, name) for name in filenames if name.lower().endswith(".md"))
    return paths


def heading_anchors(text: str) -> set[str]:
    anchors: set[str] = set()
    used: set[str] = set()
    for match in HEADING_PATTERN.finditer(text):
        title = match.group(1)
        explicit = EXPLICIT_ID_PATTERN.search(title)
        if explicit:
            anchors.add(explicit.group(1))
            title = title[: explicit.start()]
        title = re.sub(r"<[^>]*>|`|\[([^\]]+)\]\([^)]*\)", r"\1", title)
        title = unicodedata.normalize("NFKC", title).lower()
        slug = re.sub(r"[^\w -]", "", title).strip().replace(" ", "-")
        slug = re.sub(r"-+", "-", slug)
        base = slug
        suffix = 1
        while slug in used:
            slug = f"{base}-{suffix}"
            suffix += 1
        used.add(slug)
        anchors.add(slug)

    anchors.update(HTML_ID_PATTERN.findall(text))
    return anchors


def markdown_body(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    return FENCE_PATTERN.sub("", text)


def test_internal_markdown_links_resolve() -> None:
    errors: list[str] = []
    anchor_cache = {path: heading_anchors(markdown_body(path)) for path in markdown_files()}

    for source, anchors in anchor_cache.items():
        body = markdown_body(source)
        for match in LINK_PATTERN.finditer(body):
            raw_target = html.unescape(match.group(1).strip())
            destination_match = re.match(r"<([^>]+)>|([^\s]+)", raw_target)
            if destination_match is None:
                continue
            destination = destination_match.group(1) or destination_match.group(2)
            url = urlsplit(destination)
            if url.scheme or url.netloc:
                continue

            target_path = (source.parent / unquote(url.path)).resolve() if url.path else source
            if not target_path.exists():
                errors.append(f"{source.relative_to(ROOT)} links to missing file {url.path!r}")
                continue

            if url.fragment and target_path.suffix.lower() == ".md":
                target_anchors = anchor_cache.get(target_path)
                if target_anchors is None:
                    target_anchors = heading_anchors(markdown_body(target_path))
                    anchor_cache[target_path] = target_anchors
                fragment = unquote(url.fragment)
                if fragment not in target_anchors:
                    errors.append(
                        f"{source.relative_to(ROOT)} links to missing anchor "
                        f"{fragment!r} in {target_path.relative_to(ROOT)}"
                    )

    assert not errors, "\n".join(errors)
