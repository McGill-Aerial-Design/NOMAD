# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Fixtures for the report-only lexical C++ function scanner."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from scripts.dev import cpp_complexity_report


def _git(root: Path, *args: str) -> str:
    result = subprocess.run(["git", "-C", str(root), *args], capture_output=True, check=False, text=True)
    assert result.returncode == 0, result.stderr
    return result.stdout.strip()


def _init_repo(root: Path) -> str:
    _git(root, "init", "--quiet")
    _git(root, "config", "user.name", "NOMAD test")
    _git(root, "config", "user.email", "nomad-test@invalid")
    source = root / "src/core.cpp"
    source.parent.mkdir(parents=True)
    source.write_text("int core() { return 1; }\n", encoding="utf-8")
    _git(root, "add", "src/core.cpp")
    _git(root, "commit", "--quiet", "-m", "base")
    return _git(root, "rev-parse", "HEAD")


def test_nested_template_lambda_and_decisions_are_reported(tmp_path: Path):
    source = tmp_path / "sample.cpp"
    source.write_text(
        """#define TRACE_IF(value) \\
            if (value) { value = false; }
#if defined(NOMAD_TEST)
#ifdef NOMAD_NESTED
#elif defined(NOMAD_OTHER)
#endif
#endif
namespace nomad {
template <typename T>
T transform(T value) noexcept(noexcept(T{})) {
    // if (value) { this comment is ignored }
    const char *text = "while (value) {";
    if (value && value > T{}) {
        return [value](T next) -> T { return next ? next : value; }(value);
    }
    return value;
}
}
""",
        encoding="utf-8",
    )

    functions, file_metric = cpp_complexity_report._function_rows("sample.cpp", source.read_text())

    assert file_metric.macro_definition_lines == [1]
    assert file_metric.preprocessor_conditional_count == 5
    assert [item.symbol.split("(", maxsplit=1)[0] for item in functions] == ["nomad::transform", "<lambda@14>"]
    transform = functions[0]
    assert transform.span_lines == 9
    assert transform.decision_counts == {"&&": 1, "?": 1, "if": 1}
    assert transform.lexical_complexity_proxy == 4
    assert functions[1].decision_counts == {"?": 1}


def test_braced_constructor_initializer_is_reported_as_unsupported(tmp_path: Path):
    source = """struct Box {
    int value;
    Box(int input) : value{input} {}
};
"""

    functions, file_metric = cpp_complexity_report._function_rows("box.cpp", source)

    assert functions == []
    assert file_metric.braced_constructor_initializer_lines == [3]


def test_parenthesized_member_initializer_with_empty_brace_argument_is_supported():
    source = """struct Runtime {
    Runtime(int connection, int config)
        : connection_(connection), vehicle_(require(connection_), {}, config) {}
};
"""

    functions, file_metric = cpp_complexity_report._function_rows("runtime.cpp", source)

    assert len(functions) == 1
    assert functions[0].symbol.startswith("Runtime(")
    assert file_metric.braced_constructor_initializer_lines == []


def test_array_subscripts_initializers_and_attributes_are_not_lambdas():
    source = """[[nodiscard]] int lookup(int items[8], int index) {
    if (items[index]) {
        int buffer[8]{};
        return buffer[index];
    }
    return 0;
}
"""

    functions, _ = cpp_complexity_report._function_rows("array.cpp", source)

    assert len(functions) == 1
    assert functions[0].symbol.startswith("lookup(")
    assert "<lambda@" not in functions[0].symbol


def test_unbalanced_source_fails_instead_of_reporting_zero_functions():
    with pytest.raises(ValueError, match=r"unbalanced C\+\+ braces"):
        cpp_complexity_report._function_rows("broken.cpp", "int run() { if (true) { return 1; }\n")


def test_report_artifact_is_versioned_and_uses_root_relative_paths(tmp_path: Path, monkeypatch, capsys):
    head = _init_repo(tmp_path)
    (tmp_path / "src/core.cpp").write_text("int core() { return 2; }\n", encoding="utf-8")
    output = tmp_path / "artifacts/cpp-report.json"
    monkeypatch.setattr(
        sys,
        "argv",
        ["cpp_complexity_report.py", "--root", str(tmp_path), "--output", str(output)],
    )

    assert cpp_complexity_report.main() == 0

    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["schema_version"] == 1
    assert report["analyzer"] == "nomad-cpp-lexical-v1"
    assert report["language"] == "C++"
    assert report["scope"]["tracked_files_only"] is True
    assert report["scope"]["tracked_worktree_clean"] is False
    assert "local" in report["scope"]["excluded_path_parts"]
    assert "generated" in report["scope"]["excluded_path_parts"]
    assert "third_party" in report["scope"]["excluded_path_parts"]
    assert report["head_sha"] == head
    assert report["summary"] == {
        "tracked_cpp_files": 1,
        "detected_functions_and_lambdas": 1,
        "files_with_macros": 0,
        "macro_definitions": 0,
        "files_with_preprocessor_conditionals": 0,
        "preprocessor_conditional_directives": 0,
        "unsupported_braced_constructor_initializers": 0,
    }
    assert report["functions"][0]["path"] == "src/core.cpp"
    assert str(tmp_path) not in output.read_text(encoding="utf-8")
    assert "Full JSON report written" in capsys.readouterr().out


def test_analyzer_failure_is_reported_as_unavailable(tmp_path: Path, monkeypatch, capsys):
    _init_repo(tmp_path)
    (tmp_path / "src/core.cpp").write_text("int broken() {\n", encoding="utf-8")
    monkeypatch.setattr(sys, "argv", ["cpp_complexity_report.py", "--root", str(tmp_path)])

    assert cpp_complexity_report.main() == 2

    captured = capsys.readouterr()
    assert "Report unavailable" in captured.err
    assert "unbalanced C++ braces" in captured.err
    assert "C++ lexical report" not in captured.out


def test_tracked_selection_excludes_local_vendor_and_generated_paths(tmp_path: Path, monkeypatch):
    names = [
        "src/core.cpp",
        "tests/core_test.cpp",
        "third_party/vendor.cpp",
        "generated/build.cpp",
        "build-core/compiler.cpp",
        "local/private.cpp",
    ]
    for name in names:
        target = tmp_path / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text("int source() { return 1; }\n", encoding="utf-8")
    monkeypatch.setattr(cpp_complexity_report, "_run_git", lambda *_args: "\0".join(names).encode())

    result = cpp_complexity_report.tracked_cpp_paths(tmp_path)

    assert [item.relative_to(tmp_path).as_posix() for item in result] == ["src/core.cpp", "tests/core_test.cpp"]
