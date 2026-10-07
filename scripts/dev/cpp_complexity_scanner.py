# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Small lexical scanner for report-only C++ function metrics."""

from __future__ import annotations

import re
from bisect import bisect_right
from dataclasses import dataclass

CONTROL_NAMES = {
    "alignof",
    "catch",
    "decltype",
    "for",
    "if",
    "noexcept",
    "requires",
    "sizeof",
    "static_assert",
    "switch",
    "while",
}
DECISION_NAMES = {"case", "catch", "for", "if", "while"}
TOKEN_PATTERN = re.compile(r"[A-Za-z_]\w*|::|&&|\|\||->|[^\s]")
IGNORED_PATTERN = re.compile(
    r"//[^\n]*|/\*[\s\S]*?\*/|"
    r'(?:u8|u|U|L)?R"(?P<delimiter>[^\s()\\]{0,16})\([\s\S]*?\)(?P=delimiter)"|'
    r'(?:u8|u|U|L)?"(?:\\.|[^"\\])*"|'
    r"(?:u8|u|U|L)?'(?:\\.|[^'\\])*'"
)


@dataclass(frozen=True)
class Token:
    value: str
    line: int


@dataclass(frozen=True)
class FunctionMetric:
    path: str
    symbol: str
    start_line: int
    end_line: int
    span_lines: int
    decision_counts: dict[str, int]
    lexical_complexity_proxy: int
    maximum_brace_depth: int


@dataclass(frozen=True)
class FileMetric:
    path: str
    function_count: int
    macro_definition_lines: list[int]
    preprocessor_conditional_count: int
    braced_constructor_initializer_lines: list[int]


def _mask_ignored(text: str) -> tuple[str, list[int], int]:
    """Blank comments, literals and preprocessing while preserving physical lines."""
    masked = IGNORED_PATTERN.sub(lambda match: blank_text(match.group()), text)
    macros: list[int] = []
    conditional_count = 0
    cleaned: list[str] = []
    lines = masked.splitlines(keepends=True)
    index = 0
    while index < len(lines):
        line = lines[index]
        if not line.lstrip(" \t").startswith("#"):
            cleaned.append(line)
            index += 1
            continue
        start_line = index + 1
        directive = [line]
        while directive[-1].rstrip("\r\n").endswith("\\") and index + 1 < len(lines):
            index += 1
            directive.append(lines[index])
        joined = "".join(directive)
        if re.match(r"\s*#\s*define\b", joined):
            macros.append(start_line)
        if re.match(r"\s*#\s*(?:if|ifdef|ifndef|elif|else|endif)\b", joined):
            conditional_count += 1
        cleaned.append(blank_text(joined))
        index += 1
    return "".join(cleaned), macros, conditional_count


def blank_text(text: str) -> str:
    """Replace non-newline directive content with spaces."""
    return "".join("\n" if char == "\n" else " " for char in text)


def _tokens(text: str) -> list[Token]:
    line_ends = [match.end() for match in re.finditer("\n", text)]
    return [Token(match.group(), bisect_right(line_ends, match.start()) + 1) for match in TOKEN_PATTERN.finditer(text)]


def _matching(tokens: list[Token], opening: str, closing: str) -> dict[int, int]:
    pairs: dict[int, int] = {}
    pending: list[int] = []
    for index, token in enumerate(tokens):
        if token.value == opening:
            pending.append(index)
        elif token.value == closing and pending:
            pairs[pending.pop()] = index
    return pairs


def _header_start(tokens: list[Token], opening: int) -> int:
    parenthesis_depth = 0
    for index in range(opening - 1, -1, -1):
        value = tokens[index].value
        if value == ")":
            parenthesis_depth += 1
        elif value == "(" and parenthesis_depth:
            parenthesis_depth -= 1
        elif parenthesis_depth == 0 and value in {";", "{", "}"}:
            return index + 1
    return 0


def _name_before(tokens: list[Token], opening: int) -> tuple[str, int] | None:
    if opening == 0:
        return None
    end = opening - 1
    if tokens[end].value == ">":
        depth = 1
        end -= 1
        while end >= 0 and depth:
            depth += tokens[end].value == ">"
            depth -= tokens[end].value == "<"
            end -= 1
    if end < 0 or not re.fullmatch(r"[A-Za-z_]\w*", tokens[end].value):
        return None
    if tokens[end].value in CONTROL_NAMES:
        return None
    parts = [tokens[end].value]
    start = end
    cursor = end - 1
    while cursor >= 1 and tokens[cursor].value == "::":
        qualifier = tokens[cursor - 1].value
        if not re.fullmatch(r"[A-Za-z_]\w*", qualifier):
            break
        parts.insert(0, qualifier)
        start = cursor - 1
        cursor = start - 1
    return "::".join(parts), start


def _callable_signature(header: list[Token]) -> tuple[str, int, int] | None:
    parens = _matching(header, "(", ")")
    angle_depth = 0
    paren_depth = 0
    for index, token in enumerate(header):
        if token.value == "<":
            angle_depth += 1
        elif token.value == ">" and angle_depth:
            angle_depth -= 1
        elif token.value == "(" and angle_depth == 0 and paren_depth == 0:
            close = parens.get(index)
            name = _name_before(header, index)
            if close is None or name is None:
                paren_depth += 1
                continue
            symbol, name_start = name
            prefix = [item.value for item in header[:name_start]]
            if "=" in prefix or "return" in prefix or "co_return" in prefix:
                paren_depth += 1
                continue
            return symbol, index, close
        elif token.value == "(":
            paren_depth += 1
        elif token.value == ")" and paren_depth:
            paren_depth -= 1
    return None


def _lambda_start(header: list[Token]) -> int | None:
    brackets = _matching(header, "[", "]")
    for close in range(len(header) - 1, -1, -1):
        if header[close].value != "]":
            continue
        opening = next((index for index, end in brackets.items() if end == close), None)
        if opening is None or _is_attribute_bracket(header, opening):
            continue
        if opening > 0 and header[opening - 1].value not in {
            "=",
            "(",
            "{",
            ",",
            "return",
            "co_return",
            "throw",
            ":",
            "?",
            ";",
        }:
            continue
        return opening
    return None


def _is_attribute_bracket(header: list[Token], opening: int) -> bool:
    return (opening > 0 and header[opening - 1].value == "[") or (
        opening + 1 < len(header) and header[opening + 1].value == "["
    )


def _constructor_colon(header: list[Token], parameter_close: int) -> int | None:
    paren_depth = 0
    angle_depth = 0
    for index in range(parameter_close + 1, len(header)):
        value = header[index].value
        if value == "(":
            paren_depth += 1
        elif value == ")" and paren_depth:
            paren_depth -= 1
        elif value == "<":
            angle_depth += 1
        elif value == ">" and angle_depth:
            angle_depth -= 1
        elif value == ":" and paren_depth == 0 and angle_depth == 0:
            return index
    return None


def _scope_name(header: list[Token]) -> str | None:
    values = [token.value for token in header]
    for index, value in enumerate(values):
        if value == "namespace":
            cursor = index + 1
            if cursor < len(values) and values[cursor] == "inline":
                cursor += 1
            names = []
            while cursor < len(values) and re.fullmatch(r"[A-Za-z_]\w*|::", values[cursor]):
                names.append(values[cursor])
                cursor += 1
            return "".join(names) or None
        if value in {"class", "struct", "union"} and index + 1 < len(values):
            name = values[index + 1]
            if re.fullmatch(r"[A-Za-z_]\w*", name):
                return name
    return None


def _qualify(name: str, scopes: list[str]) -> str:
    if not scopes or name.startswith("::"):
        return name.lstrip(":")
    prefix = "::".join(scopes)
    if name == prefix or name.startswith(prefix + "::"):
        return name
    return f"{prefix}::{name}"


def _function_rows(path: str, text: str) -> tuple[list[FunctionMetric], FileMetric]:
    masked, macros, conditionals = _mask_ignored(text)
    tokens = _tokens(masked)
    braces = _matching(tokens, "{", "}")
    opening_count = sum(token.value == "{" for token in tokens)
    closing_count = sum(token.value == "}" for token in tokens)
    if len(braces) != opening_count or opening_count != closing_count:
        raise ValueError(f"{path}: unbalanced C++ braces; function metrics are unavailable")
    paren_depths = _delimiter_depths(tokens, "(", ")")
    bracket_depths = _delimiter_depths(tokens, "[", "]")
    functions: list[FunctionMetric] = []
    braced_initializers: list[int] = []
    scopes: list[tuple[int, str]] = []
    for opening, token in enumerate(tokens):
        while scopes and scopes[-1][0] < opening:
            scopes.pop()
        if token.value != "{":
            continue
        metric, scope, initializer_line = _scan_opening(
            path, tokens, braces, paren_depths, bracket_depths, opening, scopes
        )
        if metric:
            functions.append(metric)
        if scope:
            scopes.append((braces[opening], scope))
        if initializer_line:
            braced_initializers.append(initializer_line)
    return functions, FileMetric(path, len(functions), macros, conditionals, braced_initializers)


def _scan_opening(
    path: str,
    tokens: list[Token],
    braces: dict[int, int],
    paren_depths: list[int],
    bracket_depths: list[int],
    opening: int,
    scopes: list[tuple[int, str]],
) -> tuple[FunctionMetric | None, str | None, int | None]:
    start = _header_start(tokens, opening)
    header = tokens[start:opening]
    lambda_index = _lambda_start(header)
    if lambda_index is None and (paren_depths[opening] or bracket_depths[opening]):
        return None, None, None
    signature = _callable_signature(header)
    if lambda_index is not None:
        return _make_metric(path, tokens, braces[opening], opening, header, scopes, lambda_index, None)
    if signature is None:
        return None, _scope_name(header), None
    symbol, _, parameter_close = signature
    if _constructor_colon(header, parameter_close) is not None and _is_initializer_brace(tokens, braces[opening]):
        return None, None, tokens[opening].line
    return _make_metric(path, tokens, braces[opening], opening, header, scopes, None, signature)


def _make_metric(
    path: str,
    tokens: list[Token],
    close: int,
    opening: int,
    header: list[Token],
    scopes: list[tuple[int, str]],
    lambda_index: int | None,
    signature: tuple[str, int, int] | None,
) -> tuple[FunctionMetric, None, None]:
    if lambda_index is not None:
        name = f"<lambda@{header[lambda_index].line}>"
        parameter_open = parameter_close = -1
        signature_start = lambda_index
    else:
        assert signature is not None
        name, parameter_open, parameter_close = signature
        name = _qualify(name, [scope for _, scope in scopes])
        signature_start = 0
    signature_text = ""
    if parameter_open >= 0:
        signature_text = "".join(item.value for item in header[parameter_open : parameter_close + 1])
    start_line = header[signature_start].line
    end_line = tokens[close].line
    decisions = _decision_counts(tokens[opening : close + 1])
    return (
        FunctionMetric(
            path=path,
            symbol=f"{name}{signature_text}",
            start_line=start_line,
            end_line=end_line,
            span_lines=end_line - start_line + 1,
            decision_counts=decisions,
            lexical_complexity_proxy=1 + sum(decisions.values()),
            maximum_brace_depth=_brace_depth(tokens[opening : close + 1]),
        ),
        None,
        None,
    )


def _is_initializer_brace(tokens: list[Token], close: int) -> bool:
    return close + 1 < len(tokens) and tokens[close + 1].value in {",", "{"}


def _decision_counts(body: list[Token]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for token in body:
        if token.value in DECISION_NAMES or token.value in {"&&", "||", "?"}:
            counts[token.value] = counts.get(token.value, 0) + 1
    return dict(sorted(counts.items()))


def _brace_depth(body: list[Token]) -> int:
    depth = maximum = 0
    for token in body:
        if token.value == "{":
            depth += 1
            maximum = max(maximum, depth)
        elif token.value == "}":
            depth -= 1
    return maximum


def _delimiter_depths(tokens: list[Token], opening: str, closing: str) -> list[int]:
    depths: list[int] = []
    depth = 0
    for token in tokens:
        depths.append(depth)
        if token.value == opening:
            depth += 1
        elif token.value == closing:
            depth = max(0, depth - 1)
    return depths
