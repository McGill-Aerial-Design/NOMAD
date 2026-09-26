// SPDX-License-Identifier: Apache-2.0
#pragma once

// Shared harness for the CTest executables in this directory.
//
// One rationale has to live in exactly one place: a failing assert on Windows
// opens a dialog that blocks unattended CI runs, so checks throw a
// std::runtime_error instead of calling assert(), and main() reports that
// failure on stderr with a nonzero exit code.
//
// CHECK keeps its historical spelling so existing test bodies are unchanged.

#include <cstdio>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace nomad::test {

inline void check_impl(bool ok, const char *condition, int line) {
    if (!ok) {
        throw std::runtime_error(std::string("check failed at line ") + std::to_string(line) + ": " + condition);
    }
}

// Runs a test body, turning the first failed check into the process exit code.
inline int run_tests(const std::function<void()> &body) {
    try {
        body();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAILED: %s\n", error.what());
        return 1;
    }
    return 0;
}

inline void run_scenario(std::string_view name, const std::function<void()> &body) {
    try {
        body();
    } catch (const std::exception &error) {
        throw std::runtime_error("scenario " + std::string(name) + ": " + error.what());
    }
}

} // namespace nomad::test

#define CHECK(condition) ::nomad::test::check_impl(static_cast<bool>(condition), #condition, __LINE__)
