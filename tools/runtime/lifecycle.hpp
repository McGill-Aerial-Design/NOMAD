// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/runtime/runtime.hpp"

namespace nomad::runtime::process {

constexpr int kConfigurationFailure = 78;
using Runner = int (*)(int, char **);
int run_service(int argc, char **argv, Runner runner);
void initialize_console();
bool stop_requested();
void publish_ready(Runtime &runtime);
void publish_stopping();
void publish_error(const char *message);
int shutdown_runtime(Runtime &runtime);

#ifdef NOMAD_SERVICE_TEST
void test_begin_service();
void test_stop_service();
void test_shutdown_service();
void test_finish_service(int error);
unsigned long test_service_state();
unsigned long test_service_error();
#endif

} // namespace nomad::runtime::process
