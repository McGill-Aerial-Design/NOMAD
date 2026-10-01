// SPDX-License-Identifier: Apache-2.0
#include "lifecycle.hpp"

#include <atomic>
#include <csignal>
#include <mutex>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace nomad::runtime::process {
namespace {

volatile std::sig_atomic_t console_stop = 0;
void signal_stop(int) {
    console_stop = 1;
}

#ifdef _WIN32
// SCM callbacks require process-wide context; one service owns one runtime.
struct ServiceContext {
    std::mutex mutex;
    std::atomic_bool stopping{false};
    Runtime *runtime = nullptr;
    SERVICE_STATUS_HANDLE handle = nullptr;
    SERVICE_STATUS status{};
    Runner runner = nullptr;
    int argc = 0;
    char **argv = nullptr;
};
ServiceContext service;

void report_event(const char *message, WORD kind) {
    const auto source = RegisterEventSourceA(nullptr, "nomad-runtime");
    if (source != nullptr) {
        const char *messages[] = {message};
        ReportEventA(source, kind, 0, 1, nullptr, 1, 0, messages, nullptr);
        DeregisterEventSource(source);
    }
}

void report_status(DWORD state, int error = 0) {
    service.status.dwServiceType = SERVICE_WIN32_OWN_PROCESS;
    service.status.dwCurrentState = state;
    service.status.dwControlsAccepted = state == SERVICE_RUNNING ? SERVICE_ACCEPT_STOP | SERVICE_ACCEPT_SHUTDOWN : 0;
    service.status.dwWin32ExitCode = error == 0 ? NO_ERROR : ERROR_SERVICE_SPECIFIC_ERROR;
    service.status.dwServiceSpecificExitCode = static_cast<DWORD>(error);
    service.status.dwWaitHint = state == SERVICE_START_PENDING || state == SERVICE_STOP_PENDING ? 30000 : 0;
    service.status.dwCheckPoint = service.status.dwWaitHint == 0 ? 0 : service.status.dwCheckPoint + 1;
#ifndef NOMAD_SERVICE_TEST
    SetServiceStatus(service.handle, &service.status);
#endif
}

DWORD WINAPI control_service(DWORD control, DWORD, void *, void *) {
    if (control != SERVICE_CONTROL_STOP && control != SERVICE_CONTROL_SHUTDOWN) {
        return control == SERVICE_CONTROL_INTERROGATE ? NO_ERROR : ERROR_CALL_NOT_IMPLEMENTED;
    }
    std::lock_guard lock(service.mutex);
    service.stopping = true;
    if (service.runtime != nullptr) {
        service.runtime->request_stop();
    }
    report_status(SERVICE_STOP_PENDING);
    return NO_ERROR;
}

void WINAPI service_main(DWORD, LPSTR *) {
    service.handle = RegisterServiceCtrlHandlerExA("nomad-runtime", control_service, nullptr);
    if (service.handle == nullptr) {
        return;
    }
    report_status(SERVICE_START_PENDING);
    int result = 1;
    try {
        result = service.runner(service.argc, service.argv);
    } catch (...) {
        result = 1;
    }
    std::lock_guard lock(service.mutex);
    service.runtime = nullptr;
    report_status(SERVICE_STOPPED, result);
    report_event(result == 0 ? "NOMAD runtime stopped cleanly" : "NOMAD runtime failed; inspect service exit code",
                 result == 0 ? EVENTLOG_INFORMATION_TYPE : EVENTLOG_ERROR_TYPE);
}
#endif

} // namespace

void initialize_console() {
    std::signal(SIGINT, signal_stop);
    std::signal(SIGTERM, signal_stop);
#ifdef SIGBREAK
    std::signal(SIGBREAK, signal_stop);
#endif
}

bool stop_requested() {
#ifdef _WIN32
    return console_stop != 0 || service.stopping;
#else
    return console_stop != 0;
#endif
}

void publish_ready(Runtime &runtime) {
#ifdef _WIN32
    std::lock_guard lock(service.mutex);
    if (service.handle != nullptr) {
        service.runtime = &runtime;
        if (service.stopping) {
            runtime.request_stop();
        } else {
            report_status(SERVICE_RUNNING);
        }
    }
#else
    (void)runtime;
#endif
}

void publish_stopping() {
#ifdef _WIN32
    std::lock_guard lock(service.mutex);
    if (service.handle != nullptr) {
        service.runtime = nullptr;
        report_status(SERVICE_STOP_PENDING);
    }
#endif
}

void publish_error(const char *message) {
#ifdef _WIN32
    if (service.handle != nullptr) {
        report_event(message, EVENTLOG_ERROR_TYPE);
    }
#else
    (void)message;
#endif
}

int run_service(int argc, char **argv, Runner runner) {
#ifdef _WIN32
    service.runner = runner;
    service.argc = argc;
    service.argv = argv;
    SERVICE_TABLE_ENTRYA table[] = {{const_cast<char *>("nomad-runtime"), service_main}, {nullptr, nullptr}};
    return StartServiceCtrlDispatcherA(table) ? 0 : 1;
#else
    (void)argc;
    (void)argv;
    (void)runner;
    return kConfigurationFailure;
#endif
}

#if defined(_WIN32) && defined(NOMAD_SERVICE_TEST)
void test_begin_service() {
    service.stopping = false;
    service.runtime = nullptr;
    service.handle = reinterpret_cast<SERVICE_STATUS_HANDLE>(1);
    report_status(SERVICE_START_PENDING);
}

void test_stop_service() {
    control_service(SERVICE_CONTROL_STOP, 0, nullptr, nullptr);
}

void test_shutdown_service() {
    control_service(SERVICE_CONTROL_SHUTDOWN, 0, nullptr, nullptr);
}

void test_finish_service(int error) {
    std::lock_guard lock(service.mutex);
    service.runtime = nullptr;
    report_status(SERVICE_STOPPED, error);
}

unsigned long test_service_error() {
    return service.status.dwServiceSpecificExitCode;
}

unsigned long test_service_state() {
    return service.status.dwCurrentState;
}
#endif

} // namespace nomad::runtime::process
