// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
#include "mavsdk.hpp"

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;

struct Observation {
    std::mutex mutex;
    std::condition_variable changed;
    bool entered{false};
    bool released{false};
    bool timed_out{false};
    std::vector<std::pair<double, unsigned>> attempts;
};

bool observe_heartbeat(Observation &state, mavlink_message_t &message) {
    if (message.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
        return true;
    }
    std::unique_lock lock(state.mutex);
    if (!state.entered) {
        state.entered = true;
        state.changed.notify_all();
        if (!state.changed.wait_for(lock, std::chrono::seconds(3), [&] { return state.released; })) {
            state.timed_out = true;
        }
    }
    const auto milliseconds = std::chrono::duration<double, std::milli>(Clock::now().time_since_epoch()).count();
    state.attempts.emplace_back(milliseconds, message.seq);
    state.changed.notify_all();
    return true;
}

void report_observations(const Observation &state, bool connected) {
    std::cout << "{\"connected\":" << (connected ? "true" : "false")
              << ",\"interception_timed_out\":" << (state.timed_out ? "true" : "false") << ",\"attempts\":[";
    for (std::size_t index = 0; index < state.attempts.size(); ++index) {
        if (index != 0) {
            std::cout << ',';
        }
        std::cout << "{\"elapsed_ms\":" << state.attempts[index].first - state.attempts[0].first
                  << ",\"sequence\":" << state.attempts[index].second << '}';
    }
    std::cout << "]}\n";
}

int main(int argc, char **argv) {
    if (argc != 4) {
        return 2;
    }
    const int port = std::stoi(argv[1]);
    const int hold_ms = std::stoi(argv[2]);
    const bool add_second_connection = std::string(argv[3]) == "1";
    if (port < 1 || port > 65535 || hold_ms < 0 || hold_ms > 1500) {
        return 2;
    }
    Observation state;
    mavsdk::Mavsdk::Configuration config(mavsdk::ComponentType::GroundStation);
    config.set_always_send_heartbeats(false);
    mavsdk::Mavsdk sdk(config);
    sdk.intercept_outgoing_messages_async([&](auto &message) { return observe_heartbeat(state, message); });
    config.set_always_send_heartbeats(true);
    sdk.set_configuration(config);
    {
        std::unique_lock lock(state.mutex);
        if (!state.changed.wait_for(lock, std::chrono::seconds(3), [&] { return state.entered; })) {
            return 3;
        }
    }
    const auto connected = sdk.add_any_connection("udpout://127.0.0.1:" + std::to_string(port));
    std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
    {
        std::lock_guard lock(state.mutex);
        state.released = true;
        state.changed.notify_all();
    }
    if (add_second_connection) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        sdk.subscribe_raw_bytes_to_be_sent([](const char *, std::size_t) {});
        if (sdk.add_any_connection("raw://") != mavsdk::ConnectionResult::Success) {
            return 4;
        }
    }
    {
        std::unique_lock lock(state.mutex);
        // The final interception precedes delivery. Observe one extra attempt so
        // destruction cannot race the fourth frame required by the independent receiver.
        state.changed.wait_for(lock, std::chrono::seconds(6), [&] { return state.attempts.size() >= 5; });
        report_observations(state, connected == mavsdk::ConnectionResult::Success);
    }
    return 0;
}
