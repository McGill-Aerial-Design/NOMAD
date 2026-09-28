// SPDX-License-Identifier: Apache-2.0
// Interactive probe for the independent UDP wire qualification fixture.

#include "nomad/mavlink/mavsdk_transport.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace std::chrono_literals;

struct Gate {
    std::mutex mutex;
    std::uint64_t generation{0};
    bool admitted{false};
};

void report(const std::string &line) {
    static std::mutex output_mutex;
    std::lock_guard lock(output_mutex);
    std::cout << line << std::endl;
}

void run_command(nomad::mavlink::MavlinkConnection &connection, const std::string &kind) {
    bool accepted = false;
    if (kind == "int") {
        const nomad::mavlink::FixedWingWaypointCommand waypoint{45.5, -73.6, 10.0F, 20.0F};
        const auto session = connection.get_state().session_id;
        const auto result = connection.send_fixed_wing_waypoint(waypoint, session, 2500ms);
        accepted = result.has_value() && result->status == nomad::mavlink::CommandAck::Status::Acknowledged &&
                   result->result == 0;
    } else {
        const nomad::mavlink::Command command{183, {8.0F, 1500.0F, 0, 0, 0, 0, 0}};
        const auto result = connection.send_command(command, 2500ms);
        accepted = result.has_value() && result->status == nomad::mavlink::CommandAck::Status::Acknowledged &&
                   result->result == 0;
    }
    report(kind + (accepted ? "=accepted" : "=cancelled_or_unacknowledged"));
}

int run(const std::string &endpoint) {
    auto connection = nomad::mavlink::make_mavsdk_connection(endpoint, 1, 3000ms);
    auto gate = std::make_shared<Gate>();
    connection->set_transmission_admission_factory([gate] {
        std::uint64_t captured_generation;
        {
            std::lock_guard lock(gate->mutex);
            captured_generation = gate->generation;
        }
        return [gate, captured_generation](const std::function<void()> &send) {
            std::lock_guard lock(gate->mutex);
            if (!gate->admitted || gate->generation != captured_generation) {
                return false;
            }
            send();
            return true;
        };
    });
    if (!connection->connect() || !connection->wait_for_heartbeat(3000ms)) {
        report("connect=failed");
        return 1;
    }
    report("ready");
    std::vector<std::thread> commands;
    std::string line;
    while (std::getline(std::cin, line)) {
        if (line == "admit" || line == "revoke" || line == "handback") {
            std::lock_guard lock(gate->mutex);
            ++gate->generation;
            gate->admitted = line != "revoke";
            report(line + "=ok");
        } else if (line == "long" || line == "int") {
            commands.emplace_back([&connection, kind = line] { run_command(*connection, kind); });
            report(line + "=started");
        } else if (line == "stop") {
            {
                std::lock_guard lock(gate->mutex);
                ++gate->generation;
                gate->admitted = false;
            }
            break;
        }
    }
    for (auto &command : commands) {
        command.join();
    }
    connection->disconnect();
    report("stopped");
    return 0;
}

} // namespace

int main(int argc, char **argv) {
    if (argc != 2) {
        return 2;
    }
    return run(argv[1]);
}
