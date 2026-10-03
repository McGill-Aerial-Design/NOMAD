// SPDX-License-Identifier: Apache-2.0
// Deterministic publication phases use the same bundle and gate as live discovery.
#include "mavlink/mavsdk_mavlink_connection.hpp"
#include "support/test_harness.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace nomad::mavlink {

struct MavsdkConnectionTestAccess {
    using Connection = MavsdkMavlinkConnection;
    using Resources = Connection::ConnectionResources;

    static bool coherent(Connection &connection) {
        std::shared_lock lock(connection.plugin_lifetime_mutex_);
        const auto &value = connection.resources_;
        return !value ||
               (value->system && value->action && value->telemetry && value->passthrough && value->geofence &&
                value->param && value->offboard && value->position_handle && value->velocity_handle &&
                value->battery_handle && value->gps_handle && value->attitude_handle && value->vtol_state_handle &&
                value->landed_state_handle && value->heartbeat_handle && value->connection_handle);
    }

    static std::shared_ptr<mavsdk::System> system(Connection &connection) {
        std::shared_lock lock(connection.plugin_lifetime_mutex_);
        return connection.resources_->system;
    }

    static std::unique_ptr<Resources> prepare(Connection &connection, std::shared_ptr<mavsdk::System> selected) {
        auto candidate = std::make_unique<Resources>(std::move(selected));
        connection.subscribe(*candidate);
        return candidate;
    }

    static void fail_subscription_setup(std::shared_ptr<mavsdk::System> selected) {
        auto candidate = std::make_unique<Resources>(std::move(selected));
        candidate->position_handle = candidate->telemetry->subscribe_position([](const auto &) {});
        throw std::runtime_error("injected subscription setup failure");
    }

    static bool has_resources(Connection &connection) {
        std::shared_lock lock(connection.plugin_lifetime_mutex_);
        return connection.resources_ != nullptr;
    }

    static std::function<void()> heartbeat_copy(Resources &resources) {
        const auto callback = Connection::get_heartbeat_callback(resources.callbacks);
        return [callback] {
            mavlink_message_t message{};
            mavlink_msg_heartbeat_pack(1, 1, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 12345,
                                       MAV_STATE_ACTIVE);
            callback(message);
        };
    }

    static std::function<void()> published_callback(Connection &connection) {
        std::shared_lock lock(connection.plugin_lifetime_mutex_);
        return heartbeat_copy(*connection.resources_);
    }

    static std::function<void()> paused_callback(Connection &connection, std::promise<void> &entered,
                                                 std::shared_future<void> release) {
        std::shared_lock lock(connection.plugin_lifetime_mutex_);
        const auto gate = connection.resources_->callbacks;
        return [gate, &entered, release] {
            std::lock_guard callback_lock(gate->mutex);
            if (!gate->owner) {
                return;
            }
            entered.set_value();
            release.wait();
            gate->owner->observe_position({1.0, 2.0, 3.0F, 4.0F});
        };
    }

    static void publish(Connection &connection, std::unique_ptr<Resources> candidate) {
        std::lock_guard lock(connection.lifecycle_mutex_);
        connection.publish(std::move(candidate));
    }

    static void setup_until_released(Connection &connection, std::shared_ptr<mavsdk::System> selected,
                                     std::promise<void> &prepared, std::shared_future<void> release) {
        std::lock_guard lock(connection.lifecycle_mutex_);
        auto candidate = prepare(connection, std::move(selected));
        prepared.set_value();
        release.wait();
        connection.publish(std::move(candidate));
    }
};

} // namespace nomad::mavlink

namespace {

using Connection = nomad::mavlink::MavsdkMavlinkConnection;
using Access = nomad::mavlink::MavsdkConnectionTestAccess;
using namespace std::chrono_literals;

class Readers {
  public:
    explicit Readers(Connection &connection) {
        for (int index = 0; index < 8; ++index) {
            workers_.emplace_back([this, &connection](std::stop_token stop) {
                while (!stop.stop_requested()) {
                    static_cast<void>(connection.is_connected());
                    static_cast<void>(connection.get_state());
                    static_cast<void>(connection.get_connect_failure());
                    if (!Access::coherent(connection)) {
                        coherent_ = false;
                    }
                    ++samples_;
                    std::this_thread::yield();
                }
            });
        }
    }

    ~Readers() {
        for (auto &worker : workers_) {
            worker.request_stop();
        }
    }

    bool collect_phase() {
        const auto target = samples_.load() + 1000;
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (samples_ < target && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::yield();
        }
        return samples_ >= target && coherent_;
    }

    void observe_phase() {
        CHECK(collect_phase());
    }

  private:
    std::atomic_uint64_t samples_{0};
    std::atomic_bool coherent_{true};
    std::vector<std::jthread> workers_;
};

void check_callback_preserves_state(Connection &connection, const std::function<void()> &callback) {
    const auto before = connection.get_state();
    callback();
    const auto after = connection.get_state();
    CHECK(after.session_id == before.session_id);
    CHECK(after.custom_mode == before.custom_mode);
    CHECK(after.system_id == before.system_id);
    CHECK(after.component_id == before.component_id);
    CHECK(after.connected == before.connected);
}

void test_failed_discovery_retry(const std::string &endpoint) {
    Connection connection(endpoint, 1, 60ms);
    Readers readers(connection);
    for (int attempt = 0; attempt < 3; ++attempt) {
        CHECK(!connection.connect());
        CHECK(!connection.is_connected());
        if (!endpoint.ends_with(":0")) {
            CHECK(connection.get_connect_failure() == nomad::mavlink::ConnectFailure::NoAutopilot);
        }
        readers.observe_phase();
        connection.disconnect();
    }
}

void test_subscribed_candidate_retry(Connection &connection, Readers &readers) {
    const auto selected = Access::system(connection);
    connection.disconnect();
    bool setup_failed = false;
    try {
        Access::fail_subscription_setup(selected);
    } catch (const std::runtime_error &error) {
        setup_failed = std::string(error.what()) == "injected subscription setup failure";
    }
    CHECK(setup_failed);
    CHECK(!Access::has_resources(connection));
    readers.observe_phase();
    auto candidate = Access::prepare(connection, selected);
    const auto discarded_callback = Access::heartbeat_copy(*candidate);
    readers.observe_phase();
    CHECK(!connection.is_connected());
    check_callback_preserves_state(connection, discarded_callback);
    candidate.reset();
    check_callback_preserves_state(connection, discarded_callback);
    candidate = Access::prepare(connection, selected);
    Access::publish(connection, std::move(candidate));
    Access::published_callback(connection)();
    CHECK(connection.get_state().custom_mode == 12345);
    check_callback_preserves_state(connection, discarded_callback);
    readers.observe_phase();
    connection.disconnect();
}

void test_close_during_setup(Connection &connection, Readers &readers) {
    CHECK(connection.connect());
    const auto selected = Access::system(connection);
    connection.disconnect();
    std::promise<void> prepared;
    std::promise<void> release;
    auto setup = std::async(std::launch::async, [&] {
        Access::setup_until_released(connection, selected, prepared, release.get_future().share());
    });
    const bool setup_prepared = prepared.get_future().wait_for(5s) == std::future_status::ready;
    if (!setup_prepared) {
        release.set_value();
        setup.get();
        CHECK(setup_prepared);
    }
    auto closing = std::async(std::launch::async, [&] { connection.disconnect(); });
    const bool blocked = closing.wait_for(20ms) == std::future_status::timeout;
    const bool readers_progressed = readers.collect_phase();
    release.set_value();
    setup.get();
    closing.get();
    CHECK(blocked);
    CHECK(readers_progressed);
    CHECK(!connection.is_connected());
    CHECK(!connection.get_state().connected);
}

void test_command_retirement(Connection &connection, Readers &readers) {
    CHECK(connection.connect());
    std::promise<void> entered;
    std::promise<void> release;
    const auto released = release.get_future().share();
    connection.set_transmission_admission_factory([&] {
        return nomad::mavlink::TransmissionAdmission([&](const auto &) {
            entered.set_value();
            released.wait();
            return false;
        });
    });
    auto command =
        std::async(std::launch::async, [&] { return connection.send_command({MAV_CMD_DO_SET_RELAY, {}}, 500ms); });
    const bool command_entered = entered.get_future().wait_for(5s) == std::future_status::ready;
    const bool readers_progressed = readers.collect_phase();
    auto closing = std::async(std::launch::async, [&] { connection.disconnect(); });
    const bool blocked = closing.wait_for(20ms) == std::future_status::timeout;
    release.set_value();
    const auto result = command.get();
    closing.get();
    connection.set_transmission_admission_factory({});
    CHECK(blocked);
    CHECK(command_entered);
    CHECK(readers_progressed);
    CHECK(result && result->status == nomad::mavlink::CommandAck::Status::AdmissionCancelled);
    CHECK(!connection.send_command({MAV_CMD_DO_SET_RELAY, {}}, 50ms));
    readers.observe_phase();
}

void test_in_flight_callback_retirement(Connection &connection, Readers &readers) {
    CHECK(connection.connect());
    const auto copied_callback = Access::published_callback(connection);
    std::promise<void> entered;
    std::promise<void> release;
    const auto callback = Access::paused_callback(connection, entered, release.get_future().share());
    auto running = std::async(std::launch::async, callback);
    const bool callback_entered = entered.get_future().wait_for(5s) == std::future_status::ready;
    std::promise<void> close_started;
    auto closing = std::async(std::launch::async, [&] {
        close_started.set_value();
        connection.disconnect();
    });
    close_started.get_future().wait();
    const bool blocked = closing.wait_for(20ms) == std::future_status::timeout;
    release.set_value();
    running.get();
    closing.get();
    CHECK(blocked);
    CHECK(callback_entered);
    check_callback_preserves_state(connection, copied_callback);
    CHECK(!connection.get_state().position_valid);
    readers.observe_phase();
}

void test_generations_and_owner_destruction(const std::string &endpoint) {
    std::function<void()> retired_callback;
    {
        Connection connection(endpoint, 1, 3s);
        Readers readers(connection);
        std::uint64_t previous_session = 0;
        for (int generation = 0; generation < 6; ++generation) {
            CHECK(connection.connect());
            CHECK(connection.wait_for_heartbeat(3s));
            const auto state = connection.get_state();
            CHECK(state.session_id > previous_session);
            previous_session = state.session_id;
            if (retired_callback) {
                check_callback_preserves_state(connection, retired_callback);
            }
            retired_callback = Access::published_callback(connection);
            readers.observe_phase();
            connection.disconnect();
            check_callback_preserves_state(connection, retired_callback);
            CHECK(!connection.is_connected());
            readers.observe_phase();
        }
        CHECK(connection.connect());
        test_subscribed_candidate_retry(connection, readers);
        test_close_during_setup(connection, readers);
        test_command_retirement(connection, readers);
        test_in_flight_callback_retirement(connection, readers);
        readers.observe_phase();
    }
    retired_callback();
}

} // namespace

int main(int argc, char **argv) {
    return nomad::test::run_tests([&] {
        if (argc == 1) {
            test_failed_discovery_retry("udpin:127.0.0.1:0");
            return;
        }
        if (argc == 3 && std::string(argv[1]) == "--no-peer") {
            test_failed_discovery_retry(argv[2]);
            return;
        }
        CHECK(argc == 2);
        test_generations_and_owner_destruction(argv[1]);
    });
}
