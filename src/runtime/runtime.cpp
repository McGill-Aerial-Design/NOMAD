// SPDX-License-Identifier: Apache-2.0
#include "nomad/runtime/runtime.hpp"

#include "ipc_server.hpp"
#include "nomad/safety/velocity_config.hpp"
#include "nomad/telemetry/state.hpp"
#include "nomad/vehicle/vehicle.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace nomad::runtime {
namespace {

using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

constexpr std::string_view kProtocolName = "nomad-core";
constexpr int kProtocolVersion = 1;
constexpr std::size_t kRequestCacheCapacity = 256;
constexpr std::size_t kMaximumJsonDepth = 64;

struct Request {
    std::string id;
    std::string client_id;
    std::string type;
    Json original;
    double timeout_seconds{};
    int channel{};
    int pwm_microseconds{};
    int relay_number{};
    int motor_instance{};
    int mount_mode{};
    bool relay_on{};
};

struct ParsedRequest {
    std::optional<Request> request;
    Json error;
};

bool is_string(const Json &object, const char *key) {
    return object.contains(key) && object[key].is_string();
}

std::string field_string(const Json &object, const char *key) {
    if (!is_string(object, key)) {
        return {};
    }
    return object[key].get<std::string>();
}

Json error_response(std::string id, std::string code, std::string message) {
    Json response{{"protocol", kProtocolName}, {"version", kProtocolVersion}, {"ok", false}};
    if (!id.empty()) {
        response["id"] = std::move(id);
    }
    response["error"] = {{"code", std::move(code)}, {"message", std::move(message)}};
    return response;
}

bool read_finite_number(const Json &object, const char *key, double &value) {
    if (!object.contains(key) || !object[key].is_number()) {
        return false;
    }
    value = object[key].get<double>();
    return std::isfinite(value);
}

bool read_integer(const Json &object, const char *key, int &value) {
    if (!object.contains(key) || !object[key].is_number_integer()) {
        return false;
    }
    if (object[key].is_number_unsigned()) {
        const auto number = object[key].get<std::uint64_t>();
        if (number > 1000000) {
            return false;
        }
        value = static_cast<int>(number);
        return true;
    }
    const auto number = object[key].get<std::int64_t>();
    if (number < 0 || number > 1000000) {
        return false;
    }
    value = static_cast<int>(number);
    return true;
}

bool has_supported_version(const Json &value) {
    if (!value.is_number_integer()) {
        return false;
    }
    if (value.is_number_unsigned()) {
        return value.get<std::uint64_t>() == static_cast<std::uint64_t>(kProtocolVersion);
    }
    return value.get<std::int64_t>() == kProtocolVersion;
}

mavlink::MavlinkConnection &require_connection(
    const std::unique_ptr<mavlink::MavlinkConnection> &connection) {
    if (connection == nullptr) {
        throw std::invalid_argument("runtime requires one MAVLink connection");
    }
    return *connection;
}

vehicle::VehicleConfig make_vehicle_config(const RuntimeConfig &config) {
    vehicle::VehicleConfig vehicle_config{};
    vehicle_config.fence = config.fence_policy;
    vehicle_config.velocity = config.velocity_limits;
    return vehicle_config;
}

bool validate_request_fields(Request &request, Json &error) {
    auto &body = request.original;
    if (request.type == "hello" || request.type == "ping" || request.type == "status") {
        return true;
    }
    if (request.type == "set_servo" && read_integer(body, "channel", request.channel) &&
        read_integer(body, "pwm_microseconds", request.pwm_microseconds)) {
        return true;
    }
    if (request.type == "set_relay" && read_integer(body, "relay_number", request.relay_number) &&
        body.contains("on") && body["on"].is_boolean()) {
        request.relay_on = body["on"].get<bool>();
        return true;
    }
    if (request.type == "motor_test" && read_integer(body, "motor_instance", request.motor_instance) &&
        read_integer(body, "pwm_microseconds", request.pwm_microseconds) &&
        read_finite_number(body, "timeout_seconds", request.timeout_seconds)) {
        return true;
    }
    if (request.type == "configure_gimbal" && read_integer(body, "mount_mode", request.mount_mode)) {
        return true;
    }
    const bool known_type = request.type == "set_servo" || request.type == "set_relay" ||
                            request.type == "motor_test" ||
                            request.type == "configure_gimbal";
    if (!known_type && request.type != "hello" && request.type != "ping" && request.type != "status") {
        error = error_response(request.id, "unsupported_request", "request type is not supported in protocol v1");
        return false;
    }
    error = error_response(request.id, "invalid_request", "request fields do not match the typed request");
    return false;
}

bool has_reasonable_json_depth(std::string_view line);

ParsedRequest parse_request(std::string_view line) {
    if (line.size() > detail::kMaximumMessageBytes) {
        return {std::nullopt, error_response({}, "message_too_large", "message exceeds 65536 bytes")};
    }
    if (!has_reasonable_json_depth(line)) {
        return {std::nullopt, error_response({}, "malformed_json", "JSON nesting exceeds the v1 limit")};
    }
    const auto body = Json::parse(line, nullptr, false);
    if (body.is_discarded() || !body.is_object()) {
        return {std::nullopt, error_response({}, "malformed_json", "request must be a JSON object")};
    }
    const auto id = field_string(body, "id");
    if (id.empty() || id.size() > 64) {
        return {std::nullopt, error_response({}, "invalid_request", "id must be a non-empty string up to 64 bytes")};
    }
    if (!is_string(body, "protocol") || body["protocol"] != "nomad-core") {
        return {std::nullopt, error_response(id, "incompatible_protocol", "protocol must be nomad-core")};
    }
    if (!body.contains("version") || !has_supported_version(body["version"])) {
        return {std::nullopt, error_response(id, "incompatible_version", "supported protocol version is 1")};
    }
    const auto client_id = field_string(body, "client_id");
    const auto type = field_string(body, "type");
    if (client_id.empty() || client_id.size() > 64 || type.empty() || type.size() > 64) {
        return {std::nullopt, error_response(id, "invalid_request", "client_id and type are required strings")};
    }
    Request request{id, client_id, type, body};
    Json error;
    if (!validate_request_fields(request, error)) {
        return {std::nullopt, std::move(error)};
    }
    return {std::move(request), {}};
}

bool is_mutating(const std::string &type) {
    return type == "set_servo" || type == "set_relay" || type == "motor_test" ||
           type == "configure_gimbal";
}

std::optional<std::int64_t> age_milliseconds(Clock::time_point timestamp) {
    if (timestamp == Clock::time_point{}) {
        return std::nullopt;
    }
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - timestamp).count();
    return std::max<std::int64_t>(0, age);
}

Json optional_age(std::optional<std::int64_t> age) {
    return age.has_value() ? Json(*age) : Json(nullptr);
}

std::string cache_key(const Request &request) {
    return std::to_string(request.client_id.size()) + ":" + request.client_id + request.id;
}

bool has_reasonable_json_depth(std::string_view line) {
    bool in_string = false;
    bool escaped = false;
    std::size_t depth = 0;
    for (const auto character : line) {
        if (in_string) {
            if (escaped) {
                escaped = false;
            } else if (character == '\\') {
                escaped = true;
            } else if (character == '"') {
                in_string = false;
            }
            continue;
        }
        if (character == '"') {
            in_string = true;
        } else if (character == '{' || character == '[') {
            if (++depth > kMaximumJsonDepth) {
                return false;
            }
        } else if (character == '}' || character == ']') {
            if (depth == 0) {
                return false;
            }
            --depth;
        }
    }
    return true;
}

} // namespace

struct Runtime::Implementation {
    struct CacheEntry {
        std::string fingerprint;
        Json response;
    };

    Implementation(std::unique_ptr<mavlink::MavlinkConnection> connection, RuntimeConfig config)
        : connection_(std::move(connection)), config_(std::move(config)),
          vehicle_(require_connection(connection_), make_vehicle_config(config_)) {}

    bool start(std::string &error) {
        if (!server_.start(config_.ipc_port, [this](std::string_view request) { return handle_message(request); },
                           error)) {
            return false;
        }
        stopping_ = false;
        try {
            connection_worker_ = std::thread(&Implementation::maintain_connection, this);
        } catch (...) {
            server_.stop();
            error = "could not start MAVSDK connection worker";
            return false;
        }
        return true;
    }

    void stop() {
        stopping_ = true;
        server_.stop();
        if (connection_worker_.joinable()) {
            connection_worker_.join();
        }
        connection_->disconnect();
    }

    bool ready() const {
        return server_.running();
    }

    void maintain_connection() {
        while (!stopping_) {
            if (!connection_->is_connected()) {
                if (!connection_->connect()) {
                    std::this_thread::sleep_for(config_.reconnect_delay);
                    continue;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    std::string handle_message(std::string_view line) {
        auto parsed = parse_request(line);
        if (!parsed.request.has_value()) {
            return parsed.error.dump();
        }
        const auto &request = *parsed.request;
        if (is_mutating(request.type)) {
            return handle_mutating_request(request).dump();
        }
        return handle_read_request(request).dump();
    }

    Json handle_mutating_request(const Request &request) {
        const auto key = cache_key(request);
        const auto fingerprint = request.original.dump();
        {
            std::lock_guard lock(cache_mutex_);
            const auto cached = response_cache_.find(key);
            if (cached != response_cache_.end()) {
                if (cached->second.fingerprint != fingerprint) {
                    return error_response(request.id, "request_id_conflict", "request ID was used with different data");
                }
                return cached->second.response;
            }
            if (in_flight_.contains(key)) {
                return error_response(request.id, "request_in_progress", "request with this ID is still running");
            }
            in_flight_.insert(key);
        }

        Json response;
        try {
            response = execute_mutating_request(request);
        } catch (...) {
            response = error_response(request.id, "internal_error", "vehicle request failed internally");
        }
        remember_response(key, fingerprint, response);
        return response;
    }

    Json handle_read_request(const Request &request) {
        if (request.type == "hello") {
            return {{"protocol", kProtocolName},
                    {"version", kProtocolVersion},
                    {"id", request.id},
                    {"ok", true},
                    {"type", "hello_response"},
                    {"runtime_version", config_.version},
                    {"capabilities",
                     {"hello", "ping", "status", "set_servo", "set_relay", "motor_test",
                      "configure_gimbal"}}};
        }
        if (request.type == "ping") {
            return {{"protocol", kProtocolName}, {"version", kProtocolVersion}, {"id", request.id},
                    {"ok", true}, {"type", "pong"}};
        }
        if (request.type == "status") {
            return {{"protocol", kProtocolName}, {"version", kProtocolVersion}, {"id", request.id},
                    {"ok", true}, {"type", "status_response"}, {"status", status_snapshot()}};
        }
        return error_response(request.id, "unsupported_request", "request type is not supported in protocol v1");
    }

    Json status_snapshot() const {
        const auto state = connection_->get_state();
        const bool connection_open = connection_->is_connected();
        const auto identity = state.identity.aircraft_class;
        return {{"runtime_ready", ready()},
                {"mavsdk_connection_open", connection_open},
                {"vehicle_transport_connected", connection_open},
                {"vehicle_session_established", state.system_id != 0},
                {"vehicle_connected", state.connected},
                {"identity_resolved", identity != telemetry::AircraftClass::Unknown},
                {"aircraft_class", telemetry::aircraft_class_name(identity)},
                {"armed", state.armed},
                {"custom_mode", state.custom_mode},
                {"telemetry",
                 {{"heartbeat_fresh", state.heartbeat_fresh},
                  {"position_valid", state.position_valid},
                  {"position_age_ms", optional_age(age_milliseconds(state.position_updated_at))},
                  {"gps_valid", state.gps_valid},
                  {"gps_age_ms", optional_age(age_milliseconds(state.gps_updated_at))},
                  {"attitude_valid", state.attitude_valid},
                  {"attitude_age_ms", optional_age(age_milliseconds(state.attitude_updated_at))},
                  {"battery_valid", state.battery_valid},
                  {"battery_age_ms", optional_age(age_milliseconds(state.battery_updated_at))}}}};
    }

    Json execute_mutating_request(const Request &request) {
        if (!config_.actuation_enabled) {
            return error_response(request.id, "missing_api_key", "NOMAD_API_KEY is not set for the runtime");
        }
        std::unique_lock command_lock(command_mutex_, std::try_to_lock);
        if (!command_lock.owns_lock()) {
            return error_response(request.id, "busy", "another NOMAD command is still executing");
        }
        const auto result = invoke_vehicle(request);
        Json response{{"protocol", kProtocolName}, {"version", kProtocolVersion}, {"id", request.id},
                      {"ok", true}, {"type", "command_response"},
                      {"command_result", {{"success", result.success}, {"message", result.message}}}};
        return response;
    }

    vehicle::CommandResult invoke_vehicle(const Request &request) {
        if (request.type == "set_servo") {
            return vehicle_.set_servo(request.channel, request.pwm_microseconds);
        }
        if (request.type == "set_relay") {
            return vehicle_.set_relay(request.relay_number, request.relay_on);
        }
        if (request.type == "motor_test") {
            return vehicle_.motor_test(request.motor_instance, request.pwm_microseconds,
                                        static_cast<float>(request.timeout_seconds));
        }
        if (request.type == "configure_gimbal") {
            return vehicle_.configure_gimbal(request.mount_mode);
        }
        return {false, "request type is not supported in protocol v1"};
    }

    void remember_response(const std::string &key, const std::string &fingerprint, const Json &response) {
        std::lock_guard lock(cache_mutex_);
        in_flight_.erase(key);
        response_cache_[key] = CacheEntry{fingerprint, response};
        cache_order_.push_back(key);
        while (cache_order_.size() > kRequestCacheCapacity) {
            const auto oldest = std::move(cache_order_.front());
            cache_order_.pop_front();
            response_cache_.erase(oldest);
        }
    }

    std::unique_ptr<mavlink::MavlinkConnection> connection_;
    RuntimeConfig config_;
    vehicle::Vehicle vehicle_;
    detail::IpcServer server_;
    std::atomic_bool stopping_{false};
    std::thread connection_worker_;
    std::mutex command_mutex_;
    std::mutex cache_mutex_;
    std::unordered_map<std::string, CacheEntry> response_cache_;
    std::deque<std::string> cache_order_;
    std::unordered_set<std::string> in_flight_;
};

Runtime::Runtime(std::unique_ptr<mavlink::MavlinkConnection> connection, RuntimeConfig config)
    : implementation_(std::make_unique<Implementation>(std::move(connection), std::move(config))) {}

Runtime::~Runtime() {
    stop();
}

bool Runtime::start(std::string &error) {
    return implementation_->start(error);
}

void Runtime::stop() {
    if (implementation_ != nullptr) {
        implementation_->stop();
    }
}

bool Runtime::ready() const {
    return implementation_ != nullptr && implementation_->ready();
}

} // namespace nomad::runtime
