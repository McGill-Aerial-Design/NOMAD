// SPDX-License-Identifier: Apache-2.0
#pragma once

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
    double pitch_deg{};
    double roll_deg{};
    bool relay_on{};
    std::string incarnation;
    std::string source;
    std::uint64_t session{};
    std::uint64_t generation{};
    std::uint64_t sequence{};
    std::int64_t expires_at_ms{};
    std::shared_ptr<std::atomic_bool> admission_check_passed{std::make_shared<std::atomic_bool>(false)};
    std::shared_ptr<std::atomic_bool> ack_observed{std::make_shared<std::atomic_bool>(false)};
    std::shared_ptr<std::atomic_bool> observed_success{std::make_shared<std::atomic_bool>(false)};
};

thread_local const Request *active_request = nullptr;

class ActiveRequest {
  public:
    explicit ActiveRequest(const Request &request) : previous_(active_request) {
        active_request = &request;
    }
    ~ActiveRequest() {
        active_request = previous_;
    }

  private:
    const Request *previous_;
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

bool read_unsigned(const Json &object, const char *key, std::uint64_t &value) {
    if (!object.contains(key) || !object[key].is_number_unsigned()) {
        return false;
    }
    value = object[key].get<std::uint64_t>();
    return true;
}

std::int64_t unix_milliseconds() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

struct AuthorityGate {
    mutable std::shared_mutex mutex;
    std::string incarnation;
    std::uint64_t vehicle_session{};
    std::uint64_t generation{};
    std::uint64_t last_sequence{};
    std::string owner;
    std::uint64_t owner_session{};
    bool ever_admitted{};
    bool stopping{};
};

void update_gate_session(AuthorityGate &gate, std::uint64_t session) {
    if (gate.vehicle_session == session) {
        return;
    }
    gate.vehicle_session = session;
    if (gate.owner.empty()) {
        return;
    }
    ++gate.generation;
    gate.owner.clear();
    gate.owner_session = 0;
    gate.last_sequence = 0;
}

bool is_newer_session(std::uint64_t session, std::uint64_t current_session) {
    if (session == current_session) {
        return false;
    }
    if (current_session == 0) {
        return session != 0;
    }
    return session - current_session < (std::numeric_limits<std::uint64_t>::max() / 2);
}

bool matches_authority(const AuthorityGate &gate, const mavlink::SendAuthorityToken &token) {
    return !gate.stopping && token.runtime_incarnation == gate.incarnation && token.vehicle_session != 0 &&
           token.vehicle_session == gate.vehicle_session && token.authority_generation == gate.generation &&
           token.source == gate.owner && token.vehicle_session == gate.owner_session &&
           token.sequence != 0 && token.sequence <= gate.last_sequence && !token.request_id.empty() &&
           token.expires_at_ms >= unix_milliseconds();
}

std::string new_incarnation() {
    std::random_device random;
    constexpr char digits[] = "0123456789abcdef";
    std::string value(32, '0');
    for (auto &digit : value) {
        digit = digits[random() & 15U];
    }
    return value;
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
    if (request.type == "hello" || request.type == "ping" || request.type == "status" ||
        request.type == "admit_authority" || request.type == "revoke_authority" ||
        request.type == "handback_authority") {
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
    if (request.type == "set_gimbal_target" && read_finite_number(body, "pitch_deg", request.pitch_deg) &&
        read_finite_number(body, "roll_deg", request.roll_deg)) {
        return true;
    }
    const bool known_type = request.type == "set_servo" || request.type == "set_relay" ||
                            request.type == "motor_test" ||
                            request.type == "configure_gimbal" || request.type == "set_gimbal_target";
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
    request.incarnation = field_string(body, "runtime_incarnation");
    request.source = field_string(body, "command_source");
    read_unsigned(body, "vehicle_session", request.session);
    read_unsigned(body, "authority_generation", request.generation);
    read_unsigned(body, "sequence", request.sequence);
    if (body.contains("expires_at_ms") && body["expires_at_ms"].is_number_integer()) {
        request.expires_at_ms = body["expires_at_ms"].get<std::int64_t>();
    }
    Json error;
    if (!validate_request_fields(request, error)) {
        return {std::nullopt, std::move(error)};
    }
    return {std::move(request), {}};
}

bool is_mutating(const std::string &type) {
    return type == "set_servo" || type == "set_relay" || type == "motor_test" ||
           type == "configure_gimbal" || type == "set_gimbal_target";
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

} // namespace nomad::runtime
