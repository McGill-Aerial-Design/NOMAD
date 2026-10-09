// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/runtime/runtime.hpp"

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <string_view>

namespace nomad::runtime::detail {

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

class ActiveRequest {
  public:
    explicit ActiveRequest(const Request &request);
    ~ActiveRequest();

  private:
    const Request *previous_;
};

struct ParsedRequest {
    std::optional<Request> request;
    Json error;
};

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

extern thread_local const Request *active_request;

bool is_string(const Json &object, const char *key);
std::string field_string(const Json &object, const char *key);
Json error_response(std::string id, std::string code, std::string message);
bool read_finite_number(const Json &object, const char *key, double &value);
bool read_integer(const Json &object, const char *key, int &value);
bool read_unsigned(const Json &object, const char *key, std::uint64_t &value);
std::int64_t unix_milliseconds();
void update_gate_session(AuthorityGate &gate, std::uint64_t session);
bool is_newer_session(std::uint64_t session, std::uint64_t current_session);
bool matches_authority(const AuthorityGate &gate, const mavlink::SendAuthorityToken &token);
std::string new_incarnation();
bool has_supported_version(const Json &value);
mavlink::MavlinkConnection &require_connection(
    const std::unique_ptr<mavlink::MavlinkConnection> &connection);
bool validate_request_fields(Request &request, Json &error);
ParsedRequest parse_request(std::string_view line);
bool is_mutating(const std::string &type);
std::optional<std::int64_t> age_milliseconds(Clock::time_point timestamp);
Json optional_age(std::optional<std::int64_t> age);
std::string cache_key(const Request &request);
bool has_reasonable_json_depth(std::string_view line);

} // namespace nomad::runtime::detail
