// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace nomad::runtime::detail {

inline constexpr std::size_t kMaximumMessageBytes = 64 * 1024;
using RequestHandler = std::function<std::string(std::string_view)>;

class IpcServer {
  public:
    IpcServer() = default;
    ~IpcServer();

    IpcServer(const IpcServer &) = delete;
    IpcServer &operator=(const IpcServer &) = delete;

    bool start(std::uint16_t port, RequestHandler handler, std::string &error);
    void stop();
    bool running() const;

  private:
    struct ClientWorker {
        std::thread thread;
        std::shared_ptr<std::atomic_bool> finished;
    };

    void accept_clients();
    void serve_client(std::intptr_t socket);
    void reap_finished_clients();
    void join_clients();

    std::atomic_bool running_{false};
    std::intptr_t listener_{-1};
    RequestHandler handler_;
    std::thread accept_thread_;
    std::mutex clients_mutex_;
    std::vector<ClientWorker> clients_;
};

} // namespace nomad::runtime::detail
