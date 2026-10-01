// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace nomad::runtime::detail {

enum class FileMode { Read, Create, Lock };

// One owned native handle. Creation is exclusive; writes are explicitly synchronized.
class ProtectedFile {
  public:
    ~ProtectedFile();
    ProtectedFile() = default;
    ProtectedFile(const ProtectedFile &) = delete;
    ProtectedFile &operator=(const ProtectedFile &) = delete;
    bool open(const std::string &path, FileMode mode);
    bool read(std::string &text, std::size_t limit);
    bool append(std::string_view text);
    void close();

  private:
    std::intptr_t handle_{-1};
};

bool prepare_audit_directory(const std::string &path);
bool sync_directory(const std::string &path);

} // namespace nomad::runtime::detail
