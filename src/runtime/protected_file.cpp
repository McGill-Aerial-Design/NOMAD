// SPDX-License-Identifier: Apache-2.0
#include "protected_file.hpp"

#include <array>
#include <filesystem>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <sddl.h>
#include <aclapi.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace nomad::runtime::detail {
namespace {

#ifdef _WIN32
bool private_acl(HANDLE handle) {
    PSID owner = nullptr;
    PACL acl = nullptr;
    PSECURITY_DESCRIPTOR descriptor = nullptr;
    if (GetSecurityInfo(handle, SE_FILE_OBJECT, OWNER_SECURITY_INFORMATION | DACL_SECURITY_INFORMATION,
                        &owner, nullptr, &acl, nullptr, &descriptor) != ERROR_SUCCESS || acl == nullptr) {
        LocalFree(descriptor);
        return false;
    }
    BYTE system[SECURITY_MAX_SID_SIZE], admins[SECURITY_MAX_SID_SIZE];
    DWORD system_size = sizeof(system), admin_size = sizeof(admins);
    bool valid = CreateWellKnownSid(WinLocalSystemSid, nullptr, system, &system_size) &&
                 CreateWellKnownSid(WinBuiltinAdministratorsSid, nullptr, admins, &admin_size);
    for (DWORD index = 0; valid && index < acl->AceCount; ++index) {
        void *entry = nullptr;
        valid = GetAce(acl, index, &entry) != 0;
        if (!valid) {
            break;
        }
        const auto *ace = static_cast<ACCESS_ALLOWED_ACE *>(entry);
        if (ace->Header.AceType == ACCESS_DENIED_ACE_TYPE) {
            continue;
        }
        valid = ace->Header.AceType == ACCESS_ALLOWED_ACE_TYPE &&
                (EqualSid(const_cast<DWORD *>(&ace->SidStart), owner) ||
                 EqualSid(const_cast<DWORD *>(&ace->SidStart), system) ||
                 EqualSid(const_cast<DWORD *>(&ace->SidStart), admins));
    }
    LocalFree(descriptor);
    return valid;
}

class FileSecurity {
  public:
    FileSecurity() {
        // Protected DACL: current user, SYSTEM and administrators only.
        HANDLE token = nullptr;
        if (!OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &token)) {
            return;
        }
        DWORD size = 0;
        GetTokenInformation(token, TokenUser, nullptr, 0, &size);
        std::string buffer(size, '\0');
        const bool obtained = GetTokenInformation(token, TokenUser, buffer.data(), size, &size) != 0;
        CloseHandle(token);
        LPSTR sid = nullptr;
        if (!obtained || !ConvertSidToStringSidA(reinterpret_cast<TOKEN_USER *>(buffer.data())->User.Sid, &sid)) {
            return;
        }
        const std::string sddl = "D:P(A;;FA;;;SY)(A;;FA;;;BA)(A;;FA;;;" + std::string(sid) + ")";
        LocalFree(sid);
        if (ConvertStringSecurityDescriptorToSecurityDescriptorA(sddl.c_str(), SDDL_REVISION_1, &descriptor_,
                                                                 nullptr)) {
            attributes = {sizeof(SECURITY_ATTRIBUTES), descriptor_, FALSE};
        }
    }
    ~FileSecurity() {
        LocalFree(descriptor_);
    }
    SECURITY_ATTRIBUTES attributes{};
  private:
    PSECURITY_DESCRIPTOR descriptor_ = nullptr;
};
#else
bool private_file(int descriptor) {
    struct stat state{};
    return fstat(descriptor, &state) == 0 && S_ISREG(state.st_mode) && state.st_uid == geteuid() &&
           (state.st_mode & 0077) == 0 && state.st_nlink == 1;
}
#endif

} // namespace

ProtectedFile::~ProtectedFile() {
    close();
}

void ProtectedFile::close() {
    if (handle_ == -1) {
        return;
    }
#ifdef _WIN32
    CloseHandle(reinterpret_cast<HANDLE>(handle_));
#else
    ::close(static_cast<int>(handle_));
#endif
    handle_ = -1;
}

bool ProtectedFile::open(const std::string &path, FileMode mode) {
    close();
#ifdef _WIN32
    FileSecurity security;
    if (security.attributes.lpSecurityDescriptor == nullptr) {
        return false;
    }
    const auto access = mode == FileMode::Read ? GENERIC_READ : GENERIC_READ | GENERIC_WRITE;
    const auto creation = mode == FileMode::Read ? OPEN_EXISTING : mode == FileMode::Create ? CREATE_NEW : OPEN_ALWAYS;
    const DWORD sharing = mode == FileMode::Create ? FILE_SHARE_READ : 0;
    const auto handle = CreateFileW(std::filesystem::path(path).c_str(), access, sharing, &security.attributes,
                                   creation, FILE_FLAG_OPEN_REPARSE_POINT, nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
        return false;
    }
    handle_ = reinterpret_cast<std::intptr_t>(handle);
    BY_HANDLE_FILE_INFORMATION state{};
    if (!GetFileInformationByHandle(handle, &state) ||
        (state.dwFileAttributes & (FILE_ATTRIBUTE_REPARSE_POINT | FILE_ATTRIBUTE_DIRECTORY)) != 0 ||
        state.nNumberOfLinks != 1 || !private_acl(handle)) {
        close();
        return false;
    }
#else
    const auto flags = mode == FileMode::Read ? O_RDONLY : O_RDWR | O_CREAT;
    const auto exclusive = mode == FileMode::Create ? O_EXCL : 0;
    handle_ = ::open(path.c_str(), flags | exclusive | O_NOFOLLOW | O_CLOEXEC | O_NONBLOCK, 0600);
    if (handle_ == -1) {
        return false;
    }
    if (!private_file(static_cast<int>(handle_)) ||
        (mode == FileMode::Lock && flock(static_cast<int>(handle_), LOCK_EX | LOCK_NB) != 0)) {
        close();
        return false;
    }
#endif
    return true;
}

bool ProtectedFile::read(std::string &text, std::size_t limit) {
    std::array<char, 4096> buffer{};
    text.clear();
    while (true) {
#ifdef _WIN32
        DWORD count = 0;
        if (!ReadFile(reinterpret_cast<HANDLE>(handle_), buffer.data(), static_cast<DWORD>(buffer.size()),
                      &count, nullptr)) {
            return false;
        }
#else
        const auto count = ::read(static_cast<int>(handle_), buffer.data(), buffer.size());
        if (count < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
#endif
        if (count == 0) {
            return true;
        }
        if (text.size() + count > limit) {
            return false;
        }
        text.append(buffer.data(), count);
    }
}

bool ProtectedFile::append(std::string_view text) {
    std::size_t offset = 0;
    while (offset < text.size()) {
#ifdef _WIN32
        DWORD count = 0;
        if (!WriteFile(reinterpret_cast<HANDLE>(handle_), text.data() + offset,
                       static_cast<DWORD>(text.size() - offset), &count, nullptr) || count == 0) {
            return false;
        }
#else
        const auto count = ::write(static_cast<int>(handle_), text.data() + offset, text.size() - offset);
        if (count < 0 && errno == EINTR) {
            continue;
        }
        if (count <= 0) {
            return false;
        }
#endif
        offset += count;
    }
#ifdef _WIN32
    return FlushFileBuffers(reinterpret_cast<HANDLE>(handle_)) != 0;
#else
    return fsync(static_cast<int>(handle_)) == 0;
#endif
}

bool prepare_audit_directory(const std::string &path) {
    if (path.empty()) {
        return false;
    }
#ifdef _WIN32
    FileSecurity security;
    if (security.attributes.lpSecurityDescriptor == nullptr) {
        return false;
    }
    if (!CreateDirectoryW(std::filesystem::path(path).c_str(), &security.attributes) &&
        GetLastError() != ERROR_ALREADY_EXISTS) {
        return false;
    }
    const auto attributes = GetFileAttributesW(std::filesystem::path(path).c_str());
    const auto handle = CreateFileW(std::filesystem::path(path).c_str(), READ_CONTROL,
                                   FILE_SHARE_READ | FILE_SHARE_WRITE,
                                   nullptr, OPEN_EXISTING, FILE_FLAG_BACKUP_SEMANTICS | FILE_FLAG_OPEN_REPARSE_POINT,
                                   nullptr);
    const bool valid = attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY) != 0 &&
                       (attributes & FILE_ATTRIBUTE_REPARSE_POINT) == 0 && handle != INVALID_HANDLE_VALUE &&
                       private_acl(handle);
    if (handle != INVALID_HANDLE_VALUE) {
        CloseHandle(handle);
    }
    return valid;
#else
    if (mkdir(path.c_str(), 0700) != 0 && errno != EEXIST) {
        return false;
    }
    struct stat state{};
    return lstat(path.c_str(), &state) == 0 && S_ISDIR(state.st_mode) && state.st_uid == geteuid() &&
           (state.st_mode & 0077) == 0 && sync_directory(std::filesystem::path(path).parent_path().string());
#endif
}

bool sync_directory(const std::string &path) {
#ifdef _WIN32
    // FlushFileBuffers covers file content; Windows offers no portable directory fsync equivalent.
    return !path.empty();
#else
    const auto directory = ::open(path.empty() ? "." : path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (directory < 0) {
        return false;
    }
    const bool synced = fsync(directory) == 0;
    ::close(directory);
    return synced;
#endif
}

} // namespace nomad::runtime::detail
