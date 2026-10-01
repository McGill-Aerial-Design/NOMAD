// SPDX-License-Identifier: Apache-2.0
#include "auth_proof.hpp"

#include <array>
#include <cstdint>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <bcrypt.h>
#else
#include <openssl/hmac.h>
#include <openssl/rand.h>
#endif

namespace nomad::runtime::detail {
namespace {

bool hash(std::string_view secret, std::string_view payload, std::array<unsigned char, 32> &digest) {
#ifdef _WIN32
    BCRYPT_ALG_HANDLE algorithm = nullptr;
    if (BCryptOpenAlgorithmProvider(&algorithm, BCRYPT_SHA256_ALGORITHM, nullptr, BCRYPT_ALG_HANDLE_HMAC_FLAG) < 0) {
        return false;
    }
    BCRYPT_HASH_HANDLE handle = nullptr;
    const auto created = BCryptCreateHash(algorithm, &handle, nullptr, 0,
        reinterpret_cast<PUCHAR>(const_cast<char *>(secret.data())), static_cast<ULONG>(secret.size()), 0);
    const bool success = created >= 0 && BCryptHashData(handle,
        reinterpret_cast<PUCHAR>(const_cast<char *>(payload.data())), static_cast<ULONG>(payload.size()), 0) >= 0 &&
        BCryptFinishHash(handle, digest.data(), static_cast<ULONG>(digest.size()), 0) >= 0;
    if (handle != nullptr) {
        BCryptDestroyHash(handle);
    }
    BCryptCloseAlgorithmProvider(algorithm, 0);
    return success;
#else
    unsigned int size = 0;
    return HMAC(EVP_sha256(), secret.data(), static_cast<int>(secret.size()),
                reinterpret_cast<const unsigned char *>(payload.data()), payload.size(),
                digest.data(), &size) != nullptr &&
           size == digest.size();
#endif
}

} // namespace

std::string make_proof(std::string_view secret, std::string_view payload) {
    std::array<unsigned char, 32> digest{};
    if (!hash(secret, payload, digest)) {
        return {};
    }
    constexpr char digits[] = "0123456789abcdef";
    std::string result;
    for (const auto byte : digest) {
        result.push_back(digits[byte >> 4]);
        result.push_back(digits[byte & 15]);
    }
    return result;
}

bool equal_proof(std::string_view left, std::string_view right) {
    if (left.size() != 64 || right.size() != 64) {
        return false;
    }
    volatile std::uint32_t difference = 0;
    for (std::size_t index = 0; index < 64; ++index) {
        difference = difference | static_cast<unsigned char>(left[index] ^ right[index]);
    }
    return difference == 0;
}

std::string make_nonce() {
    std::array<unsigned char, 32> bytes{};
#ifdef _WIN32
    const bool generated = BCryptGenRandom(nullptr, bytes.data(), static_cast<ULONG>(bytes.size()),
                                          BCRYPT_USE_SYSTEM_PREFERRED_RNG) >= 0;
#else
    const bool generated = RAND_bytes(bytes.data(), static_cast<int>(bytes.size())) == 1;
#endif
    if (!generated) {
        return {};
    }
    constexpr char digits[] = "0123456789abcdef";
    std::string result;
    for (const auto byte : bytes) {
        result.push_back(digits[byte >> 4]);
        result.push_back(digits[byte & 15]);
    }
    return result;
}

} // namespace nomad::runtime::detail
