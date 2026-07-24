#pragma once
#include <cstdint>
#include <vector>
#include <array>
#include <string>
#include <functional>
namespace apfpv {
struct Crypto {
    std::function<std::array<uint8_t,32>(const std::string&, const uint8_t*, size_t)> pbkdf2_pmk;
    std::function<std::vector<uint8_t>(const uint8_t*, size_t, const std::string&, const uint8_t*, size_t, size_t)> prf;
    std::function<std::array<uint8_t,16>(const uint8_t*, const uint8_t*, size_t)> eapol_mic;
    std::function<bool(const uint8_t*, const uint8_t*, const uint8_t*, size_t, const uint8_t*, size_t, std::vector<uint8_t>&)> ccmp_decrypt;
    std::function<std::vector<uint8_t>(const uint8_t*, const uint8_t*, const uint8_t*, size_t, const uint8_t*, size_t)> ccmp_encrypt;
};
Crypto MakeWpa2Crypto();
}
