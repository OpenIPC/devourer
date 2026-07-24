#pragma once
#include <cstdint>
#include <cstddef>
namespace apfpv { namespace crypto {
void sha1(const uint8_t* data, size_t len, uint8_t out[20]);
void hmac_sha1(const uint8_t* key, size_t klen, const uint8_t* msg, size_t mlen, uint8_t out[20]);
void pbkdf2_sha1(const char* pass, const uint8_t* salt, size_t saltlen,
                 unsigned iters, uint8_t* out, size_t outlen);
}}
