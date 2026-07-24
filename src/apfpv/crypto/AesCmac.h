#pragma once
#include <cstdint>
#include <cstddef>
namespace apfpv { namespace crypto {
// AES-128-CMAC (RFC 4493). Writes a 16-byte MAC of msg[0..len) under key into out.
void aes128_cmac(const uint8_t key[16], const uint8_t* msg, size_t len, uint8_t out[16]);
// BIP-CMAC-128 (802.11w): 8-byte MIC = first 8 bytes of AES-128-CMAC. Used to verify
// protected broadcast/multicast management frames against the IGTK.
void bip_cmac_128(const uint8_t igtk[16], const uint8_t* msg, size_t len, uint8_t mic8[8]);
// RFC 4493 known-answer self-test (empty + 16-byte message). True == correct.
bool aes_cmac_selftest();
}}
