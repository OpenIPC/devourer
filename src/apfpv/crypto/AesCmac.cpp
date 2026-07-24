// ============================================================================
//  AesCmac.cpp — AES-128-CMAC (RFC 4493) + BIP-CMAC-128 (802.11w), built on the
//  vendored aes128_ecb_encrypt. No external deps. Self-tested with RFC 4493.
// ============================================================================
#include "AesCmac.h"
#include "AesCcm.h"
#include <cstring>

namespace apfpv { namespace crypto {

static void lshift1(const uint8_t in[16], uint8_t out[16]) {
    uint8_t carry = 0;
    for (int i = 15; i >= 0; --i) { uint16_t v = ((uint16_t)in[i] << 1) | carry; out[i] = (uint8_t)v; carry = (v >> 8) & 1; }
}

void aes128_cmac(const uint8_t key[16], const uint8_t* msg, size_t len, uint8_t out[16]) {
    const uint8_t Rb = 0x87;
    uint8_t zero[16] = {0}, L[16], K1[16], K2[16];
    aes128_ecb_encrypt(key, zero, L);                 // L = AES(K, 0)
    lshift1(L, K1);  if (L[0]  & 0x80) K1[15] ^= Rb;  // subkey K1
    lshift1(K1, K2); if (K1[0] & 0x80) K2[15] ^= Rb;  // subkey K2

    size_t n = (len + 15) / 16;
    bool complete;
    if (n == 0) { n = 1; complete = false; } else complete = (len % 16) == 0;

    uint8_t Mlast[16];
    const uint8_t* last = msg + (n - 1) * 16;
    if (complete) {
        for (int i = 0; i < 16; ++i) Mlast[i] = last[i] ^ K1[i];
    } else {
        uint8_t pad[16] = {0};
        size_t rem = len % 16;
        if (rem) std::memcpy(pad, last, rem);
        pad[rem] = 0x80;
        for (int i = 0; i < 16; ++i) Mlast[i] = pad[i] ^ K2[i];
    }

    uint8_t X[16] = {0}, Y[16];
    for (size_t i = 0; i + 1 < n; ++i) {
        for (int j = 0; j < 16; ++j) Y[j] = X[j] ^ msg[i * 16 + j];
        aes128_ecb_encrypt(key, Y, X);
    }
    for (int j = 0; j < 16; ++j) Y[j] = Mlast[j] ^ X[j];
    aes128_ecb_encrypt(key, Y, out);
}

void bip_cmac_128(const uint8_t igtk[16], const uint8_t* msg, size_t len, uint8_t mic8[8]) {
    uint8_t full[16];
    aes128_cmac(igtk, msg, len, full);
    std::memcpy(mic8, full, 8);                       // BIP-CMAC-128 MIC = first 8 bytes
}

bool aes_cmac_selftest() {
    const uint8_t key[16] = {0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c};
    uint8_t mac[16];
    aes128_cmac(key, nullptr, 0, mac);                // empty message
    static const uint8_t exp0[16] = {0xbb,0x1d,0x69,0x29,0xe9,0x59,0x37,0x28,0x7f,0xa3,0x7d,0x12,0x9b,0x75,0x67,0x46};
    if (std::memcmp(mac, exp0, 16) != 0) return false;
    const uint8_t m16[16] = {0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a};
    aes128_cmac(key, m16, 16, mac);                   // single full block
    static const uint8_t exp16[16] = {0x07,0x0a,0x16,0xb4,0x6b,0x4d,0x41,0x44,0xf7,0x9b,0xdd,0x9d,0xd0,0x4a,0x28,0x7c};
    return std::memcmp(mac, exp16, 16) == 0;
}

}}  // namespace apfpv::crypto
