#pragma once
#include <cstdint>
#include <cstddef>
namespace apfpv { namespace crypto {
// AES-128 expanded key (cached by caller to avoid per-packet key expansion)
struct Aes { uint8_t rk[176]; };
void aes_key_expand(const uint8_t key[16], Aes& out);
// AES-128-CCM as used by CCMP (8-byte MIC, 13-byte nonce, L=2).
// decrypt: returns true if MIC valid; out gets plaintext (clen-8 bytes).
// Pass cached expanded key (aes_key_expand) for the hot path, or nullptr.
bool aes_ccm_decrypt(const uint8_t key[16], const uint8_t* nonce, size_t nlen,
                     const uint8_t* aad, size_t aadlen,
                     const uint8_t* ct, size_t ctlen, uint8_t* out,
                     const Aes* aes_cache = nullptr);
// Hardware-AES accelerated decrypt — data RX only. ARM-CE on aarch64, AES-NI on
// x86. Returns false to fall back to the SW path. Mirrors APFPV_HAVE_AES_HW in the .cpp.
#if defined(__aarch64__) || defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
bool aes_ccm_decrypt_ce(const uint8_t key[16],const uint8_t* nonce,size_t nlen,
                        const uint8_t* aad,size_t aadlen,
                        const uint8_t* ct,size_t ctlen,uint8_t* out);
#endif
void aes_ccm_encrypt(const uint8_t key[16], const uint8_t* nonce, size_t nlen,
                     const uint8_t* aad, size_t aadlen,
                     const uint8_t* pt, size_t ptlen, uint8_t* out /* ptlen+8 */);
}}

namespace apfpv { namespace crypto {
// AES-128 ECB single-block (for key-wrap). out=enc(in) under key.
void aes128_ecb_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);
void aes128_ecb_decrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);
// RFC 3394 AES Key Unwrap. n = number of 64-bit blocks in wrapped-8. Returns
// true if integrity check (A == A6A6A6A6A6A6A6A6) passes. out = n*8 bytes.
bool aes_key_unwrap(const uint8_t kek[16], const uint8_t* wrapped, size_t wlen, uint8_t* out);
// RFC 3394 AES Key Wrap (inverse). out = (plen+8) bytes. plen multiple of 8, >= 16.
void aes_key_wrap(const uint8_t kek[16], const uint8_t* plain, size_t plen, uint8_t* out);
}}
