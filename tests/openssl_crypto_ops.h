/* openssl_crypto_ops.h — a complete CryptoOps, for tests and harnesses.
 *
 * `src/sta/` takes its crypto as a vtable so `libdevourer` gains no
 * dependency. Everything that drives it in-tree has OpenSSL available, and
 * one shared implementation is what keeps the cells from each filling the
 * vtable themselves: partial implementations of one interface, each stubbing
 * a different subset, is how they come to disagree.
 *
 * This one implements ALL FOUR methods. A caller that needs to wrap one (to
 * time its CCM calls, say) derives and overrides that method only.
 *
 * NOT constant-time beyond what OpenSSL gives, and not hardened for key
 * material in memory. It is a test facility; an embedded integrator
 * fills the same vtable with mbedTLS.
 */
#ifndef DEVOURER_TEST_OPENSSL_CRYPTO_OPS_H
#define DEVOURER_TEST_OPENSSL_CRYPTO_OPS_H

#include <openssl/evp.h>
#include <openssl/hmac.h>

#include <cstdint>
#include <cstring>

#include "ccmp_software.h"
#include "sta/CryptoOps.h"

namespace devourer {
namespace test {

struct OpenSslCryptoOps : devourer::sta::CryptoOps {
  bool aes_ccm(bool encrypt, const uint8_t key[16], const uint8_t nonce[13],
               const uint8_t* aad, size_t aad_len, const uint8_t* in,
               size_t in_len, uint8_t* out, uint8_t* tag) override {
    return ccmp_software(encrypt, key, nonce, aad, (int)aad_len, in,
                         (int)in_len, out, tag);
  }

  bool hmac_sha1(const uint8_t* key, size_t key_len, const uint8_t* data,
                 size_t data_len, uint8_t out[20]) override {
    unsigned int l = 0;

    if (!HMAC(EVP_sha1(), key, (int)key_len, data, data_len, out, &l))
      return false;
    return l == 20;
  }

  bool pbkdf2_sha1(const char* passphrase, const uint8_t* salt,
                   size_t salt_len, unsigned iterations, uint8_t* out,
                   size_t out_len) override {
    return PKCS5_PBKDF2_HMAC(passphrase, (int)std::strlen(passphrase), salt,
                             (int)salt_len, (int)iterations, EVP_sha1(),
                             (int)out_len, out) == 1;
  }

  /* RFC 3394 unwrap. The integrity check value is checked by the cipher, so a
   * false return here is a real authentication failure and the caller must
   * treat `out` as untouched rather than parse it. */
  bool aes_key_unwrap(const uint8_t* kek, size_t kek_len, const uint8_t* in,
                      size_t in_len, uint8_t* out) override {
    const EVP_CIPHER* c = kek_len == 16   ? EVP_aes_128_wrap()
                          : kek_len == 32 ? EVP_aes_256_wrap()
                                          : nullptr;
    EVP_CIPHER_CTX* ctx;
    int ol = 0, tmp = 0, ok;

    if (!c || in_len < 16 || (in_len % 8) != 0) return false;
    ctx = EVP_CIPHER_CTX_new();
    if (!ctx) return false;
    EVP_CIPHER_CTX_set_flags(ctx, EVP_CIPHER_CTX_FLAG_WRAP_ALLOW);
    ok = EVP_DecryptInit_ex(ctx, c, nullptr, kek, nullptr);
    if (ok) ok = EVP_DecryptUpdate(ctx, out, &ol, in, (int)in_len);
    if (ok) ok = EVP_DecryptFinal_ex(ctx, out + ol, &tmp);
    EVP_CIPHER_CTX_free(ctx);
    return ok == 1 && (size_t)(ol + tmp) == in_len - 8;
  }

  /* The wrap direction, which no production path needs — only an authenticator
   * does, and the only authenticators here are test fixtures. Not part of
   * CryptoOps for that reason. */
  static int key_wrap(const uint8_t* kek, size_t kek_len, const uint8_t* in,
                      size_t in_len, uint8_t* out) {
    const EVP_CIPHER* c = kek_len == 16   ? EVP_aes_128_wrap()
                          : kek_len == 32 ? EVP_aes_256_wrap()
                                          : nullptr;
    EVP_CIPHER_CTX* ctx;
    int ol = 0, tmp = 0, ok;

    if (!c || (in_len % 8) != 0) return -1;
    ctx = EVP_CIPHER_CTX_new();
    if (!ctx) return -1;
    EVP_CIPHER_CTX_set_flags(ctx, EVP_CIPHER_CTX_FLAG_WRAP_ALLOW);
    ok = EVP_EncryptInit_ex(ctx, c, nullptr, kek, nullptr);
    if (ok) ok = EVP_EncryptUpdate(ctx, out, &ol, in, (int)in_len);
    if (ok) ok = EVP_EncryptFinal_ex(ctx, out + ol, &tmp);
    EVP_CIPHER_CTX_free(ctx);
    return ok == 1 ? ol + tmp : -1;
  }
};

}  // namespace test
}  // namespace devourer

#endif /* DEVOURER_TEST_OPENSSL_CRYPTO_OPS_H */
