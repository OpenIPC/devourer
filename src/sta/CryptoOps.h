/* CryptoOps — the crypto primitives src/sta/ needs, as an interface it does
 * not implement.
 *
 * WHY AN INTERFACE AND NOT JUST OPENSSL. `libdevourer` links no crypto
 * library, and adding one for a station would be a dependency every consumer
 * of the library pays for whether or not they ever associate to anything. So
 * the protocol logic here takes its crypto as a vtable: the selftests fill it
 * with OpenSSL (tests/openssl_crypto_ops.h), checked against vectors from
 * third implementations, and an embedded integrator can fill it with mbedTLS
 * without touching a line of the state machine.
 *
 * It also makes the 4-way handshake testable without a radio OR a crypto
 * library, and it keeps the cipher, PBKDF2 and the PRF out of this tree:
 * hand-rolled crypto without known-answer tests is the defect class this
 * module is built to avoid, and the vectors in tests/ pin every primitive the
 * protocol code relies on.
 *
 * Every method returns false on failure rather than throwing; a station that
 * cannot decrypt a frame drops it and carries on.
 */
#ifndef DEVOURER_STA_CRYPTO_OPS_H
#define DEVOURER_STA_CRYPTO_OPS_H

#include <cstddef>
#include <cstdint>

namespace devourer {
namespace sta {

struct CryptoOps {
  virtual ~CryptoOps() = default;

  /* AES-128-CCM with a 13-byte nonce and an 8-byte tag — the shape 802.11
   * CCMP uses, and the only primitive Ccmp.h needs.
   *
   * encrypt: `in`/`in_len` is the plaintext, `out` takes in_len ciphertext
   *          bytes and `tag` takes 8 bytes. `out` may alias `in`.
   * decrypt: `in` is the ciphertext, `tag` the received MIC; returns false
   *          when the tag does not verify, and the caller MUST treat that as
   *          a dropped frame rather than inspecting `out`.
   *
   * The tag comparison must be constant-time in any implementation that runs
   * against untrusted input. */
  virtual bool aes_ccm(bool encrypt, const uint8_t key[16],
                       const uint8_t nonce[13], const uint8_t* aad,
                       size_t aad_len, const uint8_t* in, size_t in_len,
                       uint8_t* out, uint8_t* tag) = 0;

  /* HMAC-SHA1. The EAPOL-Key MIC and the PRF are both built on it. */
  virtual bool hmac_sha1(const uint8_t* key, size_t key_len,
                         const uint8_t* data, size_t data_len,
                         uint8_t out[20]) = 0;

  /* PBKDF2-HMAC-SHA1, 4096 iterations, 32-byte output — the WPA2-PSK PMK
   * derivation from passphrase + SSID. */
  virtual bool pbkdf2_sha1(const char* passphrase, const uint8_t* salt,
                           size_t salt_len, unsigned iterations,
                           uint8_t* out, size_t out_len) = 0;

  /* RFC 3394 AES key unwrap — the supplicant half of the GTK delivery in
   * EAPOL-Key message 3. (The AP harness wraps; a station unwraps.) Returns
   * false when the integrity check value does not match, which is a real
   * authentication failure and not a decode hiccup. */
  virtual bool aes_key_unwrap(const uint8_t* kek, size_t kek_len,
                              const uint8_t* in, size_t in_len,
                              uint8_t* out) = 0;
};

}  // namespace sta
}  // namespace devourer

#endif /* DEVOURER_STA_CRYPTO_OPS_H */
