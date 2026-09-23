// ccmp_cost_bench — the per-frame cost of devourer's software CCMP path.
//
// The AP harnesses (tests/ap_wpa2.cpp, on every backend) encrypt and decrypt
// data frames in software with OpenSSL's AES-128-CCM. Hardware CCMP on the
// MT7612U is unreached, and the open question is whether reaching it would buy
// anything: that needs the software path's cost per frame, which this measures
// on the host it runs on. It is a number about this CPU, not about the radio -
// compare it with the per-frame send cost of the transport it would sit in
// front of (txdemo at DEVOURER_TX_GAP_US=0 gives that as frames/s).
//
// Build + run: tests/ccmp_cost_bench.sh. Prints one JSON line per frame size.
#include <openssl/evp.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

static bool ccm_enc(EVP_CIPHER_CTX *c, const uint8_t *key, const uint8_t *nonce,
                    const uint8_t *aad, int aadlen, const uint8_t *in, int inlen,
                    uint8_t *out, uint8_t *tag) {
  int l;
  // Same call sequence as tests/ap_wpa2.cpp's ccm(): fixed 13-byte nonce,
  // 8-byte MIC, AAD then payload.
  if (!EVP_EncryptInit_ex(c, EVP_aes_128_ccm(), nullptr, nullptr, nullptr))
    return false;
  EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_CCM_SET_IVLEN, 13, nullptr);
  EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_CCM_SET_TAG, 8, nullptr);
  if (!EVP_EncryptInit_ex(c, nullptr, nullptr, key, nonce)) return false;
  if (!EVP_EncryptUpdate(c, nullptr, &l, nullptr, inlen)) return false;
  if (!EVP_EncryptUpdate(c, nullptr, &l, aad, aadlen)) return false;
  if (!EVP_EncryptUpdate(c, out, &l, in, inlen)) return false;
  if (!EVP_EncryptFinal_ex(c, out + l, &l)) return false;
  EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_CCM_GET_TAG, 8, tag);
  return true;
}

int main(int argc, char **argv) {
  const int iters = argc > 1 ? std::atoi(argv[1]) : 20000;
  const int sizes[] = {64, 256, 1024, 1500, 3000};
  uint8_t key[16], nonce[13], aad[22], tag[8];
  for (int i = 0; i < 16; i++) key[i] = (uint8_t)(0x11 * i);
  for (int i = 0; i < 13; i++) nonce[i] = (uint8_t)i;
  for (int i = 0; i < 22; i++) aad[i] = (uint8_t)(0xa0 + i);

  EVP_CIPHER_CTX *c = EVP_CIPHER_CTX_new();
  for (int sz : sizes) {
    std::vector<uint8_t> in((size_t)sz, 0x5a), out((size_t)sz + 16);
    // warm up
    for (int i = 0; i < 200; i++)
      if (!ccm_enc(c, key, nonce, aad, sizeof aad, in.data(), sz, out.data(), tag)) return 1;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < iters; i++) {
      nonce[12] = (uint8_t)i;  // a moving PN, so nothing is hoisted
      if (!ccm_enc(c, key, nonce, aad, sizeof aad, in.data(), sz, out.data(), tag)) return 1;
    }
    const double us = std::chrono::duration<double, std::micro>(
                          std::chrono::steady_clock::now() - t0).count();
    std::printf("{\"ev\":\"ccmp.bench\",\"bytes\":%d,\"iters\":%d,"
                "\"us_per_frame\":%.2f,\"frames_per_s\":%.0f,\"mbit_per_s\":%.0f}\n",
                sz, iters, us / iters, iters / (us / 1e6),
                (double)sz * 8 * iters / us);
  }
  EVP_CIPHER_CTX_free(c);
  return 0;
}
