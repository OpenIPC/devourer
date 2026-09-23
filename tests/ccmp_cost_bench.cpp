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
// Each timed call is the harness's ccm() whole: a fresh EVP_CIPHER_CTX per
// call, init, AAD, payload, tag, free. Encrypt and decrypt are timed
// separately - the RX path pays the second one.
//
// Build + run: tests/ccmp_cost_bench.sh. Prints one JSON line per frame size
// and direction.
#include <openssl/evp.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

// tests/ap_wpa2.cpp ccm(), verbatim in shape: context lifecycle included.
static bool ccm(bool enc, const uint8_t *key, const uint8_t *nonce,
                const uint8_t *aad, int aadlen, const uint8_t *in, int inlen,
                uint8_t *out, uint8_t *tag) {
  EVP_CIPHER_CTX *c = EVP_CIPHER_CTX_new();
  int l;
  bool ok = true;
  if (enc) {
    EVP_EncryptInit_ex(c, EVP_aes_128_ccm(), 0, 0, 0);
    EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_AEAD_SET_IVLEN, 13, 0);
    EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_AEAD_SET_TAG, 8, 0);
    EVP_EncryptInit_ex(c, 0, 0, key, nonce);
    EVP_EncryptUpdate(c, 0, &l, 0, inlen);
    EVP_EncryptUpdate(c, 0, &l, aad, aadlen);
    ok = EVP_EncryptUpdate(c, out, &l, in, inlen) == 1;
    EVP_EncryptFinal_ex(c, out + l, &l);
    EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_AEAD_GET_TAG, 8, tag);
  } else {
    EVP_DecryptInit_ex(c, EVP_aes_128_ccm(), 0, 0, 0);
    EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_AEAD_SET_IVLEN, 13, 0);
    EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_AEAD_SET_TAG, 8, tag);
    EVP_DecryptInit_ex(c, 0, 0, key, nonce);
    EVP_DecryptUpdate(c, 0, &l, 0, inlen);
    EVP_DecryptUpdate(c, 0, &l, aad, aadlen);
    ok = EVP_DecryptUpdate(c, out, &l, in, inlen) == 1;
  }
  EVP_CIPHER_CTX_free(c);
  return ok;
}

int main(int argc, char **argv) {
  const int iters = argc > 1 ? std::atoi(argv[1]) : 20000;
  const int sizes[] = {64, 256, 1024, 1500, 3000};
  uint8_t key[16], nonce[13], aad[22], tag[8];
  for (int i = 0; i < 16; i++) key[i] = (uint8_t)(0x11 * i);
  for (int i = 0; i < 13; i++) nonce[i] = (uint8_t)i;
  for (int i = 0; i < 22; i++) aad[i] = (uint8_t)(0xa0 + i);

  for (int sz : sizes) {
    std::vector<uint8_t> in((size_t)sz, 0x5a), ct((size_t)sz), pt((size_t)sz);
    for (int dir = 0; dir < 2; dir++) {
      const bool enc = dir == 0;
      // One real ciphertext + tag for the decrypt arm to verify against.
      if (!ccm(true, key, nonce, aad, sizeof aad, in.data(), sz, ct.data(), tag))
        return 1;
      for (int i = 0; i < 200; i++)  // warm up
        if (!ccm(enc, key, nonce, aad, sizeof aad,
                 enc ? in.data() : ct.data(), sz, enc ? ct.data() : pt.data(),
                 tag))
          return 1;
      const auto t0 = std::chrono::steady_clock::now();
      for (int i = 0; i < iters; i++) {
        if (!ccm(enc, key, nonce, aad, sizeof aad,
                 enc ? in.data() : ct.data(), sz, enc ? ct.data() : pt.data(),
                 tag))
          return 1;
      }
      const double us = std::chrono::duration<double, std::micro>(
                            std::chrono::steady_clock::now() - t0).count();
      std::printf("{\"ev\":\"ccmp.bench\",\"dir\":\"%s\",\"bytes\":%d,"
                  "\"iters\":%d,\"us_per_frame\":%.2f,\"frames_per_s\":%.0f,"
                  "\"mbit_per_s\":%.0f}\n",
                  enc ? "encrypt" : "decrypt", sz, iters, us / iters,
                  iters / (us / 1e6), (double)sz * 8 * iters / us);
    }
  }
  return 0;
}
