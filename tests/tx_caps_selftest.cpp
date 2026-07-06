/* Headless guard for the TX-capability derivation (src/TxCaps.h
 * tx_caps_for_chains): the load-bearing rule is that STBC needs >=2 TX chains,
 * so a 1T1R part must report stbc_ok=false — that's what the send_packet STBC
 * guard relies on to not air a malformed frame on 8811AU/8821AU/8821CU. A
 * regression here fails `ctest` instead of only surfacing as a dead link. */
#include <cstdio>

#include "TxCaps.h"

static int g_fail = 0;

static void expect(uint8_t chains, uint8_t want_nss, bool want_stbc) {
  const devourer::TxCaps c = devourer::tx_caps_for_chains(chains);
  if (c.supported && c.n_ss == want_nss && c.stbc_ok == want_stbc)
    return;
  ++g_fail;
  std::printf("FAIL: chains=%u -> n_ss=%u stbc_ok=%d, want n_ss=%u stbc_ok=%d\n",
              chains, c.n_ss, c.stbc_ok ? 1 : 0, want_nss, want_stbc ? 1 : 0);
}

int main() {
  /* 1T1R cuts (8811AU/8821AU/8821CU): STBC impossible. */
  expect(1, 1, false);
  /* 2T2R (8812AU/8822BU/8822CU/8822EU): STBC ok. */
  expect(2, 2, true);
  /* 4T4R (8814AU): STBC ok. */
  expect(4, 4, true);

  /* Defaults for the AC family. */
  const devourer::TxCaps c = devourer::tx_caps_for_chains(2);
  if (!(c.ldpc_ok && c.sgi_ok && c.bw_max_mhz == 80)) {
    ++g_fail;
    std::printf("FAIL: AC defaults (ldpc=%d sgi=%d bw=%u)\n", c.ldpc_ok,
                c.sgi_ok, c.bw_max_mhz);
  }
  /* The default-constructed (unwired) caps report unsupported. */
  if (devourer::TxCaps{}.supported) {
    ++g_fail;
    std::printf("FAIL: default TxCaps should be unsupported\n");
  }

  if (g_fail) {
    std::printf("%d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("tx-caps selftest: all OK\n");
  return 0;
}
