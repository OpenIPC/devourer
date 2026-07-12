#ifndef KESTREL_KESTREL_FW_H
#define KESTREL_KESTREL_FW_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "ChipVariant.h"
#include "RtlAdapter.h"
#include "logger.h"

namespace kestrel {

/* KestrelFw owns the firmware-download layer (milestone M1b), ported from
 * reference/rtl8852bu mac_ax: the DMAC/DLE/HFC pre-init that stands up the H2C
 * (CH12) transport, then the three-phase FWDL state machine (header H2C ->
 * section chunks -> WCPU boot poll). Runs after HalKestrel::power_on.
 *
 * The firmware image is the embedded NICCE blob (hal8852b_fw.c), cut-selected:
 * cut CCV (>=2) -> array_8852b_u3_nicce, cut CBV -> u2. */
class KestrelFw {
public:
  KestrelFw(RtlAdapter device, Logger_t logger, ChipVariant variant);

  /* Full M1b sequence: hci_func_en + dmac_pre_init (DLE/HFC for DLFW) +
   * disable/enable WCPU + download the NICCE image for `cut` + fw-ready poll.
   * `mss_idx` selects the secure-boot signature (HalKestrel::read_mss_index).
   * Returns false on any poll timeout or FW error status (checksum/security/
   * cut mismatch), all logged. */
  bool download_firmware(uint8_t cut, uint8_t mss_idx);

private:
  /* --- register-op helpers (mirror HalKestrel; kept local to this TU) --- */
  void set32(uint16_t reg, uint32_t bits);
  void clr32(uint16_t reg, uint32_t bits);
  bool poll_wcpu(uint32_t mask, uint32_t expect, const char *what);

  /* --- pre-init --- */
  bool hci_func_en();
  bool dmac_pre_init();        /* dmac_func_pre_en + dle_init + hfc_init */
  bool dle_init_dlfw();
  bool hfc_init_dlfw();
  bool chk_dle_rdy(uint16_t status_reg, uint32_t rdy_bits, const char *what);

  /* --- FWDL state machine --- */
  bool disable_cpu();
  bool enable_cpu(uint8_t boot_reason);
  bool mac_fwdl(const uint8_t *fw, uint32_t len, uint8_t mss_idx);
  bool check_fw_rdy();
  /* Build [WD 24B] (+ [fwcmd_hdr 8B] when is_header) + payload and bulk-send
   * to the CH12 endpoint. seq is the H2C sequence counter (header only). */
  bool send_fwdl_packet(const uint8_t *payload, uint32_t payload_len,
                        bool is_header, uint8_t seq);

  RtlAdapter _device;
  Logger_t _logger;
  ChipVariant _variant;
  uint8_t _ch12_ep = 0; /* resolved bulk-OUT endpoint for CH12 (BULKOUTID2) */
  std::vector<uint8_t> _txbuf; /* reused H2C packet scratch */
};

} /* namespace kestrel */

#endif /* KESTREL_KESTREL_FW_H */
