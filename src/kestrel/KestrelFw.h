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

  /* mac_hal_init pre-FWDL half: hci_func_en + dmac_pre_init (DLE/HFC for
   * DLFW). The caller then runs usb_pre_init (intf_pre_init, vendor order)
   * before download_firmware. */
  bool fw_pre_init();

  /* The FWDL state machine: WDT config + disable/enable WCPU + download the
   * NICCE image for `cut` + fw-ready poll. `mss_idx` selects the secure-boot
   * signature (HalKestrel::read_mss_index). `is_sec_ic` (OTP 0x5ED[7]) gates
   * the non-secure IDMEM + CPU-clock FWDL patch. Returns false on any poll
   * timeout or FW error status (checksum/security/cut mismatch), all logged. */
  bool download_firmware(uint8_t cut, uint8_t mss_idx, bool is_sec_ic);

  /* --- Firmware IO-offload (M3) — program BB/RF registers via H2C batches the
   * firmware replays on-chip (mac_add_cmd_ofld / halbb_fw_set_reg /
   * halrf_wrf). Usage: ofld_begin(); ofld_write(...)*N; ofld_flush(style).
   * Commands accumulate; a batch auto-flushes when full (LC forced on the last
   * buffered command — ofld_incompatible_full_cmd). Section ends flush with a
   * harmless LC sentinel: halbb appends a BB write to 0x1a24 (mask 0xff val 0),
   * halrf appends a DELAY_OFLD of 1 us (halrf_write_fwofld_trigger). --- */
  enum class OfldFlush { BB, RF };
  void ofld_begin();
  void ofld_write(uint8_t src, uint8_t type, uint8_t path, uint16_t offset,
                  uint32_t value, uint32_t mask);
  bool ofld_flush(OfldFlush style = OfldFlush::BB);

  /* Send one radio-parameter page as an OUTSRC H2C (halrf radio-to-fw): `cls` =
   * OUTSRC_CL_RADIO_A/B, `page` = page index (H2C func), `packed` = the
   * (addr<<20|data) u32 array for this page, `count` entries. */
  bool radio_page_to_fw(uint8_t cls, uint8_t page, const uint32_t *packed,
                        uint16_t count);

  bool ch12_ready() const { return _ch12_ep != 0; }

  /* Enable the per-user TX report (mac_cfg_usr_tx_rpt): H2C cat=MAC,
   * class=FW_OFLD, func=USR_TX_RPT. `mode` = mac_ax_usr_tx_rpt_mode (PERIOD /
   * LAST_PKT), started immediately (RTP_START). The firmware then emits C2H
   * USR_TX_RPT_INFO reports carrying the freerun TX-egress timestamps. */
  bool enable_usr_tx_rpt(uint8_t mode, uint8_t macid, uint8_t port,
                         uint32_t period_us = 100000);

private:
  /* Generic H2C over CH12: [WD 24B][fwcmd_hdr 8B][content]. */
  bool send_h2c_cmd(uint8_t cat, uint8_t h2c_class, uint8_t func,
                    const uint8_t *content, uint32_t len);
  /* Poll + ack the C2H-register mailbox after a cmd_ofld batch (flow control:
   * the ack releases the fw to return the H2C page). */
  bool poll_cmd_ofld_result();
  /* Append one 16-byte cmd_ofld command to the batch buffer (no flush). */
  void ofld_append(uint8_t src, uint8_t type, uint8_t path, uint16_t offset,
                   uint32_t value, uint32_t mask, bool lc);
  /* Send the accumulated batch (proc_cmd_ofld): LC must already be set on the
   * last command. Sends, sleeps accu-delay, polls the c2hreg result. */
  bool ofld_send_batch();

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
  /* Non-secure-IC FWDL patches (fwdl.c): SEC_CTRL IDMEM-share to default, and
   * the fw CPU-clock write via the indirect-access IDMEM window. */
  void idmem_share_mode_check();
  void fwdl_patch_fw_delay();
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
  std::vector<uint8_t> _ofld_buf; /* accumulated cmd_ofld batch */
  uint32_t _ofld_cmd_num = 0;     /* commands in the current batch (resets/flush) */
  uint32_t _ofld_accu_delay_us = 0; /* DELAY_OFLD host-side wait after send */
  uint8_t _h2c_seq = 0; /* fwinfo->h2c_seq: 8-bit rolling, all runtime H2Cs */
  bool _is_sec_ic = false; /* OTP 0x5ED[7]: gates the non-secure FWDL patch */
};

} /* namespace kestrel */

#endif /* KESTREL_KESTREL_FW_H */
