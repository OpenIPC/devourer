#ifndef HALMAC_8822B_FW_H
#define HALMAC_8822B_FW_H

#include <cstddef>
#include <cstdint>

#include "logger.h"
#include "RtlUsbAdapter.h"

namespace jaguar2 {

/* HalmacJaguar2Fw — the RTL8822B firmware-download (DLFW) state machine. The
 * HalMAC 88xx DLFW code (halmac_fw_88xx.c) is byte-identical between the
 * rtl88x2bu and rtl88x2cu trees and the DLFW register addresses match, so this
 * is a faithful sibling of src/jaguar3/HalmacJaguar3Fw with the ChipVariant
 * dispatch removed (8822B is a single chip). The per-chunk transport is the
 * halmac send_fwpkt/dl_rsvd_page path: build an 88xx TX descriptor for the chunk
 * (FrameParserJaguar2) and bulk-OUT it into the HIQ reserved page.
 *
 * Boot success = REG_MCUFW_CTRL == 0xC078. Requires the pre-DLFW MAC config
 * (DDMA enable, SYS_FUNC_EN) and the queue/page allocation (_rsvd_boundary) to
 * have run first — supplied by HalmacJaguar2MacInit (M4). */
class HalmacJaguar2Fw {
public:
  HalmacJaguar2Fw(RtlUsbAdapter device, Logger_t logger);

  bool download_firmware(const uint8_t *fw_bin, size_t size);
  bool download_default_firmware(); /* bundled hal8822b_fw NIC image */

  /* The reserved-page boundary FIFOPAGE_CTRL_2 is restored to after each
   * rsvd-page chunk (from the queue/page allocation). Set by MacInit before
   * download_firmware(). */
  void set_rsvd_boundary(uint16_t b) { _rsvd_boundary = b; }

private:
  bool start_dlfw(const uint8_t *fw_bin, size_t size);
  bool dlfw_to_mem(const uint8_t *fw_bin, uint32_t src, uint32_t dest,
                   uint32_t size);
  bool iddma_dlfw(uint32_t src, uint32_t dest, uint32_t len, bool first);
  bool iddma_en(uint32_t src, uint32_t dest, uint32_t ctrl);
  bool check_fw_chksum(uint32_t mem_addr);
  bool dlfw_end_flow();
  void wlan_cpu_en(bool enable);
  void pltfm_reset();
  bool chk_fw_size(const uint8_t *fw_bin, size_t size);
  bool send_fw_page(uint16_t pg_addr, const uint8_t *chunk, uint32_t size);

  uint8_t r8(uint16_t reg);
  uint16_t r16(uint16_t reg);
  uint32_t r32(uint16_t reg);
  void w8(uint16_t reg, uint8_t v);
  void w16(uint16_t reg, uint16_t v);
  void w32(uint16_t reg, uint32_t v);
  void w32_set(uint16_t reg, uint32_t bits);

  RtlUsbAdapter _device;
  Logger_t _logger;
  /* Per-chunk DLFW size. The vendor 8822B driver downloads in 4096-byte chunks
   * (4144-byte bulk-OUT incl. the 48-byte TX desc); this fits the HIQ page
   * space allocated by init_trx_cfg (64 pages) with margin. DLFW_PKT_MAX_SIZE is
   * 8192 upstream, but 8192+48 exceeds the 8192-byte HIQ, so 4096 is used. */
  uint32_t _dlfw_pkt_size = 4096;
  uint16_t _rsvd_boundary = 0;
};

} /* namespace jaguar2 */

#endif /* HALMAC_8822B_FW_H */
