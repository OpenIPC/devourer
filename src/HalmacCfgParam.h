#ifndef DEVOURER_HALMAC_CFG_PARAM_H
#define DEVOURER_HALMAC_CFG_PARAM_H

#include <cstdint>
#include <functional>
#include <vector>

#include "logger.h"

namespace devourer {

/* HalMAC firmware register IO-offload (cfg_param). Accumulates a stream of
 * static register writes into a 12-byte-per-command buffer, then flushes each
 * batch as ONE FW_OFFLOAD/CFG_PARAM H2C the on-chip firmware replays locally —
 * collapsing the thousands of per-init phy-table USB control transfers into a
 * handful of bulk transfers. Generation-agnostic (8822B/8821C/8822C/8822E all
 * speak the same cfg_param wire format); the per-generation transport is
 * injected as callbacks.
 *
 * Command wire format (12 bytes LE, matches halmac add_param_buf_88xx):
 *   word0 = LEN(0x0C)[7:0] | IO_CMD[14:8] | MSK_EN[15] |
 *           { MAC/BB: ADDR[31:16] } | { RF: RF_ADDR[23:16] | RF_PATH[31:24] }
 *   word1 = DATA (value)
 *   word2 = MASK (only when MSK_EN=1)
 * Static table entries are full-register writes, so MSK_EN=0 / MASK=0 — the
 * firmware writes the value verbatim (no on-chip read-modify-write, so the
 * masked-value shift convention is irrelevant here).
 *
 * Trigger H2C (32 bytes): {0x01, 0xff, 0x08(CFG_PARAM), 0, len, 0, seq, 0} then
 * at byte 8: NUM[15:0] | INIT_CASE[16] | LOC[31:24]. Drv mode (INIT_CASE=0)
 * places the buffer at the reserved H2C-extra-info page (LOC = its page offset
 * from the rsvd boundary). */
class HalmacCfgParam {
public:
  /* IO_CMD ids (enum halmac_parameter_cmd). */
  enum Cmd : uint8_t {
    MAC_W8 = 0x4,
    MAC_W16 = 0x5,
    MAC_W32 = 0x6,
    RF_W = 0x7,
    BB_W8 = 0x8,
    BB_W16 = 0x9,
    BB_W32 = 0xA,
    END = 0xFF,
  };

  struct Transport {
    /* DMA the packed command buffer to the reserved H2C-extra-info page
     * (dl_rsvd_page / send_fw_page with a data descriptor). */
    std::function<bool(uint16_t pg_addr, const uint8_t *buf, uint32_t len)>
        dl_rsvd_page;
    /* Send a 32-byte FW_OFFLOAD H2C packet (the CFG_PARAM trigger). */
    std::function<bool(const uint8_t pkt[32])> send_h2c_pkt;
    /* Next rolling H2C sequence number (halmac h2c_info.seq_num). */
    std::function<uint8_t()> next_seq;
    /* Block until the firmware has consumed the trigger + replayed the buffer,
     * so the extra-info page is safe to reuse for the next batch. */
    std::function<void()> settle;
    /* Read back a BB register (full dword). After each flush the last BB write
     * in the batch is read back and compared to what was sent — this both
     * confirms the firmware actually replayed the buffer (vs silently dropping
     * the H2C) and gates page reuse. A mismatch marks the offload failed so the
     * caller falls back to direct writes. Optional; skipped if unset. */
    std::function<uint32_t(uint16_t bb_addr)> verify_read;
    uint16_t cfg_pg_addr = 0; /* absolute TX-FIFO page of the extra-info buffer */
    uint16_t cfg_loc = 0;     /* LOC field: that page's offset from rsvd boundary */
    uint32_t max_cmds = 160;  /* flush threshold (extra-info page holds ~170) */
  };

  HalmacCfgParam(Transport t, Logger_t logger);

  void bb_write(uint16_t addr, uint32_t value);
  void mac_write(uint16_t addr, uint32_t value, uint8_t width);
  void rf_write(uint8_t path, uint8_t addr, uint32_t value);

  /* Flush the accumulated batch. No-op if empty. Returns false on a transport
   * failure (the caller falls back to direct writes). Auto-invoked when the
   * buffer reaches max_cmds. */
  bool flush();

  bool ok() const { return _ok; }
  uint64_t total() const { return _total; } /* commands offloaded this session */
  uint32_t pending() const { return _num; }

private:
  void push(uint8_t cmd, uint32_t addr_field, uint8_t path, uint32_t value);

  Transport _t;
  Logger_t _logger;
  std::vector<uint8_t> _buf; /* packed 12-byte commands, current batch */
  uint32_t _num = 0;
  uint64_t _total = 0;
  bool _ok = true;
  bool _last_was_bb = false; /* for the per-flush read-back verify */
  uint16_t _last_bb_addr = 0;
  uint32_t _last_bb_value = 0;
};

} // namespace devourer

#endif
