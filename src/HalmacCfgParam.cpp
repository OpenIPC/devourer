#include "HalmacCfgParam.h"

#include <chrono>
#include <cstring>
#include <thread>

namespace devourer {

namespace {
constexpr uint8_t CMD_LEN = 0x0C; /* 12-byte command */
inline void put_le32(uint8_t *p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
}
} // namespace

HalmacCfgParam::HalmacCfgParam(Transport t, Logger_t logger)
    : _t(std::move(t)), _logger(std::move(logger)) {
  _buf.reserve((static_cast<size_t>(_t.max_cmds) + 1) * CMD_LEN);
}

void HalmacCfgParam::push(uint8_t cmd, uint32_t addr_field, uint8_t /*path*/,
                          uint32_t value) {
  /* addr_field is pre-shaped by the caller into word0[31:16] (MAC/BB addr) or
   * word0[31:16] = RF_ADDR<<0 | RF_PATH<<8 already positioned. MSK_EN=0 for the
   * full-register static writes this offload carries. */
  uint32_t w0 = CMD_LEN | (static_cast<uint32_t>(cmd) << 8) |
                (addr_field & 0xffff0000u);
  const size_t off = _buf.size();
  _buf.resize(off + CMD_LEN);
  put_le32(_buf.data() + off + 0, w0);
  put_le32(_buf.data() + off + 4, value);
  put_le32(_buf.data() + off + 8, 0); /* MASK unused (MSK_EN=0) */
  _num++;
  if (_num >= _t.max_cmds)
    flush();
}

void HalmacCfgParam::bb_write(uint16_t addr, uint32_t value) {
  _last_was_bb = true;
  _last_bb_addr = addr;
  _last_bb_value = value;
  push(BB_W32, static_cast<uint32_t>(addr) << 16, 0, value);
}

void HalmacCfgParam::mac_write(uint16_t addr, uint32_t value, uint8_t width) {
  const uint8_t cmd = width == 1 ? MAC_W8 : width == 2 ? MAC_W16 : MAC_W32;
  push(cmd, static_cast<uint32_t>(addr) << 16, 0, value);
}

void HalmacCfgParam::rf_write(uint8_t path, uint8_t addr, uint32_t value) {
  /* word0[23:16] = RF_ADDR (8-bit), word0[31:24] = RF_PATH. */
  const uint32_t addr_field = (static_cast<uint32_t>(addr) << 16) |
                              (static_cast<uint32_t>(path) << 24);
  push(RF_W, addr_field, path, value & 0x000fffffu);
}

bool HalmacCfgParam::flush() {
  if (_num == 0)
    return _ok;
  /* Terminate the buffer with an END command (belt-and-suspenders alongside the
   * NUM field the trigger carries). */
  const size_t off = _buf.size();
  _buf.resize(off + CMD_LEN, 0);
  put_le32(_buf.data() + off + 0, CMD_LEN | (static_cast<uint32_t>(END) << 8));

  const uint32_t num = _num;
  bool ok = _t.dl_rsvd_page(_t.cfg_pg_addr, _buf.data(),
                            static_cast<uint32_t>(_buf.size()));
  if (ok) {
    uint8_t trig[32] = {0};
    trig[0] = 0x01;
    trig[1] = 0xff; /* CMD_ID_FW_OFFLOAD */
    trig[2] = 0x08; /* SUB_CMD_ID_CFG_PARAM */
    trig[4] = 8 + 4; /* header + 4-byte payload */
    trig[6] = _t.next_seq();
    /* byte 8..11: NUM[15:0] | INIT_CASE[16]=0 (drv mode) | LOC[31:24]. */
    put_le32(trig + 8,
             (num & 0xffffu) | (static_cast<uint32_t>(_t.cfg_loc) << 24));
    ok = _t.send_h2c_pkt(trig);
    if (ok && _t.settle)
      _t.settle(); /* trigger-consumed check (cheap; not the replay gate) */
  }

  /* Confirm the firmware actually replayed the batch by polling the last BB
   * write back until it reads what was sent. This doubles as the completion
   * gate (the extra-info page is safe to reuse once the value lands) and the
   * correctness check (a dropped/mis-parsed H2C never lands → offload fails and
   * the caller redoes the batch direct). No fixed settle — the poll returns the
   * instant the on-chip replay reaches the final write. */
  if (ok && _last_was_bb && _t.verify_read) {
    uint32_t got = 0;
    bool matched = false;
    for (int i = 0; i < 40; ++i) { /* up to ~8 ms */
      got = _t.verify_read(_last_bb_addr);
      if (got == _last_bb_value) {
        matched = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    if (!matched) {
      _logger->error("HalMAC cfg_param: readback 0x{:04x}=0x{:08x} != sent "
                     "0x{:08x} — firmware did not replay the batch",
                     _last_bb_addr, got, _last_bb_value);
      ok = false;
    }
  } else if (ok && _num > 0) {
    /* Batch ended on a non-BB (RF) write, which the BB read-back can't gate;
     * the shadow-window RF read-back isn't wired, so give the on-chip replay a
     * bounded margin before the page is reused. */
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  if (!ok) {
    _ok = false;
    _logger->error("HalMAC cfg_param: flush failed ({} cmds) — falling back",
                   num);
  } else {
    _total += num;
  }
  _buf.clear();
  _num = 0;
  _last_was_bb = false;
  return ok;
}

} // namespace devourer
