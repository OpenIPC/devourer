#ifndef DEVOURER_JAGUAR3_TXPKT_PWR_BANKS_H
#define DEVOURER_JAGUAR3_TXPKT_PWR_BANKS_H

#include <cstdint>

/* Per-packet TX-power offset banks — the Jaguar3 (8822C/8812F/8822E) hardware
 * behind the TX-descriptor TXPWR_OFSET_TYPE field (txdesc+0x14[29:28]).
 *
 * The 2-bit type selects one of four power-index offset sources the BB adds
 * on top of the frame's rate-selected TXAGC (vendor phydm.h "bb_ram" block):
 *
 *   type 0 / 1 -> per-STA BB-RAM tx_pwr_offset0/1[macid] (0x1e84 write
 *                 protocol, readback 0x2de8) — macid = descriptor MACID.
 *   type 2     -> global reg0: 0x1e70[22:16], enable 0x1e70[23]
 *   type 3     -> global reg1: 0x1e70[30:24], enable 0x1e70[31]
 *
 * Offsets are 7-bit two's-complement (-64..+63) in TX-power-index steps
 * (vendor: "each tx_pwr_ofst step will be 1dB" on the 22C class — the on-air
 * slope is bench-pinned via cfg.tuning.txpkt_step_qdb). Both chips' BB init
 * tables write 0x1E70 = 0x00001000, i.e. all banks disabled — which is why
 * the descriptor field measured inert before the banks were programmed.
 *
 * devourer's policy: type 0 is the permanent 0 dB baseline (the per-STA RAM
 * for macid 1 is cleared once, defensively, at feature enable and never
 * programmed — so every pre-existing descriptor, which carries 0 in the
 * field, keeps today's power). The two GLOBAL banks (types 2/3) carry the
 * active offsets: both live in one dword, so programming both is a single
 * masked 0x1e70[31:16] write. This planner is the pure allocation policy —
 * no I/O, unit-tested in tests/txpkt_bank_selftest.cpp. */

namespace jaguar3 {

class TxPktPwrBankPlanner {
public:
  struct Plan {
    uint8_t type;    /* TXPWR_OFSET_TYPE to put in the descriptor (0/2/3) */
    bool program;    /* true -> write reg16 into 0x1e70[31:16] first */
    uint16_t reg16;  /* the [31:16] image after this request */
  };

  /* Clamp a power-index offset request to the 7-bit two's-complement range. */
  static int clamp_idx(int idx) {
    return idx < -64 ? -64 : (idx > 63 ? 63 : idx);
  }

  /* Map a requested offset (power-index steps) onto a bank. 0 -> type 0
   * (baseline, never a write). A value already held by an enabled bank hits
   * it (no write, refreshes recency). Otherwise program a free bank, or
   * evict the least-recently-used one. */
  Plan request(int idx) {
    idx = clamp_idx(idx);
    if (idx == 0)
      return {0, false, reg16()};
    for (int b = 0; b < 2; ++b) {
      if (_b[b].en && _b[b].idx == idx) {
        _b[b].stamp = ++_tick;
        return {static_cast<uint8_t>(2 + b), false, reg16()};
      }
    }
    int victim = !_b[0].en ? 0 : (!_b[1].en ? 1 : (_b[0].stamp <= _b[1].stamp ? 0 : 1));
    _b[victim].en = true;
    _b[victim].idx = static_cast<int8_t>(idx);
    _b[victim].stamp = ++_tick;
    return {static_cast<uint8_t>(2 + victim), true, reg16()};
  }

  /* The 0x1e70[31:16] image for the current bank state:
   * bank2 (type 2) -> bits [23:16] = en<<7 | idx&0x7f (register [23] | [22:16])
   * bank3 (type 3) -> bits [31:24] = en<<7 | idx&0x7f (register [31] | [30:24]) */
  uint16_t reg16() const {
    auto byte = [](const Bank &b) -> uint16_t {
      return b.en ? static_cast<uint16_t>(0x80 | (static_cast<uint8_t>(b.idx) & 0x7f))
                  : 0;
    };
    return static_cast<uint16_t>(byte(_b[1]) << 8) | byte(_b[0]);
  }

  bool active() const { return _b[0].en || _b[1].en; }

  /* Forget both banks (bring-up reset: the BB init table rewrote 0x1e70). */
  void reset() { _b[0] = {}; _b[1] = {}; _tick = 0; }

private:
  struct Bank {
    bool en = false;
    int8_t idx = 0;
    uint32_t stamp = 0;
  };
  Bank _b[2]; /* [0] = type 2, [1] = type 3 */
  uint32_t _tick = 0;
};

} /* namespace jaguar3 */

#endif /* DEVOURER_JAGUAR3_TXPKT_PWR_BANKS_H */
