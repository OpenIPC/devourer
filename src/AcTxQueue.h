#ifndef DEVOURER_AC_TX_QUEUE_H
#define DEVOURER_AC_TX_QUEUE_H

#include <algorithm>
#include <cstdint>

#include "DeviceConfig.h"
#include "RtlAdapter.h"

namespace devourer {

/* The access-category (AC) queue registers behind DeviceConfig::Tx::
 * ac_lifetime_us and be_fast_retry (with data_on_be, which moves DATA onto
 * an AC queue). The addresses are the halmac 88xx ones, the same on Jaguar1
 * (vendor hal_com_reg.h REG_LIFETIME_CTRL 0x0426, REG_PKT_VO_VI/BE_BK_
 * LIFE_TIME 0x04C0/0x04C2), Jaguar2 and Jaguar3. Applied on the parts
 * measured: the RTL8812A (lifetime only), the RTL8822B, RTL8822C,
 * RTL8822E and RTL8733B (same addresses).
 * Pure arithmetic plus the apply, pinned headless against a fake transport
 * by tests/ac_tx_queue_selftest.cpp. */

/* True for an 802.11 DATA frame (Frame Control type bits = 2), given the
 * first Frame Control byte: the frames data_on_be moves. Same predicate as
 * jaguar3::dot11_is_data, kept here so Jaguar1 needs no other generation's
 * header. */
constexpr bool dot11_is_data(uint8_t fc0) { return ((fc0 >> 2) & 0x3) == 0x2; }

inline constexpr uint16_t kRegEdcaBeParam = 0x0508;
/* REG_LIFETIME_EN: bits 0..3 enable the packet lifetime of the VO, VI, BE
 * and BK queues. The other bits belong to other features (bit 4
 * EN_CTRL_RTYBIT, bit 5 the BA parser) and are preserved. The MGMT/HIGH
 * queues have no lifetime here. */
inline constexpr uint16_t kRegLifetimeEn = 0x0426;
inline constexpr uint8_t kLifetimeEnAc = 0x0f;
/* REG_PKT_LIFE_TIME: [15:0] the VO/VI lifetime, [31:16] the BE/BK one. */
inline constexpr uint16_t kRegPktLifeTime = 0x04c0;
/* Lifetime unit: 256 us, measured on the RTL8812A (a peer that never ACKs:
 * 20000 us gave a delivered age max of 21.0 ms, 68000 us gave 68.8-71 ms). The
 * vendor header's HAL92C_MSDU_LIFE_TIME_UNIT says 128, which would give half;
 * halmac converts with `us >> 8` on Jaguar2 and Jaguar3 too, and 30000 us
 * gave a p99 delivered age of 31.3 ms on an RTL8812BU. */
inline constexpr unsigned kLifetimeUnitUs = 256;

/* An EDCA parameter word ([7:0] AIFS, [11:8] ECWmin, [15:12] ECWmax,
 * [26:16] TXOP limit in 32 us units) with the TXOP limit cleared and ECWmax
 * pulled down to ECWmin; AIFS and ECWmin are kept. Retries of an
 * ACK-requested frame then re-air after one AIFS plus a CWmin backoff each,
 * instead of holding a TXOP and doubling the window up to CWmax. */
constexpr uint32_t edca_fast_retry(uint32_t edca) {
  return (edca & 0x00000fffu) | ((edca & 0x00000f00u) << 4);
}

/* REG_PKT_LIFE_TIME for a lifetime in microseconds, the same value in both
 * halves: rounded up to the unit, clamped to the 16-bit field (~16.8 s).
 * 0 us stays 0 (callers do not enable a zero lifetime). */
constexpr uint32_t pkt_life_time_reg(unsigned us) {
  const uint32_t units = static_cast<uint32_t>(std::min<uint64_t>(
      0xffffu, (uint64_t{us} + kLifetimeUnitUs - 1) / kLifetimeUnitUs));
  return (units << 16) | units;
}

/* Any of the AC-queue knobs set (data_on_be, be_fast_retry, ac_lifetime_us):
 * the backends warn once when the adapter does not apply them. */
constexpr bool ac_tx_queue_requested(const DeviceConfig::Tx &tx) {
  return tx.data_on_be || tx.be_fast_retry || tx.ac_lifetime_us > 0;
}

/* Programs be_fast_retry and ac_lifetime_us. Called once per bring-up,
 * after the backend's MAC/EDCA init, before a configured SetAmpduMode is
 * re-applied (on the RTL8733B the A-MPDU mode then saves and restores the
 * value set here; Jaguar2/3 SetAmpduMode does not touch EDCA) and before
 * the debug register replay (which keeps the last word).
 * Neither set: no register access. Returns false if a register write failed;
 * the backends then fail the bring-up, as for a configured ACK responder.
 * A failed write stops there: in particular the AC queues are never enabled
 * on a stale or half-written lifetime. */
inline bool apply_ac_tx_queue(RtlAdapter &dev, const DeviceConfig::Tx &tx) {
  if (tx.be_fast_retry &&
      !dev.rtw_write32(kRegEdcaBeParam,
                       edca_fast_retry(dev.rtw_read32(kRegEdcaBeParam))))
    return false;
  if (tx.ac_lifetime_us == 0)
    return true;
  if (!dev.rtw_write32(kRegPktLifeTime, pkt_life_time_reg(tx.ac_lifetime_us)))
    return false;
  return dev.rtw_write8(kRegLifetimeEn,
                        static_cast<uint8_t>(dev.rtw_read8(kRegLifetimeEn) |
                                             kLifetimeEnAc));
}

} // namespace devourer

#endif /* DEVOURER_AC_TX_QUEUE_H */
