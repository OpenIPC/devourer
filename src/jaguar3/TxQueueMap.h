/* Jaguar3 (8822C/8822E) TX queue -> bulk-OUT endpoint map. Pure, header-only,
 * so tests/txqueue_selftest.cpp can pin it without hardware.
 *
 * On a Realtek USB part the ENDPOINT selects the hardware TX queue. halmac's
 * get_usb_bulkout_id_88xx() reads QSEL out of the descriptor, looks the access
 * category up in the priority-queue map the driver programmed into
 * REG_TXDMA_PQ_MAP, and turns the resulting DMA mapping into a bulk-out index:
 * HIGH->0, NORMAL->1, LOW->2, EXTRA->3. With the enum values (EXTRA=0, LOW=1,
 * NORMAL=2, HIGH=3) that is `3 - mapping`.
 *
 * init_trx_cfg programs the vendor's 3-bulk-OUT map, whatever the endpoint
 * count. So per-queue routing is only right with >= kPerQueueMinEps bulk-OUT
 * endpoints: a 4-endpoint part uses endpoints 0..2 of the same map (EXTRA is
 * never chosen), and below the threshold there is no LOW/NORMAL endpoint for
 * that map to name. Below it every frame rides endpoint 0, and plain data
 * keeps QSEL 0x12 (MGT -> HIGH), so its descriptor and endpoint agree.
 * SetAmpduMode is the exception: its TID (0..7) is still stamped on data
 * frames, because aggregation only forms on a data queue (AmpduMode.h), so
 * on a 1- or 2-endpoint part an A-MPDU data frame carries a data QSEL down
 * endpoint 0 - a descriptor/endpoint mismatch on that shape, deliberately
 * left as it is. The DEVOURER_TX_QSEL debug override can
 * produce a mismatch on any shape. Neither the 1/2- nor the 4-endpoint shape
 * has been measured. */
#ifndef JAGUAR3_TX_QUEUE_MAP_H
#define JAGUAR3_TX_QUEUE_MAP_H

#include <cstddef>
#include <cstdint>

namespace jaguar3 {

inline constexpr size_t kPerQueueMinEps = 3;

/* True when data frames leave the HIGH queue (see above). build_tx_block and
 * peek_tx_qsel both key their data-QSEL write on this. */
constexpr bool per_queue_routing(size_t n_eps) {
  return n_eps >= kPerQueueMinEps;
}

/* Bulk-OUT endpoint INDEX (0-based, descriptor order) for a descriptor QSEL.
 * The map is the one init_trx_cfg writes: VO/VI -> NQ, BE/BK -> LQ,
 * MG/HI/BCN/CMD -> HQ. TID to access category is 802.11-2016 Table 9-1:
 * 0,3 = BE; 1,2 = BK; 4,5 = VI; 6,7 = VO. Anything else (0x10 BEACON,
 * 0x11 HIGH, 0x12 MGT, 0x13 CMD, unexpected values) goes to HIGH, where the
 * beacon must go. */
constexpr uint8_t bulkout_id_for_qsel(uint8_t qsel, size_t n_eps) {
  if (!per_queue_routing(n_eps))
    return 0;
  uint32_t mapping = 3; /* HIGH */
  switch (qsel) {
  case 0: case 1: case 2: case 3:
    mapping = 1; /* LOW */
    break;
  case 4: case 5: case 6: case 7:
    mapping = 2; /* NORMAL */
    break;
  default:
    break;
  }
  return static_cast<uint8_t>(3u - mapping);
}

/* True for an 802.11 DATA frame (Frame Control type bits = 2) - the ONE
 * frame-type predicate build_tx_block and peek_tx_qsel share. */
constexpr bool dot11_is_data(uint8_t fc0) { return ((fc0 >> 2) & 0x3) == 0x2; }

/* The final descriptor QSEL for a frame, in build_tx_block's order:
 *   1. the builder's 0x12 (MGT);
 *   2. data -> 0 (BE, LOW) when per_queue_routing(n_eps);
 *   3. SetAmpduMode's TID - DATA frames only, as the AmpduMode contract says
 *      (aggregation forms on a data queue; management keeps its queue);
 *   4. the DEVOURER_TX_QSEL debug override (debug_qsel >= 0), on EVERY frame
 *      - it is a raw register-level knob.
 * Both the peek and the build call this, so they cannot disagree. */
constexpr uint8_t tx_qsel(bool is_data, size_t n_eps, bool ampdu_enabled,
                          uint8_t ampdu_tid, int debug_qsel) {
  uint8_t q = 0x12;
  if (is_data && per_queue_routing(n_eps))
    q = 0x00;
  if (is_data && ampdu_enabled)
    q = ampdu_tid;
  if (debug_qsel >= 0)
    q = static_cast<uint8_t>(debug_qsel);
  return static_cast<uint8_t>(q & 0x1f);
}

} // namespace jaguar3

#endif /* JAGUAR3_TX_QUEUE_MAP_H */
