// cell_caps.h — the single source of truth for which AdapterCaps a radio
// needs to run an examples/cell node (a DU or a UE): write its TSF, have the
// beacon TBTT follow that TSF, control the beacon-queue backoff, and stamp the
// live TSF into the beacon at TX. Both examples/cell (caps_ok) and the
// examples/cellcaps probe use this, so the probe's verdict cannot drift from
// the gate the cell node actually applies.
#ifndef DEVOURER_EXAMPLES_CELL_CAPS_H
#define DEVOURER_EXAMPLES_CELL_CAPS_H

#include "AdapterCaps.h"

namespace devourer {

inline bool cell_node_caps_ok(const AdapterCaps &c) {
  return c.tsf_write_ok && c.tbtt_follows_tsf && c.beacon_backoff_ctl && c.hw_beacon_txtsf;
}

}  // namespace devourer

#endif  // DEVOURER_EXAMPLES_CELL_CAPS_H
