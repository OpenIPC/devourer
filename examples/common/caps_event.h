/* Shared `adapter.caps` machine-event emitter for the demos.
 *
 * One place that serializes IRtlDevice::GetAdapterCaps() (src/AdapterCaps.h) to
 * the JSONL event plane so rxdemo / txdemo / doctor / txpower all emit the same
 * schema — a dependent app or test script consumes one event instead of calling
 * the C++ API. Emit it right after CreateRtlDevice (the caps are static and
 * resolved at construction — no bring-up needed). Mirrors the txpwr.caps
 * emission in examples/txpower/main.cpp: booleans as 0/1, chip_id as a hex
 * string, bandwidths + frequency spans as arrays (absent band -> null). */
#ifndef DEVOURER_CAPS_EVENT_H
#define DEVOURER_CAPS_EVENT_H

#include "AdapterCaps.h"
#include "Event.h"
#include "IRtlDevice.h"

namespace devourer {

inline void emit_adapter_caps(EventSink &sink, IRtlDevice *dev) {
  const AdapterCaps c = dev->GetAdapterCaps();

  /* Supported channel widths as an MHz int array (kBw* -> MHz). */
  int bw[6];
  int nbw = 0;
  if (c.bw_mask & kBw5) bw[nbw++] = 5;
  if (c.bw_mask & kBw10) bw[nbw++] = 10;
  if (c.bw_mask & kBw20) bw[nbw++] = 20;
  if (c.bw_mask & kBw40) bw[nbw++] = 40;
  if (c.bw_mask & kBw80) bw[nbw++] = 80;
  if (c.bw_mask & kBw160) bw[nbw++] = 160;

  Ev ev(sink, "adapter.caps");
  ev.f("supported", c.supported ? 1 : 0)
      .f("chip", c.chip_name)
      .f("names", c.marketing_names)
      .hexf("chip_id", c.chip_id, 2)
      .f("gen", generation_name(c.generation))
      .f("variant", c.variant)
      .f("transport", c.transport)
      .f("tx_chains", c.tx_chains)
      .f("rx_chains", c.rx_chains)
      .f("n_ss", c.tx.n_ss)
      .f("stbc", c.tx.stbc_ok ? 1 : 0)
      .f("ldpc", c.tx.ldpc_ok ? 1 : 0)
      .f("sgi", c.tx.sgi_ok ? 1 : 0)
      .f("bw_max", c.tx.bw_max_mhz)
      .arr("bw", bw, static_cast<size_t>(nbw))
      .f("txpwr_max", c.txpwr.index_max)
      .f("txpwr_step_qdb", c.txpwr.step_qdb)
      .f("txpwr_step_measured", c.txpwr.step_measured ? 1 : 0)
      .f("txpwr_min_qdb", c.txpwr.offset_min_qdb)
      .f("txpwr_max_qdb", c.txpwr.offset_max_qdb);

  auto band = [&ev](const char *k, const BandRange &b) {
    if (b.valid) {
      const int v[2] = {b.min_mhz, b.max_mhz};
      ev.arr(k, v, 2);
    } else {
      ev.f(k, nullptr);
    }
  };
  band("tune_2g4", c.tune_2g4);
  band("tune_5g", c.tune_5g);
  band("char_2g4", c.characterized_2g4);
  band("char_5g", c.characterized_5g);

  ev.f("per_pkt_txpwr", c.per_packet_txpower ? 1 : 0)
      .f("narrowband", c.narrowband_ok ? 1 : 0)
      .f("fastretune", c.fastretune_ok ? 1 : 0)
      .f("per_chain_rssi", c.per_chain_rssi ? 1 : 0)
      .f("hw_rx_tsf", c.hw_rx_timestamp ? 1 : 0)
      .f("hw_beacon_txtsf", c.hw_beacon_txtsf ? 1 : 0)
      .f("xtal_cap_max", c.xtal_cap_max)
      .f("xtal_cap_default", c.xtal_cap_default);
}

} // namespace devourer

#endif /* DEVOURER_CAPS_EVENT_H */
