/* cellcaps — open the radio named in the environment (USB via DEVOURER_VID/
 * PID, or PCIe via DEVOURER_PCIE_BDF) and print the AdapterCaps that decide
 * whether it can run a cell node (examples/cell caps_ok), as JSON. Read-only
 * beyond the normal bring-up the open path does; no beacon, no TX. */
#include <cstdio>
#include <memory>

#include "radio_open.h"

int main() {
  auto logger = std::make_shared<Logger>();
  OpenedRadio radio = open_radio_from_env(logger, 0x0bda, 0);
  if (!radio.dev) {
    fprintf(stderr, "cellcaps: open failed\n");
    return 1;
  }
  const auto c = radio.dev->GetAdapterCaps();
  const bool cell_ok = c.tsf_write_ok && c.tbtt_follows_tsf && c.beacon_backoff_ctl && c.hw_beacon_txtsf;
  printf("{\"ev\":\"cellcaps\",\"radio\":\"%s\",\"chip\":\"%s\",\"variant\":\"%s\",\"transport\":\"%s\","
         "\"tsf_write_ok\":%d,\"tbtt_follows_tsf\":%d,\"beacon_backoff_ctl\":%d,\"hw_beacon_txtsf\":%d,"
         "\"ack_responder_ok\":%d,\"hw_rx_timestamp\":%d,\"hw_injected_mgmt_txtsf\":%d,"
         "\"cell_node_ok\":%d}\n",
         radio.label.c_str(), c.chip_name, c.variant, c.transport, c.tsf_write_ok, c.tbtt_follows_tsf,
         c.beacon_backoff_ctl, c.hw_beacon_txtsf, c.ack_responder_ok, c.hw_rx_timestamp,
         c.hw_injected_mgmt_txtsf, cell_ok ? 1 : 0);
  return 0;
}
