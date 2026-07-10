// pcie_txegress_tx.cpp — host-pushed DATA-frame TX over the PCIe transport, each
// frame carrying the transmitter's ReadTsf() submit timestamp in a tdma TD tag.
// Paired with tests/txegress_witness on a co-located receiver, it measures the
// submit-to-air jitter over PCIe (vs the ~117 µs measured over USB), isolating
// how much the faster/more-deterministic transport tightens the open-loop floor.
//
// It mirrors the USB timesync software master (ReadTsf *before* send, so the
// number is comparable), but opens the 8821CE through PcieTransport / vfio.
//
// Build (on a DEVOURER_PCIE=ON tree):
//   g++ -std=c++20 -O2 -Isrc -Iexamples/tdma tests/pcie_txegress_tx.cpp \
//     build-pcie/libdevourer.a $(pkg-config --cflags --libs libusb-1.0) \
//     -lpthread -o build-pcie/pcie_txegress_tx
// Run: sudo DEVOURER_PCIE_BDF=0000:01:00.0 DEVOURER_CHANNEL=36 \
//        build-pcie/pcie_txegress_tx [secs]
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>

#include "PcieTransport.h"
#include "RadiotapBuilder.h"
#include "SelectedChannel.h"
#include "WiFiDriver.h"
#include "logger.h"
#include "tdma.h"

int main(int argc, char **argv) {
  const char *bdf = std::getenv("DEVOURER_PCIE_BDF");
  if (!bdf) { fprintf(stderr, "set DEVOURER_PCIE_BDF (e.g. 0000:01:00.0)\n"); return 2; }
  uint8_t ch = 36;
  if (const char *c = std::getenv("DEVOURER_CHANNEL")) ch = (uint8_t)atoi(c);
  int secs = argc > 1 ? atoi(argv[1]) : 45;
  int gap_us = 8000;
  if (const char *g = std::getenv("GAP_US")) gap_us = atoi(g);

  auto logger = std::make_shared<Logger>();
  auto transport = devourer::PcieTransport::Open(bdf, logger);
  if (!transport) { fprintf(stderr, "pcie open failed for %s (vfio-bound?)\n", bdf); return 1; }
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevicePcie(std::move(transport));
  if (!dev) { fprintf(stderr, "CreateRtlDevicePcie failed\n"); return 1; }

  dev->InitWrite(SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
  dev->SetCcaMode(true);   // disable EDCCA — suppress CSMA backoff so the residual
                           // reflects the transport + MAC-pipeline floor, not channel load
  std::this_thread::sleep_for(std::chrono::seconds(2));

  const auto rt = devourer::build_stream_radiotap(
      devourer::parse_tx_mode_str("6M"));                  // legacy 6M, widely heard
  uint32_t seq = 0;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(secs);
  while (std::chrono::steady_clock::now() < deadline) {
    uint64_t tsf = dev->ReadTsf();                          // submit-time stamp (TX clock)
    auto f = tdma::build_frame(rt, tdma::Class::Marker, seq++, 0, tsf);
    dev->send_packet(f.data(), f.size());
    std::this_thread::sleep_for(std::chrono::microseconds(gap_us));
  }
  fprintf(stderr, "pcie_txegress_tx: %u frames on ch%d\n", seq, ch);
  return 0;
}
