// beacon_phase_observer.cpp — on-air check that several PTP-disciplined nodes
// hold their beacons on ONE timebase. One radio receives every node's beacon
// (SA 57:42:75:05:d6:<node>) and stamps each with its own RX timestamp
// (RxAtrib.tsfl), so all nodes are compared on a single clock. For each node k
// the spacing to node 0's beacon in the same beacon period is reported — if the
// network is in sync it stays at the configured phase difference, and its
// spread is the network-wide alignment error as the air sees it (PTP fabric +
// each node's TBTT loop + channel-access jitter). Independent of the loops'
// own phase estimates.
//
// Pair with tests/pcie_ptp_tsf_discipline.cpp on each node (NODE_ID=k, a
// distinct PHASE_US per node so the beacons don't collide on one channel).
//
// Build: the beacon_phase_observer CMake target (DEVOURER_PCIE=ON).
// Run: sudo DEVOURER_PCIE_BDF=0000:09:00.0 DEVOURER_CHANNEL=36 BCN_TU=100 \
//        build/beacon_phase_observer [secs] [nodes=3]
// Output: one summary line per node k>0 (median/jitter/p95/max of the spacing
// to node 0) plus beacon counts per node.
//
// Two spacings are reported. "raw" is RX(k) - RX(0) on the receiver clock:
// what the air sees, including each beacon engine's TX-start jitter (on the
// RTL8852CE a beacon leaves 22-48 us after its TBTT, varying per beacon).
// "tbtt" subtracts each beacon's own egress delay — its MPDU timestamp is the
// sender's TSF at egress, and the TBTT sits at TSF % period == 0, so
// egress % period is how late that beacon left — leaving the TBTT-to-TBTT
// alignment, i.e. the network sync error, plus receiver noise.
//
// DUMP=<path> also writes every matched beacon as "node rx_tsfl egress_phase"
// (one line each, arrival order) for offline analysis, plus one
// "-1 <second_start_tsfl> <count>" line per second with the number of OTHER
// frames heard (foreign traffic on the channel), to correlate with beacons
// that left late.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "PcieTransport.h"
#include "RxPacket.h"
#include "SelectedChannel.h"
#include "WiFiDriver.h"
#include "logger.h"

// Integer from an env var or argv string, in [lo, hi]; anything else is a
// configuration error (BCN_TU=0 would make the period a zero divisor).
static long parse_int(const char *what, const char *v, long d, long lo, long hi) {
  if (!v) return d;
  char *end = nullptr;
  const long x = std::strtol(v, &end, 0);
  if (end == v || *end != '\0' || x < lo || x > hi) {
    fprintf(stderr, "invalid %s \"%s\" (want an integer in %ld..%ld)\n", what, v, lo, hi);
    std::exit(2);
  }
  return x;
}

// One machine event = one complete line, one write, flushed (the repo's
// logging contract; a piped harness must not see half a record or wait on a
// buffer).
static void emit(const std::string &line) {
  fputs(line.c_str(), stdout);
  fputc('\n', stdout);
  fflush(stdout);
}

int main(int argc, char **argv) {
  const char *bdf = std::getenv("DEVOURER_PCIE_BDF");
  if (!bdf) { fprintf(stderr, "set DEVOURER_PCIE_BDF\n"); return 2; }
  const uint8_t ch = (uint8_t)parse_int("DEVOURER_CHANNEL", std::getenv("DEVOURER_CHANNEL"), 36, 1, 196);
  const int tu = (int)parse_int("BCN_TU", std::getenv("BCN_TU"), 100, 1, 65535);
  const int64_t period = (int64_t)tu * 1024;
  const int secs = (int)parse_int("secs", argc > 1 ? argv[1] : nullptr, 120, 1, 86400);
  const int nodes = (int)parse_int("nodes", argc > 2 ? argv[2] : nullptr, 3, 1, 256);
  const uint8_t prefix[5] = {0x57, 0x42, 0x75, 0x05, 0xd6};

  auto logger = std::make_shared<Logger>();
  auto t = devourer::PcieTransport::Open(bdf, logger);
  if (!t) { fprintf(stderr, "pcie open failed\n"); return 1; }
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRadioPcie(std::move(t));
  if (!dev) { fprintf(stderr, "create failed\n"); return 1; }

  struct Rx { int node; uint32_t tsfl; int64_t egress; };  // egress = TSF % period
  std::mutex mu;
  std::vector<Rx> rx;
  std::map<uint32_t, long> foreign;  // rx_tsfl / 1e6 -> frames that aren't ours
  std::atomic<bool> done{false};
  std::thread th([&] {
    dev->Init([&](const Packet &p) {
      if (p.RxAtrib.crc_err) return;
      const bool ours = p.Data.size() >= 24 && p.Data[0] == 0x80 &&
                        std::memcmp(p.Data.data() + 10, prefix, 5) == 0 &&
                        p.Data[15] < nodes;
      if (!ours) {
        std::lock_guard<std::mutex> lk(mu);
        foreign[p.RxAtrib.tsfl / 1000000u]++;
        return;
      }
      const int node = p.Data[15];
      const auto eg = p.TxEgressTsf();
      std::lock_guard<std::mutex> lk(mu);
      rx.push_back({node, p.RxAtrib.tsfl, eg ? (int64_t)(*eg % (uint64_t)period) : -1});
    }, SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
    done = true;
  });
  std::this_thread::sleep_for(std::chrono::seconds(secs));
  // StartRxLoop clears the stop flag on entry: repeat until the loop is gone.
  while (!done) { dev->StopRxLoop(); std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
  th.join();

  std::vector<Rx> v;
  { std::lock_guard<std::mutex> lk(mu); v = rx; }
  std::map<int, long> count;
  for (auto &e : v) count[e.node]++;
  if (const char *dump = std::getenv("DUMP")) {
    if (FILE *f = fopen(dump, "w")) {
      for (auto &e : v) fprintf(f, "%d %u %lld\n", e.node, e.tsfl, (long long)e.egress);
      std::lock_guard<std::mutex> lk(mu);
      for (auto &kv : foreign) fprintf(f, "-1 %u %ld\n", kv.first * 1000000u, kv.second);
      fclose(f);
    }
  }
  {
    std::string line = "{\"ev\":\"bcnobs.counts\"";
    for (int k = 0; k < nodes; ++k)
      line += ",\"node" + std::to_string(k) + "\":" + std::to_string(count[k]);
    emit(line + "}");
  }

  auto report = [&](int k, const char *kind, const std::vector<double> &d) {
    char buf[256];
    if (d.empty()) {
      snprintf(buf, sizeof(buf), "{\"ev\":\"bcnobs.spacing\",\"node\":%d,\"kind\":\"%s\",\"n\":0}", k, kind);
      emit(buf);
      return;
    }
    std::vector<double> s = d;
    std::sort(s.begin(), s.end());
    const double med = s[s.size() / 2];
    std::vector<double> dev_abs;
    double ss = 0;
    for (double x : d) { dev_abs.push_back(std::fabs(x - med)); ss += (x - med) * (x - med); }
    std::sort(dev_abs.begin(), dev_abs.end());
    snprintf(buf, sizeof(buf),
             "{\"ev\":\"bcnobs.spacing\",\"node\":%d,\"kind\":\"%s\",\"n\":%zu,\"median_us\":%.1f,"
             "\"jitter_rms_us\":%.2f,\"p95_abs_dev_us\":%.1f,\"max_abs_dev_us\":%.1f}",
             k, kind, d.size(), med, std::sqrt(ss / d.size()),
             dev_abs[dev_abs.size() * 95 / 100], dev_abs.back());
    emit(buf);
  };
  // For each beacon of node k, the most recent node-0 beacon within one
  // period before it gives one sample (uint32 wrap-safe difference).
  for (int k = 1; k < nodes; ++k) {
    std::vector<double> raw, tbtt;
    const Rx *last0 = nullptr;
    for (auto &e : v) {
      if (e.node == 0) { last0 = &e; continue; }
      if (e.node != k || !last0) continue;
      const int64_t dt = (int64_t)(uint32_t)(e.tsfl - last0->tsfl);
      if (dt <= 0 || dt >= period) continue;
      raw.push_back((double)dt);
      if (e.egress >= 0 && last0->egress >= 0) {
        // Egress phases are taken mod period, so a beacon that leaves just
        // before its sender's grid point (a TSF step landing near the TBTT)
        // reads ~period instead of ~0: wrap the difference into +-period/2.
        int64_t de = e.egress - last0->egress;
        if (de > period / 2) de -= period;
        if (de < -period / 2) de += period;
        tbtt.push_back((double)(dt - de));
      }
    }
    report(k, "raw", raw);
    report(k, "tbtt", tbtt);
  }
  dev->Stop();
  return 0;
}
