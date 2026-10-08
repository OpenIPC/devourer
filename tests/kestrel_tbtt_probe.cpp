// kestrel_tbtt_probe.cpp — bench probe for the AX (RTL8852CE) beacon TBTT vs
// port-TSF semantics, observed on air. Ground truth for porting PinBeaconTbtt
// to Kestrel: where does the TBTT go when the port TSF is stepped (direct
// write, TSF_SYNC from another port), and what re-latches it?
//
// Card A (BCN_BDF) runs a hardware beacon; card B (OBS_BDF) receives it on the
// same channel. The MAC stamps each beacon's timestamp field with A's live
// port-0 TSF at egress (Packet::TxEgressTsf), so `egress % period` is where on
// A's own TSF grid the beacon aired — no cross-clock mapping. A's FREERUN
// counter (0xC5C0, untouched by port-TSF writes) gives the TSF-continuity
// check: (TSF - FREERUN) only moves when the TSF is stepped.
//
// Raw register steps use a second mapping of A's BAR2 via sysfs (root, bench
// only); the radio itself runs over vfio as usual.
//
// STEPS (comma-separated, run in order):
//   wait:S          observe S seconds; print beacon phase stats for the window
//   tsfadd:US       write port-0 TSF = TSF + US (LOW then HIGH)
//   bcntx           PORT_CFG_P0 BCNTX_EN off -> on
//   portfn          PORT_CFG_P0 PORT_FUNC_EN off -> on
//   sync:SRC:US     TSF_SYNC: port0 = port SRC + US, sync-now once
//   pin:US          IRadio::PinBeaconTbtt(US)
//   regs            dump P0..P4 TSF and FREERUN
//
// Build: the kestrel_tbtt_probe CMake target (DEVOURER_PCIE=ON with
//        DEVOURER_KESTREL_8852C).
// Run: sudo BCN_BDF=0000:05:00.0 OBS_BDF=0000:09:00.0 DEVOURER_CHANNEL=36 \
//        STEPS=wait:3,tsfadd:30000,wait:3 build/kestrel_tbtt_probe
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "PcieTransport.h"
#include "RxPacket.h"
#include "SelectedChannel.h"
#include "WiFiDriver.h"
#include "logger.h"

namespace {

constexpr uint32_t PORT_CFG_P0 = 0xC400, BCNTX_EN = 1u << 12, PORT_FUNC_EN = 1u << 2;
constexpr uint32_t TSF_LO[5] = {0xC438, 0xC478, 0xC4B8, 0xC4F8, 0xC538};
constexpr uint32_t PORT0_TSF_SYNC = 0xC2A0;
constexpr uint32_t FREERUN_LO = 0xC5C0, FREERUN_HI = 0xC5C4;
const uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x01};

struct Bar {
  volatile uint8_t *p = nullptr;
  bool open(const std::string &bdf) {
    int fd = ::open(("/sys/bus/pci/devices/" + bdf + "/resource2").c_str(), O_RDWR | O_SYNC);
    if (fd < 0) return false;
    void *m = mmap(nullptr, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ::close(fd);
    if (m == MAP_FAILED) return false;
    p = static_cast<volatile uint8_t *>(m);
    return true;
  }
  uint32_t r(uint32_t a) const { return *reinterpret_cast<volatile uint32_t *>(p + a); }
  void w(uint32_t a, uint32_t v) const { *reinterpret_cast<volatile uint32_t *>(p + a) = v; }
  uint64_t r64(uint32_t lo) const {
    uint32_t h = r(lo + 4), l = r(lo);
    if (r(lo + 4) != h) { h = r(lo + 4); l = r(lo); }
    return (uint64_t)h << 32 | l;
  }
  uint64_t freerun() const {
    uint32_t h = r(FREERUN_HI), l = r(FREERUN_LO);
    if (r(FREERUN_HI) != h) { h = r(FREERUN_HI); l = r(FREERUN_LO); }
    return (uint64_t)h << 32 | l;
  }
};

std::mutex g_mu;
std::vector<int64_t> g_phase;  // egress % period, this window
uint64_t g_last_egress = 0;

int64_t median(std::vector<int64_t> v) {
  if (v.empty()) return -1;
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

}  // namespace

int main() {
  const char *a_bdf = std::getenv("BCN_BDF"), *b_bdf = std::getenv("OBS_BDF");
  if (!a_bdf || !b_bdf) { fprintf(stderr, "set BCN_BDF and OBS_BDF\n"); return 2; }
  uint8_t ch = 36; if (const char *c = std::getenv("DEVOURER_CHANNEL")) ch = atoi(c);
  int interval_tu = 100; if (const char *i = std::getenv("BCN_TU")) interval_tu = atoi(i);
  const int64_t period = (int64_t)interval_tu * 1024;
  std::string steps = std::getenv("STEPS") ? std::getenv("STEPS") : "wait:3";

  auto logger = std::make_shared<Logger>();
  WiFiDriver wifi(logger);
  auto ta = devourer::PcieTransport::Open(a_bdf, logger);
  auto tb = devourer::PcieTransport::Open(b_bdf, logger);
  if (!ta || !tb) { fprintf(stderr, "pcie open failed\n"); return 1; }
  auto A = wifi.CreateRadioPcie(std::move(ta));
  auto B = wifi.CreateRadioPcie(std::move(tb));
  if (!A || !B) { fprintf(stderr, "create failed\n"); return 1; }
  Bar bar;
  if (!bar.open(a_bdf)) { perror("sysfs resource2"); return 1; }

  SelectedChannel sc{ch, 0, CHANNEL_WIDTH_20};
  std::atomic<bool> rx_done{false};
  std::thread rx([&] {
    B->Init([&](const Packet &p) {
      if (p.RxAtrib.crc_err) return;  // a damaged timestamp would skew the phase
      if (p.Data.size() < 36 || p.Data[0] != 0x80) return;  // beacons only
      if (std::memcmp(p.Data.data() + 10, kSa, 6) != 0) return;
      auto e = p.TxEgressTsf();
      if (!e) return;
      std::lock_guard<std::mutex> lk(g_mu);
      g_phase.push_back((int64_t)(*e % (uint64_t)period));
      g_last_egress = *e;
    }, sc);
    rx_done = true;
  });
  // StartRxLoop clears the stop flag on entry, so a stop that lands while B is
  // still in bring-up is lost: repeat it until the RX thread has really ended,
  // then join. Every exit after the thread starts goes through here.
  auto stop_rx = [&] {
    while (!rx_done) {
      B->StopRxLoop();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    rx.join();
  };
  std::this_thread::sleep_for(std::chrono::seconds(4));  // let B come up

  A->InitWrite(sc);
  std::vector<uint8_t> bcn = {
      0x80,0x00,0x00,0x00, 0xff,0xff,0xff,0xff,0xff,0xff,
      kSa[0],kSa[1],kSa[2],kSa[3],kSa[4],kSa[5], kSa[0],kSa[1],kSa[2],kSa[3],kSa[4],kSa[5], 0x00,0x00,
      0,0,0,0,0,0,0,0,
      (uint8_t)(interval_tu & 0xff), (uint8_t)(interval_tu >> 8), 0x00,0x00,
      0x00,0x05,'T','B','P','R','B', 0x01,0x01,0x8c};
  if (!A->StartBeacon(bcn.data(), bcn.size(), interval_tu)) {
    fprintf(stderr, "StartBeacon failed\n");
    stop_rx();
    A->Stop();
    B->Stop();
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));
  { std::lock_guard<std::mutex> lk(g_mu); g_phase.clear(); }

  auto tsf_minus_fr = [&] { return (int64_t)(bar.r64(TSF_LO[0]) - bar.freerun()); };
  std::stringstream ss(steps);
  std::string st;
  int idx = 0;
  while (std::getline(ss, st, ',')) {
    ++idx;
    auto arg = [&](int k) {
      size_t pos = 0;
      for (int i = 0; i < k; ++i) pos = st.find(':', pos) + 1;
      return std::strtoll(st.c_str() + pos, nullptr, 0);
    };
    const int64_t d0 = tsf_minus_fr();
    if (st.rfind("wait:", 0) == 0) {
      { std::lock_guard<std::mutex> lk(g_mu); g_phase.clear(); }
      std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(arg(1) * 1000)));
      std::vector<int64_t> v;
      uint64_t last;
      { std::lock_guard<std::mutex> lk(g_mu); v = g_phase; last = g_last_egress; }
      int64_t lo = v.empty() ? -1 : *std::min_element(v.begin(), v.end());
      int64_t hi = v.empty() ? -1 : *std::max_element(v.begin(), v.end());
      printf("{\"ev\":\"tbtt.win\",\"step\":%d,\"n\":%zu,\"phase_med\":%lld,"
             "\"phase_min\":%lld,\"phase_max\":%lld,\"tsf_minus_freerun\":%lld,"
             "\"last_egress\":%llu}\n",
             idx, v.size(), (long long)median(v), (long long)lo, (long long)hi,
             (long long)tsf_minus_fr(), (unsigned long long)last);
    } else if (st.rfind("tsfadd:", 0) == 0) {
      uint64_t t = bar.r64(TSF_LO[0]) + (uint64_t)arg(1);
      bar.w(TSF_LO[0], (uint32_t)t);
      bar.w(TSF_LO[0] + 4, (uint32_t)(t >> 32));
      printf("{\"ev\":\"tbtt.tsfadd\",\"step\":%d,\"us\":%lld,\"dTSF_vs_freerun\":%lld}\n",
             idx, arg(1), (long long)(tsf_minus_fr() - d0));
    } else if (st == "bcntx" || st == "portfn") {
      const uint32_t bit = st == "bcntx" ? BCNTX_EN : PORT_FUNC_EN;
      const uint32_t v = bar.r(PORT_CFG_P0);
      bar.w(PORT_CFG_P0, v & ~bit);
      bar.w(PORT_CFG_P0, v | bit);
      printf("{\"ev\":\"tbtt.toggle\",\"step\":%d,\"what\":\"%s\",\"dTSF_vs_freerun\":%lld}\n",
             idx, st.c_str(), (long long)(tsf_minus_fr() - d0));
    } else if (st.rfind("sync:", 0) == 0) {
      const uint32_t src = (uint32_t)arg(1);
      const int64_t off = arg(2);
      uint32_t mag = (uint32_t)(off < 0 ? -off : off) & 0x3FFFF;
      if (off < 0) mag |= 1u << 18;
      uint32_t v = (src & 7u) << 24 | mag;
      bar.w(PORT0_TSF_SYNC, v);
      bar.w(PORT0_TSF_SYNC, v | (1u << 30));  // SYNC_NOW_P
      bar.w(PORT0_TSF_SYNC, v);
      printf("{\"ev\":\"tbtt.sync\",\"step\":%d,\"src\":%u,\"us\":%lld,\"dTSF_vs_freerun\":%lld}\n",
             idx, src, (long long)off, (long long)(tsf_minus_fr() - d0));
    } else if (st.rfind("pin:", 0) == 0) {
      const int32_t applied = A->PinBeaconTbtt((int32_t)arg(1));
      printf("{\"ev\":\"tbtt.pin\",\"step\":%d,\"req\":%lld,\"applied\":%d,\"dTSF_vs_freerun\":%lld}\n",
             idx, arg(1), applied, (long long)(tsf_minus_fr() - d0));
    } else if (st == "regs") {
      // One write per event line: the RX thread also emits events on stdout.
      char line[512];
      int len = snprintf(line, sizeof(line),
                         "{\"ev\":\"tbtt.regs\",\"step\":%d,\"freerun\":%llu", idx,
                         (unsigned long long)bar.freerun());
      for (int p = 0; p < 5; ++p)
        len += snprintf(line + len, sizeof(line) - len, ",\"tsf_p%d\":%llu", p,
                        (unsigned long long)bar.r64(TSF_LO[p]));
      snprintf(line + len, sizeof(line) - len, ",\"port_cfg_p0\":\"0x%08x\"}\n",
               bar.r(PORT_CFG_P0));
      fputs(line, stdout);
    } else {
      fprintf(stderr, "unknown step '%s'\n", st.c_str());
    }
    fflush(stdout);
  }
  stop_rx();
  A->Stop();
  B->Stop();
  return 0;
}
