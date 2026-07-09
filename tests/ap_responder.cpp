// ap_responder.cpp — devourer as a minimal OPEN-network AP that answers the full
// 802.11 handshake, toward the ultimate "behaves like a kernel AP": a real
// station associating. It beacons (StartBeacon, discoverable + MACID set) and,
// full-duplex, answers from its RX callback: probe-req -> probe-resp, auth-req
// (open) -> auth-resp, (re)assoc-req -> assoc-resp. Responses are BUILT in the
// callback but SENT from the main thread (send_packet from the RX event thread
// returns libusb BUSY). Management-frame timeouts are tens of ms, so the ~few-ms
// userspace RX->TX round-trip fits.
//
// STATUS (bench): FULL ASSOCIATION + DATA PLANE — a real Linux station (rtw88
// 8822cu, wlp4s0u2u4) authenticates, associates, stays "Connected to
// 02:42:75:05:d6:00", AND PINGS the AP: `ping 192.168.99.1` returns 0% loss,
// ~2 ms RTT. The AP sees AUTH/ASSOC with retry=0 (devourer HARDWARE-ACKs them —
// MACID = BSSID, set by StartBeacon) and answers the data plane below: ARP
// requests for the AP IP -> ARP replies, ICMP echo requests -> echo replies, over
// 802.11 from-DS data frames. Run with a DENSE beacon (DEVOURER_BCN_TU=25 — a fast
// channel-hopping supplicant scan misses a 100 TU beacon). The station needs a
// static IP (192.168.99.2/24; the AP answers for .1) — there is no DHCP server.
// See tests/ap_ping_demo.sh.
//
// THE FIX that unblocked it: the BSSID must be UNICAST. The canonical test SA
// 0x57.. has the I/G bit set (multicast); a station cannot unicast-auth to a
// multicast address, so rtw88 silently drops the auth before air (confirmed:
// 0x57 -> no auth on air across two station chips; 0x02 -> auth on air +
// association completes). kBssid below is 0x02.. (locally-administered unicast).
//
// The log prints each auth/assoc request with its retry bit (a no-ACK stall would
// show as repeated retries; here retry=0).
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/ap_responder.cpp \
//   examples/common/env_config.cpp build/libdevourer.a \
//   $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/ap_responder
// Run: sudo DEVOURER_PID=0xc812 DEVOURER_CHANNEL=6 DEVOURER_TX_WITH_RX=thread \
//   build/ap_responder [sec]
//   then on a kernel station: iw dev <if> connect -w devourerAP   (open network)
#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <unistd.h>
#include <libusb.h>
#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "SelectedChannel.h"
#include "TxMode.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

// BSSID MUST be UNICAST — the first octet's I/G bit (bit 0) must be 0. The
// canonical test SA 0x57... has that bit SET (multicast), which is invalid as a
// BSSID: a station cannot unicast-auth to a multicast address, so rtw88 silently
// refuses to emit the auth (bench-proven — flipping 0x57->0x02 made the station
// transmit auth). Use 0x02 (locally-administered unicast).
static const uint8_t kBssid[6] = {0x02, 0x42, 0x75, 0x05, 0xd6, 0x00};
static IRtlDevice* g_dev = nullptr;
static std::vector<uint8_t> g_rt;
static uint8_t g_chan = 6;
static std::atomic<uint64_t> g_probe{0}, g_auth{0}, g_assoc{0}, g_sent{0}, g_data{0};
static std::mutex g_q_mu;
static std::vector<std::vector<uint8_t>> g_q;      // pre-built radiotap+MPDU frames
static const uint8_t kApIp[4] = {192, 168, 99, 1}; // the AP's IP (station uses a static .2)

// 16-bit one's-complement checksum (IP / ICMP), returned host-order big-endian.
static uint16_t csum16(const uint8_t* d, int len) {
  uint32_t s = 0;
  for (int i = 0; i + 1 < len; i += 2) s += (uint32_t)(d[i] << 8) | d[i + 1];
  if (len & 1) s += (uint32_t)d[len - 1] << 8;
  while (s >> 16) s = (s & 0xffff) + (s >> 16);
  return (uint16_t)~s;
}
// Build an AP->STA data frame (from-DS): 802.11 data hdr + LLC/SNAP + payload.
static std::vector<uint8_t> build_data(const uint8_t* sta, uint16_t eth,
                                       const uint8_t* pl, int plen) {
  std::vector<uint8_t> m = {0x08, 0x02, 0x00, 0x00,      // data, from-DS
      sta[0],sta[1],sta[2],sta[3],sta[4],sta[5],          // addr1 = STA (DA)
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],  // addr2 = BSSID (TA)
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],  // addr3 = SA
      0x00, 0x00,
      0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00,                 // LLC/SNAP
      (uint8_t)(eth >> 8), (uint8_t)(eth & 0xff)};
  m.insert(m.end(), pl, pl + plen);
  return m;
}

// Common: [SSID + rates + DS] IE tail for probe/assoc responses.
static void append_ies(std::vector<uint8_t>& m, bool with_ssid) {
  if (with_ssid) { const char* s = "devourerAP";
    m.insert(m.end(), {0x00, 0x0a}); m.insert(m.end(), s, s + 10); }
  m.insert(m.end(), {0x01, 0x08, 0x82, 0x84, 0x8b, 0x96, 0x24, 0x30, 0x48, 0x6c});
  m.insert(m.end(), {0x03, 0x01, g_chan});
}
static void enqueue(std::vector<uint8_t> mpdu) {
  std::vector<uint8_t> f; f.reserve(g_rt.size() + mpdu.size());
  f.insert(f.end(), g_rt.begin(), g_rt.end());
  f.insert(f.end(), mpdu.begin(), mpdu.end());
  std::lock_guard<std::mutex> lk(g_q_mu);
  if (g_q.size() < 128) g_q.push_back(std::move(f));
}
static std::vector<uint8_t> mgmt_hdr(uint8_t subtype_fc, const uint8_t* sta) {
  return {subtype_fc, 0x00, 0x00, 0x00,
          sta[0],sta[1],sta[2],sta[3],sta[4],sta[5],
          kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
          kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
          0x00, 0x00};
}

static void on_rx(const Packet& p) {
  if (p.Data.size() < 24 || p.RxAtrib.crc_err) return;
  const uint8_t fc0 = p.Data[0], fc1 = p.Data[1];
  const uint8_t* a1 = p.Data.data() + 4;             // addr1 (RA)
  const uint8_t* sta = p.Data.data() + 10;           // addr2 (TA = station)
  bool to_us = std::memcmp(a1, kBssid, 6) == 0;
  bool bcast = (a1[0] & 0x01) != 0;
  bool retry = (fc1 & 0x08) != 0;

  if (fc0 == 0x40) {                                 // probe-request
    if (!bcast && !to_us) return;
    g_probe.fetch_add(1);
    auto m = mgmt_hdr(0x50, sta);                    // probe-response
    m.insert(m.end(), {0,0,0,0,0,0,0,0, 0x64,0x00, 0x01,0x00});  // ts, bcn int, cap
    append_ies(m, true);
    enqueue(std::move(m));
  } else if (fc0 == 0xb0 && to_us) {                 // authentication
    g_auth.fetch_add(1);
    uint16_t alg = p.Data[24] | (p.Data[25] << 8), seq = p.Data[26] | (p.Data[27] << 8);
    fprintf(stderr, "  AUTH req from %02x:%02x:%02x:%02x:%02x:%02x alg=%u seq=%u retry=%d\n",
            sta[0],sta[1],sta[2],sta[3],sta[4],sta[5], alg, seq, retry);
    auto m = mgmt_hdr(0xb0, sta);                    // auth response
    m.insert(m.end(), {0x00,0x00, 0x02,0x00, 0x00,0x00});  // open, seq 2, status 0
    enqueue(std::move(m));
  } else if ((fc0 == 0x00 || fc0 == 0x20) && to_us) {  // (re)assoc request
    g_assoc.fetch_add(1);
    fprintf(stderr, "  ASSOC req from %02x:%02x:%02x:%02x:%02x:%02x retry=%d\n",
            sta[0],sta[1],sta[2],sta[3],sta[4],sta[5], retry);
    auto m = mgmt_hdr(0x10, sta);                    // assoc response
    m.insert(m.end(), {0x01,0x00, 0x00,0x00, 0x01,0xc0});   // cap, status 0, AID 1
    append_ies(m, false);
    enqueue(std::move(m));
  } else if ((fc0 == 0x08 || fc0 == 0x88) && (fc1 & 0x01) && to_us) {  // data, to-DS
    // Data plane: answer ARP + ICMP echo so an associated station can ping the AP.
    int hlen = 24 + (fc0 == 0x88 ? 2 : 0);               // QoS data adds 2 bytes
    if ((int)p.Data.size() < hlen + 8) return;
    const uint8_t* llc = p.Data.data() + hlen;
    if (!(llc[0] == 0xaa && llc[1] == 0xaa && llc[2] == 0x03)) return;
    uint16_t eth = (llc[6] << 8) | llc[7];
    const uint8_t* pl = llc + 8;
    int pllen = (int)p.Data.size() - (hlen + 8);
    if (eth == 0x0806 && pllen >= 28) {                  // ARP
      uint16_t oper = (pl[6] << 8) | pl[7];
      const uint8_t* sha = pl + 8; const uint8_t* spa = pl + 14; const uint8_t* tpa = pl + 24;
      if (oper == 1 && std::memcmp(tpa, kApIp, 4) == 0) {  // request for the AP IP
        uint8_t a[28] = {0,1, 8,0, 6,4, 0,2,
            kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
            kApIp[0],kApIp[1],kApIp[2],kApIp[3],
            sha[0],sha[1],sha[2],sha[3],sha[4],sha[5], spa[0],spa[1],spa[2],spa[3]};
        enqueue(build_data(sta, 0x0806, a, 28)); g_data.fetch_add(1);
      }
    } else if (eth == 0x0800 && pllen >= 28) {           // IPv4
      const uint8_t* ip = pl; int ihl = (ip[0] & 0x0f) * 4;
      if (ip[9] == 1 && (int)pllen >= ihl + 8 && std::memcmp(ip + 16, kApIp, 4) == 0) {
        const uint8_t* icmp = ip + ihl;
        if (icmp[0] == 8) {                              // ICMP echo request -> reply
          std::vector<uint8_t> r(pl, pl + pllen);
          std::memcpy(r.data() + 12, kApIp, 4);          // IP src = AP
          std::memcpy(r.data() + 16, ip + 12, 4);        // IP dst = original src
          r[10] = r[11] = 0;
          uint16_t ic = csum16(r.data(), ihl); r[10] = ic >> 8; r[11] = ic & 0xff;
          r[ihl] = 0;                                    // ICMP type 0 (reply)
          r[ihl + 2] = r[ihl + 3] = 0;
          uint16_t cc = csum16(r.data() + ihl, pllen - ihl);
          r[ihl + 2] = cc >> 8; r[ihl + 3] = cc & 0xff;
          enqueue(build_data(sta, 0x0800, r.data(), pllen)); g_data.fetch_add(1);
        }
      }
    }
  }
}

int main(int argc, char** argv) {
  int sec = argc > 1 ? atoi(argv[1]) : 60;
  if (const char* c = std::getenv("DEVOURER_CHANNEL")) g_chan = (uint8_t)atoi(c);
  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  libusb_context* ctx = nullptr; libusb_init(&ctx);
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  uint16_t vid = 0x0bda, pid = 0xc812;
  if (const char* v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  if (const char* p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  auto* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open %04x:%04x fail\n", vid, pid); return 1; }
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if (devourer::claim_interface_then_reset(h, 0, logger, true, lk) != 0) return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(h, ctx, lk, devourer_config_from_env());
  g_dev = dev.get();
  if (!g_dev) return 1;
  g_rt = devourer::build_stream_radiotap(devourer::parse_tx_mode_str("6M"));
  g_dev->InitWrite(SelectedChannel{g_chan, 0, CHANNEL_WIDTH_20});
  // Beacon so the AP is discoverable + MACID/BSSID set (the ACK filter needs it).
  std::vector<uint8_t> bcn = {
      0x00,0x00,0x0a,0x00,0x00,0x80,0x00,0x00,0x08,0x00,   // radiotap
      0x80,0x00,0x00,0x00, 0xff,0xff,0xff,0xff,0xff,0xff,
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
      0x00,0x00, 0,0,0,0,0,0,0,0, 0x64,0x00, 0x01,0x00};
  { const char* s = "devourerAP"; bcn.insert(bcn.end(), {0x00,0x0a});
    bcn.insert(bcn.end(), s, s + 10);
    bcn.insert(bcn.end(), {0x01,0x08,0x82,0x84,0x8b,0x96,0x24,0x30,0x48,0x6c});
    bcn.insert(bcn.end(), {0x03,0x01,g_chan}); }
  int bcn_tu = 100;
  if (const char* iv = std::getenv("DEVOURER_BCN_TU")) bcn_tu = atoi(iv);
  bool bok = g_dev->StartBeacon(bcn.data(), bcn.size(), bcn_tu);
  std::thread rx([&]{ g_dev->StartRxLoop(on_rx); });
  fprintf(stderr, "ap_responder up on ch%d SSID devourerAP (beacon %s). %ds. "
                  "Connect a station: iw dev <if> connect -w devourerAP\n",
          g_chan, bok ? "OK" : "FAIL", sec);
  auto end = std::chrono::steady_clock::now() + std::chrono::seconds(sec);
  while (std::chrono::steady_clock::now() < end) {
    std::vector<std::vector<uint8_t>> batch;
    { std::lock_guard<std::mutex> lk2(g_q_mu); batch.swap(g_q); }
    for (auto& f : batch) if (g_dev->send_packet(f.data(), f.size())) g_sent.fetch_add(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  fprintf(stderr, "probe=%llu auth=%llu assoc=%llu data(arp/icmp)=%llu  responses_sent=%llu\n",
          (unsigned long long)g_probe.load(), (unsigned long long)g_auth.load(),
          (unsigned long long)g_assoc.load(), (unsigned long long)g_data.load(),
          (unsigned long long)g_sent.load());
  _exit(0);
}
