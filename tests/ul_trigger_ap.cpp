// ul_trigger_ap.cpp — tier-C end-to-end harness for 802.11ax scheduled UL.
//
// devourer runs as an HE (802.11ax) access point on a Kestrel adapter (default
// the RTL8832CU / 8852C, 35bc:0101): it beacons with HT+VHT+HE Capabilities /
// Operation IEs, answers probe/auth/(re)assoc, and serves the ARP/ICMP/DHCP
// data plane (so an associated station stays up and generates UL traffic). On a
// completed association it registers the station as a peer (RegisterPeerSta) and
// then drives the firmware UL-OFDMA scheduler (ConfigureUlOfdma tf_periodic +
// periodic SendTrigger) to grant that station UL transmit opportunities. An HE
// station responds to a Basic Trigger with a TB PPDU at trigger+SIFS in
// hardware — the AP's full-duplex RX logs frames from the station's MAC with
// their PPDU type, so an HE TB PPDU (vs a normal contention UL frame) is
// visible.
//
// The station side is a real kernel HE supplicant — the RTL8852BU under
// rtw89_8852bu (the one HE-capable USB kernel driver on the bench). Orchestrated
// by tests/ul_ofdma_e2e.sh.
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/ul_trigger_ap.cpp \
//   examples/common/env_config.cpp build/libdevourer.a \
//   $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/ul_trigger_ap
// Run: sudo DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_CHANNEL=36 \
//   DEVOURER_TX_WITH_RX=thread build/ul_trigger_ap [sec]
#include <atomic>
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
#include "TriggerTwt.h"
#include "TxMode.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

// Locally-administered unicast BSSID (a station cannot unicast-auth to a
// multicast address — see ap_responder.cpp).
static const uint8_t kBssid[6] = {0x02, 0x42, 0x75, 0x05, 0xd6, 0x00};
static IRtlDevice* g_dev = nullptr;
static std::vector<uint8_t> g_rt;
static uint8_t g_chan = 36;
static std::atomic<uint64_t> g_probe{0}, g_auth{0}, g_assoc{0}, g_sent{0}, g_data{0};
static std::atomic<uint64_t> g_ul{0}, g_ul_tb{0}, g_trig{0}, g_snd{0};
static std::atomic<bool> g_associated{false};
// Sounding mode (DEVOURER_SND=1): drive the production NDPA/NDP/BFRP path
// (StartSounding) instead of the F2P/UL_FIXINFO triggers, and advertise the AP
// as an HE beamformer so the STA acts as a beamformee and answers the BFRP with
// an HE TB PPDU report.
static bool g_snd_mode = false;
static uint8_t g_sta[6] = {0};
static std::mutex g_q_mu;
static std::vector<std::vector<uint8_t>> g_q;
static const uint8_t kApIp[4] = {192, 168, 99, 1};
static const uint8_t kLeaseIp[4] = {192, 168, 99, 2};
// The single granted station: AID 1 / macid 1 / addr-cam index 1.
static constexpr uint8_t kStaAid = 1, kStaMacid = 1, kStaCamIdx = 1;

static uint16_t csum16(const uint8_t* d, int len) {
  uint32_t s = 0; for (int i = 0; i + 1 < len; i += 2) s += (d[i] << 8) | d[i + 1];
  if (len & 1) s += d[len - 1] << 8;
  while (s >> 16) s = (s & 0xffff) + (s >> 16);
  return ~s;
}
static std::vector<uint8_t> build_data(const uint8_t* sta, uint16_t eth,
                                       const uint8_t* pl, int pllen) {
  std::vector<uint8_t> m = {0x08, 0x02, 0x00, 0x00,
      sta[0],sta[1],sta[2],sta[3],sta[4],sta[5],
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5], 0x00,0x00,
      0xaa,0xaa,0x03,0x00,0x00,0x00, (uint8_t)(eth >> 8), (uint8_t)(eth & 0xff)};
  m.insert(m.end(), pl, pl + pllen);
  return m;
}
static std::vector<uint8_t> build_dhcp_reply(const uint8_t* sta, const uint8_t* xid,
                                             uint8_t msgtype) {
  std::vector<uint8_t> bootp(240, 0);
  bootp[0] = 2; bootp[1] = 1; bootp[2] = 6;
  std::memcpy(&bootp[4], xid, 4);
  std::memcpy(&bootp[16], kLeaseIp, 4);
  std::memcpy(&bootp[20], kApIp, 4);
  std::memcpy(&bootp[28], sta, 6);
  bootp[236] = 0x63; bootp[237] = 0x82; bootp[238] = 0x53; bootp[239] = 0x63;
  std::vector<uint8_t> opt = {53,1,msgtype, 54,4,kApIp[0],kApIp[1],kApIp[2],kApIp[3],
      51,4,0,0,0x0e,0x10, 1,4,255,255,255,0, 3,4,kApIp[0],kApIp[1],kApIp[2],kApIp[3],
      255};
  bootp.insert(bootp.end(), opt.begin(), opt.end());
  int ul = 8 + (int)bootp.size();
  std::vector<uint8_t> pl(20 + ul, 0);
  pl[0] = 0x45; int tot = 20 + ul; pl[2] = tot >> 8; pl[3] = tot & 0xff;
  pl[8] = 64; pl[9] = 17; std::memcpy(&pl[12], kApIp, 4);
  pl[16]=pl[17]=pl[18]=pl[19]=0xff;
  uint16_t ic = csum16(pl.data(), 20); pl[10] = ic >> 8; pl[11] = ic & 0xff;
  pl[20] = 0; pl[21] = 67; pl[22] = 0; pl[23] = 68;
  pl[24] = ul >> 8; pl[25] = ul & 0xff;
  std::memcpy(&pl[28], bootp.data(), bootp.size());
  return build_data(sta, 0x0800, pl.data(), (int)pl.size());
}

// ---- 802.11ax HE information elements for a 5 GHz 20 MHz HE AP ----
// Minimal-but-valid HT/VHT/HE Capabilities + Operation so an HE station
// negotiates HE. 1SS. Lengths are the element bodies (the 0xff extension
// elements count the extension-ID byte in their length). Tweak here if a
// station's supplicant rejects the association (iterate against dmesg / iw).
static void append_he_ies(std::vector<uint8_t>& m) {
  // HT Capabilities (tag 45, len 26): SMPS disabled, 20 MHz, MCS0-7 (1SS).
  m.insert(m.end(), {0x2d, 0x1a,
      0x0c, 0x00,                                     // HT cap info (SMPS disabled)
      0x03,                                           // A-MPDU params
      0xff, 0,0,0,0,0,0,0,0,0, 0,0, 0,0,0,0,          // supported MCS set (1SS)
      0x00, 0x00,                                     // HT extended cap
      0x00, 0x00, 0x00, 0x00,                         // TX beamforming
      0x00});                                         // ASEL
  // HT Operation (tag 61, len 22): primary channel, 20 MHz, basic MCS0-7.
  m.insert(m.end(), {0x3d, 0x16,
      g_chan, 0x00, 0x00, 0x00, 0x00, 0x00,           // primary + op info
      0xff, 0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0});         // basic MCS set
  // VHT Capabilities (tag 191, len 12): 1SS MCS0-9.
  m.insert(m.end(), {0xbf, 0x0c,
      0x00, 0x00, 0x00, 0x00,                         // VHT cap info
      0xfe, 0xff, 0x00, 0x00, 0xfe, 0xff, 0x00, 0x00}); // RX/TX MCS maps (1SS)
  // VHT Operation (tag 192, len 5): 20/40 MHz, basic MCS0-7 (1SS).
  m.insert(m.end(), {0xc0, 0x05, 0x00, 0x00, 0x00, 0xfc, 0xff});
  // HE Capabilities (ext tag 255 / ext 35, len 22): MAC(6)+PHY(11)+MCS/NSS(4).
  // In sounding mode advertise the AP as an SU+MU beamformer with 1 sounding
  // dimension (HE PHY caps: SU Beamformer B31=byte3 bit7, SU Beamformee B32 +
  // MU Beamformer B33 = byte4 bits 0/1, Number Of Sounding Dimensions<=80
  // B40-42 = byte5 bits 0-2) so the STA responds to our BFRP with a TB report.
  const uint8_t phy3 = g_snd_mode ? 0x80 : 0x00;
  const uint8_t phy4 = g_snd_mode ? 0x03 : 0x00;
  const uint8_t phy5 = g_snd_mode ? 0x01 : 0x00;
  m.insert(m.end(), {0xff, 0x16, 0x23,
      0x00,0x00,0x00,0x00,0x00,0x00,                  // HE MAC cap
      0x02,0x00,0x00,phy3,phy4,phy5,0x00,0x00,0x00,0x00,0x00, // HE PHY cap
      0xfe,0xff, 0xfe,0xff});                          // HE-MCS/NSS <=80MHz (1SS 0-9)
  // HE Operation (ext tag 255 / ext 36, len 7): params(3)+color(1)+basic MCS(2).
  m.insert(m.end(), {0xff, 0x07, 0x24,
      0x00, 0x00, 0x00,                               // HE op params
      0x01,                                           // BSS color 1
      0xfc, 0xff});                                   // basic HE-MCS (1SS 0-7)
}

// Supported Rates IE, band-correct: CCK+OFDM on 2.4 GHz, OFDM-only on 5 GHz.
// CCK basic rates (1/2/5.5/11) do not exist on 5 GHz — advertising them makes a
// 5 GHz station skip the BSS with "rate sets do not match" (no association).
static void append_rates(std::vector<uint8_t>& m) {
  if (g_chan <= 14)  // 2.4 GHz: 1*,2*,5.5*,11*,18,24,36,54
    m.insert(m.end(), {0x01, 0x08, 0x82, 0x84, 0x8b, 0x96, 0x24, 0x30, 0x48, 0x6c});
  else               // 5 GHz: 6*,9,12*,18,24*,36,48,54 (basic = high bit set)
    m.insert(m.end(), {0x01, 0x08, 0x8c, 0x12, 0x98, 0x24, 0xb0, 0x48, 0x60, 0x6c});
}

// [SSID +] rates + DS + HE IE tail for probe/assoc responses.
static void append_ies(std::vector<uint8_t>& m, bool with_ssid) {
  if (with_ssid) { const char* s = "devourerAP";
    m.insert(m.end(), {0x00, 0x0a}); m.insert(m.end(), s, s + 10); }
  append_rates(m);
  m.insert(m.end(), {0x03, 0x01, g_chan});
  append_he_ies(m);
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
          kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5], 0x00, 0x00};
}

static void on_rx(const Packet& p) {
  if (p.Data.size() < 24 || p.RxAtrib.crc_err) return;
  const uint8_t fc0 = p.Data[0], fc1 = p.Data[1];
  const uint8_t* a1 = p.Data.data() + 4;
  const uint8_t* sta = p.Data.data() + 10;
  bool to_us = std::memcmp(a1, kBssid, 6) == 0;
  bool bcast = (a1[0] & 0x01) != 0;
  bool retry = (fc1 & 0x08) != 0;

  if (fc0 == 0x40) {                                 // probe-request
    if (!bcast && !to_us) return;
    g_probe.fetch_add(1);
    auto m = mgmt_hdr(0x50, sta);
    m.insert(m.end(), {0,0,0,0,0,0,0,0, 0x64,0x00, 0x01,0x00});
    append_ies(m, true);
    enqueue(std::move(m));
  } else if (fc0 == 0xb0 && to_us) {                 // authentication
    g_auth.fetch_add(1);
    fprintf(stderr, "  AUTH from %02x:%02x:%02x:%02x:%02x:%02x retry=%d\n",
            sta[0],sta[1],sta[2],sta[3],sta[4],sta[5], retry);
    auto m = mgmt_hdr(0xb0, sta);
    m.insert(m.end(), {0x00,0x00, 0x02,0x00, 0x00,0x00});
    enqueue(std::move(m));
  } else if ((fc0 == 0x00 || fc0 == 0x20) && to_us) {  // (re)assoc request
    g_assoc.fetch_add(1);
    fprintf(stderr, "  ASSOC from %02x:%02x:%02x:%02x:%02x:%02x retry=%d\n",
            sta[0],sta[1],sta[2],sta[3],sta[4],sta[5], retry);
    auto m = mgmt_hdr(0x10, sta);
    // AID field: aid in bits 0-13, bits 14-15 set (802.11 convention) — LE, so
    // low byte = aid[7:0], high byte = 0xc0 | aid[13:8]. Encoding it the other
    // way round gives the STA a WRONG AID (e.g. 0xC1=193 instead of 1), so a
    // BFRP addressed to kStaAid is ignored — no TB report.
    m.insert(m.end(), {0x01,0x00, 0x00,0x00,
                       (uint8_t)(kStaAid & 0xff),
                       (uint8_t)(0xc0 | ((kStaAid >> 8) & 0x3f))});  // cap, status, AID
    append_ies(m, false);
    enqueue(std::move(m));
    // Latch the station MAC + associated flag ONLY here (RX thread). The peer
    // registration + all UL-OFDMA H2Cs are issued from the main thread — every
    // H2C funnels through KestrelFw's non-thread-safe scratch/seq state, so
    // firing them from the RX callback while the main thread also issues H2Cs
    // corrupts the H2C queue (peer_reg silently failed that way).
    if (!g_associated.exchange(true))
      std::memcpy(g_sta, sta, 6);
  } else if ((fc0 == 0x08 || fc0 == 0x88 || fc0 == 0xe8) && (fc1 & 0x01) && to_us) {
    // Uplink data from the associated station. Count it + log its PPDU type:
    // an HE TB PPDU (trigger-based) is the scheduled-UL response we're after.
    if (g_associated && std::memcmp(sta, g_sta, 6) == 0) {
      g_ul.fetch_add(1);
      const uint8_t ppt = p.RxAtrib.ppdu_type;
      // RxPacket.h ppdu_type: 2=OFDM 3=HT 5/6=VHT 7=HE_SU 8=HE_ERSU 9=HE_MU
      // 10=HE_TB. HE_TB is the trigger-based UL response we're after.
      bool tb = (ppt == 10);
      if (tb) g_ul_tb.fetch_add(1);
      static uint64_t logged = 0;
      if (logged++ < 40 || tb)
        fprintf(stderr, "{\"ev\":\"ul.rx\",\"ppdu_type\":%d,\"rate\":\"0x%x\","
                        "\"len\":%zu,\"tb\":%s,\"tsfl\":%u}\n",
                ppt, p.RxAtrib.data_rate, p.Data.size(), tb ? "true" : "false",
                p.RxAtrib.tsfl);
    }
    // Keep the data plane alive (ARP/ICMP/DHCP) so the station stays associated
    // and keeps generating UL traffic — the BSR the fw needs to grant a trigger.
    int hlen = 24 + ((fc0 == 0x88 || fc0 == 0xe8) ? 2 : 0);
    if ((int)p.Data.size() < hlen + 8) return;
    const uint8_t* llc = p.Data.data() + hlen;
    if (!(llc[0] == 0xaa && llc[1] == 0xaa && llc[2] == 0x03)) return;
    uint16_t eth = (llc[6] << 8) | llc[7];
    const uint8_t* pl = llc + 8;
    int pllen = (int)p.Data.size() - (hlen + 8);
    if (eth == 0x0806 && pllen >= 28) {
      uint16_t oper = (pl[6] << 8) | pl[7];
      const uint8_t* sha = pl + 8; const uint8_t* spa = pl + 14; const uint8_t* tpa = pl + 24;
      if (oper == 1 && std::memcmp(tpa, kApIp, 4) == 0) {
        uint8_t a[28] = {0,1, 8,0, 6,4, 0,2,
            kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
            kApIp[0],kApIp[1],kApIp[2],kApIp[3],
            sha[0],sha[1],sha[2],sha[3],sha[4],sha[5], spa[0],spa[1],spa[2],spa[3]};
        enqueue(build_data(sta, 0x0806, a, 28)); g_data.fetch_add(1);
      }
    } else if (eth == 0x0800 && pllen >= 28) {
      const uint8_t* ip = pl; int ihl = (ip[0] & 0x0f) * 4;
      if (ip[9] == 1 && (int)pllen >= ihl + 8 && std::memcmp(ip + 16, kApIp, 4) == 0) {
        const uint8_t* icmp = ip + ihl;
        if (icmp[0] == 8) {
          std::vector<uint8_t> r(pl, pl + pllen);
          std::memcpy(r.data() + 12, kApIp, 4);
          std::memcpy(r.data() + 16, ip + 12, 4);
          r[10] = r[11] = 0;
          uint16_t ic = csum16(r.data(), ihl); r[10] = ic >> 8; r[11] = ic & 0xff;
          r[ihl] = 0; r[ihl + 2] = r[ihl + 3] = 0;
          uint16_t cc = csum16(r.data() + ihl, pllen - ihl);
          r[ihl + 2] = cc >> 8; r[ihl + 3] = cc & 0xff;
          enqueue(build_data(sta, 0x0800, r.data(), pllen)); g_data.fetch_add(1);
        }
      } else if (ip[9] == 17 && (int)pllen >= ihl + 8 + 240) {
        const uint8_t* udp = ip + ihl;
        if (((udp[2] << 8) | udp[3]) == 67) {
          const uint8_t* dh = udp + 8; const uint8_t* end = pl + pllen;
          uint8_t mt = 0;
          for (const uint8_t* o = dh + 240; o + 1 < end && *o != 0xff; ) {
            if (*o == 0) { o++; continue; }
            if (*o == 53 && o + 2 < end) mt = o[2];
            o += 2 + o[1];
          }
          uint8_t reply = (mt == 1) ? 2 : (mt == 3) ? 5 : 0;
          if (reply) { enqueue(build_dhcp_reply(sta, dh + 4, reply)); g_data.fetch_add(1); }
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
  uint16_t vid = 0x35bc, pid = 0x0101;   // default AP = RTL8832CU (8852C)
  if (const char* v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  if (const char* p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  auto* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open %04x:%04x fail\n", vid, pid); return 1; }
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if (devourer::claim_interface_then_reset(h, devourer::find_wifi_interface(h), logger, true, lk) != 0) return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(h, ctx, lk, devourer_config_from_env());
  g_dev = dev.get();
  if (!g_dev) return 1;
  if (!g_dev->GetAdapterCaps().trigger_ul_ok) {
    fprintf(stderr, "AP adapter has no trigger-UL support (Kestrel-only)\n");
    return 1;
  }
  // Sounding mode: the production NDPA/NDP/BFRP trigger path (advertises the AP
  // as an HE beamformer + drives StartSounding). Must be set before the beacon
  // IEs are built. Requires the sounding cap.
  g_snd_mode = std::getenv("DEVOURER_SND") && atoi(std::getenv("DEVOURER_SND"));
  if (g_snd_mode && !g_dev->GetAdapterCaps().sounding_ok) {
    fprintf(stderr, "AP adapter has no HE sounding support\n");
    return 1;
  }
  g_rt = devourer::build_stream_radiotap(devourer::parse_tx_mode_str("6M"));
  g_dev->InitWrite(SelectedChannel{g_chan, 0, CHANNEL_WIDTH_20});

  std::vector<uint8_t> bcn = {
      0x00,0x00,0x0a,0x00,0x00,0x80,0x00,0x00,0x08,0x00,
      0x80,0x00,0x00,0x00, 0xff,0xff,0xff,0xff,0xff,0xff,
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],
      0x00,0x00, 0,0,0,0,0,0,0,0, 0x64,0x00, 0x01,0x00};
  { const char* s = "devourerAP"; bcn.insert(bcn.end(), {0x00,0x0a});
    bcn.insert(bcn.end(), s, s + 10);
    append_rates(bcn);
    bcn.insert(bcn.end(), {0x03,0x01,g_chan});
    append_he_ies(bcn); }
  int bcn_tu = 100;
  if (const char* iv = std::getenv("DEVOURER_BCN_TU")) bcn_tu = atoi(iv);
  bool bok = g_dev->StartBeacon(bcn.data(), bcn.size(), bcn_tu);
  std::thread rx([&]{ g_dev->StartRxLoop(on_rx); });
  fprintf(stderr, "ul_trigger_ap: HE AP up on ch%d SSID devourerAP (beacon %s). "
                  "%ds. Connect an HE station.\n", g_chan, bok ? "OK" : "FAIL", sec);

  // Trigger cadence: DEVOURER_TRIG_GAP_MS between SendTrigger calls once a
  // station is associated (0 = rely only on the UL_FIXINFO tf_periodic engine).
  long trig_gap_ms = 50;
  if (const char* e = std::getenv("DEVOURER_TRIG_GAP_MS")) trig_gap_ms = atol(e);
  // Sounding cadence: DEVOURER_SND_PERIOD>0 (default 50) arms fw-autonomous
  // PERIODIC sounding from a single H2C (SNDF2P_ADD) — the robust path, since a
  // fast per-sounding H2C loop floods and wedges the CH12 H2C queue (rc=-7).
  // SND_PERIOD=0 falls back to a slow one-shot re-issue at SND_GAP_MS.
  long snd_period = 50;
  if (const char* e = std::getenv("DEVOURER_SND_PERIOD")) snd_period = atol(e);
  long snd_gap_ms = 200;
  if (const char* e = std::getenv("DEVOURER_SND_GAP_MS")) snd_gap_ms = atol(e);
  bool peer_registered = false, ul_fixinfo_done = false, bfee_registered = false;
  bool snd_started = false;
  auto next_trig = std::chrono::steady_clock::now();
  auto next_snd = std::chrono::steady_clock::now();
  uint8_t snd_dialog = 0;

  auto end = std::chrono::steady_clock::now() + std::chrono::seconds(sec);
  while (std::chrono::steady_clock::now() < end) {
    { std::vector<std::vector<uint8_t>> batch;
      { std::lock_guard<std::mutex> lk2(g_q_mu); batch.swap(g_q); }
      for (auto& f : batch) if (g_dev->send_packet(f.data(), f.size())) g_sent.fetch_add(1); }

    if (g_associated && g_snd_mode) {
      // Sounding path: register the STA as a beamformee, then air the NDPA/NDP/
      // BFRP sequence (StartSounding). The fw builds+airs the trigger and arms
      // RX for the report HE TB PPDU (ppdu_type==10, counted as g_ul_tb below).
      if (!bfee_registered) {
        devourer::StaBfCaps bf;      // Nc-1=0 (1 col), Nr-1=1 (AP 2rx), Ng4, cb.
        bf.nc = 0; bf.nr = 1; bf.ng = 0; bf.cb = 1; bf.csi_bw = 0;
        bool ok = g_dev->RegisterBeamformee(g_sta, kStaMacid, kStaCamIdx, bf);
        fprintf(stderr, "{\"ev\":\"ap.bfee\",\"macid\":%d,\"aid\":%d,"
                        "\"reg\":%s}\n", kStaMacid, kStaAid, ok ? "true":"false");
        bfee_registered = true;
      }
      if (snd_period > 0) {
        // Periodic: one H2C arms fw-autonomous repeated sounding.
        if (!snd_started) {
          devourer::SoundingConfig sc = devourer::make_he_bfrp_sounding(
              kBssid, kBssid, g_sta, kStaAid, kStaMacid, snd_dialog++,
              (uint16_t)snd_period);
          bool ok = g_dev->StartSounding(sc);
          if (ok) g_snd.fetch_add(1);
          fprintf(stderr, "{\"ev\":\"ap.sounding\",\"periodic\":true,\"period\":%ld,"
                          "\"ok\":%s}\n", snd_period, ok ? "true" : "false");
          snd_started = true;
        }
      } else if (snd_gap_ms > 0 && std::chrono::steady_clock::now() >= next_snd) {
        // One-shot fallback at a deliberately slow cadence (>=100 ms) so the
        // CH12 H2C queue drains between soundings.
        devourer::SoundingConfig sc = devourer::make_he_bfrp_sounding(
            kBssid, kBssid, g_sta, kStaAid, kStaMacid, snd_dialog++, 0);
        if (g_dev->StartSounding(sc)) g_snd.fetch_add(1);
        next_snd = std::chrono::steady_clock::now() +
                   std::chrono::milliseconds(snd_gap_ms < 100 ? 100 : snd_gap_ms);
      }
    } else if (g_associated) {
      // Register the associated station as a peer (main thread — serialized H2C).
      if (!peer_registered) {
        bool ok = g_dev->RegisterPeerSta(g_sta, kStaMacid, kStaCamIdx);
        fprintf(stderr, "{\"ev\":\"ap.assoc\",\"sta\":\"%02x:%02x:%02x:%02x:%02x:%02x\","
                        "\"macid\":%d,\"aid\":%d,\"peer_reg\":%s}\n",
                g_sta[0],g_sta[1],g_sta[2],g_sta[3],g_sta[4],g_sta[5],
                kStaMacid, kStaAid, ok ? "true" : "false");
        peer_registered = true;
      }
      // Program the fw UL-OFDMA scheduler once (tf_periodic grants the STA).
      if (!ul_fixinfo_done) {
        devourer::UlOfdmaConfig ul;
        ul.mode = 3; ul.interval_s = 1; ul.tf_type = 0; ul.ppdu_bw = 0;
        ul.n_stas = 1; ul.stas[0].macid = kStaMacid; ul.stas[0].ru_pos = 61;
        ul.stas[0].mcs = 0; ul.stas[0].ss = 1; ul.stas[0].tgt_rssi_dbm = -50;
        bool uok = g_dev->ConfigureUlOfdma(ul);
        fprintf(stderr, "{\"ev\":\"ap.ul_ofdma\",\"ok\":%s}\n", uok ? "true" : "false");
        ul_fixinfo_done = true;
      }
      // Also fire explicit Basic Triggers granting the STA (belt + suspenders).
      if (trig_gap_ms > 0 && std::chrono::steady_clock::now() >= next_trig) {
        devourer::TriggerConfig cfg;
        cfg.ul_bw = 0; cfg.n_users = 1;
        cfg.users[0].aid12 = kStaAid; cfg.users[0].macid = kStaMacid;
        cfg.users[0].ru_alloc = devourer::he_ru_alloc(242, 0);
        cfg.users[0].ul_mcs = 0; cfg.users[0].ss = 1; cfg.users[0].tgt_rssi_dbm = -50;
        if (g_dev->SendTrigger(cfg)) g_trig.fetch_add(1);
        next_trig = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(trig_gap_ms);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  fprintf(stderr, "{\"ev\":\"ap.summary\",\"probe\":%llu,\"auth\":%llu,\"assoc\":%llu,"
                  "\"associated\":%s,\"mode\":\"%s\",\"triggers\":%llu,\"soundings\":%llu,"
                  "\"ul_rx\":%llu,\"ul_tb\":%llu,\"data\":%llu,\"sent\":%llu}\n",
          (unsigned long long)g_probe.load(), (unsigned long long)g_auth.load(),
          (unsigned long long)g_assoc.load(), g_associated ? "true" : "false",
          g_snd_mode ? "sounding" : "trigger",
          (unsigned long long)g_trig.load(), (unsigned long long)g_snd.load(),
          (unsigned long long)g_ul.load(),
          (unsigned long long)g_ul_tb.load(), (unsigned long long)g_data.load(),
          (unsigned long long)g_sent.load());
  _exit(0);
}
