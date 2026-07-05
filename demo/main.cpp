#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <libusb.h>

#include "BfReportDetect.h"
#include "RxPacket.h"
#include "SweepSpec.h"
#if defined(DEVOURER_HAVE_JAGUAR1)
#include "jaguar1/RtlJaguarDevice.h"
#endif
#include "RtlUsbAdapter.h"
#include "SignalStop.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"

#define USB_VENDOR_ID 0x0bda

/* Known USB product IDs for the Realtek Jaguar 802.11ac family driven by this
 * library: RTL8812AU (2T2R), RTL8811AU (1T1R cut), and RTL8814AU (4T4R RF /
 * 3-SS baseband). */
static constexpr uint16_t kRealtekProductIds[] = {
    0x8812, /* RTL8812AU (also seen on some 8811AU boards) */
    0x0811, /* RTL8811AU */
    0xa811, /* RTL8811AU */
    0xb811, /* RTL8811AU/8821AU variants */
    0x8813, /* RTL8814AU (Realtek demoboard PID, used by CF-938AC/CF-960AC) */
    0xb812, /* RTL8822BU WiFi-only (Jaguar2) */
    0xb82c, /* RTL8822BU (Jaguar2) */
    /* OEM-rebadged RTL8822BU (e.g. TP-Link Archer T3U 2357:012d) enumerate under
     * a non-Realtek VID — reach those with DEVOURER_VID / DEVOURER_PID. */
    0xc82c, /* RTL8822CU (Jaguar3) */
    0xc82e, /* RTL8822CU (Jaguar3) */
    0xc812, /* RTL8812CU WiFi-only (Jaguar3) */
    0x881a, /* RTL8812EU variant (Jaguar3 EU) */
    0x881b, /* RTL8812EU variant (Jaguar3 EU) */
    0x881c, /* RTL8812EU variant (Jaguar3 EU) */
    0xa81a, /* RTL8812EU — LB-LINK BL-M8812EU2 (Jaguar3 EU) */
    0xe822, /* RTL8822EU (Jaguar3 EU) */
    0xa82a, /* RTL8822EU (Jaguar3 EU) */
};

static int g_rx_count = 0;
#if defined(DEVOURER_HAVE_JAGUAR1)
static RtlJaguarDevice *g_rtl_device = nullptr;
#endif

/* Process-start reference for the init-timing lines (see src/InitTimer.h).
 * `init-timing: demo.first_rx_frame` is the end-to-end "ready to RX" mark:
 * exec → first 802.11 frame delivered to the packet processor. */
static const std::chrono::steady_clock::time_point g_proc_start =
    std::chrono::steady_clock::now();
static long long ms_since_start() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - g_proc_start)
      .count();
}

/* DEVOURER_TX_STATUS=1: surface chip-side C2H frames (TX status reports,
 * various diagnostic pings) on `<devourer-c2h>` with a raw hex dump, plus
 * a best-effort decode of the 8814A TX_RPT payload layout. The C2H
 * sub-type ID isn't enumerated in the vendored headers, so the decode is
 * speculative — the raw hex stays in the line so an observer can
 * validate the sub-type against on-air capture.
 *
 * DEVOURER_QUEUE_POLL_MS=N: periodic snapshot of the 8814A REG_FIFOPAGE_INFO
 * registers, throttled to one `<devourer-queue>` line per second on RX hook.
 * 8814-only (8812/8821 don't expose these registers as per-queue free pages). */
static const bool g_tx_status_enabled =
    std::getenv("DEVOURER_TX_STATUS") != nullptr;
static const uint32_t g_qd_poll_ms = []() -> uint32_t {
  const char *e = std::getenv("DEVOURER_QUEUE_POLL_MS");
  return e ? static_cast<uint32_t>(std::strtoul(e, nullptr, 0)) : 0u;
}();

/* DEVOURER_THERMAL_POLL_MS=N: periodic snapshot of the chip thermal meter
 * (RF[A][0x42][15:10]), one `<devourer-thermal>` line per interval. Works on
 * every Jaguar member. 0 = disabled. DEVOURER_THERMAL_WARN_DELTA overrides the
 * warn threshold (thermal units above the EFUSE baseline; default 15). */
static const uint32_t g_thermal_poll_ms = []() -> uint32_t {
  const char *e = std::getenv("DEVOURER_THERMAL_POLL_MS");
  return e ? static_cast<uint32_t>(std::strtoul(e, nullptr, 0)) : 0u;
}();
static const int g_thermal_warn_delta = []() -> int {
  const char *e = std::getenv("DEVOURER_THERMAL_WARN_DELTA");
  return e ? std::atoi(e) : 15;
}();

/* DEVOURER_RX_DUMP_CSI=hex,hex,... (or "0x1a,0x20,0x40"): F2 research
 * spike. On each canonical-SA RX frame (first N frames), read BB
 * dbgport 0x8FC at each selector and emit
 *   <devourer-csi>selector=0xNN value=0xNNNNNNNN
 *
 * This is a SELECTOR-SWEEP framework — the actual per-subcarrier IQ
 * selector is missing from in-tree sources (see BbDbgportReader.h for
 * details), so this knob exists so a researcher can try selectors at
 * runtime, capture the resulting words, and look for plausible
 * IQ-like patterns without recompiling. Throttled to the first 8
 * canonical-SA frames to bound brick-risk.
 *
 * BRICK RISK: enabling this writes to 0x8FC while RX is live. If the
 * chip stops responding after a sweep, the reader self-wedges (see
 * <devourer-csi-wedged>) and refuses further writes; recover with
 * libusb_reset_device / usbreset / power-cycle. */
static const std::vector<uint32_t> g_csi_selectors = []() -> std::vector<uint32_t> {
  const char *e = std::getenv("DEVOURER_RX_DUMP_CSI");
  if (!e || !*e) return {};
  std::vector<uint32_t> out;
  std::string s = e;
  size_t pos = 0;
  while (pos < s.size()) {
    size_t comma = s.find(',', pos);
    std::string tok = s.substr(pos, comma == std::string::npos
                                        ? std::string::npos
                                        : comma - pos);
    if (!tok.empty()) {
      out.push_back(static_cast<uint32_t>(std::strtoul(tok.c_str(), nullptr, 0)));
    }
    if (comma == std::string::npos) break;
    pos = comma + 1;
  }
  return out;
}();
static constexpr int kCsiMaxFrames = 8;

/* DEVOURER_RX_ENERGY_MS=N: periodic frame-free RX energy / channel-busy
 * telemetry — the read side of DEVOURER_CW_TONE. Each interval emits one
 * <devourer-energy> line combining the chip's phydm FA/CCA counters + IGI
 * (IRtlDevice::GetRxEnergy, frame-free, all three generations) with a rolling
 * per-frame RSSI/SNR aggregate. A second adapter running this detects the first
 * adapter's CW carrier as a jump in cca_ofdm / fa_ofdm and a rise in igi.
 * 0 = disabled. */
static const uint32_t g_rx_energy_ms = []() -> uint32_t {
  const char *e = std::getenv("DEVOURER_RX_ENERGY_MS");
  return (e && *e) ? static_cast<uint32_t>(std::strtoul(e, nullptr, 0)) : 0;
}();

/* DEVOURER_RX_SWEEP="1,6,11" | "36-48/4" | "5170-5250/5": live coarse spectrum
 * sweep. Cycle the listed bins (SweepSpec grammar: channels, channel ranges,
 * or MHz ranges — the latter for issue-#149-style narrowband maps), dwelling
 * DEVOURER_RX_SWEEP_DWELL_MS (default 300) on each, and emit one
 * <devourer-energy>ch=N line per bin. The RX loop runs on a worker thread so
 * the main thread is free to retune (FastRetune) between reads — a live
 * energy-vs-frequency map that localizes an interferer (peaks, or dips on the
 * 1T1R parts that saturate, at the tone's channel). Empty = disabled. */
static const std::vector<int> g_rx_sweep =
    devourer::parse_sweep_spec(std::getenv("DEVOURER_RX_SWEEP"));
static const uint32_t g_rx_sweep_dwell_ms = []() -> uint32_t {
  const char *e = std::getenv("DEVOURER_RX_SWEEP_DWELL_MS");
  return (e && *e) ? static_cast<uint32_t>(std::strtoul(e, nullptr, 0)) : 300;
}();

/* Rolling per-frame RSSI/SNR/EVM aggregate (the frame-driven signal, all
 * gens): updated per received frame in packetProcessor, drained each interval
 * by the energy emitter and per dwell by the sweep loop. EVM counts only
 * frames that report one (CCK / non-type1 phy-status leaves it 0), so a mixed
 * stream doesn't bias the mean toward 0. */
static std::mutex g_rxagg_mu;
struct RxAgg {
  uint32_t n = 0;
  int32_t rssi_sum = 0, rssi_max = -128, snr_sum = 0, snr_min = 127;
  int32_t evm_sum = 0;
  uint32_t evm_n = 0;
  void add(int rssi, int snr, int evm) {
    ++n;
    rssi_sum += rssi;
    if (rssi > rssi_max) rssi_max = rssi;
    snr_sum += snr;
    if (snr < snr_min) snr_min = snr;
    if (evm != 0) { evm_sum += evm; ++evm_n; }
  }
};
static RxAgg g_rxagg;

/* The canonical txdemo beacon SA (same constant as txdemo/main.cpp and
 * tests/regress.py CANONICAL_SA — change all three together): the
 * <devourer-tx-hit> matcher and the "canon" aggregate filter below. */
static const uint8_t kTxSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};

/* DEVOURER_RX_AGG_SA: restrict the per-frame aggregate to frames whose SA
 * matches — the active-sounding filter, so ambient traffic doesn't pollute the
 * per-bin link stats (a sweep dwell hears everything on the bin; H(f) wants
 * only the probe TX). "canon" = kTxSa; "aa:bb:cc:dd:ee:ff" = that address;
 * unset/"any" = every frame (existing behaviour). */
static bool g_agg_sa_filter = false;
static uint8_t g_agg_sa[6];
static const bool g_agg_sa_parsed = []() {
  const char *e = std::getenv("DEVOURER_RX_AGG_SA");
  if (!e || !*e || std::strcmp(e, "any") == 0)
    return false;
  if (std::strcmp(e, "canon") == 0) {
    std::memcpy(g_agg_sa, kTxSa, 6);
    g_agg_sa_filter = true;
    return true;
  }
  unsigned b[6];
  if (std::sscanf(e, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4],
                  &b[5]) == 6) {
    for (int i = 0; i < 6; i++)
      g_agg_sa[i] = static_cast<uint8_t>(b[i]);
    g_agg_sa_filter = true;
    return true;
  }
  return false;
}();
static bool agg_sa_match(const Packet &packet) {
  if (!g_agg_sa_filter)
    return true;
  /* SA at offset 10 (FC + duration + addr1) — same layout the tx-hit matcher
   * keys on; frames too short to carry it don't count. */
  return packet.Data.size() >= 16 &&
         std::memcmp(packet.Data.data() + 10, g_agg_sa, 6) == 0;
}

/* Emit the frame-free NHM power histogram (IRtlDevice::GetRxEnergy fills it) as
 * a distinct <devourer-nhm> line so it never disturbs the <devourer-energy>
 * format its regex consumers key on. `peak` = the fullest bucket (0 = quiet
 * noise floor, higher = energy is landing in a higher power band, e.g. under an
 * interferer); `busy` = percent of samples above the lowest bucket; `hist` =
 * the 12 raw bucket counts (IGI-referenced, low→high power). ch<0 omits the
 * channel field (steady-state emitter); ch>=0 tags it (sweep). */
static void emit_nhm(const RxEnergy &e, int ch) {
  if (!e.valid_nhm)
    return;
  uint32_t total = 0, peak = 0;
  int peak_k = 0;
  for (int k = 0; k < 12; k++) {
    total += e.nhm[k];
    if (e.nhm[k] > peak) { peak = e.nhm[k]; peak_k = k; }
  }
  int busy = total ? static_cast<int>(100 * (total - e.nhm[0]) / total) : 0;
  char hist[96];
  int off = 0;
  for (int k = 0; k < 12; k++)
    off += std::snprintf(hist + off, sizeof(hist) - off, "%s%u",
                         k ? "," : "", e.nhm[k]);
  if (ch >= 0)
    printf("<devourer-nhm>ch=%d peak=%d busy=%d dur=%u hist=%s\n", ch, peak_k,
           busy, e.nhm_duration, hist);
  else
    printf("<devourer-nhm>peak=%d busy=%d dur=%u hist=%s\n", peak_k, busy,
           e.nhm_duration, hist);
  fflush(stdout);
}

static void packetProcessor(const Packet &packet) {
  /* C2H packets carry chip-side status updates, not 802.11 frames. Handle
   * them up front so the rest of this function (which assumes a normal
   * 802.11 MPDU layout) doesn't try to read SA bytes from a C2H payload. */
  if (packet.RxAtrib.pkt_rpt_type == RX_PACKET_TYPE::C2H_PACKET) {
    if (g_tx_status_enabled) {
      printf("<devourer-c2h>len=%zu bytes=", packet.Data.size());
      for (size_t i = 0; i < packet.Data.size(); ++i)
        printf("%02x", packet.Data[i]);
      printf("\n");
      /* Best-effort 8814A TX_RPT decode. The GET_8814A_C2H_TX_RPT_*
       * macros (hal/rtl8814a_cmd.h:118-125) read from a "_Header" pointer
       * — which, in upstream Realtek code, points one or two bytes past
       * the C2H frame start (after cmd_id [+ seq]). We try the two most
       * common offsets (1 and 2) and emit each; an observer can pick the
       * one whose queue_id / rate / retry values look plausible. */
      if (packet.Data.size() >= 8) {
        for (size_t hoff : {size_t(1), size_t(2)}) {
          if (packet.Data.size() < hoff + 6) continue;
          const uint8_t *h = packet.Data.data() + hoff;
          uint8_t  queue   = h[0] & 0x1f;
          uint8_t  retry   = h[2] & 0x3f;
          uint16_t qt_raw  = static_cast<uint16_t>(h[3] | (h[4] << 8));
          uint32_t qt_us   = static_cast<uint32_t>(qt_raw) * 256u;
          uint8_t  rate    = h[5];
          printf("<devourer-tx-status>hoff=%zu queue=%u retry=%u "
                 "airtime_us=%u rate=%u\n",
                 hoff, queue, retry, qt_us, rate);
        }
      }
      fflush(stdout);
    }
    return;
  }

  ++g_rx_count;

  /* Feed the rolling per-frame RSSI/SNR/EVM aggregate for DEVOURER_RX_ENERGY_MS
   * and the sweep's per-dwell frame stats (the frame-driven half of the energy
   * telemetry). path-A chain; DEVOURER_RX_AGG_SA optionally restricts it to the
   * sounding probe's SA. */
  if ((g_rx_energy_ms > 0 || !g_rx_sweep.empty()) && agg_sa_match(packet)) {
    std::lock_guard<std::mutex> lk(g_rxagg_mu);
    g_rxagg.add(packet.RxAtrib.rssi[0], packet.RxAtrib.snr[0],
                packet.RxAtrib.evm[0]);
  }

  if (g_rx_count == 1) {
    printf("<devourer>init-timing: demo.first_rx_frame = %lld ms\n",
           ms_since_start());
    fflush(stdout);
  }

  if (g_rx_count <= 10 || g_rx_count % 100 == 0) {
    printf("<devourer>RX pkt #%d (len=%zu)\n", g_rx_count, packet.Data.size());
    fflush(stdout);
  }
  /* DEVOURER_RX_DUMP_ALL=1: emit a `<devourer-corrupt-any>` line for EVERY
   * frame regardless of SA, with chip-flag bits and phy-soft metrics.
   * Consumed by tools/precoder/corruption_survey.py for the FEC-design
   * corruption-pattern survey. Pairs with DEVOURER_RX_KEEP_CORRUPTED to
   * also pass through chip-FCS-error frames. The body is omitted from this
   * line by design (a hot survey would inflate the log past usable size);
   * pkt_len + the chip flags + phy metrics is what aggregates carry. */
  static const bool dump_all = std::getenv("DEVOURER_RX_DUMP_ALL") != nullptr;
  if (dump_all) {
    printf("<devourer-corrupt-any>len=%zu crc_err=%u icv_err=%u "
           "rate=%u bw=%u stbc=%u ldpc=%u sgi=%u rssi=%d,%d evm=%d,%d snr=%d,%d\n",
           packet.Data.size(),
           packet.RxAtrib.crc_err ? 1u : 0u,
           packet.RxAtrib.icv_err ? 1u : 0u,
           packet.RxAtrib.data_rate,
           packet.RxAtrib.bw, packet.RxAtrib.stbc,
           packet.RxAtrib.ldpc, packet.RxAtrib.sgi,
           packet.RxAtrib.rssi[0], packet.RxAtrib.rssi[1],
           packet.RxAtrib.evm[0], packet.RxAtrib.evm[1],
           packet.RxAtrib.snr[0], packet.RxAtrib.snr[1]);
    fflush(stdout);
  }
  /* BF self-sounding report detector (DEVOURER_BF_DETECT_REPORT modes 1-4) —
   * shared with WiFiDriverTxDemo's single-radio capture, see BfReportDetect.h. */
  devourer::bf::detect_report(packet);

  /* TX-validation hook: detect frames whose SA matches the txdemo's hardcoded
   * injected beacon (57:42:75:05:d6:00). When running this RX demo against
   * one adapter while WiFiDriverTxDemo runs against another on the same
   * channel, each hit confirms an injected frame made it over the air. */
  if (packet.Data.size() >= 16) {
    if (std::memcmp(packet.Data.data() + 10, kTxSa, 6) == 0) {
      static int hits = 0;
      ++hits;
      if (hits <= 10 || hits % 100 == 0) {
        printf("<devourer-tx-hit>txdemo SA match: hits=%d total_rx=%d len=%zu\n",
               hits, g_rx_count, packet.Data.size());
        fflush(stdout);
      }
#if defined(DEVOURER_HAVE_JAGUAR1)
      /* F2: BB-dbgport sweep on the first kCsiMaxFrames canonical-SA frames.
       * Jaguar1-only (RtlJaguarDevice); g_rtl_device is null on Jaguar3. */
      if (!g_csi_selectors.empty() && g_rtl_device != nullptr &&
          hits <= kCsiMaxFrames && !g_rtl_device->bb_dbgport_wedged()) {
        for (uint32_t sel : g_csi_selectors) {
          uint32_t v = g_rtl_device->read_bb_dbgport(sel);
          if (g_rtl_device->bb_dbgport_wedged()) {
            printf("<devourer-csi-wedged>after selector=0x%08x — reader "
                   "refusing further writes. Recover with "
                   "libusb_reset_device / usbreset.\n", sel);
            fflush(stdout);
            break;
          }
          printf("<devourer-csi>hit=%d selector=0x%08x value=0x%08x\n",
                 hits, sel, v);
        }
        fflush(stdout);
      }
#endif
      /* DEVOURER_DUMP_SCRAMBLER=1: print the descrambler seed the chip
       * recovered from this frame's SERVICE field. Consumed by
       * tools/precoder/seed_probe.py --mode rx to learn the seed a precoder TX
       * chip uses. CAVEAT: the seed is only trustworthy when *this* RX adapter
       * is an RTL8814AU — the 8812/8821 RX descriptor doesn't expose it there
       * (see FrameParser.cpp). On 8812/8821 prefer seed_probe.py --mode
       * bruteforce. Gated + SA-filtered so it doesn't flood. */
      static const bool dump_scrambler =
          std::getenv("DEVOURER_DUMP_SCRAMBLER") != nullptr;
      if (dump_scrambler && (hits <= 20 || hits % 100 == 0)) {
        printf("<devourer-scrambler>seed=0x%02x rate=%u hits=%d len=%zu\n",
               packet.RxAtrib.scrambler, packet.RxAtrib.data_rate, hits,
               packet.Data.size());
        fflush(stdout);
      }
      /* DEVOURER_DUMP_BODY=1: print the RX rate index (DESC_RATE*: 0x04=6M
       * OFDM, 0x00=1M CCK, 0x0c+=HT/VHT MCS) and the 802.11 frame body
       * (everything after the 24-byte mgmt header) as hex. Consumed by
       * tests/precoder_roundtrip.py to confirm a PrecoderDemo frame flew as
       * 6M OFDM and that its shaped PSDU bytes round-tripped intact — the
       * two-adapter, no-SDR verification. First few hits only. */
      static const bool dump_body = std::getenv("DEVOURER_DUMP_BODY") != nullptr;
      /* DEVOURER_STREAM_OUT=1: like DEVOURER_DUMP_BODY but uncapped — print
       * every canonical-SA frame's body for the stream RX driver
       * (tools/precoder/stream_rx.py) to decode. Tag is distinct so the
       * regular dump_body capture stays uncluttered. */
      static const bool stream_out = std::getenv("DEVOURER_STREAM_OUT") != nullptr;
      /* DEVOURER_RX_KEEP_CORRUPTED=1: surface the body even when the chip
       * flagged CRC/ICV error. Default is to filter them out for the byte-
       * stream consumer (stream_rx.py), since a body with a wrong tail is
       * the byte-mode parser's worst-case input. The flag is the entry
       * point for the corruption_analysis.py tool — by-design opt-in so
       * accidental enablement doesn't cause IP-stack misery. */
      static const bool keep_corrupted =
          std::getenv("DEVOURER_RX_KEEP_CORRUPTED") != nullptr;
      const bool corrupted = packet.RxAtrib.crc_err || packet.RxAtrib.icv_err;
      /* DEVOURER_RX_ALLPATHS=1: emit all four RX chains (A,B,C,D) of per-stream
       * RSSI / SNR / EVM on a distinct <devourer-rxpath> line. Opt-in and
       * separate so the canonical two-path <devourer-stream>/<devourer-body>
       * format its regex consumers key on stays untouched. Paths C/D are
       * non-zero only on the 8814AU (4T4R); on 8812/8821 they read 0. Consumed
       * by tests/antenna_decorrelation.py to measure inter-chain envelope
       * correlation and realised diversity gain (spatial-diversity axis). */
      static const bool rxpath_out =
          std::getenv("DEVOURER_RX_ALLPATHS") != nullptr;
      if (rxpath_out && (!corrupted || keep_corrupted)) {
        printf("<devourer-rxpath>seq=%u rssi=%d,%d,%d,%d snr=%d,%d,%d,%d "
               "evm=%d,%d,%d,%d\n",
               packet.RxAtrib.seq_num, packet.RxAtrib.rssi[0],
               packet.RxAtrib.rssi[1], packet.RxAtrib.rssi[2],
               packet.RxAtrib.rssi[3], packet.RxAtrib.snr[0],
               packet.RxAtrib.snr[1], packet.RxAtrib.snr[2],
               packet.RxAtrib.snr[3], packet.RxAtrib.evm[0],
               packet.RxAtrib.evm[1], packet.RxAtrib.evm[2],
               packet.RxAtrib.evm[3]);
        fflush(stdout);
      }
      if (stream_out && (!corrupted || keep_corrupted)) {
        /* Per-stream phy soft metrics (RSSI / EVM / SNR for paths A,B; on
         * 8814AU paths C,D would also be non-zero but we surface only A,B
         * here to stay aligned with <devourer-body>'s format). These are
         * link-quality measurements at the PHY before decoding — same
         * source as the Tier-2 diagnostics — so a consumer like
         * corruption_analysis.py can correlate BER with link quality on a
         * per-frame basis instead of relying on aggregated statistics. */
        /* seq + tsfl: chip-side sequence number (12-bit u16) and TSF low
         * (full 32-bit u32). Consumers can dedup by seq and measure
         * one-way latency by diffing TSF against the host clock. Optional
         * fields — pre-#84 regex consumers tolerate them via the same
         * pass-through pattern. */
        /* Decoded PHY descriptor fields (bw/stbc/ldpc/sgi) alongside the rate
         * index: these let an SDR-as-TX completeness harness assert that the
         * frame devourer received carries the bandwidth / STBC / FEC / guard
         * interval the transmitter encoded. Valid on 8812/8821; on 8814AU the
         * RX descriptor doesn't expose these at this offset (FrameParser.cpp),
         * so they read as the chip's defaults there. Inserted before body= so
         * the trailing hex-dump pattern downstream regex consumers key on is
         * unchanged. */
        printf("<devourer-stream>rate=%u len=%zu crc_err=%u icv_err=%u "
               "rssi=%d,%d evm=%d,%d snr=%d,%d seq=%u tsfl=%u "
               "bw=%u stbc=%u ldpc=%u sgi=%u body=",
               packet.RxAtrib.data_rate, packet.Data.size(),
               packet.RxAtrib.crc_err ? 1u : 0u,
               packet.RxAtrib.icv_err ? 1u : 0u,
               packet.RxAtrib.rssi[0], packet.RxAtrib.rssi[1],
               packet.RxAtrib.evm[0], packet.RxAtrib.evm[1],
               packet.RxAtrib.snr[0], packet.RxAtrib.snr[1],
               packet.RxAtrib.seq_num, packet.RxAtrib.tsfl,
               packet.RxAtrib.bw, packet.RxAtrib.stbc,
               packet.RxAtrib.ldpc, packet.RxAtrib.sgi);
        for (size_t i = 24; i < packet.Data.size(); ++i)
          printf("%02x", packet.Data[i]);
        printf("\n");
        fflush(stdout);
      }
      if (dump_body && hits <= 5) {
        /* Tier-2 health diagnostics alongside the byte mirror: rate (0x04 =
         * 6M OFDM), per-stream RSSI/EVM/SNR (link quality — content-blind),
         * crc (always 0: CRC-failed frames are dropped upstream, so reaching
         * here is itself the decode-sanity signal). Then the body hex. */
        printf("<devourer-body>rate=%u rssi=%d,%d evm=%d,%d snr=%d,%d crc=%d "
               "len=%zu body=",
               packet.RxAtrib.data_rate, packet.RxAtrib.rssi[0],
               packet.RxAtrib.rssi[1], packet.RxAtrib.evm[0],
               packet.RxAtrib.evm[1], packet.RxAtrib.snr[0],
               packet.RxAtrib.snr[1], packet.RxAtrib.crc_err ? 1 : 0,
               packet.Data.size());
        for (size_t i = 24; i < packet.Data.size(); ++i)
          printf("%02x", packet.Data[i]);
        printf("\n");
        fflush(stdout);
      }
    }
  }
}

int main() {
  libusb_context *ctx;
  int rc;

  auto logger = std::make_shared<Logger>();

  /* SIGINT/SIGTERM -> clean shutdown (Stop() below). Without this the harness's
   * `timeout` SIGTERM killed us mid-RX, leaving the chip's USB core hung. */
  install_devourer_signal_handlers();

  rc = libusb_init(&ctx);
  if (rc < 0) {
    return rc;
  }

  /* libusb log level: WARNING by default. DEBUG is opt-in via
   * DEVOURER_USB_DEBUG=1 — it emits ~7 MB per 15s run (has filled /tmp and
   * wedged the harness), and bench_init.py measured it adding 0.5-0.8s to
   * time-to-first-RX-frame even with stderr discarded; on a slow sink
   * (SSH/serial/embedded flash) the cost is unbounded. DEVOURER_USB_QUIET
   * is accepted as a no-op for backwards compatibility with older scripts. */
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL,
                    std::getenv("DEVOURER_USB_DEBUG")
                        ? LIBUSB_LOG_LEVEL_DEBUG
                        : LIBUSB_LOG_LEVEL_WARNING);

  /* DEVOURER_PID env var (hex, e.g. "0x8813") restricts the open loop to a
   * single PID. Useful when multiple Realtek adapters are plugged.
   * DEVOURER_VID overrides the VID (default 0x0bda Realtek) — needed to reach
   * OEM-rebadged Jaguar dongles like the TP-Link Archer T2U Plus (2357:0120). */
  const char *pid_env = std::getenv("DEVOURER_PID");
  uint16_t target_pid = 0;
  if (pid_env != nullptr) {
    target_pid = static_cast<uint16_t>(std::strtoul(pid_env, nullptr, 0));
    logger->info("DEVOURER_PID={:04x} (limiting to this PID)", target_pid);
  }
  uint16_t target_vid = USB_VENDOR_ID;
  if (const char *vid_env = std::getenv("DEVOURER_VID")) {
    target_vid = static_cast<uint16_t>(std::strtoul(vid_env, nullptr, 0));
    logger->info("DEVOURER_VID={:04x} (overriding default VID)", target_vid);
  }
  libusb_device_handle *dev_handle = nullptr;

  /* DEVOURER_USB_BUS (+ optional DEVOURER_USB_PORT) select a specific device by
   * USB topology when several share one VID:PID and even the serial — e.g. two
   * RTL8814AU dongles (CF-938AC vs CF-960AC) that enumerate identically, so only
   * the bus/port tells them apart. DEVOURER_USB_PORT is the dotted libusb port
   * path (as in sysfs `devpath` / `lsusb -t`, e.g. "2.3.2"). When bus is unset,
   * the VID:PID open loop below runs as before. */
  if (const char *bus_env = std::getenv("DEVOURER_USB_BUS")) {
    const auto want_bus = static_cast<uint8_t>(std::strtoul(bus_env, nullptr, 0));
    const char *port_env = std::getenv("DEVOURER_USB_PORT");
    libusb_device **list = nullptr;
    ssize_t n = libusb_get_device_list(ctx, &list);
    for (ssize_t i = 0; i < n && dev_handle == NULL; ++i) {
      libusb_device_descriptor dd{};
      if (libusb_get_device_descriptor(list[i], &dd) != 0) continue;
      if (dd.idVendor != target_vid) continue;
      if (target_pid != 0 && dd.idProduct != target_pid) continue;
      if (libusb_get_bus_number(list[i]) != want_bus) continue;
      if (port_env != nullptr) {
        uint8_t ports[8];
        int pc = libusb_get_port_numbers(list[i], ports, sizeof(ports));
        std::string path;
        for (int p = 0; p < pc; ++p)
          path += (path.empty() ? "" : ".") + std::to_string(ports[p]);
        if (path != port_env) continue;
      }
      if (libusb_open(list[i], &dev_handle) == 0)
        logger->info("Opened device {:04x}:{:04x} on bus {} port {}", dd.idVendor,
                     dd.idProduct, want_bus, port_env ? port_env : "(any)");
    }
    if (list != nullptr) libusb_free_device_list(list, 1);
    if (dev_handle == NULL)
      logger->error("DEVOURER_USB_BUS={} PORT={} matched no device", want_bus,
                    port_env ? port_env : "(any)");
  }

  for (uint16_t pid : kRealtekProductIds) {
    if (dev_handle != NULL) break;
    if (target_pid != 0 && pid != target_pid) continue;
    dev_handle = libusb_open_device_with_vid_pid(ctx, target_vid, pid);
    if (dev_handle != NULL) {
      logger->info("Opened device {:04x}:{:04x}", target_vid, pid);
      break;
    }
  }
  /* DEVOURER_PID can name a PID not in kRealtekProductIds (e.g. 0x0120 for the
   * T2U Plus). Try that direct combination once before giving up. */
  if (dev_handle == NULL && target_pid != 0) {
    dev_handle = libusb_open_device_with_vid_pid(ctx, target_vid, target_pid);
    if (dev_handle != NULL) {
      logger->info("Opened device {:04x}:{:04x} (via DEVOURER_PID)",
                   target_vid, target_pid);
    }
  }
  if (dev_handle == NULL) {
    logger->error("Cannot find any supported device under VID {:04x}",
                  target_vid);
    libusb_exit(ctx);
    return 1;
  }

  logger->info("init-timing: demo.open_device = {} ms", ms_since_start());
  /* Claim-before-reset: the kernel's exclusive interface claim is the primary
   * guard against a second devourer driving this adapter — it returns BUSY, and
   * bailing on BUSY *before* the reset keeps a second launch from re-enumerating
   * the adapter out from under the process that already owns it. DEVOURER_SKIP_RESET
   * still suppresses the reset for a warm pickup (firmware already running).
   * See src/UsbOpen.h. */
  std::shared_ptr<devourer::UsbDeviceLock> usb_lock;
  rc = devourer::claim_interface_then_reset(
      dev_handle, 0, logger, std::getenv("DEVOURER_SKIP_RESET") == nullptr,
      usb_lock);
  logger->info("init-timing: demo.usb_reset = {} ms", ms_since_start());
  if (rc != 0) {
    /* BUSY => another process owns the adapter; any other error => open failed.
     * Either way, exit cleanly rather than asserting. */
    libusb_close(dev_handle);
    libusb_exit(ctx);
    return 1;
  }

  WiFiDriver wifi_driver(logger);
  auto rtlDevice = wifi_driver.CreateRtlDevice(dev_handle, ctx, usb_lock);
  if (!rtlDevice) {
    /* The factory returns null when the plugged chip's generation wasn't
     * compiled in (per-chip CMake options); it already logged which. */
    logger->error("No driver for this chip in this build — exiting");
    return 1;
  }
  logger->info("init-timing: demo.create_device = {} ms", ms_since_start());
  /* The BB-debug-port / queue-depth / thermal research helpers are Jaguar1-only,
   * so they live on RtlJaguarDevice rather than the IRtlDevice interface. The
   * whole block compiles out when Jaguar1 support isn't built; when it is, the
   * dynamic_cast yields nullptr for a Jaguar3 device, disabling them cleanly. */
#if defined(DEVOURER_HAVE_JAGUAR1)
  g_rtl_device = dynamic_cast<RtlJaguarDevice *>(rtlDevice.get());
  std::atomic<bool> qd_emitter_stop{false};
  std::thread qd_emitter;
  if (g_qd_poll_ms > 0 && g_rtl_device != nullptr) {
    logger->info("DEVOURER_QUEUE_POLL_MS={} — starting queue-depth poller",
                 g_qd_poll_ms);
    g_rtl_device->start_queue_depth_poller(g_qd_poll_ms);
    /* Self-driven emitter — ticks at the poll cadence so the queue snapshot
     * surfaces even when the RX hook is sparse (e.g. broken-RX 8814 cells).
     * Idempotent w.r.t. the poller; just reads the atomic snapshot. */
    qd_emitter = std::thread([&qd_emitter_stop]() {
      while (!qd_emitter_stop.load()) {
        if (g_rtl_device != nullptr) {
          auto q = g_rtl_device->get_queue_depth();
          printf("<devourer-queue>q1=0x%08x q2=0x%08x q3=0x%08x q4=0x%08x "
                 "q5=0x%08x\n", q[0], q[1], q[2], q[3], q[4]);
          fflush(stdout);
        }
        for (uint32_t slept = 0; slept < g_qd_poll_ms && !qd_emitter_stop.load();
             slept += 50) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      }
    });
  }
  std::atomic<bool> therm_emitter_stop{false};
  std::thread therm_emitter;
  if (g_thermal_poll_ms > 0) {
    logger->info("DEVOURER_THERMAL_POLL_MS={} warn_delta={} — starting thermal "
                 "poller", g_thermal_poll_ms, g_thermal_warn_delta);
    /* Thermal poller is a Jaguar1 (RtlJaguarDevice) feature; g_rtl_device is
     * null on Jaguar3. */
    if (g_rtl_device != nullptr)
      g_rtl_device->start_thermal_poller(g_thermal_poll_ms, g_thermal_warn_delta);
    therm_emitter = std::thread([&therm_emitter_stop]() {
      while (!therm_emitter_stop.load()) {
        if (g_rtl_device != nullptr) {
          auto t = g_rtl_device->get_thermal_snapshot();
          if (t.valid) {
            printf("<devourer-thermal>raw=%u baseline=%u delta=%+d status=%s\n",
                   t.raw, t.baseline, t.delta, ThermalBucket(t));
          } else {
            printf("<devourer-thermal>raw=%u baseline=none status=%s\n",
                   t.raw, ThermalBucket(t));
          }
          fflush(stdout);
        }
        for (uint32_t slept = 0;
             slept < g_thermal_poll_ms && !therm_emitter_stop.load();
             slept += 50) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      }
    });
  }
#endif /* DEVOURER_HAVE_JAGUAR1 */

  /* DEVOURER_RX_ENERGY_MS: frame-free RX energy / channel-busy telemetry — the
   * read side of DEVOURER_CW_TONE. Cross-generation (IRtlDevice::GetRxEnergy),
   * so it runs off the base device pointer, not the Jaguar1 downcast. The thread
   * sleeps one interval first (so its first read lands after bring-up completes,
   * not mid-init), then each interval reads GetRxEnergy() + drains the rolling
   * frame aggregate and emits one <devourer-energy> line. Concurrency caveat:
   * the FA/CCA reads share libusb with the RX bulk loop (like the thermal
   * poller) — keep the cadence conservative (>= a few hundred ms). */
  std::atomic<bool> energy_emitter_stop{false};
  std::thread energy_emitter;
  if (g_rx_energy_ms > 0) {
    logger->info("DEVOURER_RX_ENERGY_MS={} — starting RX energy telemetry",
                 g_rx_energy_ms);
    IRtlDevice *dev = rtlDevice.get();
    energy_emitter = std::thread([&energy_emitter_stop, dev]() {
      auto nap = [&](uint32_t ms) {
        for (uint32_t s = 0; s < ms && !energy_emitter_stop.load(); s += 50)
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
      };
      nap(g_rx_energy_ms); /* let bring-up finish before the first read */
      while (!energy_emitter_stop.load()) {
        RxEnergy e = dev->GetRxEnergy();
        RxAgg agg;
        {
          std::lock_guard<std::mutex> lk(g_rxagg_mu);
          agg = g_rxagg;
          g_rxagg = RxAgg{};
        }
        char fao[16], fac[16], cco[16], ccc[16], igi[16];
        auto u = [](char *b, bool v, uint32_t x) {
          if (v) std::snprintf(b, 16, "%u", x); else std::snprintf(b, 16, "-");
        };
        u(fao, e.valid_fa, e.fa_ofdm);
        u(fac, e.valid_fa, e.fa_cck);
        u(cco, e.valid_fa, e.cca_ofdm);
        u(ccc, e.valid_fa, e.cca_cck);
        if (e.valid_igi) std::snprintf(igi, 16, "%u", e.igi);
        else std::snprintf(igi, 16, "-");
        int rssi_mean = agg.n ? agg.rssi_sum / static_cast<int>(agg.n) : 0;
        int snr_mean = agg.n ? agg.snr_sum / static_cast<int>(agg.n) : 0;
        int evm_mean =
            agg.evm_n ? agg.evm_sum / static_cast<int>(agg.evm_n) : 0;
        printf("<devourer-energy>cca_ofdm=%s cca_cck=%s fa_ofdm=%s fa_cck=%s "
               "igi=%s frames=%u rssi_mean=%d rssi_max=%d snr_mean=%d "
               "snr_min=%d evm_mean=%d\n",
               cco, ccc, fao, fac, igi, agg.n, rssi_mean,
               agg.n ? agg.rssi_max : 0, snr_mean, agg.n ? agg.snr_min : 0,
               evm_mean);
        fflush(stdout);
        emit_nhm(e, -1);
        nap(g_rx_energy_ms);
      }
    });
  }

  /* Default channel 36 (5 GHz) for the 8812 reference. Override with
   * DEVOURER_CHANNEL=N env var (e.g. DEVOURER_CHANNEL=6 for busy 2.4 GHz). */
  int channel = 36;
  if (const char *ch_env = std::getenv("DEVOURER_CHANNEL")) {
    channel = std::atoi(ch_env);
    logger->info("DEVOURER_CHANNEL set — tuning to channel {}", channel);
  }
  /* RX bandwidth: 20 MHz by default. DEVOURER_BW=40|80 selects a wide monitor
   * channel (for receiving HT40 / VHT80 frames); DEVOURER_CHOFFSET picks the
   * secondary half (1 = secondary above the primary, 2 = secondary below).
   * DEVOURER_NB_BW=5|10 re-clocks the baseband to narrowband (Jaguar3 only). */
  ChannelWidth_t width = CHANNEL_WIDTH_20;
  uint8_t ch_offset = 0;
  if (const char *bw_env = std::getenv("DEVOURER_BW")) {
    int bw = std::atoi(bw_env);
    if (bw == 40 || bw == 80) {
      width = (bw == 40) ? CHANNEL_WIDTH_40 : CHANNEL_WIDTH_80;
      ch_offset = 1; // default: secondary channel above the primary
      if (const char *off_env = std::getenv("DEVOURER_CHOFFSET"))
        ch_offset = static_cast<uint8_t>(std::atoi(off_env));
      logger->info("DEVOURER_BW={} — {} MHz RX, channel-offset {}", bw, bw,
                   ch_offset);
    }
  }
  if (const char *nb = std::getenv("DEVOURER_NB_BW")) {
    int mhz = std::atoi(nb);
    if (mhz == 5)
      width = CHANNEL_WIDTH_5;
    else if (mhz == 10)
      width = CHANNEL_WIDTH_10;
    logger->info("DEVOURER_NB_BW={} — RX bandwidth {} MHz", nb, mhz);
  }

  /* DEVOURER_RX_SWEEP: live spectrum sweep. Run the (blocking) RX bring-up +
   * loop on a worker thread so the main thread is free to retune between energy
   * reads. StopRxLoop unblocks it on SIGINT. */
  if (!g_rx_sweep.empty()) {
    logger->info("DEVOURER_RX_SWEEP: {} bins, dwell {} ms — live spectrum map",
                 g_rx_sweep.size(), g_rx_sweep_dwell_ms);
    IRtlDevice *dev = rtlDevice.get();
    SelectedChannel first{static_cast<uint8_t>(g_rx_sweep[0]), ch_offset, width};
    std::thread rx([dev, first]() {
      try {
        dev->Init(packetProcessor, first);
      } catch (const std::exception &e) {
        printf("<devourer>RX-sweep bring-up failed: %s\n", e.what());
        fflush(stdout);
      }
    });
    /* Let bring-up complete before the first retune: a retune racing the
     * worker thread's init (FW download, DACK/IQK on Jaguar3) interleaves
     * register writes with calibration. The first RX frame proves bring-up
     * finished; a silent channel falls through after the 10 s cap. */
    for (int s = 0; s < 10000 && !g_devourer_should_stop && g_rx_count == 0;
         s += 50)
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    size_t bi = 0;
    /* Dwell retunes ride FastRetune (lean intra-band fast path on every
     * generation, internal full-path fallback on a band change);
     * DEVOURER_RX_SWEEP_FULL=1 forces the full SetMonitorChannel per dwell
     * (A/B escape hatch). */
    const bool sweep_full = std::getenv("DEVOURER_RX_SWEEP_FULL") != nullptr;
    while (!g_devourer_should_stop) {
      int ch = g_rx_sweep[bi % g_rx_sweep.size()];
      ++bi;
      const auto rt0 = std::chrono::steady_clock::now();
      if (sweep_full)
        dev->SetMonitorChannel(
            SelectedChannel{static_cast<uint8_t>(ch), ch_offset, width});
      else
        dev->FastRetune(static_cast<uint8_t>(ch), /*cache_rf=*/true);
      const long long retune_us =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - rt0)
              .count();
      {
        /* Drop frames captured before/during the retune — the dwell's frame
         * stats must only cover this bin. */
        std::lock_guard<std::mutex> lk(g_rxagg_mu);
        g_rxagg = RxAgg{};
      }
      for (uint32_t s = 0; s < g_rx_sweep_dwell_ms && !g_devourer_should_stop;
           s += 50)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      RxEnergy e = dev->GetRxEnergy();
      RxAgg agg;
      {
        std::lock_guard<std::mutex> lk(g_rxagg_mu);
        agg = g_rxagg;
        g_rxagg = RxAgg{};
      }
      char cco[16], ccc[16], fao[16], fac[16], igi[16];
      auto u = [](char *b, bool v, uint32_t x) {
        if (v) std::snprintf(b, 16, "%u", x); else std::snprintf(b, 16, "-");
      };
      u(cco, e.valid_fa, e.cca_ofdm);
      u(ccc, e.valid_fa, e.cca_cck);
      u(fao, e.valid_fa, e.fa_ofdm);
      u(fac, e.valid_fa, e.fa_cck);
      u(igi, e.valid_igi, e.igi);
      int rssi_mean = agg.n ? agg.rssi_sum / static_cast<int>(agg.n) : 0;
      int snr_mean = agg.n ? agg.snr_sum / static_cast<int>(agg.n) : 0;
      int evm_mean = agg.evm_n ? agg.evm_sum / static_cast<int>(agg.evm_n) : 0;
      printf("<devourer-energy>ch=%d cca_ofdm=%s cca_cck=%s fa_ofdm=%s "
             "fa_cck=%s igi=%s retune_us=%lld frames=%u rssi_mean=%d "
             "rssi_max=%d snr_mean=%d snr_min=%d evm_mean=%d\n",
             ch, cco, ccc, fao, fac, igi, retune_us, agg.n, rssi_mean,
             agg.n ? agg.rssi_max : 0, snr_mean, agg.n ? agg.snr_min : 0,
             evm_mean);
      fflush(stdout);
      emit_nhm(e, ch);
    }
    dev->StopRxLoop();
    if (rx.joinable())
      rx.join();
    dev->Stop();
    rc = libusb_release_interface(dev_handle, 0);
    if (rc != 0)
      logger->info("libusb_release_interface rc={}", rc);
    libusb_close(dev_handle);
    libusb_exit(ctx);
    return 0;
  }

  rtlDevice->Init(packetProcessor, SelectedChannel{
                                       .Channel = static_cast<uint8_t>(channel),
                                       .ChannelOffset = ch_offset,
                                       .ChannelWidth = width,
                                   });

  /* Stop the energy telemetry thread before de-init (it reads chip registers). */
  energy_emitter_stop = true;
  if (energy_emitter.joinable())
    energy_emitter.join();

  /* Clean chip de-init before dropping the interface (card-disable PWR_SEQ), so
   * the adapter re-enumerates instead of hanging its USB core. */
  rtlDevice->Stop();

  rc = libusb_release_interface(dev_handle, 0);
  assert(rc == 0);

  libusb_close(dev_handle);

  libusb_exit(ctx);

  return 0;
}
