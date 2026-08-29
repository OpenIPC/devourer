#pragma once

/* DeviceConfig — construction-time configuration, passed to
 * WiFiDriver::CreateRtlDevice (defaulted: CreateRtlDevice(handle) gives stock
 * behaviour). Fields are fixed for the device's lifetime; knobs that change
 * mid-session are runtime setters on IRtlDevice / the concrete device classes
 * (SetTxMode, SetTxPowerOffsetQdb, SetRxPathMask, SetCcaMode, ...).
 *
 * The example binaries populate this from DEVOURER_* environment variables via
 * examples/common/env_config.{h,cpp}; each field's `env:` tag names its
 * spelling in that translator and its value grammar. */

#include <array>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <string_view>

#include "AmpduMode.h"

namespace devourer {

/* 6-byte MAC address carrier (replaces the per-site uint8_t[6] + sscanf). */
struct MacAddr {
  std::array<uint8_t, 6> bytes{};
  const uint8_t *data() const { return bytes.data(); }
};

/* "aa:bb:cc:dd:ee:ff" (hex, case-insensitive) -> MacAddr; nullopt on any
 * malformed input. */
inline std::optional<MacAddr> parse_mac(std::string_view s) {
  MacAddr m;
  unsigned v[6];
  char tail;
  /* string_view is not NUL-terminated in general — bounce through a string. */
  std::string z(s);
  if (std::sscanf(z.c_str(), "%x:%x:%x:%x:%x:%x%c", &v[0], &v[1], &v[2], &v[3],
                  &v[4], &v[5], &tail) != 6)
    return std::nullopt;
  for (int i = 0; i < 6; ++i) {
    if (v[i] > 0xFF)
      return std::nullopt;
    m.bytes[i] = static_cast<uint8_t>(v[i]);
  }
  return m;
}

/* Jaguar-1 regulatory domain for the efuse TX-power limit tables. */
enum class Regulation : uint8_t { FCC = 0, ETSI = 1, MKK = 2, WW = 3 };

/* 8814A firmware-download path select. */
enum class Fwdl8814Path : uint8_t {
  KernelBracket, /* kernel-faithful bracket (default) */
  Rtw88,         /* legacy rtw88-mimic sequence (A/B fallback) */
};

/* 8822E DPDT antenna-transfer-switch routing (board A/B). */
enum class Dpdt8822eMode : uint8_t {
  EfemPinmux, /* kernel _efem_pinmux_config port — DPDT under HW TX/RX
                 control, both RX chains live (default) */
  Legacy,     /* improvised b5a6df7 static write: 0x4c[24] set, [22] clear
                 (TX-favoring, RX chain B antenna-less) */
  Bit24,      /* 0x4c[24] only (halmac WL_DPDT_SEL bit alone) */
  Skip,       /* leave the DPDT/pin-mux untouched */
};

/* USB RX-ring servicing strategy (UsbTransport::rx_loop). The default async
 * ring runs the consumer inline on the libusb pump thread before resubmitting
 * each URB, so a slow consumer under a burst can stall resubmit and collapse
 * the armed-URB depth. The other modes trade that coupling off for A/B study:
 * a blocking single-buffer reference, and two buffer-pool variants that re-arm
 * the ring with a spare buffer before consuming the received one. */
enum class RxMode : uint8_t {
  Async,       /* current: consume inline, then resubmit the same URB */
  Sync,        /* blocking single-buffer bulk-IN read loop (legacy reference) */
  ReorderPool, /* swap a spare buffer onto the wire, resubmit, then consume */
  SpscFat,     /* pool + SPSC hand-off to a consumer thread (deep absorption) */
  Decoupled,   /* naive worker-thread hand-off (known-bad A/B control) */
};

/* Hardware retry rate-fallback control for injected frames. The HalMAC
 * generations default to the firmware's fallback ladder, whose rate-space is
 * the descriptor's RA group (rateid_for_mgn, RateDefinitions.h) — with the
 * family-correct group the ladder is MCS-native, witness-measured per copy
 * (tests/retry_ladder_probe.sh): 8822B MCS3 -> MCS2 -> MCS1 -> MCS0 (pure),
 * 8822C MCS3 x4 -> MCS2 -> 6M floor (CCK floor at 2.4 GHz; VHT steps
 * M7 -> M4 -> M1 -> M0 -> 6M). Jaguar1's fw does not step at all — every
 * retry at the descriptor rate regardless of this knob (8821AU-measured).
 * Off pins every retry at the descriptor DATARATE (DISDATAFB / DISABLE_FB =
 * 1; measured: 1,200/1,200 retried frames re-aired at the original rate) —
 * for constant-rate links where a step-down retry costs extra airtime. A
 * DATA_RTY_LOWEST_RATE floor form was measured and REJECTED: the fw
 * reinterprets the bound inside the RA-group rate space, wandering retries
 * into VHT rates (final_rate 45 = VHT1SS_MCS1 on an HT frame) with a 20x
 * retry inflation — the field is not a plain DESC_RATE bound on this fw. */
enum class RetryFallback : uint8_t { Default, Off };

/* What the spsc-fat ring does when its buffer pool runs dry (consumer behind
 * under sustained overload). The choice decides which side of the hardware-ARQ
 * contract survives overload — the chip ACKs on FIFO admission, so anything
 * the HOST discards afterwards is a loss the TX peer believes delivered. */
enum class PoolExhaust : uint8_t {
  Backpressure, /* park the URB unarmed: the chip FIFO fills and the chip
                 * declines further ACKs — congestion loss stays ARQ-visible
                 * (bench: 14,214/14,214 forced drops reported ok=0 and
                 * retried). Ring re-arms as the consumer returns buffers. */
  Drop,         /* keep the ring armed and discard the payload — the chip
                 * already ACKed it, so the TX peer logs it delivered and
                 * never retries (counted as rx.ring pool_dropped). Smoother
                 * under overload, but only for consumers that accept silent
                 * loss (no ARQ / delivery accounting on top). */
};

struct DeviceConfig {
  /* ---- RX ------------------------------------------------------------- */
  struct Rx {
    /* env: DEVOURER_RX_KEEP_CORRUPTED — pass frames that fail the 802.11 FCS
     * (CRC32) or decryption-ICV check up to the host instead of dropping them
     * at the WMAC filter (sets RCR ACRC32|AICV). Jaguar1 + Jaguar2. */
    bool keep_corrupted = false;
    /* env: DEVOURER_TX_WITH_RX — Jaguar3 only: keep the RX filters open and
     * enable the RX path during a TX (InitWrite) bring-up so StartRxLoop can
     * run on the same handle. Must be decided before InitWrite (retrofitting
     * RX onto a TX-only bring-up is unreliable on this generation). */
    bool enable_with_tx = false;
    /* env: DEVOURER_RX_CSI_MASK — "<f_lo>[-<f_hi>][/wgt]" MHz: de-weight a
     * frequency range in the RX equalizer's per-tone CSI mask (ToneMask.h).
     * Applied when the RX loop starts; a channel switch reverts it. */
    std::optional<std::string> csi_mask;
    /* env: DEVOURER_RX_NBI — "<f_mhz>": arm the narrowband-interference notch
     * at one in-channel frequency (LUT-quantized). Same lifecycle as
     * csi_mask. */
    std::optional<std::string> nbi;
    /* env: DEVOURER_RX_PATHS — Jaguar1 RX-chain mask spec: "0xNN" applies one
     * mask (sticky across channel sets, see SetRxPathMask); the toggle form
     * "0xAA:0xBB[:0xCC]@<ms>" cycles masks on a timer thread. */
    std::optional<std::string> path_spec;
    /* env: DEVOURER_RX_URBS — Jaguar1 async bulk-IN URB queue depth
     * (default 8, clamped to >= 1). */
    std::optional<int> urbs;
    /* env: DEVOURER_RX_URB_BYTES — per-URB bulk-IN buffer size on the 11ac
     * generations (default 16384, clamped to >= 4096). Keep it at 16 KB unless
     * you also raise the device-side RX aggregation thresholds: some MediaTek
     * Android xhci hosts never complete a bulk-IN read larger than 16 KB
     * (LIBUSB_ERROR_TIMEOUT forever, zero RX — OpenIPC/PixelPilot#6, fixed by
     * #19, regressed by the #213 transport split), and the device-side
     * aggregation caps (J1 0x3/4KB-pages, J2/J3 0x3) mean a bigger URB never
     * fills past 16 KB anyway. Kestrel ignores this knob: the 8852C RXAGG
     * LEN_TH is ~20 KB and its ring must hold a full aggregate (32 KB). */
    std::optional<int> urb_bytes;
    /* env: DEVOURER_RX_MODE — USB RX-ring servicing strategy (RxMode above).
     * Default async = the current inline-consume-then-resubmit ring. The other
     * modes are A/B / fix vehicles for the burst-starvation study. */
    RxMode rx_mode = RxMode::Async;
    /* env: DEVOURER_RX_POOL_SPARE — extra RX buffers beyond the URB count for
     * the buffer-pool modes (ReorderPool/SpscFat): a deeper pool absorbs a
     * burst backlog host-side instead of overflowing the chip RX FIFO. 0 =
     * pool of exactly urbs buffers (no spare). Ignored by async/sync. */
    int pool_spare = 0;
    /* env: DEVOURER_RX_POOL_EXHAUST — "backpressure" (default) | "drop":
     * the SpscFat pool-exhaustion policy (see PoolExhaust above). Only read
     * by SpscFat; the other modes never drop host-side (async/reorder degrade
     * to inline consume, which backpressures the chip by construction). */
    PoolExhaust pool_exhaust = PoolExhaust::Backpressure;
    /* env: DEVOURER_RX_RING_MS — cadence (ms) for the diagnostic rx.ring
     * telemetry event (armed-URB depth, min depth in the window, resubmit
     * failures, max inline-consume latency). Unset/0 = no telemetry (the
     * default RX path is then byte-for-byte unchanged). */
    std::optional<int> ring_ms;
    /* env: DEVOURER_8821C_NO_PHYST (inverted) — 8821C: prepend the 32-byte
     * PHY-status to RX frames (per-frame RSSI/SNR/EVM). Disable only for the
     * leanest possible RX path. */
    bool phy_status_8821c = true;
    /* env: DEVOURER_RX_NOISE_FLOOR — opt-in active/frame-free ABSOLUTE noise
     * floor (dBm) in GetRxEnergy/GetRxQuality (RxEnergy.abs_noise_floor_dbm).
     * OFF by default: the vendor idle-noise measurement adds ~10 ms of USB
     * round-trips. Jaguar2 measures it live (wedge-free HW idle-noise report);
     * Jaguar1 8812A/8821A measure it RX-idle (a CAL bracket); Jaguar3 and others
     * leave it invalid (no vendor path). */
    bool abs_noise_floor = false;
    /* env: DEVOURER_IGI — Jaguar2 fixed initial-gain index override, 7 bits
     * (unset = 0x40, the FA-rate-validated default). */
    std::optional<uint8_t> igi;
    /* env: DEVOURER_ACK_RESPONDER=<unicast mac> — arm the hardware ACK
     * responder at the end of bring-up (src/AckResponder.h): the MAC
     * auto-ACKs unicast frames addressed to this MAC while monitor RX and
     * injection continue unchanged. Runtime equivalent: SetAckResponder /
     * ClearAckResponder. OPT-IN: makes a passive monitor transmit. */
    std::optional<MacAddr> ack_responder;
  } rx;

  /* ---- TX ------------------------------------------------------------- */
  struct Tx {
    /* env: DEVOURER_TX_EP — bulk-OUT endpoint override (unset = first
     * descriptor-walked bulk-OUT endpoint, fallback 0x02). */
    std::optional<uint8_t> ep;
    /* env: DEVOURER_TX_TIMEOUT_MS — TX bulk-OUT transfer timeout (unset =
     * USB_TIMEOUT). */
    std::optional<unsigned> timeout_ms;
    /* env: DEVOURER_TX_LEGACY_8812_DESC — 8814A: keep the legacy 8812-style
     * TX-descriptor bits instead of the 8814-native layout. */
    bool legacy_8812_desc = false;
    /* env: DEVOURER_TX_PWR — Jaguar2: flat TXAGC index (0..63) applied at the
     * end of InitWrite via SetTxPowerIndexOverride (composes with the offset
     * knob and shows up in GetTxPowerState). Debug/SDR-visibility knob. */
    std::optional<int> power_index;
    /* env: DEVOURER_TX_RF_BW — Jaguar3 40 MHz: force the 0x9b0 TX_RF_BW field
     * (2 bits; kernel-stall study knob). */
    std::optional<uint8_t> rf_bw;
    /* env: DEVOURER_CW_TONE / DEVOURER_CW_TONE_GAIN — radiate a bare RF LO
     * carrier (MP single-tone) after bring-up; gain = RF 0x00[4:0], 0..31.
     * Runtime equivalent: StartCwTone/StopCwTone on the concrete device. */
    bool cw_tone = false;
    uint8_t cw_tone_gain = 0;
    /* env: DEVOURER_TX_RETRY_LIMIT — per-frame hardware retry limit (0..63;
     * Kestrel ceiling 62 — its attempts-counting WD field folds +1). Maps to
     * the TX descriptor DATA_RETRY_LIMIT / RTS_DATA_RTY_LMT field on the
     * 11ac generations and wd_info DATA_TXCNT_LMT on Kestrel. 0 = no retries
     * (WFB default: FEC provides reliability, not MAC retries). On a busy
     * half-duplex link retries flood the air and blind the receiver.
     * Hardware-ARQ (SetAckResponder + unicast TA, docs/scheduled-mac.md)
     * needs a nonzero value. Inert on the 8814A die only (vendor
     * DATA_RETRY_LIMIT=0 carve-out kept pending its bench). */
    int retry_limit = 0;
    /* env: DEVOURER_ACK_TIMEOUT_US — hardware ACK response window in µs
     * (1..255, clamped), the hardware-ARQ RANGE lever: the MAC writes a
     * frame off (and retries) when no ACK is counted within this window,
     * and round-trip propagation eats ~6.7 µs per km. ONE default, 128 µs,
     * programmed identically on every generation at bring-up — the same
     * knob value means the same range budget (~15 km round trip) no matter
     * which die is plugged. 128 is the vendor's interop-blessed J1/J2
     * value and covers the slowest narrowband ACK in the tree (the 5 MHz
     * per-bandwidth vendor value is 117 µs), so it also replaces the
     * per-chip / per-bandwidth vendor defaults (which ranged 33..128 µs
     * and made hardware-ARQ range silently die-dependent). The register:
     * REG_ACKTO 0x640 on the 11ac generations, R_AX_RSP_CHK_SIG 0xCC00
     * byte0 on Kestrel; the CTS window (REG_CTS2TO 0x641) is separate and
     * untouched. Sizing: ~6.7 µs x round-trip km + ~50 µs ACK flight and
     * detection margin; a longer window is NOT free — every retry of a
     * LOST frame waits the full window, measured (dead RA, retry 8, max
     * duty): 2719 write-offs/8 s at 33 µs vs 2015 at 128 vs 1507 at 255.
     * Bench proof the register gates the ARQ verdict: at 8 µs (below the
     * ACK's flight time) retries pin at the limit with 0% ok against a
     * live responder; at 128/255 the responder cell runs 100% ok,
     * retries ~0. */
    int ack_timeout_us = 128;
    /* env: DEVOURER_TX_RETRY_FALLBACK — "off" | unset. Unset = the firmware
     * fallback ladder with its own floor (the current behaviour, descriptors
     * byte-identical). "off" disables per-retry rate fallback (DISDATAFB /
     * DISABLE_FB = 1): retries re-air at the descriptor rate. See the
     * RetryFallback enum for why there is no floor form. */
    RetryFallback retry_fallback = RetryFallback::Default;
    /* env: DEVOURER_TX_USB_AGG — USB TX aggregation: max frames packed into
     * one bulk-OUT URB by send_packets (0 = off, the default: send_packets
     * degrades to a per-frame loop and every TX path is byte-identical to
     * before the knob existed). Clamped to the 8-bit agg-num field (255; 64
     * on Jaguar1) and the vendor packing rules (see src/TxAggPlan.h). USB
     * only; the PCIe rings take frames individually. */
    unsigned usb_agg_max = 0;
    /* env: DEVOURER_TX_REPORT — per-frame TX-status reports (src/TxReport.h):
     * sets SPE_RPT in every data TX descriptor so the firmware answers each
     * transmission with a CCX report (delivered/retry-drop + hardware retry
     * count + queue time + final rate), decoded off the C2H RX path into
     * `tx.report` events. The TX-side link sensor. On the HalMAC chips the
     * descriptor SW_DEFINE also carries a rotating 8-bit tag the report
     * echoes (per-frame correlation). Default off (descriptors
     * byte-identical). Needs an RX loop to deliver the C2H reports.
     *
     * Value = sampling divisor N: 1 requests a report on EVERY frame, N > 1
     * on every Nth (0..255). The CCX emission path saturates at ~1.3–1.4 k
     * reports/s (docs/scheduled-mac.md), so above ~1.25 k fps pick
     * N >= fps/1300 and coverage of the SAMPLED frames stays deterministic
     * instead of load-collapsing. HalMAC dies stamp the SW_DEFINE tag on
     * every frame regardless of sampling, so consecutive received-report
     * tags differ by exactly N — any other delta is a dropped report
     * (consumers must know N; tests/txrpt_coverage_attrib.py --sample-n). */
    int report = 0;
    /* env: DEVOURER_TX_AMPDU_MODE="tid/maxnum[/density[/noack[/maxtime_hex]]]"
     * — arm the first-class A-MPDU TX mode (src/AmpduMode.h) at the end of
     * bring-up: mark data frames aggregatable and program the MAC pacing
     * registers. The product form of the DEVOURER_TX_QSEL / DEVOURER_TX_AMPDU
     * spike knobs (which still compose on top). Runtime equivalent:
     * SetAmpduMode. Unset = off (byte-identical). Pair with a deep feed
     * (send_packets / DEVOURER_TX_THREADS) for the goodput win. */
    std::optional<AmpduMode> ampdu;
  } tx;

  /* ---- Beamforming (bring-up-time arming; see docs/beamforming-*.md) --- */
  struct Bf {
    /* env: DEVOURER_BF_ARM_SOUNDER — arm the MAC's hardware sounding engine
     * (beamformer side) so a descriptor-marked NDPA is followed by a
     * hardware-generated NDP. */
    bool arm_sounder = false;
    /* env: DEVOURER_BF_ARM_SOUNDER=<mac> — Jaguar2/3 also program the self-MAC
     * (0x0610) the sounding engine matches the NDPA TA against. Unset = bare
     * arming (Jaguar-1 semantics: MAC left unprogrammed). */
    std::optional<MacAddr> sounder_self_mac;
    /* env: DEVOURER_BF_ARM_BFEE=<mac> — arm the hardware CSI responder
     * (beamformee side) for NDPA+NDP from this beamformer MAC. */
    std::optional<MacAddr> beamformee_of;
    /* env: DEVOURER_BF_ARM_BFEE_MU — upgrade the responder to MU beamformee
     * (per-subcarrier delta-SNR report). Jaguar2/3, needs beamformee_of. */
    bool mu = false;
    /* env: DEVOURER_BF_TXBF=<mac> — Jaguar3: configure the beamformer entry
     * for closed-loop TX-beamforming apply toward this peer (the apply itself
     * gates on the first CBR seen by the RX loop; needs rx.enable_with_tx). */
    std::optional<MacAddr> txbf_peer;
    /* env: DEVOURER_TX_NDPA — 0 = off; 1 = mark every injected frame NDPA
     * (self-sounding capture); N>1 = every Nth frame (Jaguar3 periodic
     * re-sounding; Jaguar1/2 treat any nonzero as on). */
    int ndpa_period = 0;
  } bf;

  /* ---- Bring-up / calibration experiment gates ------------------------ */
  struct Tuning {
    bool skip_iqk = false;          /* env: DEVOURER_SKIP_IQK (J2, J3) */
    bool force_iqk = false;         /* env: DEVOURER_FORCE_IQK (J1) */
    bool disable_iqk = false;       /* env: DEVOURER_DISABLE_IQK (J1) */
    bool skip_txpwr = false;        /* env: DEVOURER_SKIP_TXPWR (J1, J2) */
    bool skip_txgapk = false;       /* env: DEVOURER_SKIP_TXGAPK (8822E) */
    bool skip_trx_reassert = false; /* env: DEVOURER_SKIP_TRX_REASSERT (J2) */
    bool skip_rfe_init = false;     /* env: DEVOURER_SKIP_RFEINIT (J2) */
    bool skip_coex = false;         /* env: DEVOURER_SKIP_COEX (J2) */
    bool skip_dig = false;          /* env: DEVOURER_SKIP_DIG (J2) */
    /* env: DEVOURER_THERMAL_TRACK — Jaguar2 thermal TX-power tracking
     * (default ON; "0" disables). A ~2 s tick reads the RF 0x42 thermal
     * meter, computes the delta vs the efuse baseline, and writes the vendor
     * MIX_MODE swing compensation (0xc94/0xe94 TXAGC + 0xc1c/0xe1c BB scale)
     * so on-air power holds flat as the PA heats over a sustained TX link. */
    bool thermal_track = true;
    /* env: DEVOURER_FASTRETUNE_FW — FastRetune firmware fast path (H2C 0x1D
     * SINGLE_CHANNELSWITCH_V2, the switch the vendor drivers gate behind
     * rtw_ch_switch_offload) on the Jaguar2 dies (8822B, 8821C) and the
     * Jaguar3 dies (8822C, 8822E): 0 = off (software compose path), 1 = firmware switch for
     * intra-band 20/40 MHz hops, 2 = additionally accept cross-band hops
     * (the firmware reprograms the band block; TXAGC baseline stays the
     * bring-up band's — active power knobs re-fold). Bench + protocol:
     * docs/experiments/kernel-channel-switch-offload.md. */
    int fastretune_fw = 0;
    /* env: DEVOURER_KFR_OFLD — Kestrel FastRetune firmware IO-offload (mac_ax
     * fwofld.c): batch the whole same-sub-band hop (RF18/0xcf channel set + BB
     * reset + fixed TX power) into ONE FW_OFLD/CMD_OFLD_REG H2C the on-chip fw
     * replays, collapsing ~20 USB register round-trips into one bulk-OUT
     * (~9 ms -> ~0.15 ms host-side). Default 1 (on); 0 forces the direct
     * write-only path (self-paces the ~1.5 ms RF synth settle instead of
     * returning before it). No rtw89 firmware channel-switch H2C exists, so
     * this offloads the raw register writes, not a fw switch primitive. The
     * hop returns before the synth settles — the caller honours a ~1.5 ms
     * settle before TX (the demos' admission window). 8852B only. */
    int kestrel_fastretune_ofld = 1;
    /* env: DEVOURER_FW_TABLE_OFLD — Jaguar2 init-time firmware register
     * IO-offload (HalMAC cfg_param): route the static BB/AGC/RF phy-table write
     * stream through one FW_OFFLOAD/CFG_PARAM H2C per ~160-command batch that
     * the firmware replays on-chip, collapsing the per-init USB register control
     * transfers into a handful of bulk transfers. Bit flags: 1 = BB + AGC
     * tables, 2 = also the RF radio tables (RF_W, routed through the firmware's
     * RF write path). 0 = off (direct writes, byte-identical to before). Default
     * 0 — init is higher blast-radius than a hop. Jaguar3 (8822C/E) is a
     * follow-up: the fw replays cfg_param but lazily, and its rsvd-page download
     * stalls per batch. */
    int fw_table_offload = 0;
    /* env: DEVOURER_DIS_CCA — MAC carrier-sense disable at bring-up, every
     * generation (primary CCA 0x520[14] + EDCCA [15]; Jaguar1 additionally
     * programs/parks its BB EDCCA thresholds 0x8a4 — its init table leaves
     * them at never-trigger, and the explicit apply is what makes EDCCA
     * real on that family): injected/beacon TX stops deferring to a
     * busy channel and punches through co-channel traffic. Default false =
     * carrier-sense + EDCCA ENABLED (the standards-compliant default) on
     * Jaguar1/2/3 and the Kestrel 8852C; the Kestrel 8852B TX bring-up
     * clears the gates and warns pending its measurement arm
     * (tests/kestrel_cca_default_check.sh). Runtime equivalent: SetCcaMode.
     * Default-on on the streamtx FPV downlink (the link owns the
     * channel). */
    bool disable_cca = false;
    /* env: DEVOURER_TXPKT_STEP_QDB — Jaguar3 per-packet power-bank step size
     * in quarter-dB: the dB weight of one 0x1e70 offset-index step
     * (SetTxPacketPowerOffsetQdb / radiotap DBM_TX_POWER).
     * Default 4 (= 1 dB), the vendor-stated step for the 22C class ("each
     * tx_pwr_ofst step will be 1dB", phydm.h bb_ram block); override for
     * bench slope calibration (tests/txpkt_pwr_ofset_onair.sh). */
    int txpkt_step_qdb = 4;
    /* env: DEVOURER_RFE — Jaguar2 RFE type override (antenna/LNA switch
     * variant; unset = efuse, blank efuse falls back per vendor). */
    std::optional<uint8_t> rfe_type;
    /* env: DEVOURER_NB_DAC — 5/10 MHz divider-mapping experiments only.
     * Jaguar3: force the 3-bit DAC-divider code (0x9b4[10:8]). Jaguar2:
     * force the 0x8ac DAC clock field — bits [1:0] -> 0x8ac[21:20],
     * bit 2 -> 0x8ac[28]. RTL8733B: force the 0x9b4[10:8] code the
     * experimental narrowband path writes (stub default 1/2). */
    std::optional<uint8_t> nb_dac;
    /* env: DEVOURER_NB_ADC — 5/10 MHz divider-mapping experiments only.
     * Jaguar2: force the 0x8ac ADC clock field — bits [1:0] -> 0x8ac[9:8],
     * bit 2 -> 0x8ac[16]. RTL8733B: force the 0x9f0[3:0] ADC-clock code
     * (stub default 0xa/0xb). */
    std::optional<uint8_t> nb_adc;
    /* env: DEVOURER_XTAL_CAP — crystal-cap trim code applied at the end of
     * bring-up (IRtlDevice::SetXtalCap). The CFO lever for narrowband at the
     * edge of its budget; unset = efuse/default. Raw code, 0..0x3f (Jaguar1/2)
     * or 0..0x7f (Jaguar3). */
    std::optional<uint8_t> xtal_cap;
    /* env: DEVOURER_CFO_TRACK — closed-loop CFO tracking (Jaguar3): auto-trim
     * the crystal cap from the RX-measured per-frame CFO, tracking warm-up
     * drift so narrowband stays locked (issue #217). Off by default. */
    bool cfo_track = false;
    /* env: DEVOURER_REGULATION — Jaguar1 regulatory domain override for the
     * TX-power limit tables (unset = efuse). */
    std::optional<Regulation> regulation;
    /* env: DEVOURER_ENABLE_TXPWR_BY_RATE — Jaguar1: apply the efuse PG-table
     * per-rate offsets on top of the base index. */
    bool txpwr_by_rate = false;
    /* env: DEVOURER_PHYDM_WATCHDOG — Jaguar1: run the optional periodic phydm
     * watchdog (FA stats + DIG) thread. */
    bool phydm_watchdog = false;
    /* env: DEVOURER_8814_FWDL / DEVOURER_8814_FWDL_CHUNK — 8814A firmware
     * download path + reserved-page chunk size (validated 64..MAX at the use
     * site; out-of-range values are ignored). */
    Fwdl8814Path fwdl_8814 = Fwdl8814Path::KernelBracket;
    std::optional<uint32_t> fwdl_8814_chunk;
    /* env: DEVOURER_DPDT_MODE=efem|legacy|bit24|skip — 8822E DPDT
     * antenna-transfer-switch routing for board A/B. Default (efem) ports the
     * kernel eFEM pin-mux so the DPDT follows TX/RX in hardware (both RX chains
     * live); the others are the pre-fix static routes. See docs/8822e-quirks.md. */
    Dpdt8822eMode dpdt_8822e = Dpdt8822eMode::EfemPinmux;

    /* env: DEVOURER_TEARDOWN_POWER_DOWN=0|1 — Jaguar1: run the die's
     * card-disable power sequence at Stop()/destruction (default on), instead
     * of leaving the chip in ACT with its RF front end live indefinitely.
     *
     * Turning it OFF is what a post-mortem needs: a powered-down chip answers
     * every register read with the CARDEMU fill (0xEA...), so the state a
     * session left behind is only inspectable (examples/chipstate) while the
     * chip is still up. It is also the A/B lever for measuring what the
     * power-down is actually worth. */
    bool teardown_power_down = true;
  } tuning;

  /* ---- Diagnostics output --------------------------------------------- */
  struct Debug {
    /* env: DEVOURER_DUMP_CANARY — dump the per-chip BB/MAC/RF canary register
     * set after every channel set / fast retune (kernel-parity diffing). */
    bool dump_canary = false;
    /* env: DEVOURER_BB_DUMP — Jaguar2: full BB/RF dump in rtw_proc format. */
    bool bb_dump = false;
    /* env: DEVOURER_EFUSE_DUMP — Jaguar2: dump the efuse logical map. */
    bool efuse_dump = false;
    /* env: DEVOURER_LOG_WRITES — emit every vendor register write as a
     * debug.wreg event (addr/width/val mirror tests/decode_wseq.py). */
    bool log_writes = false;
    /* env: DEVOURER_LOG_TXPWR — Jaguar1: log the per-channel/per-rate TX-power
     * derivation. */
    bool log_txpwr = false;
    /* env: DEVOURER_HOP_PROF — per-stage FastRetune timing on stderr. */
    bool hop_prof = false;
    /* env: DEVOURER_GAINTAB_DBG — 8822E: log the TXGAPK gain-table backup. */
    bool gaintab_dbg = false;
    /* env: DEVOURER_REPLAY_WSEQ — golden-init replay (Jaguar2): path to a
     * register write-sequence file (lines: "<addr_hex> <width 1|2|4>
     * <val_hex>", e.g. extracted from a kernel-driver usbmon capture).
     * Applied verbatim at the end of Init, OVERRIDING devourer's own
     * configuration — a debugging lever for hardware-diffing devourer
     * against the vendor driver's end state. */
    std::string replay_wseq;
    /* env: DEVOURER_TX_QSEL — EXPERIMENTAL (A-MPDU spike, tests/ampdu_spike):
     * override the data TX-descriptor QSEL (default 0x12 = MGMT queue, the
     * monitor-inject convention). Data-queue values are the TID (0..7);
     * hardware A-MPDU formation is expected only on data queues. */
    std::optional<uint8_t> tx_qsel;
    /* env: DEVOURER_TX_RATEID — EXPERIMENTAL (retry-ladder probe,
     * tests/retry_ladder_probe.sh): override the TX-descriptor RATE_ID (the
     * RA group). The group defines the rate-space the firmware's retry
     * fallback ladder walks — the inherited inject value (9 = VHT_2SS on
     * the 8822C-era table; 8 = the CCK-only B group) is why an MCS frame's
     * retries fall back through legacy OFDM. Vendor group ids:
     * reference ieee80211.h RATEID_IDX_* (4 = GN 2SS, 5 = GN 1SS, ...). */
    std::optional<uint8_t> tx_rateid;
    /* env: DEVOURER_TX_AMPDU="max[/density]" — EXPERIMENTAL (A-MPDU spike):
     * set AGG_EN=1 + MAX_AGG_NUM (1..0x1f; hardware units of 2 MPDUs) +
     * AMPDU_DENSITY (0..7, default 0) on every data TX descriptor, asking the
     * MAC to aggregate co-queued same-RA/TID frames into A-MPDUs. Whether the
     * hardware honours it for host-pushed USE_RATE frames is exactly what the
     * spike measures. Pair with tx_qsel, QoS-Data frames (txdemo
     * DEVOURER_TX_QOS_DATA) and USB TX agg for co-queueing. */
    std::optional<uint8_t> tx_ampdu_max;
    uint8_t tx_ampdu_density = 0;
    /* Optional third component of DEVOURER_TX_AMPDU ("max[/density[/rty]]"):
     * override the per-frame data retry limit (RTY_LMT_EN stays 1). The
     * A-MPDU spike found the MAC retries an aggregate to the limit waiting
     * for a BlockAck no monitor-mode receiver sends — rty=0 airs each
     * aggregate exactly once (the broadcast/no-ack flavor). */
    std::optional<uint8_t> tx_ampdu_rty;
    /* env: DEVOURER_KESTREL_FWLOG — Kestrel diagnostic: route the firmware log
     * to C2H packets in InitWrite (mac_fw_log_cfg output=C2H). A decisive probe
     * of whether async packet-C2H (rpkt_type=10) reaches the host at all. */
    bool kestrel_fw_log = false;
    /* env: DEVOURER_KESTREL_CCA_ON — Kestrel experiment: leave the CMAC CCA
     * medium-busy gates enabled (carrier-sense TX) instead of clearing them,
     * to test whether the ported RX-DCK calibration fixed the perpetual-busy. */
    bool kestrel_cca_on = false;
    /* env: DEVOURER_KESTREL_TRIGGER_F2P — force SendTrigger down the fw F2P_TEST
     * command instead of host-injecting the raw Basic Trigger through the mgmt
     * TX path. F2P is an MP-only entry the shipped client fw silently drops (it
     * airs nothing on-air), so the default (false) host-injects a real Basic
     * Trigger frame that actually leaves the antenna. Set true only to reproduce
     * the F2P no-op for comparison. */
    bool kestrel_trigger_f2p = false;
  } debug;

  /* ---- USB / process environment -------------------------------------- */
  struct Usb {
    /* env: TMPDIR — directory for the
     * devourer-usb-*.lock files. Empty = "/tmp". */
    std::string lock_dir;
    /* env: DEVOURER_RX_ZEROCOPY — allocate the async RX ring from kernel DMA
     * memory (libusb_dev_mem_alloc / USBDEVFS_ALLOC) so incoming frames DMA
     * straight into the userspace buffer, eliminating the per-URB usbfs copy on
     * reap. Linux + capable HCD only; the alloc falls back to heap buffers (the
     * historical copy-on-reap path) when unsupported, so leaving it on is safe.
     * 1 = on (default), 0 = force the heap path. */
    bool rx_zerocopy = true;
  } usb;

  /* ---- PCIe (DEVOURER_PCIE builds; see src/PcieTransport.h) ------------ */
  struct Pcie {
    /* env: DEVOURER_PCIE_RX_POLL_US — RX ring hardware-index poll interval in
     * microseconds (unset = the transport default, 200). Demo-side only: the
     * demos fold it into PcieTransport::Config before Open(); device selection
     * (DEVOURER_PCIE_BDF) is likewise demo-local, like USB device selection. */
    std::optional<int> rx_poll_us;
  } pcie;
};

} // namespace devourer
