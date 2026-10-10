/* Aggregate adapter-capability report — what the opened radio actually IS and
 * what it can do, resolved from the chip identity devourer already determines at
 * construction (SYS_CFG2 chip-id + USB PID + EFUSE RF-type). A dependent app
 * (OpenIPC-FPV tooling, a link manager, a test harness) can enumerate the RF
 * frequency coverage, channel widths, spatial-stream / chain count, and the
 * per-family feature levers (per-packet TX power, narrowband, fast retune)
 * without hardcoding a chip table of its own or scraping bring-up logs.
 *
 * This is the identity+radio superset of the narrower GetTxCaps (modulation
 * features) and GetTxPowerCaps (power-knob range), which it composes by value so
 * there is one source of truth per fact. Like those, it is STATIC — resolved at
 * construction, safe from any thread, callable before Init/InitWrite. The live
 * "which antennas look connected" question is deliberately NOT here (it needs
 * traffic); see IRadio::GetActiveRxPaths / ActiveRxPaths in RxQuality.h.
 *
 * FREQUENCY COVERAGE. The 5 GHz synthesizer on these parts tunes well past the
 * regulatory UNII channels (the vendor rtl88x2bu "monitor_chan_override" hack:
 * chan 16..253, freq = 5000 + 5*chan, ~5080..6165 MHz). devourer drives that
 * whole span in monitor mode; `tune_5g` reports the tunable range while
 * `characterized_5g` reports the sub-range backed by the generated txpwr_lmt /
 * EFUSE PG tables (outside it, per-channel constants and TX power are
 * extrapolated from the nearest characterized channel — the radio still tunes,
 * but power is uncalibrated). Regulatory compliance is the CALLER's problem: the
 * library enforces none.
 */
#ifndef DEVOURER_ADAPTER_CAPS_H
#define DEVOURER_ADAPTER_CAPS_H

#include <cstdint>

#include "TxCaps.h"
#include "TxPower.h"

namespace devourer {

enum class ChipGeneration : uint8_t {
  Unknown = 0,
  Jaguar1,
  Jaguar2,
  Jaguar3,
  Rtl8733b, /* HALMAC 87xx 802.11n: RTL8731BU / RTL8733BU */
  Kestrel,  /* Wi-Fi 6 / 802.11ax (RTL8852BU/8852CU) */
  /* MediaTek MT7662 MAC (MT7612U / MT7662U, 2T2R 11ac USB) — the first
   * non-Realtek generation. Register width, the vendor-request opcodes and the
   * in-band MCU plane all differ; nothing that switches on this value may
   * assume a Realtek register map. */
  Mt7612u
};

inline const char *generation_name(ChipGeneration g) {
  switch (g) {
  case ChipGeneration::Jaguar1:
    return "jaguar1";
  case ChipGeneration::Jaguar2:
    return "jaguar2";
  case ChipGeneration::Jaguar3:
    return "jaguar3";
  case ChipGeneration::Rtl8733b:
    return "rtl8733b";
  case ChipGeneration::Kestrel:
    return "kestrel";
  case ChipGeneration::Mt7612u:
    return "mt7612u";
  default:
    return "unknown";
  }
}

/* Supported channel widths, one bit per width (MHz). A mask, not a max, because
 * the set is not contiguous per family: Jaguar2/Jaguar3 add 5/10 MHz narrowband
 * BELOW the 20/40/80 all AC families do. */
constexpr uint8_t kBw5 = 1u << 0;
constexpr uint8_t kBw10 = 1u << 1;
constexpr uint8_t kBw20 = 1u << 2;
constexpr uint8_t kBw40 = 1u << 3;
constexpr uint8_t kBw80 = 1u << 4;
constexpr uint8_t kBw160 = 1u << 5;

/* J1 does 20/40/80; J2 and J3 add the 5/10 MHz narrowband re-clock (J2 packs
 * the ADC/DAC clock word into 0x8ac, J3 into 0x9b0/0x9b4 — same RF-stays-20MHz
 * concept; the J2 8822B additionally needs an RF18 re-latch edge after the
 * re-clock). J1 has no vendor narrowband reference (the rtl8812au trees carry
 * only dead enum values). Pure; unit-tested in tests/adapter_caps_selftest.cpp. */
inline uint8_t bw_mask_for_generation(ChipGeneration g) {
  const uint8_t ac = kBw20 | kBw40 | kBw80;
  /* Kestrel (11ax): 5/10 MHz is the BB small-BW field on both dies (vendor
   * bw_sup declares BW_CAP_5M|10M); 160 MHz is 8852C-only (rtl8852c_halinit.c
   * bw_sup has BW_CAP_160M, rtl8852b_halinit.c tops at 80) and is OR'd in by
   * the device layer per variant. */
  /* RTL8733B: 10 MHz qualified (SDR OBW + two-way cross-decode with a
   * Jaguar3 peer, both bands); 5 MHz is refused — its BB small-BW mode airs
   * no packets on this die (docs/rtl8733b.md "Narrowband status"). */
  /* MT7612U: 20/40/80 and nothing narrower. MT_RATE_BW encodes only
   * 20/40/80/160, so there is no 5 or 10 MHz to select — unlike the Realtek
   * BB small-BW modes the trailing arm below is describing. Named explicitly
   * because that trailing arm is the permissive one: without this case a
   * MediaTek adapter would inherit kBw5|kBw10 and advertise two bandwidths the
   * part cannot represent. 160 MHz is likewise absent (docs/mt7612u.md). */
  return g == ChipGeneration::Rtl8733b ? (kBw10 | kBw20 | kBw40)
         : g == ChipGeneration::Jaguar1  ? ac
         : g == ChipGeneration::Mt7612u  ? ac
         : g == ChipGeneration::Unknown ? 0
                                        : (ac | kBw5 | kBw10);
}

/* A tunable / characterized frequency span (MHz). valid=false = band absent. */
struct BandRange {
  bool valid = false;
  uint16_t min_mhz = 0;
  uint16_t max_mhz = 0;
};

struct AdapterCaps; /* fwd for the frequency-range helper below */

/* Fill the 2.4 + 5 GHz tunable / characterized spans shared by all three Jaguar
 * families. tune_* = what the synthesizer reaches in monitor mode; the 5 GHz
 * span runs past the UNII channels (the extended synth ~5080..6165 MHz, chan
 * 16..253 — per-chip lock varies, validated on the bench). characterized_* =
 * the sub-range the generated txpwr_lmt / EFUSE PG tables cover; outside it TX
 * power is extrapolated from the nearest characterized channel. Defined
 * out-of-line below (needs the full AdapterCaps). */
inline void set_standard_freq_ranges(AdapterCaps &c);

struct AdapterCaps {
  bool supported = false; /* false on a generation that hasn't wired this */

  /* --- identity --- */
  const char *chip_name = "";      /* silicon die, no bus suffix: "RTL8822C" */
  const char *marketing_names = "";/* alias list, e.g. "RTL8812CU/RTL8822CU" */
  uint8_t chip_id = 0;             /* SYS_CFG2 (0x00FC) dispatch byte */
  ChipGeneration generation = ChipGeneration::Unknown;
  const char *variant = "";        /* per-family variant tag ("C8822B", ICType) */
  const char *transport = "";      /* "usb" | "pcie" */

  /* --- chains (EFUSE RF-type on Jaguar1; per-variant on Jaguar2/3) --- */
  uint8_t tx_chains = 0;
  uint8_t rx_chains = 0;

  /* --- composed sub-caps (single source of truth) --- */
  TxCaps tx;          /* = GetTxCaps() — modulation features */
  TxPowerCaps txpwr;  /* = GetTxPowerCaps() — power-knob range/step */

  /* --- bandwidth + frequency --- */
  uint8_t bw_mask = 0;                    /* kBw* bits */
  BandRange tune_2g4, tune_5g;            /* synthesizer-tunable spans */
  BandRange characterized_2g4, characterized_5g; /* txpwr-table-backed spans */

  /* --- FEC RX (bench-derived truth table — deliberately NOT the vendor
   * driver's HAL_DEF_RX_LDPC, which is 2013-era interop-advertisement policy:
   * all-false on Jaguar1 while the 8812A baseband demonstrably decodes LDPC
   * on-air. The TX side lives in TxCaps.ldpc_ok. HT and VHT are separate
   * decoder paths in silicon, so the flags split: the RTL8821A field failure
   * ("PixelPilot can't RX LDPC from Eachine Sphere Link") is VHT-only. --- */
  bool ldpc_rx_ht = false;  /* baseband decodes LDPC-coded HT PPDUs */
  bool ldpc_rx_vht = false; /* baseband decodes LDPC-coded VHT PPDUs */
  /* Per-frame LDPC *reporting*: RxAtrib.ldpc is populated (RX-descriptor bit
   * on the 8812A die, PHY-status byte7[5] on Jaguar2/3). False on the 8814A —
   * it decodes LDPC fine but the vendor wired no per-frame indicator (rxdesc
   * offsets 16/20 unparsed, and the Jaguar1 phy_status_rpt has no ldpc bit). */
  bool ldpc_rx_flag = false;

  /* Bench-derived like the ldpc_rx_* trio above, and a TRANSMIT claim: the
   * baseband emits a VHT PPDU that a peer decodes on the 2.4 GHz band.
   * 802.11ac is a 5 GHz standard, so nothing guarantees a 2.4 GHz VHT frame
   * works at all; devourer's TX path
   * never reads the band when resolving a rate, which makes it *selectable*
   * everywhere, and this flag is the separate question of whether it flies.
   * False means unmeasured on that chip, not incapable.
   *
   * Scope: VHT *format* on 2.4 GHz. The 256-QAM points that motivate the
   * extension (the "NitroQAM" / "TurboQAM" marketing) are confirmed on the
   * 8812A only — VHT1SS_MCS8 at 20 MHz, decoded by an 8822BU peer. Measuring
   * them needs a chip that has been VBUS cold-cycled: high-order constellation
   * TX degrades across warm re-inits until 64-QAM and up stop decoding, which
   * reads exactly like a link too weak to carry them (docs/vht-on-2g4.md).
   * Note VHT MCS9 is not a legal rate at 20 MHz for 1-2 streams; hardware
   * falls back to MCS8 there, so 40 MHz is required to exercise MCS9 at all.
   *
   * A standards-only 802.11n receiver decodes none of it either way: this is a
   * strong-link, close-range mode, the opposite of a range mode. */
  bool vht_2g4_ok = false;

  /* --- hardware-ARQ capability (bench-derived truth table, on-air responder
   * matrix + retry-knob A/B; the measured contract is docs/scheduled-mac.md).
   * ack_responder_ok: SetAckResponder measurably closes a hardware-ARQ loop
   * as the RESPONDER (SIFS ACKs that a soliciting TX's CCX reports confirm).
   * Measured true: 8812A (works, degraded — intermittent SIFS ACKs), 8814A,
   * 8821A (61–64% single-shot MCS3, 94% at retry 8 — an earlier "broken"
   * verdict was a harness artifact: the responder's arm was never verified,
   * so a silently dead responder read as on=0/off=0),
   * 8822B, 8812C/8822C, 8812E/8822E (the 8811A rides the 8812 die path and
   * inherits its row), 8733B (1725/1725 frames ACKed at retries_mean 0.00,
   * and retarget-proof: re-armed on a different MAC, 1728/1728 —
   * tests/ack_txreport_matrix.sh run with the 8733B as the responder).
   *
   * The `on`, `retarget`, and legacy `off` rows establish arming, retargeting,
   * and a never-armed control. A backend-owned `disarmed` row supports only
   * the live-disarm claim for the responder used in that run. On the reference
   * RTL8812AU, the old gate-only clear left every soliciting report ACKed;
   * restoring the captured pre-arm MACID produced no ACKs with retries pinned
   * at the configured limit. The own-MAC adversary also showed why an arm equal
   * to the captured MACID must be refused. docs/scheduled-mac.md owns the exact
   * counts. The implementation additionally restores and readback-verifies
   * BSSID as port-state hygiene; the ACK-rate result does not attribute the
   * behavioral change to BSSID. The implementation covers the shared CHIP_8812
   * path, but its 1T1R RTL8811AU cut was not separately measured; 8814A/8821A
   * and the HalMAC generations do not inherit the result.
   *
   * On the 8733B the net_type gate is INERT and the engine matches MACID
   * alone: at single-shot ACK rate a never-armed port answers on its own EFUSE
   * MAC at 85.2%/82.5% against 0.0% for an address nobody holds, and 83.3%
   * when deliberately armed. Two consequences: every never-armed monitor
   * session on that die already auto-ACKs unicast to its own MAC, and a disarm
   * there can only move the identity, never silence the port
   * (Rtl8733bDevice::disarm_ack_responder). Not known to hold on any other
   * generation — the AP-mode work proved the gate where it was measured.
   * False-as-unmeasured (the
   * vht_2g4_ok reading: unmeasured, not incapable): the 8821C — it shares
   * the recipe but no 8821CU/CE cell has run. FALSE on Kestrel:
   * SetAckResponder is not implemented on the AX generation.
   * tx_retry_limit_ok: DEVOURER_TX_RETRY_LIMIT drives hardware autonomous
   * retransmission (measured 12/0/12 A/B: 8821AU, 8812BU, 8822CU; the 8733B
   * by airtime dose-response instead, 0/3/12 -> 1.00/4.00/12.32–12.33 airings per
   * frame, because that die has no CCX path to judge its own frames
   * (tests/rtl8733b_retry_limit_onair.sh); Kestrel
   * 8832CU witness-measured — the AX WD DATA_TXCNT_LMT field counts
   * ATTEMPTS, folded +1 to the N-retries contract, limits {0,2,8} -> modal
   * on-air copies {1,3,8-9}; the MT7612U by its own TX status FIFO -
   * mt7612uprobe txs, an unacknowledged 1400-byte frame settles at 6
   * attempts with limit 5 and 16 on the initvals (short limit 15; frames
   * over 2032 bytes use the long limit, 31 - the knob sets both), one run
   * each (docs/mt7612u-tx-retry.md); a global register there, not per
   * frame, programmed from retry_limit at every bring-up). FALSE on the 8814A die (the vendor
   * DATA_RETRY_LIMIT=0 carve-out is kept — knob inert) and
   * false-as-unmeasured on the 8821C. */
  bool ack_responder_ok = false;
  bool tx_retry_limit_ok = false;

  /* station_mode_ok: IRadio::SetStationIdentity can program this MAC for the
   * STATION half of an infrastructure BSS, and the behaviour a station needs
   * from the silicon has been measured on air. Gate station-mode callers on
   * this rather than on SetStationIdentity's return value alone, so a caller
   * can refuse before it starts a handshake it cannot finish.
   *
   * False means "not ported / not measured", never "the silicon cannot". Do
   * not set it from a code-reading: the bar is an on-air cell showing this
   * adapter receiving unicast addressed to it and being ACKed for what it
   * sends - the same shape of evidence ack_responder_ok carries, measured per
   * die. (The MT7612U acknowledgement cells use a raw injector and an armed
   * ACK responder as the peer, not an AP.)
   *
   * TRUE on MT7612U, and read docs/mt7612u-station-identity.md - its
   * retraction section first - before quoting a number from it. Both halves
   * of the bar are measured there, with controls: a Realtek peer's own CCX
   * reports show this MAC acknowledging 100% of unicast addressed to it with
   * nothing armed (0.45 mean retries, 1279 reports) against three controls
   * pinned at the peer's 12-retry limit (a destination nobody holds, the DUT
   * absent, and MT_AUTO_RSP_EN cleared); and the MAC's own TX status FIFO
   * shows its uplink acknowledged 200/200 at 0.0 mean retries against a
   * 0/200 control run to the full ladder. The uplink cell sent from the
   * bring-up tool with an ACK-requesting TXWI and a retry limit of 15 - not
   * a library session, whose defaults (NOACK stream radiotap, tx.retry_limit
   * 0) send each unicast once; see IRadio::SetStationIdentity. Note the
   * limits the measurements do NOT clear, which a caller should know:
   *
   *   - every cell ran an UNASSOCIATED station receiving traffic it had not
   *     negotiated, so power save, TIM parsing, cross-BSS duplicate detection
   *     and hardware key lookup are untested;
   *   - those cells did not drive SetStationIdentity itself. On this part the
   *     seam writes no identity register and installs the managed receive
   *     filter the cells ran (0x00015f97; the RX loop's monitor filter comes
   *     back on clear), so the measured hardware state is the state a
   *     successful arm leaves behind, but the literal "arm through IRadio,
   *     then measure" path is not what the cells ran;
   *   - two units, one peer model, one channel, near field, no soak; the
   *     second unit reproduced the acknowledgement and uplink cells (its
   *     uplink at 1.9 mean retries against the first unit's 0.0), not the
   *     BSSID receive table.
   *
   * TRUE on the Jaguar3 8822C and the Jaguar2 8822B dies, through the
   * Realtek arm (src/StationArm.h has how it differs from the MT7612U: it
   * configures the port rather than checking it, and refuses the other
   * port-0 claimants rather than being dropped by them). Both halves were
   * measured by tests/realtek_station_onair.sh, which arms through the seam
   * itself and reads the transmitter's own CCX reports; one RTL8812CU and
   * one RTL8812BU, each the other's peer. Read docs/realtek-station-arm.md
   * - its limits section above all (one unit, two runs on one rig, near
   * field, one AP type; what "received" means; the report gap) - before
   * quoting:
   *
   *                  8812CU station          8812BU station
   *   A armed        100.0% ok, 0.03 retries 100.0% ok, 0.33 retries
   *   B nobody       0.0%, 12.00             0.0%, 12.00
   *   C DUT absent   0.0%, 12.00             0.0%, 12.00
   *   D unarmed      0.0%, 12.00             0.0%, 12.00
   *   E cleared      0.0%, 12.00             0.0%, 12.00
   *   F uplink->AP   100.0%, 0.09            100.0%, 0.20
   *   G uplink->none 0.0%, 12.00             0.0%, 12.00
   *   H unarmed F    100.0%, 0.08            100.0%, 0.20
   *
   * (Current record; an earlier record on the same rig matches it.) The
   * flag rests on both halves met WHILE ARMED. H shows the uplink half holds
   * without the arm too - the AP acknowledges by address - so on these dies
   * the arm is what the DOWN half needs (D and E at 0%).
   *
   * FALSE on the other Realtek dies, where SetStationIdentity is ported and
   * unmeasured: the 8822E (not measured by this cell), the 8821C, and every
   * Jaguar1 die (8812, 8814A, 8821A, the 8811AU cut).
   *
   * FALSE on Kestrel and the RTL8733B: not ported. */
  bool station_mode_ok = false;

  /* TxMode::no_agg is honoured: a frame carrying the radiotap TX_FLAGS
   * kRadiotapTxFlagNoAgg bit (RadiotapTxFlags.h) airs as its own PPDU at its
   * own rate and bandwidth even while SetAmpduMode is on. TRUE on Jaguar3,
   * on-air-measured on one 8812EU and one 8812CU (tests/tx_no_agg_onair.sh;
   * the 8822C folds deeper and pays more for the flag, docs/aggregation.md).
   * False everywhere else: the bit is ignored and a flagged frame can still
   * be folded into an aggregate at its neighbour's rate. */
  bool tx_no_agg_ok = false;

  /* --- feature flags --- */
  /* Per-packet TX power: a per-frame power trim driven by radiotap
   * DBM_TX_POWER (dB delta vs the calibrated table / session base) or a
   * session default. Three hardware shapes:
   *   - Jaguar2 (8822B/8821C) + 8814A: a fixed 6-rung LUT in the descriptor
   *     ({0,-3,-7,-11,+3,+6} dB) — per_pkt_txpwr_steps = 6, step_qdb = 0.
   *   - Jaguar3 (8822C/8822E): a 2-bit bank selector; the banks are
   *     programmable 7-bit signed offsets (0x1e70) in per_pkt_txpwr_step_qdb
   *     units (nominally 4 = 1 dB), 2 concurrent non-zero levels —
   *     per_pkt_txpwr_steps = 0 (continuous), min/max give the travel.
   *   - Kestrel (8852B/8852C): no descriptor field; the fixed-dBm BB target
   *     is rewritten between frames on value change (2 RMWs, free while
   *     constant; global, so HW beacons follow) — per_pkt_txpwr_steps = 0,
   *     step_qdb = 1 (0.25 dB).
   * per_pkt_txpwr_measured stays false until the family's path has been
   * proven to move on-air power (tests/txpkt_pwr_ofset_onair.sh) — the
   * honest flag for a vendor-defined-but-unvalidated field (the 8814A
   * today). */
  bool per_packet_txpower = false;
  uint8_t per_pkt_txpwr_steps = 0;    /* 6 = LUT rungs; 0 = continuous qdb */
  uint8_t per_pkt_txpwr_step_qdb = 0; /* Jaguar3 bank step (qdB); 0 for LUT */
  int16_t per_pkt_txpwr_min_qdb = 0;  /* most negative per-packet trim */
  int16_t per_pkt_txpwr_max_qdb = 0;  /* most positive per-packet trim */
  bool per_pkt_txpwr_measured = false; /* on-air-confirmed for this family */
  bool narrowband_ok = false;      /* narrowband BB re-clock exists; which
                                    * widths via bw_mask kBw5/kBw10 (the
                                    * RTL8733B is 10 MHz only) */
  uint8_t xtal_cap_max = 0;        /* crystal-cap trim range top (0 = no trim;
                                    * 0x3f on Jaguar1/2, 0x7f on Jaguar3) */
  uint8_t xtal_cap_default = 0;    /* efuse/default crystal-cap code */
  bool fastretune_ok = false;      /* lean FastRetune override exists */
  /* HE ER SU (802.11ax extended range, Kestrel only): TX airs the ER SU PPDU
   * per-packet via radiotap-HE FORMAT=EXT_SU (242-tone RU MCS0-2; 106-tone RU
   * MCS0 via a BW_RU_ALLOC of 106) plus HE DCM, and RX classifies the format
   * in RxAtrib.ppdu_type (7=HE_SU, 8=HE_ERSU). Pre-AX generations have no ER
   * equivalent. */
  bool he_er_su_ok = false;
  bool per_chain_rssi = false;     /* frame parser fills per-chain rssi (>=2ch) */

  /* Frame-free sensing. These exist because a successful
   * dynamic_cast<IRtlRadio*> is not a correct discriminator and never was: the
   * RTL8733B derives from IRtlRadio and implements no GetRxEnergy at all, so
   * the cast reports a sensor that returns nothing.
   *
   * The two flags below are also independent of each other, and the RTL8733B
   * is where that stops being theoretical: it has a working CCX CLM engine
   * and no phydm FA/CCA block, so busy_airtime_ok is true while rx_energy_ok
   * is false. Do not read either from the other.
   *
   * busy_airtime_ok: the backend HAS a hardware busy-airtime engine that
   * IRadio::GetChannelBusy can report — the Realtek CCX CLM engine
   * (Jaguar1/2/3 and the RTL8733B, which the vendor phydm puts on the JGR3
   * map) or the MediaTek MAC channel timers. FALSE on Kestrel, whose NHM
   * rides the halbb glue rather than NhmReader, so it has no CLM.
   *
   * It does NOT promise that an unarmed GetChannelBusy() answers. On the
   * RTL8733B it does not: that backend feeds its sampled path from
   * GetRxEnergy, which it does not implement, so an unarmed call reports no
   * reading and IRadio::ArmChannelBusy is the only way to get a number out of
   * it. A consumer that wants a reading from an arbitrary backend should arm
   * rather than sample; one that samples must handle "no reading" from a
   * backend whose flag is true.
   *
   * busy_airtime_measured: that reading has been separated from a quiet
   * channel ON AIR for this family, not merely implemented. The harness is
   * tests/busy_window_probe.sh, which pits an armed window against the quiet
   * floor under a known load, and against each way a window can be spoiled.
   * True today on all five backends that set busy_airtime_ok. The flag stays
   * because the two facts are independent: a port can land the engine before
   * anyone has run it on air, and false-as-unmeasured is the house rule for
   * that state.
   *
   * rx_energy_ok: IRtlRadio::GetRxEnergy returns real phydm FA/CCA/IGI
   * counters. Always false on a non-Realtek radio; false on the RTL8733B and
   * Kestrel, which is exactly the false positive the cast produced. */
  bool busy_airtime_ok = false;
  bool busy_airtime_measured = false;
  bool rx_energy_ok = false;
  /* Hardware timing. hw_rx_timestamp: every received frame is stamped with the
   * MAC's microsecond TSF at receive (RxPacket.RxAtrib.tsfl) — true on all
   * generations. hw_beacon_txtsf: this adapter, as a transmitter, inserts its
   * live hardware TSF into the beacons it airs at the instant of transmission
   * (a genuine sub-µs TX-egress timestamp a receiver reads via
   * Packet::TxEgressTsf) — rides the hardware beacon function (StartBeacon);
   * true on all generations. Together they are the primitives for one-way
   * hardware time distribution (see TsfSync). tsf_write_ok: IRadio::WriteTsf
   * drives a standalone write the part's counter loads (the static half of the
   * WriteTsf contract; its bool return is the per-call transport result).
   * Readback-measured through WriteTsf: 8822B (Jaguar2; incl. an RTL8812BU,
   * 40/40 forward and backward writes landed), 8822C (Jaguar3). The
   * 8821AU (Jaguar1) is measured on the raw REG_TSFTR pair with a scratch probe
   * (both word orders, no beacon armed), which is exactly what the Jaguar1
   * WriteTsf writes; the override itself has not run on Jaguar1 hardware. The
   * 8812A/8814A, 8821C (USB and PCIe) and 8822E ride the same pair and code
   * path and are not separately measured. Kestrel: true over PCIe (RTL8852CE,
   * readback against the FREERUN counter: +30000 µs -> +29998), false over USB
   * (unmeasured). FALSE on the MT7612U (measured: its DW0/DW1 registers
   * do not load the counter, docs/mt7612u.md), and on the RTL8733B (no TSF
   * write in the source — not a bench fact).
   *
   * tbtt_follows_tsf: an active hardware beacon's TBTT moves with a WriteTsf
   * (the TBTT is hardware-locked to the TSF grid), so disciplining the TSF
   * steers the beacon — the actuator tests/pcie_ptp_tsf_discipline.cpp needs.
   * Measured on air through WriteTsf on the RTL8852CE (Kestrel, PCIe). FALSE on
   * Jaguar2/3, whose TBTT is deaf to a TSF write (measured — use
   * PinBeaconTbtt there), and left false on Jaguar1: its TBTT is recorded as
   * TSF-locked from the PinBeaconTbtt bench, but not measured through
   * WriteTsf.
   *
   * hw_injected_mgmt_txtsf: the MAC ALSO overwrites the timestamp field of a
   * management frame the HOST injects through send_packet (a probe response or
   * a beacon-shaped frame) with its live egress TSF, so a caller can air an
   * egress-stamped frame on any channel at any time without the hardware
   * beacon function. Measured with a constant in the field and an independent
   * witness reading it back (tests/probe_resp_egress_tsf_check.sh): true on
   * Jaguar2 (8812BU), Jaguar3 (8812CU, 8812EU) and Kestrel (8832CU), both FC
   * 0x50 and 0x80, arrival−egress spread 34–41 µs over 15 s. FALSE on Jaguar1:
   * the 8821AU rewrites the field, but with a free-running counter that is
   * neither TSF port (both read live during the run) and repeats for ~7
   * frames at a time; its hardware TBTT beacon IS stamped (3.2 µs spread), so
   * on this family the egress pair rides StartBeacon. The 8812AU/8814AU are
   * unmeasured and inherit false. False on the RTL8733B (no TSF read, no
   * beacon) and the MT7612U (unmeasured). */
  bool hw_rx_timestamp = false;
  bool hw_beacon_txtsf = false;
  bool hw_injected_mgmt_txtsf = false;
  bool tsf_write_ok = false;
  bool tbtt_follows_tsf = false;
  /* 802.11ax scheduled UL (Kestrel/RTL8852 only). trigger_ul_ok: the adapter
   * can air an HE Trigger frame (UL-OFDMA grant) and program the fw UL-OFDMA
   * scheduler (SendTrigger / ConfigureUlOfdma). twt_ok: the fw exposes the TWT
   * agreement surface (ConfigureTwt / TwtBindSta). Pre-AX generations have no
   * trigger/TWT firmware surface. */
  bool trigger_ul_ok = false;
  bool twt_ok = false;
  /* sounding_ok: the adapter exposes the HE sounding command surface (NDPA ->
   * NDP -> BFRP via StartSounding / RegisterBeamformee). NB the shipped client
   * NIC firmware accepts SET_SND_PARA but does not air the sequence (the fw
   * sounding-transmit engine is AP-firmware-only, like the MP-only F2P path);
   * host-injected SendTrigger is what puts a Trigger on the air. */
  bool sounding_ok = false;
};

inline void set_standard_freq_ranges(AdapterCaps &c) {
  c.tune_2g4 = BandRange{true, 2412, 2484};
  c.characterized_2g4 = BandRange{true, 2412, 2484};
  c.tune_5g = BandRange{true, 5080, 6165};
  c.characterized_5g = BandRange{true, 5180, 5825};
}

} // namespace devourer

#endif /* DEVOURER_ADAPTER_CAPS_H */
