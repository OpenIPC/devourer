#include "Halmac8733bMac.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>
#include <utility>

namespace rtl8733b {
namespace {

constexpr uint16_t kRegEfuseCtrl = 0x0030;
constexpr uint32_t kEfuseReady = 1u << 31;
constexpr uint32_t kEfuseAddressMask = 0x3ffu << 8;
constexpr uint32_t kEfuseDataMask = 0xff;
constexpr uint16_t kLogicalId = 0x000;
constexpr uint16_t kLogicalXtal = 0x0b9;
constexpr uint16_t kLogicalThermal = 0x0ba;
constexpr uint16_t kLogicalTrxPath = 0x0c9;
constexpr uint16_t kLogicalRfe = 0x0ca;
constexpr uint16_t kLogicalVid = 0x100;
constexpr uint16_t kLogicalPid = 0x102;
constexpr uint16_t kLogicalMac = 0x108;
constexpr uint16_t kExpectedId = 0x8129;

/* HALMAC 87xx / RTL8733B normal-mode register plane. Values are transcribed
 * from halmac_reg2.h and halmac_init_8733b.c at vendor commit 9e5f684. */
constexpr uint16_t kRegCr = 0x0100;
constexpr uint16_t kRegTxdmaPqMap = 0x010c;
constexpr uint16_t kRegRxBoundary = 0x0116;
constexpr uint16_t kRegRqpnHlpq = 0x0200;
constexpr uint16_t kRegDwbcn0Ctrl = 0x0208;
constexpr uint16_t kRegTxdmaOffsetChk = 0x020c;
constexpr uint16_t kRegRqpnNpq = 0x0214;
constexpr uint16_t kRegAutoLlt = 0x0224;
constexpr uint16_t kRegRxdmaAgg = 0x0280;
constexpr uint16_t kRegRxdmaMode = 0x0290;
constexpr uint16_t kRegFwhwTxqCtrl = 0x0420;
constexpr uint16_t kRegBcnqBoundary = 0x0424;
constexpr uint16_t kRegLifetimeEn = 0x0426;
constexpr uint16_t kRegSpecSifs = 0x0428;
constexpr uint16_t kRegDarfRc = 0x0430;
constexpr uint16_t kRegDarfRcH = 0x0434;
constexpr uint16_t kRegRarfRcH = 0x043c;
constexpr uint16_t kRegRrsr = 0x0440;
constexpr uint16_t kRegArfr0 = 0x0444;
constexpr uint16_t kRegArfrh0 = 0x0448;
constexpr uint16_t kRegArfr1 = 0x044c;
constexpr uint16_t kRegArfrh1 = 0x0450;
constexpr uint16_t kRegCckCheck = 0x0454;
constexpr uint16_t kRegBcnq2Boundary = 0x0455;
constexpr uint16_t kRegTxHangCtrl = 0x045e;
constexpr uint16_t kRegIniRtsRate = 0x0480;
constexpr uint16_t kRegArfr4 = 0x049c;
constexpr uint16_t kRegArfrh4 = 0x04a0;
constexpr uint16_t kRegArfr5 = 0x04a4;
constexpr uint16_t kRegArfrh5 = 0x04a8;
constexpr uint16_t kRegProtMode = 0x04c8;
constexpr uint16_t kRegBarMode = 0x04cc;
constexpr uint16_t kRegPrecnt = 0x04e5;
constexpr uint16_t kRegEdcaVo = 0x0500;
constexpr uint16_t kRegEdcaVi = 0x0504;
constexpr uint16_t kRegEdcaBe = 0x0508;
constexpr uint16_t kRegEdcaBk = 0x050c;
constexpr uint16_t kRegPifs = 0x0512;
constexpr uint16_t kRegSifs = 0x0514;
constexpr uint16_t kRegSlot = 0x051b;
constexpr uint16_t kRegTxPtcl = 0x0520;
constexpr uint16_t kRegTxPause = 0x0522;
constexpr uint16_t kRegDisTxReqClr = 0x0523;
constexpr uint16_t kRegRdCtrl = 0x0524;
constexpr uint16_t kRegTbttProhibit = 0x0540;
constexpr uint16_t kRegRdNavNext = 0x0544;
constexpr uint16_t kRegBcnCtrl = 0x0550;
constexpr uint16_t kRegDriverEarlyInt = 0x0558;
constexpr uint16_t kRegBcnDmaTime = 0x0559;
constexpr uint16_t kRegUsTimeTsf = 0x055c;
constexpr uint16_t kRegBcnMaxErr = 0x055d;
constexpr uint16_t kRegRxTsfOffsetCck = 0x055e;
constexpr uint16_t kRegMiscCtrl = 0x0577;
constexpr uint16_t kRegTcr = 0x0604;
constexpr uint16_t kRegRcr = 0x0608;
constexpr uint16_t kRegRxPacketLimit = 0x060c;
constexpr uint16_t kRegMacId = 0x0610;
constexpr uint16_t kRegMar = 0x0620;
constexpr uint16_t kRegUsTimeEdca = 0x0638;
constexpr uint16_t kRegAckTimeoutCck = 0x0639;
constexpr uint16_t kRegRespSifsCck = 0x063c;
constexpr uint16_t kRegRespSifsOfdm = 0x063e;
constexpr uint16_t kRegAckTimeout = 0x0640;
constexpr uint16_t kRegEifs = 0x0642;
constexpr uint16_t kRegNavCtrl = 0x0650;
constexpr uint16_t kRegWmacTrxPtcl = 0x0668;
constexpr uint16_t kRegWmacTrxPtclH = 0x066c;
constexpr uint16_t kRegRxFilter0 = 0x06a0;
constexpr uint16_t kRegRxFilter2 = 0x06a4;
constexpr uint16_t kRegBbpsfCtrl = 0x06dc;
constexpr uint16_t kRegSndPtclCtrl = 0x0718;
constexpr uint16_t kRegWmacOption1 = 0x07d4;
constexpr uint16_t kRegWmacOption2 = 0x07d8;
constexpr uint16_t kRegWsecOption = 0x07ec;
constexpr uint16_t kRegSpsLdoVoltCtrl1 = 0x0024;
constexpr uint16_t kRegExtSysClkCtrl = 0x1008;
constexpr uint16_t kRegRxPsfCtrl = 0x1610;
constexpr uint16_t kRegWmacCsiDmaCfg = 0x169c;
constexpr uint16_t kRegBf0Time = 0x1428;
constexpr uint16_t kRegBf1Time = 0x142c;
constexpr uint16_t kRegBfTimeout = 0x1430;
constexpr uint16_t kRegFastEdcaVoVi = 0x1448;
constexpr uint16_t kRegFastEdcaBeBk = 0x144c;

constexpr uint16_t kNormalPqMap = 0xf5a0;
constexpr uint8_t kMacTrxEnable = 0xff;
constexpr uint16_t kTxPages = 32768 >> 7;
constexpr uint16_t kReservedPages = 16 + 8 + 4;
constexpr uint8_t kReservedBoundary = kTxPages - kReservedPages;
constexpr uint8_t kQueuePages = 8;
constexpr uint16_t kPublicPages =
    kReservedBoundary - 3 * kQueuePages - 1;
constexpr uint16_t kRxBoundary = 16384 - 256 - 1;
constexpr uint32_t kRcrNormal = 0xe410220e;
/* RX aggregates have to fit inside one bulk-IN URB: a vendor-default 20 KiB
 * aggregate can be split by xHCI, leaving the descriptor tail in one completion
 * and its body in the next. The HALMAC size field is in 4 KiB pages, so 0x03
 * caps aggregates at kRxAggregateBytes8733b (12 KiB) while retaining the
 * vendor's 0x20 timeout. This matches Devourer's Jaguar2/Jaguar3 USB invariant
 * and remains compatible with hosts that cannot complete larger bulk-IN reads.
 * The RX loop floors its URB size at the same constant. */
constexpr uint16_t kUsbRxAgg8733b = 0x2003;
static_assert(kRxAggregateBytes8733b == ((kUsbRxAgg8733b & 0x0f) * 4096),
              "RX URB floor must match the programmed RXDMA_AGG page count");

uint16_t le16(const uint8_t *data) {
  return static_cast<uint16_t>(data[0] |
                               (static_cast<uint16_t>(data[1]) << 8));
}

uint32_t le32(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

int8_t signed_nibble(uint8_t value) {
  value &= 0x0f;
  return static_cast<int8_t>((value & 0x08) ? value - 16 : value);
}

template <size_t N>
bool valid_power_bases(const std::array<uint8_t, N> &values) {
  return std::all_of(values.begin(), values.end(),
                     [](uint8_t value) { return value <= 0x7f; });
}

} // namespace

bool EfuseInfo::mac_valid() const {
  const bool all_ff = std::all_of(mac.begin(), mac.end(),
                                  [](uint8_t b) { return b == 0xff; });
  const bool all_zero = std::all_of(mac.begin(), mac.end(),
                                    [](uint8_t b) { return b == 0x00; });
  return !all_ff && !all_zero && (mac[0] & 1u) == 0;
}

bool MacState::matches_normal_usb3out() const {
  return (pq_map & 0xfff0) == kNormalPqMap && (pq_map & (1u << 2)) != 0 &&
         (rqpn_hlpq & 0x00ffffff) ==
             ((static_cast<uint32_t>(kPublicPages) << 16) |
              (static_cast<uint32_t>(kQueuePages) << 8) | kQueuePages) &&
         /* RQPN_NPQ[15:8] reads back as a hardware status mirror (0x08 on
          * both cut-D units) although HALMAC writes zero there. Only the
          * writable NPQ byte and zero EXQ byte are contractual. */
         (rqpn_npq & 0x000000ff) == kQueuePages &&
         (rqpn_npq & 0x00ff0000) == 0 &&
         reserved_boundary == kReservedBoundary &&
         rx_boundary == kRxBoundary && cr == kMacTrxEnable &&
         (rxdma_mode & 0x0e) == 0x0e && rx_agg == kUsbRxAgg8733b &&
         rcr == kRcrNormal && (ldpc_control & 0x0300u) == 0;
}

Halmac8733bMac::Halmac8733bMac(RtlAdapter device, Logger_t logger)
    : _device(std::move(device)), _logger(std::move(logger)) {}

bool Halmac8733bMac::read_physical_byte(uint16_t address, uint8_t &value) {
  if (address >= kPhysicalEfuseSize)
    return false;
  uint32_t ctrl = _device.rtw_read32(kRegEfuseCtrl);
  ctrl &= ~(kEfuseAddressMask | kEfuseDataMask | kEfuseReady);
  ctrl |= static_cast<uint32_t>(address) << 8;
  _device.rtw_write32(kRegEfuseCtrl, ctrl);
  for (unsigned attempt = 0; attempt < 1000; ++attempt) {
    ctrl = _device.rtw_read32(kRegEfuseCtrl);
    if (ctrl & kEfuseReady) {
      value = static_cast<uint8_t>(ctrl);
      return true;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
  _logger->error("RTL8733B EFUSE: physical read timed out at 0x{:03x}",
                 address);
  value = 0xff;
  return false;
}

bool Halmac8733bMac::read_physical_efuse(
    std::array<uint8_t, kPhysicalEfuseSize> &map) {
  map.fill(0xff);
  const size_t logical_pg_end = kPhysicalEfuseSize - kProtectedEfuseSize;
  for (size_t address = 0; address < logical_pg_end; ++address)
    if (!read_physical_byte(static_cast<uint16_t>(address), map[address]))
      return false;

  /* Match the non-Windows HALMAC physical-dump policy: do not expose the
   * reserved checksum prefix or final reserved block, but retain the factory
   * calibration span between them. RTL8733B TSSI trims live at 0x1d4-0x1dd. */
  const size_t calibration_begin = logical_pg_end + kReservedCsEfuseSize;
  const size_t calibration_end = kPhysicalEfuseSize - kReservedEfuseSize;
  for (size_t address = calibration_begin; address < calibration_end;
       ++address) {
    if (!read_physical_byte(static_cast<uint16_t>(address), map[address]))
      return false;
  }
  return true;
}

bool Halmac8733bMac::parse_physical_efuse(const uint8_t *physical,
                                          size_t physical_len,
                                          uint8_t *logical,
                                          size_t logical_len,
                                          size_t *used_bytes) {
  if (physical == nullptr || logical == nullptr || logical_len == 0)
    return false;
  std::memset(logical, 0xff, logical_len);
  size_t pos = 0;
  while (pos < physical_len) {
    const uint8_t header = physical[pos++];
    if (header == 0xff) {
      if (used_bytes != nullptr)
        *used_bytes = pos - 1;
      return true;
    }

    uint16_t block = 0;
    uint8_t word_enable = 0;
    if ((header & 0x1f) == 0x0f) {
      if (pos >= physical_len)
        return false;
      const uint8_t extended = physical[pos++];
      if (extended == 0xff) {
        if (used_bytes != nullptr)
          *used_bytes = pos - 1;
        return true;
      }
      if ((extended & 0x0f) == 0x0f)
        continue;
      block = static_cast<uint16_t>(((extended & 0xf0) >> 1) |
                                    ((header >> 5) & 0x07));
      word_enable = extended & 0x0f;
    } else {
      block = (header >> 4) & 0x0f;
      word_enable = header & 0x0f;
    }

    const size_t base = static_cast<size_t>(block) << 3;
    for (unsigned word = 0; word < 4; ++word) {
      if (word_enable & (1u << word))
        continue;
      if (pos + 2 > physical_len)
        return false;
      const size_t target = base + word * 2;
      if (target + 1 >= logical_len)
        return false;
      logical[target] = physical[pos++];
      logical[target + 1] = physical[pos++];
    }
  }
  /* Running off the end of the readable span is address-space exhaustion, not
   * corruption: a fully-written EFUSE has no 0xff terminator left to find, and
   * every header consumed up to here decoded cleanly. The vendor walkers treat
   * this as success-with-whatever-parsed; failing instead would brick bring-up
   * on a heavily reprogrammed but perfectly valid map. Malformed input is
   * still rejected — the early returns above cover truncated headers and
   * out-of-range block targets. */
  if (used_bytes != nullptr)
    *used_bytes = physical_len;
  return true;
}

TxPowerPgMode8733b Halmac8733bMac::tx_power_pg_mode(uint8_t calibrate) {
  const uint8_t mode = calibrate >> 4;
  if (mode <= 3)
    return TxPowerPgMode8733b::DirectIndex;
  if (mode <= 7)
    return TxPowerPgMode8733b::TssiOffset;
  return TxPowerPgMode8733b::Unknown;
}

bool Halmac8733bMac::parse_direct_tx_power_efuse(
    const uint8_t *logical, size_t logical_len, DirectTxPowerInfo8733b &out) {
  out = {};
  if (logical == nullptr)
    return false;

  /* Common direct-index PG layout: per path, 18 bytes of 2.4 GHz data
   * followed by 24 bytes of 5 GHz data. RTL8733B uses four TX-gain steps per
   * dBm, while the signed EFUSE nibbles are in half-dB units, hence x2. */
  constexpr size_t kPathBytes = 18 + 24;
  constexpr size_t kLastRequired =
      kTxPowerEfuseOffset8733b + kPathBytes + 18 + 14;
  if (logical_len <= kLastRequired)
    return false;

  for (size_t path = 0; path < out.path.size(); ++path) {
    const size_t base = kTxPowerEfuseOffset8733b + path * kPathBytes;
    auto &power = out.path[path];
    std::copy_n(logical + base, power.cck_base_2g.size(),
                power.cck_base_2g.begin());
    std::copy_n(logical + base + 6, power.bw40_base_2g.size(),
                power.bw40_base_2g.begin());
    const uint8_t diff2g = logical[base + 11];
    power.bw20_diff_2g =
        static_cast<int8_t>(signed_nibble(diff2g >> 4) * 2);
    power.ofdm_diff_2g =
        static_cast<int8_t>(signed_nibble(diff2g) * 2);
    power.valid_2g = valid_power_bases(power.cck_base_2g) &&
                     valid_power_bases(power.bw40_base_2g);

    const size_t base5g = base + 18;
    std::copy_n(logical + base5g, power.bw40_base_5g.size(),
                power.bw40_base_5g.begin());
    const uint8_t diff5g = logical[base5g + 14];
    power.bw20_diff_5g =
        static_cast<int8_t>(signed_nibble(diff5g >> 4) * 2);
    power.ofdm_diff_5g =
        static_cast<int8_t>(signed_nibble(diff5g) * 2);
    power.valid_5g = valid_power_bases(power.bw40_base_5g);
  }
  return true;
}

bool Halmac8733bMac::parse_tssi_power_efuse(
    const uint8_t *logical, size_t logical_len, const uint8_t *physical,
    size_t physical_len, TssiPowerInfo8733b &out) {
  out = {};
  if (logical == nullptr || physical == nullptr || logical_len <= 0x44 ||
      physical_len <= 0x1dd)
    return false;

  size_t index = 0;
  for (size_t offset = 0x10; offset <= 0x1a; ++offset)
    out.path_a_de[index++] = static_cast<int8_t>(logical[offset]);
  for (size_t offset = 0x22; offset <= 0x2f; ++offset)
    out.path_a_de[index++] = static_cast<int8_t>(logical[offset]);
  for (size_t offset = 0x3a; offset <= 0x44; ++offset)
    out.path_b_de[offset - 0x3a] = static_cast<int8_t>(logical[offset]);

  out.path_a_programmed = std::any_of(
      out.path_a_de.begin(), out.path_a_de.end(),
      [](int8_t value) { return value != static_cast<int8_t>(-1); });
  out.path_b_programmed = std::any_of(
      out.path_b_de.begin(), out.path_b_de.end(),
      [](int8_t value) { return value != static_cast<int8_t>(-1); });
  if (!out.path_a_programmed)
    out.path_a_de.fill(0);
  if (!out.path_b_programmed)
    out.path_b_de.fill(0);

  constexpr std::array<size_t, 8> kPathATrimOffsets = {
      0x1dd, 0x1db, 0x1d9, 0x1d8, 0x1d7, 0x1d6, 0x1d5, 0x1d4};
  constexpr std::array<size_t, 2> kPathBTrimOffsets = {0x1dc, 0x1da};
  for (size_t group = 0; group < kPathATrimOffsets.size(); ++group)
    out.trim[group][0] =
        static_cast<int8_t>(physical[kPathATrimOffsets[group]]);
  for (size_t group = 0; group < kPathBTrimOffsets.size(); ++group)
    out.trim[group][1] =
        static_cast<int8_t>(physical[kPathBTrimOffsets[group]]);
  const bool path_a_trim_programmed = std::any_of(
      kPathATrimOffsets.begin(), kPathATrimOffsets.end(),
      [physical](size_t offset) { return physical[offset] != 0xff; });
  const bool path_b_trim_programmed = std::any_of(
      kPathBTrimOffsets.begin(), kPathBTrimOffsets.end(),
      [physical](size_t offset) { return physical[offset] != 0xff; });
  out.trim_programmed = path_a_trim_programmed || path_b_trim_programmed;
  if (!out.trim_programmed)
    out.trim = {};
  return true;
}

bool Halmac8733bMac::read_efuse(EfuseInfo &out) {
  out = {};
  out.physical.fill(0xff);
  out.logical.fill(0xff);
  out.mac.fill(0xff);
  if (!read_physical_efuse(out.physical))
    return false;
  const size_t readable = kPhysicalEfuseSize - kProtectedEfuseSize;
  if (!parse_physical_efuse(out.physical.data(), readable, out.logical.data(),
                            out.logical.size(), &out.used_bytes)) {
    _logger->error("RTL8733B EFUSE: packed-map parse failed");
    return false;
  }

  out.id = le16(&out.logical[kLogicalId]);
  out.vid = le16(&out.logical[kLogicalVid]);
  out.pid = le16(&out.logical[kLogicalPid]);
  std::copy_n(out.logical.begin() + kLogicalMac, out.mac.size(),
              out.mac.begin());
  out.xtal = out.logical[kLogicalXtal];
  out.thermal = out.logical[kLogicalThermal];
  out.trx_path = out.logical[kLogicalTrxPath];
  out.rfe_type = out.logical[kLogicalRfe];
  out.tx_power_calibrate = out.logical[kTxPowerCalibrateOffset8733b];
  out.tx_power_tracking_mode = out.tx_power_calibrate >> 4;
  out.tx_power_mode = tx_power_pg_mode(out.tx_power_calibrate);
  if (out.tx_power_mode == TxPowerPgMode8733b::DirectIndex) {
    out.direct_tx_power_parsed = parse_direct_tx_power_efuse(
        out.logical.data(), out.logical.size(), out.direct_tx_power);
  } else if (out.tx_power_mode == TxPowerPgMode8733b::TssiOffset) {
    out.tssi_power_parsed = parse_tssi_power_efuse(
        out.logical.data(), out.logical.size(), out.physical.data(),
        out.physical.size(), out.tssi_power);
  }
  out.valid = out.id == kExpectedId;

  _logger->info(
      "RTL8733B EFUSE: used={} id=0x{:04x} VID:PID={:04x}:{:04x} "
      "MAC={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} rfe=0x{:02x} "
      "xtal=0x{:02x} thermal=0x{:02x} path=0x{:02x}",
      out.used_bytes, out.id, out.vid, out.pid, out.mac[0], out.mac[1],
      out.mac[2], out.mac[3], out.mac[4], out.mac[5], out.rfe_type, out.xtal,
      out.thermal, out.trx_path);
  if (out.tx_power_mode == TxPowerPgMode8733b::DirectIndex &&
      out.direct_tx_power_parsed) {
    const auto &a = out.direct_tx_power.path[0];
    const auto &b = out.direct_tx_power.path[1];
    _logger->info(
        "RTL8733B EFUSE TX power: mode=direct raw=0x{:02x} selector={} "
        "usable={} A2G={} cck6={} ht40g2={} "
        "ofdm_diff={} ht20_diff={} B2G={} cck6={} ht40g2={} "
        "ofdm_diff={} ht20_diff={} A5G={} ht40g0={} ofdm_diff={} "
        "ht20_diff={}",
        out.tx_power_calibrate, out.tx_power_tracking_mode,
        out.direct_tx_power.usable(), a.valid_2g, a.cck_base_2g[2],
        a.bw40_base_2g[2], a.ofdm_diff_2g, a.bw20_diff_2g,
        b.valid_2g, b.cck_base_2g[2], b.bw40_base_2g[2],
        b.ofdm_diff_2g, b.bw20_diff_2g, a.valid_5g,
        a.bw40_base_5g[0], a.ofdm_diff_5g, a.bw20_diff_5g);
  } else if (out.tx_power_mode == TxPowerPgMode8733b::DirectIndex) {
    _logger->warn(
        "RTL8733B EFUSE TX power: mode=direct raw=0x{:02x} selector={} "
        "layout is truncated",
        out.tx_power_calibrate, out.tx_power_tracking_mode);
  } else if (out.tx_power_mode == TxPowerPgMode8733b::TssiOffset) {
    if (out.tssi_power_parsed) {
      _logger->info(
          "RTL8733B EFUSE TX power: mode=TSSI-offset raw=0x{:02x} "
          "selector={} A={} B={} trim={} ch6_cck={}/{} ch6_ofdm={}/{} "
          "ch6_trim={}/{}",
          out.tx_power_calibrate, out.tx_power_tracking_mode,
          out.tssi_power.path_a_programmed,
          out.tssi_power.path_b_programmed, out.tssi_power.trim_programmed,
          out.tssi_power.path_a_de[2], out.tssi_power.path_b_de[2],
          out.tssi_power.path_a_de[8], out.tssi_power.path_b_de[8],
          out.tssi_power.trim[0][0], out.tssi_power.trim[0][1]);
    } else {
      _logger->warn(
          "RTL8733B EFUSE TX power: mode=TSSI-offset raw=0x{:02x} "
          "selector={} layout is truncated",
          out.tx_power_calibrate, out.tx_power_tracking_mode);
    }
  } else {
    _logger->warn(
        "RTL8733B EFUSE TX power: mode=unknown raw=0x{:02x} selector={}",
        out.tx_power_calibrate, out.tx_power_tracking_mode);
  }
  if (!out.valid)
    _logger->error("RTL8733B EFUSE: logical ID 0x{:04x}, expected 0x{:04x}",
                   out.id, kExpectedId);
  if (!out.mac_valid())
    _logger->warn("RTL8733B EFUSE: permanent MAC is not usable");
  return out.valid;
}

bool Halmac8733bMac::init_queues() {
  _device.rtw_write32(
      kRegRqpnHlpq, (static_cast<uint32_t>(kPublicPages) << 16) |
                        (static_cast<uint32_t>(kQueuePages) << 8) |
                        kQueuePages);
  _device.rtw_write32(kRegRqpnNpq, kQueuePages);
  _device.rtw_write8(kRegRqpnHlpq + 3, 0x80);

  _device.rtw_write8(kRegDwbcn0Ctrl + 1, kReservedBoundary);
  _device.rtw_write8(kRegBcnqBoundary, kReservedBoundary);
  _device.rtw_write8(kRegBcnq2Boundary, kReservedBoundary);
  _device.rtw_write16(kRegRxBoundary, kRxBoundary);

  uint8_t dwbcn = _device.rtw_read8(kRegDwbcn0Ctrl);
  _device.rtw_write8(kRegDwbcn0Ctrl,
                     static_cast<uint8_t>((dwbcn & 0x0f) | (3u << 4)));
  _device.rtw_write8(
      kRegTxdmaOffsetChk + 1,
      static_cast<uint8_t>(_device.rtw_read8(kRegTxdmaOffsetChk + 1) |
                           (1u << 1)));

  _device.rtw_write32(kRegAutoLlt,
                      _device.rtw_read32(kRegAutoLlt) | (1u << 16));
  for (unsigned count = 1000; count != 0; --count) {
    if ((_device.rtw_read32(kRegAutoLlt) & (1u << 16)) == 0) {
      _device.rtw_write8(kRegCr + 3, 0); // normal transfer mode
      return true;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
  _logger->error("RTL8733B MAC: auto-LLT initialization timed out");
  return false;
}

bool Halmac8733bMac::init_trx() {
  if (_device.bulk_out_ep_count() != 3) {
    _logger->error("RTL8733B MAC: normal USB profile requires 3 bulk-OUT "
                   "endpoints, found {}",
                   _device.bulk_out_ep_count());
    return false;
  }
  _device.rtw_write16(kRegTxdmaPqMap, kNormalPqMap);
  _device.rtw_write8(kRegCr, 0);
  _device.rtw_write8(kRegCr, kMacTrxEnable);
  return init_queues();
}

void Halmac8733bMac::init_protocol() {
  _device.rtw_write8(
      kRegFwhwTxqCtrl,
      static_cast<uint8_t>(_device.rtw_read8(kRegFwhwTxqCtrl) | (1u << 7)));
  _device.rtw_write8(kRegFwhwTxqCtrl + 1, 0x1f);

  _device.rtw_write16(kRegSpecSifs, 0x100a);
  _device.rtw_write32(kRegSifs, 0x0e0a0e0a);
  _device.rtw_write16(kRegRespSifsCck, 0x0a0a);
  _device.rtw_write16(kRegRespSifsOfdm, 0x0e0e);

  _device.rtw_write32(kRegDarfRc, 0x01000000);
  _device.rtw_write32(kRegDarfRcH, 0x08070504);
  _device.rtw_write32(kRegRarfRcH, 0x08070504);
  _device.rtw_write32(kRegArfr0, 0xfe01f010);
  _device.rtw_write32(kRegArfrh0, 0x40000000);
  _device.rtw_write32(kRegArfr1, 0x003ff010);
  _device.rtw_write32(kRegArfrh1, 0x40000000);
  _device.rtw_write32(kRegArfr4, 0x0600f010);
  _device.rtw_write32(kRegArfrh4, 0x400003e0);
  _device.rtw_write32(kRegArfr5, 0x0600f015);
  _device.rtw_write32(kRegArfrh5, 0x000000e0);

  _device.rtw_write8(0x0456, 0x70); // REG_AMPDU_MAX_TIME
  _device.rtw_write8(
      kRegTxHangCtrl,
      static_cast<uint8_t>(_device.rtw_read8(kRegTxHangCtrl) | (1u << 4)));
  _device.rtw_write8(kRegPrecnt, 0x3a);
  _device.rtw_write8(kRegPrecnt + 1, 0x08);
  _device.rtw_write32(kRegProtMode, 0x203f08ff);
  _device.rtw_write16(kRegBarMode + 2, 0x0801);
  _device.rtw_write8(kRegFastEdcaVoVi, 0x06);
  _device.rtw_write8(kRegFastEdcaVoVi + 2, 0x06);
  _device.rtw_write8(kRegFastEdcaBeBk, 0x06);
  _device.rtw_write8(kRegFastEdcaBeBk + 2, 0x06);
  _device.rtw_write8(
      kRegLifetimeEn,
      static_cast<uint8_t>(_device.rtw_read8(kRegLifetimeEn) & ~(1u << 5)));

  uint32_t value = _device.rtw_read32(kRegBf0Time) & ~(1u << 29);
  _device.rtw_write32(kRegBf0Time, value | (1u << 28));
  value = _device.rtw_read32(kRegBf1Time) & ~(1u << 29);
  _device.rtw_write32(kRegBf1Time, value | (1u << 28));
  _device.rtw_write32(kRegBfTimeout,
                      _device.rtw_read32(kRegBfTimeout) & ~0x03u);
  _device.rtw_write32(kRegRrsr,
                      _device.rtw_read32(kRegRrsr) & ~(0x0fu << 21));
  _device.rtw_write8(
      kRegIniRtsRate,
      static_cast<uint8_t>(_device.rtw_read8(kRegIniRtsRate) | (1u << 5)));
  _device.rtw_write8(
      kRegCckCheck,
      static_cast<uint8_t>(_device.rtw_read8(kRegCckCheck) &
                           ~((1u << 3) | (1u << 6))));
  _device.rtw_write8(
      kRegFwhwTxqCtrl + 2,
      static_cast<uint8_t>(_device.rtw_read8(kRegFwhwTxqCtrl + 2) |
                           (1u << 2)));
  _device.rtw_write8(kRegFwhwTxqCtrl + 3, 0xff);
}

void Halmac8733bMac::init_edca() {
  _device.rtw_write32(kRegEdcaVo, 0x002fa226);
  _device.rtw_write32(kRegEdcaVi, 0x005ea328);
  _device.rtw_write32(kRegEdcaBe, 0x005ea42b);
  _device.rtw_write32(kRegEdcaBk, 0x0000a44f);
  _device.rtw_write8(kRegPifs, 0x1c);
  _device.rtw_write8(
      kRegTxPtcl + 1,
      static_cast<uint8_t>(_device.rtw_read8(kRegTxPtcl + 1) & ~(1u << 4)));
  _device.rtw_write8(
      kRegRdCtrl + 1,
      static_cast<uint8_t>(_device.rtw_read8(kRegRdCtrl + 1) | 0x07));

  uint8_t clk = _device.rtw_read8(kRegSpsLdoVoltCtrl1 + 2);
  _device.rtw_write8(kRegSpsLdoVoltCtrl1 + 2,
                     static_cast<uint8_t>((clk & 0xcf) | 0x10));
  _device.rtw_write8(
      kRegExtSysClkCtrl,
      static_cast<uint8_t>(_device.rtw_read8(kRegExtSysClkCtrl) & 0xf3));
  _device.rtw_write8(kRegUsTimeTsf, 0x14);
  _device.rtw_write8(kRegUsTimeEdca, 0x14);

  _device.rtw_write8(
      kRegMiscCtrl,
      static_cast<uint8_t>(_device.rtw_read8(kRegMiscCtrl) | 0x0b));
  _device.rtw_write16(kRegTxPause, 0);
  _device.rtw_write8(kRegSlot, 0x09);
  _device.rtw_write32(kRegRdNavNext, 0x001b0005);
  _device.rtw_write16(kRegRxTsfOffsetCck, 0x3030);
  _device.rtw_write8(
      kRegBcnCtrl,
      static_cast<uint8_t>(_device.rtw_read8(kRegBcnCtrl) | (1u << 3)));
  _device.rtw_write32(kRegTbttProhibit, 0x00006404);
  _device.rtw_write8(kRegDriverEarlyInt, 0x04);
  _device.rtw_write8(kRegBcnCtrl + 1, 0x10);
  _device.rtw_write8(kRegBcnDmaTime, 0x02);
  _device.rtw_write8(kRegBcnMaxErr, 0xff);
  _device.rtw_write8(
      kRegDisTxReqClr,
      static_cast<uint8_t>(_device.rtw_read8(kRegDisTxReqClr) | (1u << 7)));
}

void Halmac8733bMac::init_wmac() {
  _device.rtw_write32(kRegMar, 0xffffffff);
  _device.rtw_write32(kRegMar + 4, 0xffffffff);
  _device.rtw_write8(kRegBbpsfCtrl + 2, 0x84);
  _device.rtw_write8(kRegAckTimeout, 0x21);
  _device.rtw_write8(kRegAckTimeoutCck, 0x6a);
  _device.rtw_write16(kRegEifs, 0x0040);
  _device.rtw_write8(kRegNavCtrl + 2, 0xc8);
  _device.rtw_write8(
      kRegWmacTrxPtcl + 2,
      static_cast<uint8_t>(_device.rtw_read8(kRegWmacTrxPtcl + 2) &
                           ~(1u << 4)));
  _device.rtw_write16(
      kRegWmacTrxPtclH,
      static_cast<uint16_t>(_device.rtw_read16(kRegWmacTrxPtclH) & 0xfefc));
  _device.rtw_write8(kRegWmacTrxPtclH + 2, 0x05);
  _device.rtw_write32(kRegRxFilter0, 0xffffffff);
  _device.rtw_write16(kRegRxFilter2, 0xffff);
  _device.rtw_write32(kRegRcr, kRcrNormal);
  _device.rtw_write8(
      kRegRxPsfCtrl + 2,
      static_cast<uint8_t>(_device.rtw_read8(kRegRxPsfCtrl + 2) | 0x0e));
  _device.rtw_write8(kRegRxPacketLimit, 24);
  _device.rtw_write8(kRegTcr + 2, 0x30);
  _device.rtw_write8(kRegTcr + 1, 0x30);
  _device.rtw_write16(
      kRegWmacCsiDmaCfg,
      static_cast<uint16_t>((_device.rtw_read16(kRegWmacCsiDmaCfg) & 0xf000) |
                            0x00fc));
  _device.rtw_write8(
      kRegWsecOption + 2,
      static_cast<uint8_t>(_device.rtw_read8(kRegWsecOption + 2) &
                           ~((1u << 3) | (1u << 4) | (1u << 5) |
                             (1u << 6))));
  _device.rtw_write8(
      kRegSndPtclCtrl,
      static_cast<uint8_t>(_device.rtw_read8(kRegSndPtclCtrl) | (1u << 6)));
  _device.rtw_write32(kRegWmacOption2, 0xb1810041);
  _device.rtw_write8(kRegWmacOption1, 0x18); // 0x98 with early-drop disabled
}

void Halmac8733bMac::init_usb() {
  uint8_t mode = 0x0e; // DMA mode + burst count 3
  if (_device.rtw_read8(0x00ff) != 0x20) {
    const bool high_speed = (_device.rtw_read8(0xfe11) & 0x03) == 1;
    mode = static_cast<uint8_t>(mode | ((high_speed ? 1u : 2u) << 4));
  }
  _device.rtw_write8(kRegRxdmaMode, mode);
  _device.rtw_write16(
      kRegTxdmaOffsetChk,
      static_cast<uint16_t>(_device.rtw_read16(kRegTxdmaOffsetChk) |
                            (1u << 9)));

  _device.rtw_write32(kRegRxdmaAgg,
                      _device.rtw_read32(kRegRxdmaAgg) & ~(1u << 29));
  _device.rtw_write8(
      kRegTxdmaPqMap,
      static_cast<uint8_t>(_device.rtw_read8(kRegTxdmaPqMap) | (1u << 2)));
  _device.rtw_write8(
      kRegRxdmaAgg + 3,
      static_cast<uint8_t>(_device.rtw_read8(kRegRxdmaAgg + 3) & ~(1u << 7)));
  _device.rtw_write16(kRegRxdmaAgg, kUsbRxAgg8733b);
}

void Halmac8733bMac::program_mac(const std::array<uint8_t, 6> &mac) {
  _device.rtw_write32(kRegMacId, le32(mac.data()));
  _device.rtw_write16(kRegMacId + 4, le16(mac.data() + 4));
}

MacState Halmac8733bMac::read_mac_state() {
  MacState state;
  state.pq_map = _device.rtw_read16(kRegTxdmaPqMap);
  state.rqpn_hlpq = _device.rtw_read32(kRegRqpnHlpq);
  state.rqpn_npq = _device.rtw_read32(kRegRqpnNpq);
  state.reserved_boundary = _device.rtw_read8(kRegBcnqBoundary);
  state.rx_boundary = _device.rtw_read16(kRegRxBoundary);
  state.cr = _device.rtw_read8(kRegCr);
  state.rxdma_mode = _device.rtw_read8(kRegRxdmaMode);
  state.rx_agg = _device.rtw_read16(kRegRxdmaAgg);
  state.rcr = _device.rtw_read32(kRegRcr);
  state.ldpc_control = _device.rtw_read16(kRegBfTimeout);
  return state;
}

bool Halmac8733bMac::initialize(const EfuseInfo &efuse) {
  if (!efuse.valid || !efuse.mac_valid()) {
    _logger->error("RTL8733B MAC: refusing initialization without valid EFUSE");
    return false;
  }
  if (!init_trx()) {
    stop();
    return false;
  }
  init_protocol();
  init_edca();
  init_wmac();
  init_usb();
  program_mac(efuse.mac);

  /* RTL8733B monitor injection is BCC-only. The vendor's two-MAC path also
   * clears REG_BF_TIMEOUT_EN[9:8] (VHT/HT LDPC global enables); leaving those
   * firmware-owned bits set can suppress raw HT injection even when the
   * per-frame DATA_LDPC descriptor bit is zero. Keep both layers explicit. */
  const uint16_t ldpc_before = _device.rtw_read16(kRegBfTimeout);
  _device.rtw_write16(kRegBfTimeout,
                      static_cast<uint16_t>(ldpc_before & 0xfcffu));

  const MacState state = read_mac_state();
  const bool mac_matches =
      _device.rtw_read32(kRegMacId) == le32(efuse.mac.data()) &&
      _device.rtw_read16(kRegMacId + 4) == le16(efuse.mac.data() + 4);
  if (!state.matches_normal_usb3out() || !mac_matches) {
    _logger->error(
        "RTL8733B MAC: readback mismatch pq=0x{:04x} hlpq=0x{:08x} "
        "npq=0x{:08x} bdny=0x{:02x} rxbdny=0x{:04x} CR=0x{:02x} "
        "RXDMA=0x{:02x} RXAGG=0x{:04x} RCR=0x{:08x} "
        "LDPC=0x{:04x} mac={}",
        state.pq_map, state.rqpn_hlpq, state.rqpn_npq,
        state.reserved_boundary, state.rx_boundary, state.cr,
        state.rxdma_mode, state.rx_agg, state.rcr, state.ldpc_control,
        mac_matches);
    stop();
    return false;
  }
  _logger->info(
      "RTL8733B MAC: initialized pq=0x{:04x} boundary={} public={} "
      "RXDMA=0x{:02x} RXAGG=0x{:04x} RCR=0x{:08x} "
      "LDPC=0x{:04x}->0x{:04x}",
      state.pq_map, state.reserved_boundary, kPublicPages, state.rxdma_mode,
      state.rx_agg, state.rcr, ldpc_before, state.ldpc_control);
  return true;
}

bool Halmac8733bMac::configure_monitor_rx(bool keep_corrupted) {
  /* Vendor set_opmode_monitor plus HALMAC_DRV_INFO_PHY_STATUS. AAP makes the
   * receive path promiscuous, APP_PHYSTS appends 32 bytes of PHY status, and
   * APP_FCS preserves the trailing FCS expected by Packet::Data. */
  uint32_t rcr = (1u << 31) | (1u << 28) | 1u;
  if (keep_corrupted)
    rcr |= (1u << 8) | (1u << 9);
  _device.rtw_write32(kRegRcr, rcr);
  _device.rtw_write8(kRegRxPacketLimit + 3, 4); // REG_RX_DRVINFO_SZ, 8-B units
  _device.rtw_write16(kRegRxFilter0, 0xffff);
  _device.rtw_write16(kRegRxFilter0 + 2, 0xffff);
  _device.rtw_write16(kRegRxFilter2, 0xffff);
  _device.rtw_write16(kRegCr, 0x06ff); // full DMA/protocol/MAC TX+RX enable

  const bool ok = _device.rtw_read32(kRegRcr) == rcr &&
                  (_device.rtw_read8(kRegRxPacketLimit + 3) & 0x0f) == 4 &&
                  _device.rtw_read16(kRegCr) == 0x06ff;
  _logger->info(
      "RTL8733B monitor RX: ready={} RCR=0x{:08x} DRVINFO={} CR=0x{:04x}",
      ok, _device.rtw_read32(kRegRcr),
      _device.rtw_read8(kRegRxPacketLimit + 3) & 0x0f,
      _device.rtw_read16(kRegCr));
  return ok;
}

void Halmac8733bMac::stop() {
  _device.rtw_write32(kRegRcr, 0);
  _device.rtw_write8(
      kRegTxdmaPqMap,
      static_cast<uint8_t>(_device.rtw_read8(kRegTxdmaPqMap) & ~(1u << 2)));
  _device.rtw_write16(kRegTxPause, 0xffff);
  _device.rtw_write16(kRegCr, 0);
}

} // namespace rtl8733b
