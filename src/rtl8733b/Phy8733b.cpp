#include "Phy8733b.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

#include "Event.h"
#include "hal8733b_tables.h"

namespace rtl8733b {
namespace {

constexpr uint32_t kDwordMask = 0xffffffffu;
constexpr uint32_t kRfMask = 0x000fffffu;
constexpr uint16_t kRfBase[2] = {0x3c00, 0x4c00};

/* Sanity bound for the audit_tssi_* probe stages only (rtl8733bprobe): those
 * stages arm the PA-facing TSSI analog/enable sequence and roll it straight
 * back, so a meter that has moved this far above the EFUSE baseline across a
 * sub-second window means the readback is not describing the sequence under
 * test. It is a probe verdict, never a runtime gate — the device path treats
 * the meter as telemetry, like every other generation. */
constexpr int kTssiAuditThermalCeiling = 25;

unsigned bit_shift(uint32_t mask) {
  if (mask == 0)
    return 0;
  unsigned shift = 0;
  while ((mask & 1u) == 0) {
    mask >>= 1;
    ++shift;
  }
  return shift;
}

void delay_us(unsigned us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

void delay_ms(unsigned ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

size_t count_table(const uint32_t *table, size_t len,
                   const JaguarPhyContext &ctx) {
  size_t count = 0;
  PhyTableLoader::Load(table, len, ctx,
                       [&count](uint32_t, uint32_t) { ++count; });
  return count;
}

bool legal_20mhz(uint8_t channel) {
  if (channel >= 1 && channel <= 14)
    return true;
  if (channel >= 36 && channel <= 64)
    return (channel % 4) == 0;
  if (channel >= 100 && channel <= 144)
    return (channel % 4) == 0;
  if (channel >= 149 && channel <= 177)
    return ((channel - 149) % 4) == 0;
  return false;
}

bool legal_40_center(uint8_t center) {
  constexpr std::array<uint8_t, 14> centers = {
      38, 46, 54, 62, 102, 110, 118,
      126, 134, 142, 151, 159, 167, 175,
  };
  for (uint8_t candidate : centers)
    if (candidate == center)
      return true;
  return false;
}

unsigned width_mhz(ChannelWidth_t width) {
  return width == CHANNEL_WIDTH_5    ? 5
         : width == CHANNEL_WIDTH_10 ? 10
         : width == CHANNEL_WIDTH_40 ? 40
                                     : 20;
}

} // namespace

bool PhyState::matches(uint8_t xtal) const {
  const uint32_t xcap = static_cast<uint32_t>(xtal & 0x7f);
  const uint32_t expected_xtal = xcap | (xcap << 7);
  return (sys_func_en & 0x3u) == 0x3u && (rf_ctrl & 0x7u) == 0x7u &&
         (hci_opt_ctrl & (7u << 24)) == (7u << 24) &&
         (bb_enable & 0x3u) == 0x3u &&
         (wire_control & 0x10000003u) == 0x10000003u &&
         ((xtal_control & 0x00fffc00u) >> 10) == expected_xtal &&
         rf_a_57 != 0 && rf_a_57 != kRfMask &&
         rf_b_57 != 0 && rf_b_57 != kRfMask &&
         (rf_a_87 & 0x7u) == 0x7u && (rf_b_87 & 0x7u) == 0x3u &&
         rf_a_ef == 0 && rf_b_ef == 0;
}

bool ChannelState8733b::matches(const ChannelPlan8733b &plan) const {
  const uint32_t channel_mask = 0x00030fffu;
  uint32_t expected18 = plan.center;
  if (!plan.is_2g)
    expected18 |= (1u << 16) | (1u << 8);
  expected18 |= plan.width == CHANNEL_WIDTH_40 ? (1u << 11)
                                               : (3u << 10);
  uint32_t expected19 = 0;
  if (!plan.is_2g) {
    if (plan.center > 144)
      expected19 = 1u << 19;
    else if (plan.center > 80)
      expected19 = 1u << 18;
  }
  bool bb_ok = false;
  if (plan.width == CHANNEL_WIDTH_40) {
    bb_ok = (bb_9b0 & 0x0000ffcfu) ==
            ((static_cast<uint32_t>(plan.primary_index |
                                    (plan.primary_index << 4))
              << 8) |
             0x5u);
  } else if (plan.width == CHANNEL_WIDTH_5 ||
             plan.width == CHANNEL_WIDTH_10) {
    const uint32_t small_bw = plan.width == CHANNEL_WIDTH_5 ? 1u : 2u;
    bb_ok = (bb_9b0 & 0x0000ffcfu) == (small_bw << 6) &&
            ((bb_9b4 & 0x00000700u) >> 8) == plan.dac_code() &&
            (bb_9f0 & 0xfu) == plan.adc_code() && (bb_81c & 0xfu) == 0;
  } else {
    bb_ok = (bb_9b0 & 0x0000ffcfu) == 0;
  }
  const uint32_t mac_bw = wmac_trxptcl & ((1u << 7) | (1u << 8));
  return (rf_a_18 & channel_mask) == expected18 &&
         (rf_b_18 & channel_mask) == expected18 &&
         (rf_a_19 & 0x000c0000u) == expected19 &&
         (rf_b_19 & 0x000c0000u) == expected19 && bb_ok &&
         (data_sc & 0xfu) == plan.primary_index &&
         mac_bw == (plan.width == CHANNEL_WIDTH_40 ? (1u << 7) : 0u);
}

bool TxAgcState8733b::matches_flat(uint8_t index) const {
  if (index > 0x7f || cck_ref_a != index || cck_ref_b != index ||
      ofdm_ref_a != index || ofdm_ref_b != index)
    return false;
  return std::all_of(rate_diffs.begin(), rate_diffs.end(),
                     [](uint32_t value) { return value == 0; });
}

bool TssiDeState8733b::matches_disabled(
    const TssiDePlan8733b &expected) const {
  const uint32_t expected_a =
      static_cast<uint8_t>(expected.ofdm[0]) |
      (static_cast<uint32_t>(static_cast<uint8_t>(expected.ht40[0])) << 8) |
      (static_cast<uint32_t>(static_cast<uint8_t>(expected.ht40[0])) << 16) |
      (static_cast<uint32_t>(static_cast<uint8_t>(expected.ht20[0])) << 24);
  const uint32_t expected_b =
      static_cast<uint8_t>(expected.ht40[1]) |
      (static_cast<uint32_t>(static_cast<uint8_t>(expected.ofdm[1])) << 8) |
      (static_cast<uint32_t>(static_cast<uint8_t>(expected.ht40[1])) << 16) |
      (static_cast<uint32_t>(static_cast<uint8_t>(expected.ht40[1])) << 24);
  return !enabled && de.cck == expected.cck && de.ht40 == expected.ht40 &&
         de.ofdm == expected.ofdm && de.ht20 == expected.ht20 &&
         path_a_lanes == expected_a && path_b_lanes == expected_b;
}

bool TssiBbState8733b::matches_disabled(
    const TssiBbPlan8733b &expected) const {
  return !enabled && path == expected.path &&
         rate_offsets == expected.rate_offsets &&
         reg_4304 == expected.reg_4304 && reg_4308 == expected.reg_4308 &&
         reg_430c == expected.reg_430c && reg_4320 == expected.reg_4320 &&
         reg_4328 == expected.reg_4328 && reg_432c == expected.reg_432c &&
         reg_438c == expected.reg_438c && reg_4390 == expected.reg_4390 &&
         reg_4394 == expected.reg_4394 && reg_4398 == expected.reg_4398 &&
         reg_439c == expected.reg_439c && reg_43a8 == expected.reg_43a8;
}

bool TssiThermalState8733b::matches_disabled(
    const TssiThermalPlan8733b &expected) const {
  return !enabled && control == expected.control() &&
         offsets == expected.offsets;
}

bool TssiAnalogState8733b::matches_setup() const {
  return !enabled && (bb_1860 & (1u << 30)) == 0 &&
         bb_1c38 == 0xffb5005eu && (bb_1d40 & (1u << 3)) == 0 &&
         (bb_1e1c & ((1u << 31) | (1u << 26) | 0x0000f000u)) ==
             ((1u << 31) | (1u << 26) | 0x0000b000u) &&
         (bb_1ca4 & (1u << 31)) != 0 &&
         (rf_55[0] & (1u << 7)) != 0 &&
         (rf_55[1] & (1u << 7)) != 0;
}

bool TssiAnalogState8733b::matches_snapshot(
    const TssiAnalogState8733b &expected) const {
  return enabled == expected.enabled && bb_1860 == expected.bb_1860 &&
         bb_1c38 == expected.bb_1c38 && bb_1d40 == expected.bb_1d40 &&
         bb_1e1c == expected.bb_1e1c && bb_1ca4 == expected.bb_1ca4 &&
         (rf_55[0] & (1u << 7)) == (expected.rf_55[0] & (1u << 7)) &&
         (rf_55[1] & (1u << 7)) == (expected.rf_55[1] & (1u << 7)) &&
         (rf_7f[0] & (1u << 8)) == (expected.rf_7f[0] & (1u << 8)) &&
         (rf_7f[1] & (1u << 8)) == (expected.rf_7f[1] & (1u << 8));
}

Phy8733b::Phy8733b(RtlAdapter device, Logger_t logger)
    : _device(std::move(device)), _logger(std::move(logger)) {}

JaguarPhyContext Phy8733b::table_context(uint8_t cut, uint8_t rfe_type) {
  JaguarPhyContext ctx{};
  ctx.cut_version = cut;
  ctx.support_interface = 0x02; // ODM_ITRF_USB
  ctx.support_platform = 0x04;  // ODM_CE
  ctx.package_type = 0;         // vendor check_positive maps 0 to wildcard 15
  ctx.rfe_type = rfe_type;
  return ctx;
}

PhyTableCounts Phy8733b::count_selected_tables(uint8_t cut,
                                               uint8_t rfe_type) {
  const JaguarPhyContext ctx = table_context(cut, rfe_type);
  PhyTableCounts counts;
  counts.mac = count_table(array_mp_8733b_mac_reg,
                           array_mp_8733b_mac_reg_len, ctx);
  counts.phy = count_table(array_mp_8733b_phy_reg,
                           array_mp_8733b_phy_reg_len, ctx);
  counts.agc = count_table(array_mp_8733b_agc_tab,
                           array_mp_8733b_agc_tab_len, ctx);
  counts.power_targets = array_mp_8733b_phy_reg_pg_len / 6;
  counts.cal_init = count_table(array_mp_8733b_cal_init,
                                array_mp_8733b_cal_init_len, ctx);
  counts.radio_a = count_table(array_mp_8733b_radioa,
                               array_mp_8733b_radioa_len, ctx);
  counts.radio_b = count_table(array_mp_8733b_radiob,
                               array_mp_8733b_radiob_len, ctx);
  return counts;
}

std::optional<ChannelPlan8733b>
Phy8733b::channel_plan(SelectedChannel channel) {
  ChannelPlan8733b plan;
  plan.primary = channel.Channel;
  plan.offset = channel.ChannelOffset;
  plan.width = channel.ChannelWidth;
  if (!legal_20mhz(plan.primary) || channel.Band == 6)
    return std::nullopt;
  /* WIDTH_5 is refused outright: on this die the 5 MHz BB small-BW mode airs
   * no packets across the entire DAC/ADC divider code space, and was observed
   * airing a continuous carrier from some warm states (docs/rtl8733b.md
   * "Narrowband status"). 10 MHz is qualified and stays. */
  if (plan.width == CHANNEL_WIDTH_5)
    return std::nullopt;
  if (plan.width == CHANNEL_WIDTH_20 || plan.width == CHANNEL_WIDTH_10) {
    if (plan.offset != 0)
      return std::nullopt;
    plan.center = plan.primary;
    plan.primary_index = 0;
  } else if (plan.width == CHANNEL_WIDTH_40) {
    if (plan.offset != 1 && plan.offset != 2)
      return std::nullopt;
    const int center = static_cast<int>(plan.primary) +
                       (plan.offset == 1 ? 2 : -2);
    if (center <= 0 || center > 255)
      return std::nullopt;
    plan.center = static_cast<uint8_t>(center);
    if (plan.primary <= 14) {
      if (plan.center < 3 || plan.center > 11 || plan.primary == 14)
        return std::nullopt;
    } else if (!legal_40_center(plan.center)) {
      return std::nullopt;
    }
    // Vendor VHT_DATA_SC encoding: primary upper=1, primary lower=2.
    plan.primary_index = plan.offset == 2 ? 1 : 2;
  } else {
    return std::nullopt;
  }
  plan.is_2g = plan.center <= 14;
  return plan;
}

std::optional<TssiDePlan8733b>
Phy8733b::tssi_de_plan(const TssiPowerInfo8733b &power, uint8_t channel) {
  size_t de_group = 0;
  size_t trim_group = 0;
  size_t cck_group = 0;
  const bool is_2g = channel >= 1 && channel <= 14;

  if (channel == 0)
    return std::nullopt;
  if (channel <= 2) {
    cck_group = 0;
    de_group = 6;
  } else if (channel <= 5) {
    cck_group = 1;
    de_group = 7;
  } else if (channel <= 8) {
    cck_group = 2;
    de_group = 8;
  } else if (channel <= 11) {
    cck_group = 3;
    de_group = 9;
  } else if (channel <= 14) {
    cck_group = channel == 14 ? 5 : 4;
    de_group = 10;
  } else if (channel >= 16 && channel <= 40) {
    de_group = 11;
  } else if (channel >= 42 && channel <= 48) {
    de_group = 12;
  } else if (channel >= 50 && channel <= 58) {
    de_group = 13;
  } else if (channel >= 60 && channel <= 96) {
    de_group = 14;
  } else if (channel >= 100 && channel <= 104) {
    de_group = 15;
  } else if (channel >= 106 && channel <= 112) {
    de_group = 16;
  } else if (channel >= 114 && channel <= 120) {
    de_group = 17;
  } else if (channel >= 122 && channel <= 128) {
    de_group = 18;
  } else if (channel >= 130 && channel <= 136) {
    de_group = 19;
  } else if (channel >= 138 && channel <= 144) {
    de_group = 20;
  } else if (channel >= 149 && channel <= 153) {
    de_group = 21;
  } else if (channel >= 155 && channel <= 161) {
    de_group = 22;
  } else if (channel >= 163 && channel <= 169) {
    de_group = 23;
  } else if (channel >= 171 && channel <= 253) {
    de_group = 24;
  } else {
    return std::nullopt;
  }

  if (channel <= 7)
    trim_group = 0;
  else if (channel <= 14)
    trim_group = 1;
  else if (channel <= 50)
    trim_group = 2;
  else if (channel <= 98)
    trim_group = 3;
  else if (channel <= 128)
    trim_group = 4;
  else if (channel <= 144)
    trim_group = 5;
  else if (channel <= 163)
    trim_group = 6;
  else
    trim_group = 7;

  auto add = [](int8_t lhs, int8_t rhs) {
    return static_cast<int8_t>(std::clamp(static_cast<int>(lhs) +
                                              static_cast<int>(rhs),
                                          -128, 127));
  };
  auto subtract_two = [](int8_t value) {
    return static_cast<int8_t>(
        (std::max)(-128, static_cast<int>(value) - 2));
  };

  TssiDePlan8733b plan;
  for (size_t path = 0; path < 2; ++path) {
    const int8_t trim = power.trim[trim_group][path];
    if (is_2g) {
      const int8_t cck = path == 0 ? power.path_a_de[cck_group]
                                   : power.path_b_de[cck_group];
      plan.cck[path] = add(cck, trim);
    }
    const int8_t ofdm = path == 0
                            ? power.path_a_de[de_group]
                            : is_2g ? power.path_b_de[de_group]
                                    : static_cast<int8_t>(0);
    plan.ht40[path] = add(ofdm, trim);
    plan.ofdm[path] = subtract_two(plan.ht40[path]);
    plan.ht20[path] = subtract_two(plan.ht40[path]);
  }
  return plan;
}

bool Phy8733b::parse_tx_power_targets(const uint32_t *table, size_t len,
                                      TxPowerTargets8733b &out) {
  for (auto &band : out.qdbm)
    for (auto &path : band)
      path.fill(0xff);
  out.present = {};
  if (table == nullptr || len == 0 || (len % 6) != 0)
    return false;

  for (size_t record = 0; record < len; record += 6) {
    const uint32_t band = table[record];
    const uint32_t path = table[record + 1];
    const uint32_t tx_num = table[record + 2];
    const uint32_t reg = table[record + 3];
    const uint32_t mask = table[record + 4];
    const uint32_t value = table[record + 5];
    if (band > 1 || path > 1 || tx_num != 0 || mask != kDwordMask)
      return false;

    size_t first_rate = 0;
    switch (reg & 0xfffu) {
    case 0xc20:
    case 0xe20:
      first_rate = 0;
      break;
    case 0xc24:
    case 0xe24:
      first_rate = 4;
      break;
    case 0xc28:
    case 0xe28:
      first_rate = 8;
      break;
    case 0xc2c:
    case 0xe2c:
      first_rate = 12;
      break;
    case 0xc30:
    case 0xe30:
      first_rate = 16;
      break;
    default:
      return false;
    }
    for (size_t byte = 0; byte < 4; ++byte)
      out.qdbm[band][path][first_rate + byte] =
          static_cast<uint8_t>(value >> (byte * 8));
    out.present[band][path] = true;
  }
  return true;
}

std::optional<std::array<int8_t, 20>>
Phy8733b::tssi_rate_offsets(const TxPowerTargets8733b &targets, uint8_t band,
                            uint8_t path, uint8_t max_target_qdbm,
                            int offset_qdb, TssiOffsetSat8733b *sat) {
  if (band > 1 || path > 1 || !targets.present[band][path])
    return std::nullopt;
  std::array<int8_t, 20> offsets{};
  /* Accumulated locally and published only on success: a caller that gets
   * nullopt must not find rail flags set by the rates that were computed
   * before the one that failed. */
  TssiOffsetSat8733b rails;
  for (size_t rate = 0; rate < offsets.size(); ++rate) {
    const uint8_t target = targets.qdbm[band][path][rate];
    if (target == 0xff) {
      if (band == 1 && rate < 4)
        continue; // CCK is not defined outside 2.4 GHz.
      return std::nullopt;
    }
    /* Cap first, then shift: the ceiling is the safety limit on absolute
     * power, the offset is the caller's relative move around it, in either
     * direction. A rate the ceiling already pulled down keeps its calibrated
     * distance from the others through the shift.
     *
     * The shift is deliberately NOT re-clamped at the factory target on the
     * way up. The PG table is the vendor's calibration, not a hardware rail,
     * and a per-unit EFUSE trimmed too cold is exactly the case an operator
     * calibrates their own way out of — src/TxPower.h states for every family
     * that headroom above the generated table is deliberate and compliance is
     * the caller's. Clamping here would make this the one backend where a
     * measured operating point cannot be commanded. */
    int shifted =
        static_cast<int>((std::min)(target, max_target_qdbm)) + offset_qdb;
    /* Both rails are the int8 delta field, nothing softer. The field is signed
     * against the 64 qdBm anchor, so it spans targets from -16 dBm to
     * +47.75 dBm, and the loop was measured to keep reducing power the whole
     * way down to the bottom of it — no sign wrap, power still falling 7.3 dB
     * between a 0 dBm and a -4 dBm target. Clamping at 0 dBm, as an earlier cut
     * of this did, threw away ~9 dB of working backoff on the guess that a
     * negative absolute target was meaningless. Where the loop actually stops
     * responding (~-8 dBm) is a measured property documented in
     * GetTxPowerCaps, not a limit imposed here. */
    if (shifted < -64) {
      shifted = -64;
      rails.low = true;
    }
    int delta = shifted - 64;
    if (delta > 127) {
      delta = 127;
      rails.high = true;
    }
    offsets[rate] = static_cast<int8_t>(delta);
  }
  if (sat)
    *sat = rails;
  return offsets;
}

std::optional<TssiBbPlan8733b>
Phy8733b::tssi_bb_plan(const TxPowerTargets8733b &targets, uint8_t channel,
                       uint8_t rfe_type, uint8_t path,
                       uint8_t max_target_qdbm, int offset_qdb,
                       TssiOffsetSat8733b *sat) {
  if (path > 1 || !legal_20mhz(channel))
    return std::nullopt;
  const uint8_t band = channel <= 14 ? 0 : 1;
  const auto offsets =
      tssi_rate_offsets(targets, band, path, max_target_qdbm, offset_qdb, sat);
  if (!offsets)
    return std::nullopt;

  TssiBbPlan8733b plan;
  plan.path = path;
  for (size_t word = 0; word < plan.rate_offsets.size(); ++word) {
    for (size_t byte = 0; byte < 4; ++byte) {
      plan.rate_offsets[word] |=
          static_cast<uint32_t>(static_cast<uint8_t>(
              (*offsets)[word * 4 + byte]))
          << (byte * 8);
    }
  }
  const bool rfe_2g_path_b =
      band == 0 && (rfe_type == 2 || rfe_type == 4 || rfe_type == 9);
  plan.reg_4304 = rfe_2g_path_b ? 0x00008080u : 0x00000000u;
  plan.reg_4308 = 0x5c545c50u;
  plan.reg_430c = 0x3f3f3f3fu;
  plan.reg_4320 = 0x03883100u;
  plan.reg_4328 = 0x03280200u;
  plan.reg_432c = 0x1000ff55u;
  plan.reg_438c = 0xa0a04040u;
  plan.reg_4390 = 0x80808080u;
  plan.reg_4394 = 0xa4a44040u;
  plan.reg_4398 = 0x80808080u;
  plan.reg_439c = 0x00800801u;
  plan.reg_43a8 = rfe_2g_path_b ? 0x7747fd00u : 0x77470d00u;
  return plan;
}

TssiThermalPlan8733b Phy8733b::tssi_thermal_plan(uint8_t efuse_thermal,
                                                 bool cck) {
  TssiThermalPlan8733b plan;
  plan.baseline = efuse_thermal == 0xff ? 0x20 : efuse_thermal;
  plan.cck = cck;
  if (!cck)
    return plan;

  /* Generated txpowertracktssi data: the four 2G CCK path/direction tables
   * are identical. Entries 0..17 are zero, 18..19 are 3, and 20..29 are 5.
   * Reproduce _halrf_tssi_set_tmeter_tbl_8733b's signed 64-entry layout. */
  std::array<int8_t, 64> signed_offsets{};
  auto swing = [](size_t index) -> int8_t {
    return index < 18 ? 0 : index < 20 ? 3 : 5;
  };
  for (size_t i = 0; i < 32; ++i)
    signed_offsets[i] =
        static_cast<int8_t>(-swing((std::min)(i, size_t{29})));
  size_t swing_index = 1;
  for (int i = 63; i >= 32; --i) {
    signed_offsets[static_cast<size_t>(i)] =
        swing((std::min)(swing_index, size_t{29}));
    ++swing_index;
  }
  for (size_t word = 0; word < plan.offsets.size(); ++word) {
    for (size_t byte = 0; byte < 4; ++byte) {
      plan.offsets[word] |=
          static_cast<uint32_t>(static_cast<uint8_t>(
              signed_offsets[word * 4 + byte]))
          << (byte * 8);
    }
  }
  return plan;
}

void Phy8733b::set_bb(uint16_t reg, uint32_t mask, uint32_t data) {
  if (mask == kDwordMask) {
    _device.rtw_write32(reg, data);
    return;
  }
  const uint32_t original = _device.rtw_read32(reg);
  const unsigned shift = bit_shift(mask);
  _device.rtw_write32(reg, (original & ~mask) | ((data << shift) & mask));
}

uint32_t Phy8733b::get_bb(uint16_t reg, uint32_t mask) {
  return (_device.rtw_read32(reg) & mask) >> bit_shift(mask);
}

void Phy8733b::bb_table_write(uint32_t addr, uint32_t data) {
  switch (addr) {
  case 0xfe: delay_ms(50); return;
  case 0xfd: delay_ms(5); return;
  case 0xfc: delay_ms(1); return;
  case 0xfb: delay_us(50); return;
  case 0xfa: delay_us(5); return;
  case 0xf9: delay_us(1); return;
  default: break;
  }
  if (addr > 0xffffu)
    throw std::runtime_error("RTL8733B BB table address exceeds USB window");
  set_bb(static_cast<uint16_t>(addr), kDwordMask, data);
}

uint32_t Phy8733b::read_rf(uint8_t path, uint8_t reg, uint32_t mask) {
  if (path > 1 || (mask & kRfMask) == 0)
    return 0xffffffffu;
  const uint16_t direct =
      static_cast<uint16_t>(kRfBase[path] + (static_cast<uint16_t>(reg) << 2));
  return get_bb(direct, mask & kRfMask);
}

bool Phy8733b::write_rf(uint8_t path, uint8_t reg, uint32_t mask,
                        uint32_t data) {
  if (path > 1 || (mask & kRfMask) == 0)
    return false;
  const uint16_t direct =
      static_cast<uint16_t>(kRfBase[path] + (static_cast<uint16_t>(reg) << 2));
  set_bb(direct, mask & kRfMask, data);
  delay_us(1);
  return true;
}

void Phy8733b::rf_table_write(uint8_t path, uint32_t addr, uint32_t data) {
  if (addr == 0xffe) {
    delay_ms(50);
    return;
  }
  if (addr == 0xfe) {
    delay_us(100);
    return;
  }
  if (addr == 0xffff) {
    delay_us(1);
    return;
  }
  if (!write_rf(path, static_cast<uint8_t>(addr & 0xffu), kRfMask, data))
    throw std::runtime_error("RTL8733B RF table contains invalid path");
}

void Phy8733b::parameter_gate(bool post) {
  // config_phydm_parameter_init_8733b: enable 3-wire, gate CCK/OFDM,
  // then toggle the BB reset through MAC register 0 bit 16.
  set_bb(0x180c, 0x3u, 0x3u);
  set_bb(0x180c, 1u << 28, 1u);
  set_bb(0x1c3c, 0x3u, post ? 0x3u : 0x0u);
  set_bb(0x0000, 1u << 16, 1u);
  set_bb(0x0000, 1u << 16, 0u);
  set_bb(0x0000, 1u << 16, 1u);
}

void Phy8733b::set_crystal_cap(uint8_t xtal) {
  const uint32_t cap = static_cast<uint32_t>(xtal & 0x7fu);
  set_bb(0x103c, 0x00fffc00u, cap | (cap << 7));
}

PhyState Phy8733b::read_state() {
  PhyState state;
  state.sys_func_en = _device.rtw_read16(0x0002);
  state.rf_ctrl = _device.rtw_read8(0x001f);
  state.hci_opt_ctrl = _device.rtw_read32(0x0074);
  state.bb_enable = _device.rtw_read32(0x1c3c);
  state.wire_control = _device.rtw_read32(0x180c);
  state.xtal_control = _device.rtw_read32(0x103c);
  state.rf_a_57 = read_rf(0, 0x57);
  state.rf_b_57 = read_rf(1, 0x57);
  state.rf_a_87 = read_rf(0, 0x87);
  state.rf_b_87 = read_rf(1, 0x87);
  state.rf_a_ef = read_rf(0, 0xef);
  state.rf_b_ef = read_rf(1, 0xef);
  state.rf_b_66 = read_rf(1, 0x66);
  return state;
}

void Phy8733b::bb_reset() {
  set_bb(0x0000, 1u << 16, 1u);
  set_bb(0x0000, 1u << 16, 0u);
  set_bb(0x0000, 1u << 16, 1u);
}

void Phy8733b::igi_toggle() {
  const uint32_t igi = get_bb(0x1d70, 0x7fu);
  if (igi > 2)
    set_bb(0x1d70, 0x7fu, igi - 2);
  set_bb(0x1d70, 0x7fu, igi);
}

bool Phy8733b::program_synth(uint32_t rf18) {
  for (unsigned i = 0; i < 20; ++i) {
    write_rf(0, 0x18, kRfMask, rf18);
    write_rf(1, 0x18, kRfMask, rf18);
    delay_us(250);
    if (read_rf(0, 0xc5, 1u << 15) == 1)
      return true;
  }
  // Vendor cut-D flow treats this as an advisory settle poll and proceeds
  // after all 20 attempts. Preserve the bit in ChannelState for diagnostics.
  return true;
}

bool Phy8733b::switch_band(const ChannelPlan8733b &plan) {
  uint32_t rf18 = read_rf(0, 0x18);
  if (rf18 == 0xffffffffu)
    return false;
  if (plan.is_2g) {
    set_bb(0x2a24, 1u << 13, 0);
    set_bb(0x2a00, 1u << 1, 0);
    set_bb(0x0454, 1u << 7, 0);
    set_bb(0x1c80, 0x3f000000u, 0xf);
    rf18 &= ~((3u << 16) | (3u << 8) | 0xffu);
    rf18 |= plan.center;
    set_bb(0x1884, 1u << 21, 0);
    set_bb(0x1884, 1u << 20,
           (_rfe_type == 2 || _rfe_type == 4 || _rfe_type == 9) ? 1 : 0);
    set_bb(0x1968, 1u << 0, 1);
    set_bb(0x1968, 1u << 8, 1);
    set_bb(0x1968, 1u << 14, 1);
  } else {
    if (_rfe_type == 2 || _rfe_type == 3)
      return false;
    set_bb(0x2a24, 1u << 13, 1);
    set_bb(0x2a00, 1u << 1, 1);
    set_bb(0x0454, 1u << 7, 1);
    set_bb(0x1c80, 0x3f000000u, 0xf);
    rf18 &= ~((1u << 17) | (1u << 9) | 0xffu);
    rf18 |= (1u << 16) | (1u << 8) | plan.center;
    set_bb(0x1884, (1u << 21) | (1u << 20), 0);
    set_bb(0x1968, 1u << 0, 0);
    set_bb(0x1968, 1u << 8, 0);
    set_bb(0x1968, 1u << 14, 0);
  }
  const bool ready = program_synth(rf18);
  bb_reset();
  return ready;
}

void Phy8733b::spur_cancellation() {
  set_bb(0x0818, 1u << 11, 0);
  set_bb(0x1940, 1u << 31, 0);
  set_bb(0x1ce8, 1u << 28, 0);
  set_bb(0x0db4, 1u << 0, 0);
  set_bb(0x0c10, 1u << 9, 0);
  set_bb(0x0c24, 0x000000ffu, 0xff);
  set_bb(0x0c24, 0x0000ff00u, 0);
  set_bb(0x0884, 0x0001c000u, 4);
  set_bb(0x1900, 0xfu, 6);
  set_bb(0x1908, 0xf0u, 9);
}

/* Channel-keyed constants shared by switch_channel (which writes them on
 * every full set) and fast_retune (which writes them only when their bucket
 * changes across the hop). One copy, so full-vs-fast register parity cannot
 * silently diverge. */
namespace {

constexpr uint32_t kScoFc2a38_2g[15] = {
    0, 0x1cfea, 0x1d0e1, 0x1d1d7, 0x1d2cd, 0x1d3c3, 0x1d4b9,
    0x1d5b0, 0x1d6a6, 0x1d79c, 0x1d892, 0x1d988, 0x1da7f,
    0x1db75, 0x1ddc4};
constexpr uint32_t kScoFc2a3c_2g[15] = {
    0, 0x27de3, 0x27f35, 0x28088, 0x281da, 0x2832d, 0x2847f,
    0x285d2, 0x28724, 0x28877, 0x289c9, 0x28b1c, 0x28c6e,
    0x28dc1, 0x290ed};

constexpr uint32_t agc_bucket_5g(uint8_t center) {
  return center <= 64 ? 6u : center <= 144 ? 7u : 0u;
}

constexpr uint32_t sco_bucket_0c30(uint8_t center) {
  return center >= 173  ? 0x411u
         : center >= 120 ? 0x412u
         : center >= 112 ? 0x452u
         : center >= 56  ? 0x453u
         : center >= 52  ? 0x493u
         : center >= 16  ? 0x494u
         : center >= 13  ? 0x969u
         : center >= 11  ? 0x96au
                         : 0x9aau;
}

constexpr std::array<uint32_t, 8> tx_shape_2g(uint8_t center) {
  return {center == 14 ? 0x452484u : 0x7847cfu,
          center == 14 ? 0x0fe3c8u : 0x57a6b1u,
          center == 14 ? 0u : 0x1f2af412u,
          center == 14 ? 0u : 0x09717du,
          center == 14 ? 0u : 0xfb9003u,
          center == 14 ? 0u : 0xfb1fa5u,
          center == 14 ? 0u : 0xfe2fcau,
          center == 14 ? 0u : 0xffcff3u};
}

} // namespace

bool Phy8733b::switch_channel(const ChannelPlan8733b &plan) {
  uint32_t rf18 = read_rf(0, 0x18);
  uint32_t rf19 = read_rf(0, 0x19);
  if (rf18 == 0xffffffffu || rf19 == 0xffffffffu)
    return false;
  if (plan.is_2g) {
    rf18 &= ~((3u << 16) | (3u << 8) | 0xffu);
    rf18 |= plan.center;
  } else {
    rf18 &= ~((1u << 17) | (1u << 9) | 0xffu);
    rf18 |= (1u << 16) | (1u << 8) | plan.center;
    rf19 &= ~((1u << 19) | (1u << 18));
    if (plan.center > 144)
      rf19 |= 1u << 19;
    else if (plan.center > 80)
      rf19 |= 1u << 18;
  }
  if (!program_synth(rf18))
    return false;
  write_rf(0, 0x19, kRfMask, rf19);
  write_rf(1, 0x19, kRfMask, rf19);

  if (plan.is_2g) {
    set_bb(0x1ea8, 1u << 7, 1);
    set_bb(0x18ac, 0x0000f000u, 5);
    set_bb(0x18ac, 0x000001f0u, 4);
  } else {
    set_bb(0x1ea8, 1u << 7, 0);
    set_bb(0x18ac, 0x000001f0u, agc_bucket_5g(plan.center));
  }

  if (plan.center <= 14) {
    set_bb(0x2a38, 1u << 27, 0);
    set_bb(0x2a38, 0x07ffff00u, kScoFc2a38_2g[plan.center]);
    set_bb(0x2a3c, 0x000fffffu, kScoFc2a3c_2g[plan.center]);
  }
  set_bb(0x0c30, 0xfffu, sco_bucket_0c30(plan.center));

  if (plan.is_2g) {
    const auto shape = tx_shape_2g(plan.center);
    for (unsigned i = 0; i < 8; ++i)
      set_bb(static_cast<uint16_t>(0x1a00 + i * 4),
             i == 2 ? kDwordMask : 0x00ffffffu, shape[i]);
  }
  set_bb(0x0808, 0x7fu, plan.center == 13 ? 0x30 : 0x40);
  bb_reset();
  igi_toggle();
  spur_cancellation();
  return true;
}

bool Phy8733b::switch_bandwidth(const ChannelPlan8733b &plan) {
  uint32_t rf18 = read_rf(0, 0x18);
  if (rf18 == 0xffffffffu)
    return false;
  rf18 &= ~((1u << 11) | (1u << 10));
  const bool narrow = plan.width == CHANNEL_WIDTH_5 ||
                      plan.width == CHANNEL_WIDTH_10;
  if (plan.width != CHANNEL_WIDTH_40) {
    set_bb(0x0810, 0x3ff0u, 0x19b);
    set_bb(0x09b0, 0xffc0u, 0);
    set_bb(0x09b4, 0x0700u, 3);
    set_bb(0x09f0, 0xfu, 0xc);
    set_bb(0x081c, 0xfu, 9);
    set_bb(0x09b0, 0xfu, 0);
    rf18 |= (1u << 11) | (1u << 10);
    set_bb(0x0cbc, 1u << 21, 0);
  } else {
    set_bb(0x09b0, 0xfu, 5);
    set_bb(0x09b0, 0xc0u, 0);
    const uint32_t pri = plan.primary_index |
                         (static_cast<uint32_t>(plan.primary_index) << 4);
    set_bb(0x09b0, 0xff00u, pri);
    rf18 |= 1u << 11;
    set_bb(0x0cbc, 1u << 21, 1);
    set_bb(0x0808, 0x000f0000u, 0xd);
    set_bb(0x081c, 0xfu, 9);
  }
  if (plan.is_2g) {
    set_bb(0x18ac, 0x0000f000u, 5);
    set_bb(0x18ac, 0x000001f0u, 4);
  }
  if (!program_synth(rf18))
    return false;
  write_rf(0, 0x1a, 1u, plan.width == CHANNEL_WIDTH_40 ? 1 : 0);
  write_rf(1, 0x1a, 1u, plan.width == CHANNEL_WIDTH_40 ? 1 : 0);
  if (plan.width == CHANNEL_WIDTH_40) {
    write_rf(0, 0x1a, 1u << 16, 0);
    write_rf(1, 0x1a, 1u << 16, 0);
  }
  /* The vendor tree does not advertise 5/10 MHz in hal_spec. Its later
   * monitor-mode patch first completes the ordinary 20 MHz RF programming,
   * then applies this four-register BB re-clock. Keep that ordering verbatim:
   * applying the same values before the RF writes produced no RF output for
   * the patch author. This remains experimental until independently measured
   * occupied bandwidth and cross-receiver decode both pass. */
  if (narrow) {
    const uint32_t small_bw = plan.width == CHANNEL_WIDTH_5 ? 1u : 2u;
    set_bb(0x09b0, 0xc0u, small_bw);
    set_bb(0x09b4, 0x0700u, plan.dac_code());
    set_bb(0x09f0, 0xfu, plan.adc_code());
    set_bb(0x081c, 0xfu, 0);
  }
  spur_cancellation();
  bb_reset();
  igi_toggle();
  return true;
}

ChannelState8733b Phy8733b::read_channel_state() {
  ChannelState8733b state;
  state.rf_a_18 = read_rf(0, 0x18);
  state.rf_b_18 = read_rf(1, 0x18);
  state.rf_a_19 = read_rf(0, 0x19);
  state.rf_b_19 = read_rf(1, 0x19);
  state.bb_9b0 = _device.rtw_read32(0x09b0);
  state.bb_9b4 = _device.rtw_read32(0x09b4);
  state.bb_9f0 = _device.rtw_read32(0x09f0);
  state.bb_81c = _device.rtw_read32(0x081c);
  state.data_sc = _device.rtw_read8(0x0483);
  state.wmac_trxptcl = _device.rtw_read32(0x0668);
  state.synth_ready = read_rf(0, 0xc5, 1u << 15) == 1;
  return state;
}

TxAgcState8733b Phy8733b::read_txagc_state() {
  TxAgcState8733b state;
  const uint32_t refs = _device.rtw_read32(0x4308);
  state.cck_ref_a = static_cast<uint8_t>(refs & 0x7f);
  state.ofdm_ref_a = static_cast<uint8_t>((refs >> 8) & 0x7f);
  state.cck_ref_b = static_cast<uint8_t>((refs >> 16) & 0x7f);
  state.ofdm_ref_b = static_cast<uint8_t>((refs >> 24) & 0x7f);
  for (size_t i = 0; i < state.rate_diffs.size(); ++i)
    state.rate_diffs[i] =
        _device.rtw_read32(static_cast<uint16_t>(0x3a00 + i * 4));
  return state;
}

bool Phy8733b::set_flat_tx_power(uint8_t index) {
  if (!_initialized || index > 0x7f)
    return false;

  /* config_phydm_set_txagc_to_hw_8733b, reduced to a flat table. 0x4308
   * carries path A/B CCK and OFDM references; 0x3a00..0x3a10 carry signed
   * differences for hardware rates 0..19 (CCK, OFDM, HT MCS0..7). */
  set_bb(0x4308, 0x0000007fu, index);
  set_bb(0x4308, 0x00007f00u, index);
  set_bb(0x4308, 0x007f0000u, index);
  set_bb(0x4308, 0x7f000000u, index);
  for (uint16_t reg = 0x3a00; reg <= 0x3a10; reg += 4)
    set_bb(reg, kDwordMask, 0);

  const TxAgcState8733b state = read_txagc_state();
  const bool ok = state.matches_flat(index);
  _logger->info(
      "RTL8733B TXAGC: ready={} flat=0x{:02x} refs={:02x}/{:02x}/"
      "{:02x}/{:02x} diffs={:08x}/{:08x}/{:08x}/{:08x}/{:08x}",
      ok, index, state.cck_ref_a, state.ofdm_ref_a, state.cck_ref_b,
      state.ofdm_ref_b, state.rate_diffs[0], state.rate_diffs[1],
      state.rate_diffs[2], state.rate_diffs[3], state.rate_diffs[4]);
  devourer::Ev(_logger->events(), "rtl8733b.txagc")
      .f("ok", ok)
      .hexf("flat", index, 2)
      .hexf("cck_a", state.cck_ref_a, 2)
      .hexf("ofdm_a", state.ofdm_ref_a, 2)
      .hexf("cck_b", state.cck_ref_b, 2)
      .hexf("ofdm_b", state.ofdm_ref_b, 2);
  return ok;
}

TssiBbState8733b Phy8733b::read_tssi_bb_state() {
  TssiBbState8733b state;
  state.enabled = get_bb(0x4318, 0x70000000u) != 0;
  state.path = static_cast<uint8_t>(get_bb(0x1884, 1u << 20));
  for (size_t i = 0; i < state.rate_offsets.size(); ++i)
    state.rate_offsets[i] =
        _device.rtw_read32(static_cast<uint16_t>(0x3a00 + i * 4));
  state.reg_4304 = _device.rtw_read32(0x4304);
  state.reg_4308 = _device.rtw_read32(0x4308);
  state.reg_430c = _device.rtw_read32(0x430c);
  state.reg_4320 = _device.rtw_read32(0x4320);
  state.reg_4328 = _device.rtw_read32(0x4328);
  state.reg_432c = _device.rtw_read32(0x432c);
  state.reg_438c = _device.rtw_read32(0x438c);
  state.reg_4390 = _device.rtw_read32(0x4390);
  state.reg_4394 = _device.rtw_read32(0x4394);
  state.reg_4398 = _device.rtw_read32(0x4398);
  state.reg_439c = _device.rtw_read32(0x439c);
  state.reg_43a8 = _device.rtw_read32(0x43a8);
  return state;
}

bool Phy8733b::prepare_tssi_bb(SelectedChannel channel,
                               const EfuseInfo &efuse) {
  const auto channel_cfg = planned(channel);
  const uint8_t path = static_cast<uint8_t>(get_bb(0x1884, 1u << 20));
  const auto plan =
      tssi_bb_plan(_tx_power_targets, channel.Channel, _rfe_type, path);
  if (!_initialized || !channel_cfg || !plan ||
      efuse.tx_power_mode != TxPowerPgMode8733b::TssiOffset) {
    _logger->error(
        "RTL8733B TSSI-BB: reject initialized={} channel={} mode={} path={}",
        _initialized, channel.Channel,
        static_cast<unsigned>(efuse.tx_power_tracking_mode), path);
    return false;
  }
  if (!read_channel_state().matches(*channel_cfg)) {
    _logger->error("RTL8733B TSSI-BB: channel state does not match request");
    return false;
  }

  /* This is the vendor digital common/DCK/slope state, reduced to deltas from
   * the immutable PHY image. Closed-loop tracking stays off. In particular,
   * this stage does not touch ANAPAR, RF 0x55/0x7f, the thermal table, or run
   * calibration/transmit packets. Factory DE is applied by the next seam. */
  set_bb(0x4318, 0x70000000u, 0);
  for (size_t i = 0; i < plan->rate_offsets.size(); ++i)
    set_bb(static_cast<uint16_t>(0x3a00 + i * 4), kDwordMask,
           plan->rate_offsets[i]);
  set_bb(0x4304, kDwordMask, plan->reg_4304);
  set_bb(0x4308, kDwordMask, plan->reg_4308);
  set_bb(0x430c, kDwordMask, plan->reg_430c);
  set_bb(0x4320, kDwordMask, plan->reg_4320);
  set_bb(0x4328, kDwordMask, plan->reg_4328);
  set_bb(0x432c, kDwordMask, plan->reg_432c);
  set_bb(0x4368, kDwordMask, 0x00000002u);
  set_bb(0x436c, kDwordMask, 0x00000000u);
  set_bb(0x4378, kDwordMask, 0x00000002u);
  set_bb(0x438c, kDwordMask, plan->reg_438c);
  set_bb(0x4390, kDwordMask, plan->reg_4390);
  set_bb(0x4394, kDwordMask, plan->reg_4394);
  set_bb(0x4398, kDwordMask, plan->reg_4398);
  set_bb(0x439c, kDwordMask, plan->reg_439c);
  set_bb(0x43a8, kDwordMask, plan->reg_43a8);
  set_bb(0x1ca4, 1u << 30, 1);
  set_bb(0x1c84, 0x0000fc00u, 8);
  set_bb(0x1c84, 0x000003c0u, 1);

  // Remove stale DE state before the following factory-offset stage.
  set_bb(0x4334, kDwordMask, 0);
  set_bb(0x433c, kDwordMask, 0);
  set_bb(0x4344, kDwordMask, 0);
  set_bb(0x434c, kDwordMask, 0);
  set_bb(0x43b0, kDwordMask, 0);
  set_bb(0x43b4, kDwordMask, 0);
  set_bb(0x43b8, kDwordMask, 0);

  const TssiBbState8733b state = read_tssi_bb_state();
  const bool ok = state.matches_disabled(*plan);
  _logger->info(
      "RTL8733B TSSI-BB: ready={} enabled={} ch={} path={} "
      "rates={:08x}/{:08x}/{:08x}/{:08x}/{:08x} "
      "CTRL={:08x}/{:08x}/{:08x}",
      ok, state.enabled, channel.Channel, state.path, state.rate_offsets[0],
      state.rate_offsets[1], state.rate_offsets[2], state.rate_offsets[3],
      state.rate_offsets[4], state.reg_4320, state.reg_4328,
      state.reg_432c);
  devourer::Ev(_logger->events(), "rtl8733b.tssi_bb")
      .f("ok", ok)
      .f("enabled", state.enabled)
      .f("channel", channel.Channel)
      .f("path", state.path)
      .hexf("rate_0_3", state.rate_offsets[0], 8)
      .hexf("rate_16_19", state.rate_offsets[4], 8)
      .hexf("reg_4320", state.reg_4320, 8)
      .hexf("reg_4328", state.reg_4328, 8)
      .hexf("reg_432c", state.reg_432c, 8);
  return ok;
}

TssiThermalState8733b Phy8733b::read_tssi_thermal_state() {
  TssiThermalState8733b state;
  state.enabled = get_bb(0x4318, 0x70000000u) != 0;
  state.control = _device.rtw_read32(0x4380);
  for (size_t i = 0; i < state.offsets.size(); ++i)
    state.offsets[i] =
        _device.rtw_read32(static_cast<uint16_t>(0x4200 + i * 4));
  return state;
}

bool Phy8733b::prepare_tssi_thermal(const EfuseInfo &efuse, bool cck) {
  if (!_initialized ||
      efuse.tx_power_mode != TxPowerPgMode8733b::TssiOffset ||
      get_bb(0x4318, 0x70000000u) != 0) {
    _logger->error(
        "RTL8733B TSSI thermal: reject initialized={} mode={} enabled={}",
        _initialized, static_cast<unsigned>(efuse.tx_power_tracking_mode),
        get_bb(0x4318, 0x70000000u) != 0);
    return false;
  }
  const TssiThermalPlan8733b plan = tssi_thermal_plan(efuse.thermal, cck);
  set_bb(0x4380, kDwordMask, plan.control());
  for (size_t i = 0; i < plan.offsets.size(); ++i)
    set_bb(static_cast<uint16_t>(0x4200 + i * 4), kDwordMask,
           plan.offsets[i]);

  const TssiThermalState8733b state = read_tssi_thermal_state();
  const bool ok = state.matches_disabled(plan);
  /* Report words that can actually distinguish the two tables. The CCK and
   * OFDM/HT plans differ only in words 4..11 — the swing ramp either side of
   * the baseline — and are identical (all zero) at words 0 and 15, so logging
   * the table edges could never show whether a CCK<->OFDM swap took effect.
   * `fold` is an XOR over all 16 words, so any word changing moves it. */
  uint32_t fold = 0;
  for (uint32_t word : state.offsets)
    fold ^= word;
  _logger->info(
      "RTL8733B TSSI thermal table: ready={} enabled={} cck={} "
      "baseline={} control={:08x} swing={:08x}/{:08x} fold={:08x}",
      ok, state.enabled, cck, plan.baseline, state.control, state.offsets[4],
      state.offsets[8], fold);
  devourer::Ev(_logger->events(), "rtl8733b.tssi_thermal_table")
      .f("ok", ok)
      .f("enabled", state.enabled)
      .f("cck", cck)
      .f("baseline", plan.baseline)
      .hexf("control", state.control, 8)
      .hexf("swing_lo", state.offsets[4], 8)
      .hexf("swing_hi", state.offsets[8], 8)
      .hexf("fold", fold, 8);
  return ok;
}

TssiAnalogState8733b Phy8733b::read_tssi_analog_state() {
  TssiAnalogState8733b state;
  state.bb_1860 = _device.rtw_read32(0x1860);
  state.bb_1c38 = _device.rtw_read32(0x1c38);
  state.bb_1d40 = _device.rtw_read32(0x1d40);
  state.bb_1e1c = _device.rtw_read32(0x1e1c);
  state.bb_1ca4 = _device.rtw_read32(0x1ca4);
  for (uint8_t path = 0; path < 2; ++path) {
    state.rf_55[path] = read_rf(path, 0x55);
    state.rf_7f[path] = read_rf(path, 0x7f);
  }
  state.enabled = get_bb(0x4318, 0x70000000u) != 0;
  return state;
}

void Phy8733b::write_tssi_anapar(bool tssi, bool is_2g) {
  const uint32_t active = is_2g ? 0x44u : 0x48u;
  static constexpr std::array<uint32_t, 16> baseline = {
      0x700b8041u, 0x701f0041u, 0x702f0041u, 0x703f0041u,
      0x704f0041u, 0x705f0041u, 0x706f0041u, 0x707b8041u,
      0x708b8041u, 0x709b8041u, 0x70ab8041u, 0x70bb8041u,
      0x70cb8041u, 0x70db8041u, 0x70eb8041u, 0x70fb8041u};
  for (size_t i = 0; i < baseline.size(); ++i) {
    uint32_t command = baseline[i];
    if (tssi && i >= 1 && i <= 4)
      command = (command & ~0xffu) | active;
    if (tssi && i == 6)
      command = 0x70644041u;
    set_bb(0x1830, kDwordMask, command);
  }
}

void Phy8733b::restore_tssi_analog(
    const TssiAnalogState8733b &snapshot, bool is_2g) {
  for (uint8_t path = 0; path < 2; ++path) {
    write_rf(path, 0x55, 1u << 7,
             (snapshot.rf_55[path] >> 7) & 1u);
    write_rf(path, 0x7f, 1u << 8,
             (snapshot.rf_7f[path] >> 8) & 1u);
  }
  write_tssi_anapar(false, is_2g);
  set_bb(0x1860, kDwordMask, snapshot.bb_1860);
  set_bb(0x1c38, kDwordMask, snapshot.bb_1c38);
  set_bb(0x1d40, kDwordMask, snapshot.bb_1d40);
  set_bb(0x1e1c, kDwordMask, snapshot.bb_1e1c);
  set_bb(0x1ca4, kDwordMask, snapshot.bb_1ca4);
  set_bb(0x4318, 0x70000000u, 0);
}

void Phy8733b::apply_tssi_analog(
    bool is_2g, std::array<uint32_t, 2> *rf_7f_immediate) {
  write_tssi_anapar(true, is_2g);
  set_bb(0x1860, 1u << 30, 0);
  set_bb(0x1c38, kDwordMask, 0xffb5005eu);
  set_bb(0x1d40, 1u << 3, 0);
  set_bb(0x1e1c, 1u << 31, 1);
  set_bb(0x1e1c, 1u << 26, 1);
  set_bb(0x1e1c, 0x0000f000u, 0xb);
  set_bb(0x1ca4, 1u << 31, 1);
  for (uint8_t path = 0; path < 2; ++path) {
    write_rf(path, 0x7f, 1u << 8, 1);
    if (rf_7f_immediate != nullptr)
      (*rf_7f_immediate)[path] = read_rf(path, 0x7f);
    write_rf(path, 0x55, 1u << 7, 1);
  }
}

bool Phy8733b::audit_tssi_analog(SelectedChannel channel,
                                 const EfuseInfo &efuse) {
  const auto channel_cfg = planned(channel);
  if (!_initialized || !channel_cfg ||
      efuse.tx_power_mode != TxPowerPgMode8733b::TssiOffset ||
      get_bb(0x4318, 0x70000000u) != 0 ||
      !read_channel_state().matches(*channel_cfg)) {
    _logger->error("RTL8733B TSSI analog: unsafe precondition");
    return false;
  }
  const TssiAnalogState8733b snapshot = read_tssi_analog_state();
  TssiAnalogState8733b setup;
  std::array<uint32_t, 2> rf_7f_immediate{};
  bool setup_ok = false;
  uint8_t setup_thermal = 0;
  try {
    // RF 0x7f[8] is a vendor command/strobe and does not latch on cut D.
    apply_tssi_analog(channel_cfg->is_2g, &rf_7f_immediate);
    setup = read_tssi_analog_state();
    setup_ok = setup.matches_setup();
    setup_thermal = read_thermal();
  } catch (...) {
    restore_tssi_analog(snapshot, channel_cfg->is_2g);
    throw;
  }
  restore_tssi_analog(snapshot, channel_cfg->is_2g);
  const TssiAnalogState8733b restored = read_tssi_analog_state();
  const uint8_t restored_thermal = read_thermal();
  const bool rollback_ok = restored.matches_snapshot(snapshot);
  const bool thermal_ok =
      setup_thermal != 0 && restored_thermal != 0 &&
      setup_thermal < efuse.thermal + kTssiAuditThermalCeiling &&
      restored_thermal < efuse.thermal + kTssiAuditThermalCeiling;
  const bool ok = setup_ok && rollback_ok && thermal_ok;
  _logger->info(
      "RTL8733B TSSI analog: ready={} setup={} rollback={} thermal={}/{} "
      "applied-BB={:08x}/{:08x}/{:08x}/{:08x}/{:08x} "
      "applied-RF55={:05x}/{:05x} RF7F-immediate={:05x}/{:05x}",
      ok, setup_ok, rollback_ok, setup_thermal, restored_thermal,
      setup.bb_1860, setup.bb_1c38, setup.bb_1d40, setup.bb_1e1c,
      setup.bb_1ca4, setup.rf_55[0], setup.rf_55[1],
      rf_7f_immediate[0], rf_7f_immediate[1]);
  devourer::Ev(_logger->events(), "rtl8733b.tssi_analog")
      .f("ok", ok)
      .f("setup", setup_ok)
      .f("rollback", rollback_ok)
      .f("setup_thermal", setup_thermal)
      .f("restored_thermal", restored_thermal)
      .hexf("bb_1860", setup.bb_1860, 8)
      .hexf("bb_1c38", setup.bb_1c38, 8)
      .hexf("bb_1d40", setup.bb_1d40, 8)
      .hexf("bb_1e1c", setup.bb_1e1c, 8)
      .hexf("bb_1ca4", setup.bb_1ca4, 8)
      .hexf("rf_a55", setup.rf_55[0], 5)
      .hexf("rf_b55", setup.rf_55[1], 5)
      .hexf("rf_a7f_immediate", rf_7f_immediate[0], 5)
      .hexf("rf_b7f_immediate", rf_7f_immediate[1], 5);
  return ok;
}

bool Phy8733b::audit_tssi_enable(SelectedChannel channel,
                                 const EfuseInfo &efuse,
                                 uint8_t max_target_qdbm) {
  const auto channel_cfg = planned(channel);
  const uint8_t path = static_cast<uint8_t>(get_bb(0x1884, 1u << 20));
  const auto capped = tssi_bb_plan(_tx_power_targets, channel.Channel,
                                   _rfe_type, path, max_target_qdbm);
  if (!_initialized || !channel_cfg || !capped ||
      max_target_qdbm > kSafeTssiTargetQdbm8733b ||
      efuse.tx_power_mode != TxPowerPgMode8733b::TssiOffset ||
      get_bb(0x4318, 0x70000000u) != 0 ||
      !read_channel_state().matches(*channel_cfg)) {
    _logger->error("RTL8733B TSSI enable: unsafe precondition");
    return false;
  }

  const TssiBbState8733b digital_snapshot = read_tssi_bb_state();
  const TssiAnalogState8733b analog_snapshot = read_tssi_analog_state();
  bool enabled_ok = false;
  uint8_t enabled_thermal = 0;
  try {
    for (size_t i = 0; i < capped->rate_offsets.size(); ++i)
      set_bb(static_cast<uint16_t>(0x3a00 + i * 4), kDwordMask,
             capped->rate_offsets[i]);
    apply_tssi_analog(channel_cfg->is_2g);
    set_bb(0x4320, 1u << 24, 0);
    set_bb(0x439c, 0x0ffffff0u, 0x080080);
    set_bb(0x4318, 0x70000000u, 0);
    set_bb(0x4318, 0x70000000u, 7);
    const TssiBbState8733b enabled = read_tssi_bb_state();
    TssiAnalogState8733b enabled_analog = read_tssi_analog_state();
    enabled_analog.enabled = false;
    enabled_ok = enabled.enabled &&
                 get_bb(0x4318, 0x70000000u) == 7 &&
                 enabled.rate_offsets == capped->rate_offsets &&
                 (enabled.reg_4320 & (1u << 24)) == 0 &&
                 enabled.reg_439c == 0x00800801u &&
                 enabled_analog.matches_setup();
    enabled_thermal = read_thermal();
  } catch (...) {
    set_bb(0x4318, 0x70000000u, 0);
    restore_tssi_analog(analog_snapshot, channel_cfg->is_2g);
    throw;
  }

  set_bb(0x4318, 0x70000000u, 0);
  restore_tssi_analog(analog_snapshot, channel_cfg->is_2g);
  for (size_t i = 0; i < digital_snapshot.rate_offsets.size(); ++i)
    set_bb(static_cast<uint16_t>(0x3a00 + i * 4), kDwordMask,
           digital_snapshot.rate_offsets[i]);
  set_bb(0x4320, kDwordMask, digital_snapshot.reg_4320);
  set_bb(0x439c, kDwordMask, digital_snapshot.reg_439c);

  const TssiBbState8733b restored_digital = read_tssi_bb_state();
  const TssiAnalogState8733b restored_analog = read_tssi_analog_state();
  const uint8_t restored_thermal = read_thermal();
  const bool rollback_ok =
      !restored_digital.enabled &&
      restored_digital.rate_offsets == digital_snapshot.rate_offsets &&
      restored_digital.reg_4320 == digital_snapshot.reg_4320 &&
      restored_digital.reg_439c == digital_snapshot.reg_439c &&
      restored_analog.matches_snapshot(analog_snapshot);
  const uint8_t baseline = efuse.thermal == 0xff ? 0x20 : efuse.thermal;
  const bool thermal_ok =
      enabled_thermal != 0 && restored_thermal != 0 &&
      enabled_thermal < baseline + kTssiAuditThermalCeiling &&
      restored_thermal < baseline + kTssiAuditThermalCeiling;
  const bool ok = enabled_ok && rollback_ok && thermal_ok;
  _logger->info(
      "RTL8733B TSSI enable: ready={} enabled={} rollback={} ceiling={} "
      "rates={:08x}/{:08x}/{:08x}/{:08x}/{:08x} thermal={}/{}",
      ok, enabled_ok, rollback_ok, max_target_qdbm, capped->rate_offsets[0],
      capped->rate_offsets[1], capped->rate_offsets[2],
      capped->rate_offsets[3], capped->rate_offsets[4], enabled_thermal,
      restored_thermal);
  devourer::Ev(_logger->events(), "rtl8733b.tssi_enable")
      .f("ok", ok)
      .f("enabled", enabled_ok)
      .f("rollback", rollback_ok)
      .f("ceiling_qdbm", max_target_qdbm)
      .hexf("rate_0_3", capped->rate_offsets[0], 8)
      .hexf("rate_16_19", capped->rate_offsets[4], 8)
      .f("enabled_thermal", enabled_thermal)
      .f("restored_thermal", restored_thermal);
  return ok;
}

bool Phy8733b::enable_tssi_tracking(SelectedChannel channel,
                                    const EfuseInfo &efuse,
                                    uint8_t max_target_qdbm, int offset_qdb) {
  const auto channel_cfg = planned(channel);
  const uint8_t path = static_cast<uint8_t>(get_bb(0x1884, 1u << 20));
  const auto capped = tssi_bb_plan(_tx_power_targets, channel.Channel,
                                   _rfe_type, path, max_target_qdbm,
                                   offset_qdb);
  if (!_initialized || !channel_cfg || !capped || max_target_qdbm > 64 ||
      efuse.tx_power_mode != TxPowerPgMode8733b::TssiOffset ||
      _tssi_digital_snapshot || _tssi_analog_snapshot ||
      get_bb(0x4318, 0x70000000u) != 0 ||
      !read_channel_state().matches(*channel_cfg)) {
    _logger->error("RTL8733B TSSI tracking: unsafe enable precondition");
    return false;
  }

  const TssiBbState8733b digital_snapshot = read_tssi_bb_state();
  const TssiAnalogState8733b analog_snapshot = read_tssi_analog_state();
  try {
    for (size_t i = 0; i < capped->rate_offsets.size(); ++i)
      set_bb(static_cast<uint16_t>(0x3a00 + i * 4), kDwordMask,
             capped->rate_offsets[i]);
    apply_tssi_analog(channel_cfg->is_2g);
    set_bb(0x4320, 1u << 24, 0);
    set_bb(0x439c, 0x0ffffff0u, 0x080080);
    set_bb(0x4318, 0x70000000u, 7);

    const TssiBbState8733b enabled = read_tssi_bb_state();
    TssiAnalogState8733b enabled_analog = read_tssi_analog_state();
    enabled_analog.enabled = false;
    /* Verdict is register readback only. No thermal read here: this runs on
     * every CCK<->OFDM rate-table transition, i.e. inside send_packet, and
     * read_thermal() is 3 RF writes + a 15 us settle + an RF read — several
     * USB control transfers per frame. The meter is also the wrong instrument
     * for the question: it tracks PA bias, not whether these BB writes landed
     * (docs/warm-tx-degradation.md). Thermal telemetry rides
     * GetThermalStatus / DEVOURER_THERMAL_POLL_MS at the caller's cadence. */
    const bool ok = enabled.enabled &&
                    get_bb(0x4318, 0x70000000u) == 7 &&
                    enabled.rate_offsets == capped->rate_offsets &&
                    (enabled.reg_4320 & (1u << 24)) == 0 &&
                    enabled.reg_439c == 0x00800801u &&
                    enabled_analog.matches_setup();
    if (!ok) {
      set_bb(0x4318, 0x70000000u, 0);
      restore_tssi_analog(analog_snapshot, channel_cfg->is_2g);
      for (size_t i = 0; i < digital_snapshot.rate_offsets.size(); ++i)
        set_bb(static_cast<uint16_t>(0x3a00 + i * 4), kDwordMask,
               digital_snapshot.rate_offsets[i]);
      set_bb(0x4320, kDwordMask, digital_snapshot.reg_4320);
      set_bb(0x439c, kDwordMask, digital_snapshot.reg_439c);
      _logger->error("RTL8733B TSSI tracking: enable readback failed");
      return false;
    }

    _tssi_digital_snapshot = digital_snapshot;
    _tssi_analog_snapshot = analog_snapshot;
    _tssi_is_2g = channel_cfg->is_2g;
    /* fast_retune bookkeeping: the offsets the chip now carries and the RF
     * path bit (RFE routing — static per session), for the in-place
     * per-channel rewrite. */
    _fr_tssi_offsets = capped->rate_offsets;
    _fr_tssi_path = path;
    _logger->info(
        "RTL8733B TSSI tracking enabled: ch={} path={} ceiling={} offset={} "
        "rates={:08x}/{:08x}/{:08x}/{:08x}/{:08x}",
        channel.Channel, path, max_target_qdbm, offset_qdb,
        capped->rate_offsets[0], capped->rate_offsets[1],
        capped->rate_offsets[2], capped->rate_offsets[3],
        capped->rate_offsets[4]);
    devourer::Ev(_logger->events(), "rtl8733b.tssi_tracking")
        .f("enabled", true)
        .f("channel", channel.Channel)
        .f("path", path)
        .f("ceiling_qdbm", max_target_qdbm)
        .f("offset_qdb", offset_qdb);
    return true;
  } catch (...) {
    set_bb(0x4318, 0x70000000u, 0);
    restore_tssi_analog(analog_snapshot, channel_cfg->is_2g);
    for (size_t i = 0; i < digital_snapshot.rate_offsets.size(); ++i)
      set_bb(static_cast<uint16_t>(0x3a00 + i * 4), kDwordMask,
             digital_snapshot.rate_offsets[i]);
    set_bb(0x4320, kDwordMask, digital_snapshot.reg_4320);
    set_bb(0x439c, kDwordMask, digital_snapshot.reg_439c);
    throw;
  }
}

bool Phy8733b::set_tssi_offset(SelectedChannel channel,
                               uint8_t max_target_qdbm, int offset_qdb,
                               TssiOffsetSat8733b *sat) {
  /* Refuse before the first write, like fast_retune: tracking not live means
   * there is no loop to retarget (the flat-PG path has no actuator at all),
   * and a channel the radio is not on would install the wrong band's targets. */
  if (!_initialized || !_tssi_digital_snapshot || !_fr_tssi_offsets ||
      max_target_qdbm > kSafeTssiTargetQdbm8733b ||
      (_fr_plan && _fr_plan->primary != channel.Channel)) {
    _logger->error("RTL8733B TSSI offset: refused (tracking not live or "
                   "channel/ceiling mismatch)");
    return false;
  }
  const auto plan =
      tssi_bb_plan(_tx_power_targets, channel.Channel, _rfe_type,
                   _fr_tssi_path, max_target_qdbm, offset_qdb, sat);
  if (!plan) {
    _logger->error("RTL8733B TSSI offset: no target plan for ch{}",
                   channel.Channel);
    return false;
  }
  if (plan->rate_offsets == *_fr_tssi_offsets)
    return true;

  /* In place, tracking left enabled — the #389 shape. Only the five packed
   * per-rate target dwords carry the power level; every other register in the
   * plan is a constant the enable path already wrote. */
  for (size_t i = 0; i < plan->rate_offsets.size(); ++i)
    set_bb(static_cast<uint16_t>(0x3a00 + i * 4), kDwordMask,
           plan->rate_offsets[i]);
  const TssiBbState8733b now = read_tssi_bb_state();
  const bool ok = now.enabled && now.rate_offsets == plan->rate_offsets;
  if (!ok) {
    /* Roll back to what the chip was carrying rather than leaving the loop on
     * a half-written target. _fr_tssi_offsets stays as it was — it describes
     * the state being restored. */
    for (size_t i = 0; i < _fr_tssi_offsets->size(); ++i)
      set_bb(static_cast<uint16_t>(0x3a00 + i * 4), kDwordMask,
             (*_fr_tssi_offsets)[i]);
    _logger->error("RTL8733B TSSI offset: readback failed, rolled back");
  } else {
    _fr_tssi_offsets = plan->rate_offsets;
  }
  _logger->info("RTL8733B TSSI offset: ok={} ch={} ceiling={} offset={} "
                "sat={}/{} rates={:08x}/{:08x}/{:08x}/{:08x}/{:08x}",
                ok, channel.Channel, max_target_qdbm, offset_qdb,
                sat && sat->low, sat && sat->high, plan->rate_offsets[0],
                plan->rate_offsets[1], plan->rate_offsets[2],
                plan->rate_offsets[3], plan->rate_offsets[4]);
  devourer::Ev(_logger->events(), "rtl8733b.tssi_offset")
      .f("ok", ok)
      .f("channel", channel.Channel)
      .f("ceiling_qdbm", max_target_qdbm)
      .f("offset_qdb", offset_qdb)
      .f("saturated_low", sat && sat->low)
      .f("saturated_high", sat && sat->high)
      .hexf("rate_0_3", plan->rate_offsets[0], 8)
      .hexf("rate_16_19", plan->rate_offsets[4], 8);
  return ok;
}

bool Phy8733b::tssi_offsets_confirmed(SelectedChannel channel,
                                      uint8_t max_target_qdbm,
                                      int offset_qdb) {
  if (!_initialized || !_tssi_digital_snapshot)
    return false;
  const auto plan =
      tssi_bb_plan(_tx_power_targets, channel.Channel, _rfe_type,
                   _fr_tssi_path, max_target_qdbm, offset_qdb);
  if (!plan)
    return false;
  /* read_txagc_state's rate_diffs array IS 0x3a00..0x3a10 — the same five
   * dwords that carry the loop's per-rate targets while tracking is on. */
  return read_txagc_state().rate_diffs == plan->rate_offsets;
}

bool Phy8733b::disable_tssi_tracking() {
  if (!_tssi_digital_snapshot || !_tssi_analog_snapshot)
    return true;

  const TssiBbState8733b digital_snapshot = *_tssi_digital_snapshot;
  const TssiAnalogState8733b analog_snapshot = *_tssi_analog_snapshot;
  set_bb(0x4318, 0x70000000u, 0);
  restore_tssi_analog(analog_snapshot, _tssi_is_2g);
  for (size_t i = 0; i < digital_snapshot.rate_offsets.size(); ++i)
    set_bb(static_cast<uint16_t>(0x3a00 + i * 4), kDwordMask,
           digital_snapshot.rate_offsets[i]);
  set_bb(0x4320, kDwordMask, digital_snapshot.reg_4320);
  set_bb(0x439c, kDwordMask, digital_snapshot.reg_439c);

  const TssiBbState8733b restored_digital = read_tssi_bb_state();
  const TssiAnalogState8733b restored_analog = read_tssi_analog_state();
  const bool ok = !restored_digital.enabled &&
                  restored_digital.rate_offsets ==
                      digital_snapshot.rate_offsets &&
                  restored_digital.reg_4320 == digital_snapshot.reg_4320 &&
                  restored_digital.reg_439c == digital_snapshot.reg_439c &&
                  restored_analog.matches_snapshot(analog_snapshot);
  if (ok) {
    _tssi_digital_snapshot.reset();
    _tssi_analog_snapshot.reset();
  }
  _fr_tssi_offsets.reset();
  _fr_tssi_de.reset();
  _fr_tssi_power.reset();
  _logger->info("RTL8733B TSSI tracking disabled: rollback={}", ok);
  devourer::Ev(_logger->events(), "rtl8733b.tssi_tracking")
      .f("enabled", false)
      .f("rollback", ok);
  return ok;
}

TssiDeState8733b Phy8733b::read_tssi_de_state() {
  TssiDeState8733b state;
  state.enabled = get_bb(0x4318, 0x70000000u) != 0;
  state.path_a_lanes = _device.rtw_read32(0x43b0);
  state.path_b_lanes = _device.rtw_read32(0x43b4);
  state.de.cck[0] = static_cast<int8_t>(get_bb(0x433c, 0x0ff00000u));
  state.de.cck[1] = static_cast<int8_t>(get_bb(0x434c, 0x0ff00000u));
  state.de.ht40[0] = static_cast<int8_t>(get_bb(0x4334, 0x0ff00000u));
  state.de.ht40[1] = static_cast<int8_t>(get_bb(0x4344, 0x0ff00000u));
  state.de.ofdm[0] = static_cast<int8_t>(get_bb(0x43b0, 0x000000ffu));
  state.de.ofdm[1] = static_cast<int8_t>(get_bb(0x43b4, 0x0000ff00u));
  state.de.ht20[0] = static_cast<int8_t>(get_bb(0x43b0, 0xff000000u));
  state.de.ht20[1] = static_cast<int8_t>(get_bb(0x43b8, 0x000000ffu));
  return state;
}

bool Phy8733b::prepare_tssi_offsets(SelectedChannel channel,
                                    const EfuseInfo &efuse) {
  const auto channel_cfg = planned(channel);
  const auto de = tssi_de_plan(efuse.tssi_power, channel.Channel);
  if (!_initialized || !channel_cfg || !de ||
      efuse.tx_power_mode != TxPowerPgMode8733b::TssiOffset ||
      !efuse.tssi_power_parsed) {
    _logger->error(
        "RTL8733B TSSI-DE: reject initialized={} channel={} mode={} parsed={}",
        _initialized, channel.Channel,
        static_cast<unsigned>(efuse.tx_power_tracking_mode),
        efuse.tssi_power_parsed);
    return false;
  }
  const ChannelState8733b channel_state = read_channel_state();
  if (!channel_state.matches(*channel_cfg)) {
    _logger->error("RTL8733B TSSI-DE: channel state does not match request");
    return false;
  }

  /* Program factory DE values while closed-loop TSSI remains explicitly off.
   * This is the reversible boundary before analog setup, DCK, thermal tables,
   * slope setup, and final enable are ported and validated separately. */
  set_bb(0x4318, 0x70000000u, 0);
  set_bb(0x433c, 0x0ff00000u, static_cast<uint8_t>(de->cck[0]));
  set_bb(0x434c, 0x0ff00000u, static_cast<uint8_t>(de->cck[1]));
  set_bb(0x4334, 0x0ff00000u, static_cast<uint8_t>(de->ht40[0]));
  set_bb(0x4344, 0x0ff00000u, static_cast<uint8_t>(de->ht40[1]));
  set_bb(0x43b0, 0x000000ffu, static_cast<uint8_t>(de->ofdm[0]));
  set_bb(0x43b0, 0x0000ff00u, static_cast<uint8_t>(de->ht40[0]));
  set_bb(0x43b0, 0x00ff0000u, static_cast<uint8_t>(de->ht40[0]));
  set_bb(0x43b4, 0x0000ff00u, static_cast<uint8_t>(de->ofdm[1]));
  set_bb(0x43b4, 0x000000ffu, static_cast<uint8_t>(de->ht40[1]));
  set_bb(0x43b4, 0x00ff0000u, static_cast<uint8_t>(de->ht40[1]));
  set_bb(0x43b4, 0xff000000u, static_cast<uint8_t>(de->ht40[1]));
  set_bb(0x43b0, 0xff000000u, static_cast<uint8_t>(de->ht20[0]));
  set_bb(0x43b8, 0x000000ffu, static_cast<uint8_t>(de->ht20[1]));

  const TssiDeState8733b state = read_tssi_de_state();
  const bool ok = state.matches_disabled(*de);
  if (ok) {
    /* fast_retune bookkeeping: the DE plan the chip now carries and the
     * EFUSE calibration that derives it, for the in-place per-channel
     * rewrite on bucket-crossing hops. */
    _fr_tssi_de = *de;
    _fr_tssi_power = efuse.tssi_power;
  }
  _logger->info(
      "RTL8733B TSSI-DE: ready={} enabled={} ch={} cck={}/{} ht40={}/{} "
      "ofdm={}/{} ht20={}/{}",
      ok, state.enabled, channel.Channel, state.de.cck[0], state.de.cck[1],
      state.de.ht40[0], state.de.ht40[1], state.de.ofdm[0], state.de.ofdm[1],
      state.de.ht20[0], state.de.ht20[1]);
  devourer::Ev(_logger->events(), "rtl8733b.tssi_de")
      .f("ok", ok)
      .f("enabled", state.enabled)
      .f("channel", channel.Channel)
      .f("cck_a", state.de.cck[0])
      .f("cck_b", state.de.cck[1])
      .f("ht40_a", state.de.ht40[0])
      .f("ht40_b", state.de.ht40[1])
      .f("ofdm_a", state.de.ofdm[0])
      .f("ofdm_b", state.de.ofdm[1])
      .f("ht20_a", state.de.ht20[0])
      .f("ht20_b", state.de.ht20[1]);
  return ok;
}

uint8_t Phy8733b::read_thermal() {
  if (!_initialized)
    return 0;
  /* Vendor _dpk_thermal_read_8733b: the die has one meter on RF path A.
   * Pulse RF 0x42[19], wait 15 us, then read the 6-bit value at [6:1]. */
  if (!write_rf(0, 0x42, 1u << 19, 1) ||
      !write_rf(0, 0x42, 1u << 19, 0) ||
      !write_rf(0, 0x42, 1u << 19, 1))
    return 0;
  delay_us(15);
  return static_cast<uint8_t>(read_rf(0, 0x42, 0x7e));
}

bool Phy8733b::set_channel(SelectedChannel channel) {
  const auto plan = planned(channel);
  if (!_initialized || !plan) {
    _logger->error(
        "RTL8733B channel: reject primary={} width={} offset={} initialized={}",
        channel.Channel, static_cast<unsigned>(channel.ChannelWidth),
        channel.ChannelOffset, _initialized);
    return false;
  }
  if (!switch_band(*plan) || !switch_channel(*plan) ||
      !switch_bandwidth(*plan))
    return false;

  // HALMAC cfg_ch_bw_87xx: primary index, bandwidth, band, and MAC clocks.
  _device.rtw_write8(0x0483, plan->primary_index);
  set_bb(0x0668, (1u << 7) | (1u << 8),
         plan->width == CHANNEL_WIDTH_40 ? 1 : 0);
  _device.rtw_write8(0x0026,
                     static_cast<uint8_t>((_device.rtw_read8(0x0026) & 0xcfu) |
                                          0x10u));
  _device.rtw_write8(0x1008,
                     static_cast<uint8_t>(_device.rtw_read8(0x1008) & 0xf3u));
  _device.rtw_write8(0x055c, 0x14);
  _device.rtw_write8(0x0638, 0x14);

  const ChannelState8733b state = read_channel_state();
  const bool ok = state.matches(*plan);
  /* fast_retune bookkeeping: record where the radio is, and invalidate the
   * RF compose cache — the switch functions above rewrote both words, so a
   * cached copy is stale until the next fast hop re-primes it. */
  if (ok)
    _fr_plan = *plan;
  else
    _fr_plan.reset();
  _fr_rf18.reset();
  _fr_rf19.reset();
  _logger->info(
      "RTL8733B channel: ready={} primary={} center={} width={} offset={} "
      "RF18={:05x}/{:05x} RF19={:05x}/{:05x} BB9B0={:08x} "
      "BB9B4={:08x} BB9F0={:08x} BB81C={:08x} DATA_SC={:02x} "
      "WMAC={:08x} synth={}",
      ok, plan->primary, plan->center,
      width_mhz(plan->width), plan->offset,
      state.rf_a_18, state.rf_b_18, state.rf_a_19, state.rf_b_19,
      state.bb_9b0, state.bb_9b4, state.bb_9f0, state.bb_81c,
      state.data_sc, state.wmac_trxptcl,
      state.synth_ready);
  devourer::Ev(_logger->events(), "rtl8733b.channel")
      .f("ok", ok).f("primary", plan->primary).f("center", plan->center)
      .f("width", width_mhz(plan->width))
      .f("offset", plan->offset).hexf("rf_a18", state.rf_a_18, 5)
      .hexf("rf_b18", state.rf_b_18, 5).hexf("rf_a19", state.rf_a_19, 5)
      .hexf("rf_b19", state.rf_b_19, 5).hexf("bb_9b0", state.bb_9b0, 8)
      .hexf("bb_9b4", state.bb_9b4, 8).hexf("bb_9f0", state.bb_9f0, 8)
      .hexf("bb_81c", state.bb_81c, 8)
      .hexf("data_sc", state.data_sc, 2)
      .hexf("wmac", state.wmac_trxptcl, 8).f("synth", state.synth_ready);
  return ok;
}

bool Phy8733b::fast_retune(SelectedChannel channel, bool tssi_live,
                           uint8_t max_target_qdbm, bool cache_rf,
                           int offset_qdb) {
  const auto plan = planned(channel);
  if (!_initialized || !plan || !_fr_plan)
    return false;
  const ChannelPlan8733b cur = *_fr_plan;
  if (plan->is_2g != cur.is_2g || plan->width != cur.width ||
      plan->offset != cur.offset)
    return false;
  if (plan->center == cur.center && plan->primary == cur.primary)
    return true;

  /* Everything that can refuse is computed BEFORE the first chip write, so a
   * declined hop leaves the radio untouched for the caller's full-path
   * fallback rather than half-hopped. */
  std::optional<TssiBbPlan8733b> tssi;
  std::optional<TssiDePlan8733b> de;
  if (tssi_live && _fr_tssi_offsets) {
    /* The live runtime offset rides along: without it a hop would recompute
     * the targets from the bare ceiling and silently walk the caller's TX-power
     * backoff back up. */
    tssi = tssi_bb_plan(_tx_power_targets, channel.Channel, _rfe_type,
                        _fr_tssi_path, max_target_qdbm, offset_qdb);
    if (!tssi)
      return false;
  }
  if (tssi_live && _fr_tssi_de && _fr_tssi_power) {
    de = tssi_de_plan(*_fr_tssi_power, channel.Channel);
    if (!de)
      return false;
  }
  if (!cache_rf || !_fr_rf18) {
    const uint32_t rf18 = read_rf(0, 0x18);
    if (rf18 == 0xffffffffu)
      return false;
    _fr_rf18 = rf18;
  }
  if (!cache_rf || !_fr_rf19) {
    const uint32_t rf19 = read_rf(0, 0x19);
    if (rf19 == 0xffffffffu)
      return false;
    _fr_rf19 = rf19;
  }

  uint32_t rf18 = *_fr_rf18;
  uint32_t rf19 = *_fr_rf19;
  if (plan->is_2g) {
    rf18 &= ~((3u << 16) | (3u << 8) | 0xffu);
    rf18 |= plan->center;
  } else {
    rf18 &= ~((1u << 17) | (1u << 9) | 0xffu);
    rf18 |= (1u << 16) | (1u << 8) | plan->center;
    rf19 &= ~((1u << 19) | (1u << 18));
    if (plan->center > 144)
      rf19 |= 1u << 19;
    else if (plan->center > 80)
      rf19 |= 1u << 18;
  }
  /* From the first chip write onward a transport failure leaves the radio
   * part-hopped, so the catch drops the fast-path bookkeeping: the next call
   * declines (stale _fr_plan gone) and the caller's full-path fallback
   * reprograms everything, instead of a later fast hop bucket-comparing
   * against a baseline the chip no longer holds. */
  try {
    if (!program_synth(rf18)) {
      _fr_plan.reset();
      return false;
    }
    _fr_rf18 = rf18;
    if (rf19 != *_fr_rf19) {
      write_rf(0, 0x19, kRfMask, rf19);
      write_rf(1, 0x19, kRfMask, rf19);
      _fr_rf19 = rf19;
    }

    /* Channel-keyed constants from switch_channel, written only when their
     * bucket changes across the hop. Band-keyed constants (0x1ea8, the
     * 2.4 GHz 0x18ac pair, spur cancellation) were set by the last full set
     * at this band and are untouched — that is the fast path's contract. */
    if (plan->is_2g) {
      if (plan->center <= 14) {
        set_bb(0x2a38, 0x07ffff00u, kScoFc2a38_2g[plan->center]);
        set_bb(0x2a3c, 0x000fffffu, kScoFc2a3c_2g[plan->center]);
      }
      if ((plan->center == 14) != (cur.center == 14)) {
        const auto shape = tx_shape_2g(plan->center);
        for (unsigned i = 0; i < 8; ++i)
          set_bb(static_cast<uint16_t>(0x1a00 + i * 4),
                 i == 2 ? kDwordMask : 0x00ffffffu, shape[i]);
      }
    } else {
      if (agc_bucket_5g(plan->center) != agc_bucket_5g(cur.center))
        set_bb(0x18ac, 0x000001f0u, agc_bucket_5g(plan->center));
    }
    if (sco_bucket_0c30(plan->center) != sco_bucket_0c30(cur.center))
      set_bb(0x0c30, 0xfffu, sco_bucket_0c30(plan->center));
    if ((plan->center == 13) != (cur.center == 13))
      set_bb(0x0808, 0x7fu, plan->center == 13 ? 0x30 : 0x40);

    bb_reset();
    igi_toggle();

    /* TSSI: tracking stays enabled; the per-channel rate-offset dwords and
     * the channel-bucketed DE offsets are rewritten, in place, each only
     * when the new channel's plan differs (the #389 shape). Intra-band the
     * rate offsets are band-keyed and never change — the DE buckets, ~3
     * channels wide at 2.4 GHz, are the calibration a hop actually moves.
     * The rollback snapshots are untouched — they record the pre-enable
     * state, which these rewrites do not change. */
    if (tssi && tssi->rate_offsets != *_fr_tssi_offsets) {
      for (size_t i = 0; i < tssi->rate_offsets.size(); ++i)
        set_bb(static_cast<uint16_t>(0x3a00 + i * 4), kDwordMask,
               tssi->rate_offsets[i]);
      _fr_tssi_offsets = tssi->rate_offsets;
    }
    if (de && !(*de == *_fr_tssi_de)) {
      /* The prepare_tssi_offsets field sequence, minus its 0x4318
       * tracking-disable write — the loop stays live across the rewrite. */
      set_bb(0x433c, 0x0ff00000u, static_cast<uint8_t>(de->cck[0]));
      set_bb(0x434c, 0x0ff00000u, static_cast<uint8_t>(de->cck[1]));
      set_bb(0x4334, 0x0ff00000u, static_cast<uint8_t>(de->ht40[0]));
      set_bb(0x4344, 0x0ff00000u, static_cast<uint8_t>(de->ht40[1]));
      set_bb(0x43b0, 0x000000ffu, static_cast<uint8_t>(de->ofdm[0]));
      set_bb(0x43b0, 0x0000ff00u, static_cast<uint8_t>(de->ht40[0]));
      set_bb(0x43b0, 0x00ff0000u, static_cast<uint8_t>(de->ht40[0]));
      set_bb(0x43b4, 0x0000ff00u, static_cast<uint8_t>(de->ofdm[1]));
      set_bb(0x43b4, 0x000000ffu, static_cast<uint8_t>(de->ht40[1]));
      set_bb(0x43b4, 0x00ff0000u, static_cast<uint8_t>(de->ht40[1]));
      set_bb(0x43b4, 0xff000000u, static_cast<uint8_t>(de->ht40[1]));
      set_bb(0x43b0, 0xff000000u, static_cast<uint8_t>(de->ht20[0]));
      set_bb(0x43b8, 0x000000ffu, static_cast<uint8_t>(de->ht20[1]));
      _fr_tssi_de = *de;
    }
    _fr_plan = *plan;
    return true;
  } catch (...) {
    _fr_plan.reset();
    _fr_rf18.reset();
    _fr_rf19.reset();
    throw;
  }
}

bool Phy8733b::initialize(uint8_t cut, const EfuseInfo &efuse) {
  if (!efuse.valid || efuse.rfe_type == 0xff || efuse.xtal == 0xff) {
    _logger->error("RTL8733B PHY: refusing initialization without EFUSE data");
    return false;
  }

  const JaguarPhyContext ctx = table_context(cut, efuse.rfe_type);
  const PhyTableCounts counts = count_selected_tables(cut, efuse.rfe_type);
  if (!parse_tx_power_targets(array_mp_8733b_phy_reg_pg,
                              array_mp_8733b_phy_reg_pg_len,
                              _tx_power_targets) ||
      !_tx_power_targets.usable()) {
    _logger->error("RTL8733B PHY: target-power table is invalid");
    return false;
  }
  _logger->info(
      "RTL8733B PHY: tables selected cut={} rfe={} mac={} phy={} agc={} "
      "targets={} rfk-init={} rf-a={} rf-b={}",
      cut, efuse.rfe_type, counts.mac, counts.phy, counts.agc,
      counts.power_targets, counts.cal_init, counts.radio_a, counts.radio_b);

  // rtl8733b_phy_init calls rtw_halmac_phy_power_switch(true) before PRE.
  // This is load-bearing on warm init because system pre-init deliberately
  // cleared these gates while deciding whether card-enable was needed.
  _device.rtw_write8(0x0002,
                     static_cast<uint8_t>(_device.rtw_read8(0x0002) | 0x03u));
  _device.rtw_write8(0x001f,
                     static_cast<uint8_t>(_device.rtw_read8(0x001f) | 0x07u));
  _device.rtw_write32(0x0074,
                      _device.rtw_read32(0x0074) | (7u << 24));

  // rtl8733b_phy_init_mac_register precedes rtl8733b_phy_init in the vendor
  // HAL. Keep its final 0xffff row: the vendor walker performs that byte write.
  PhyTableLoader::Load(
      array_mp_8733b_mac_reg, array_mp_8733b_mac_reg_len, ctx,
      [this](uint32_t addr, uint32_t data) {
        if (addr > 0xffffu)
          throw std::runtime_error("RTL8733B MAC table address is invalid");
        _device.rtw_write8(static_cast<uint16_t>(addr),
                           static_cast<uint8_t>(data));
      });

  parameter_gate(false);
  PhyTableLoader::Load(array_mp_8733b_phy_reg, array_mp_8733b_phy_reg_len,
                       ctx, [this](uint32_t a, uint32_t v) {
                         bb_table_write(a, v);
                       });
  PhyTableLoader::Load(array_mp_8733b_agc_tab, array_mp_8733b_agc_tab_len,
                       ctx, [this](uint32_t a, uint32_t v) {
                         bb_table_write(a, v);
                       });
  set_crystal_cap(efuse.xtal);

  // CONFIG_BB_RF_CAL_INIT is a static BB/NCTL image applied before radioa/b;
  // it does not execute the later, environment-dependent IQK/DPK routines.
  PhyTableLoader::Load(array_mp_8733b_cal_init,
                       array_mp_8733b_cal_init_len, ctx,
                       [this](uint32_t a, uint32_t v) {
                         bb_table_write(a, v);
                       });
  PhyTableLoader::Load(array_mp_8733b_radioa, array_mp_8733b_radioa_len, ctx,
                       [this](uint32_t a, uint32_t v) {
                         rf_table_write(0, a, v);
                       });
  PhyTableLoader::Load(array_mp_8733b_radiob, array_mp_8733b_radiob_len, ctx,
                       [this](uint32_t a, uint32_t v) {
                         rf_table_write(1, a, v);
                       });
  parameter_gate(true);

  const PhyState state = read_state();
  const bool ok = state.matches(efuse.xtal);
  _cut = cut;
  _rfe_type = efuse.rfe_type;
  _initialized = ok;
  _logger->info(
      "RTL8733B PHY: ready={} SYS_FUNC=0x{:04x} RF_CTRL=0x{:02x} "
      "HCI_OPT=0x{:08x} BB_EN=0x{:08x} 3WIRE=0x{:08x} "
      "XTAL=0x{:08x} RF-A57=0x{:05x} RF-B57=0x{:05x} "
      "RF-A87=0x{:05x} RF-B87=0x{:05x} RF-AEF=0x{:05x} "
      "RF-BEF=0x{:05x} RF-B66=0x{:05x}",
      ok, state.sys_func_en, state.rf_ctrl, state.hci_opt_ctrl,
      state.bb_enable, state.wire_control, state.xtal_control,
      state.rf_a_57, state.rf_b_57, state.rf_a_87, state.rf_b_87,
      state.rf_a_ef, state.rf_b_ef, state.rf_b_66);
  devourer::Ev(_logger->events(), "rtl8733b.phy")
      .f("ok", ok)
      .f("cut", cut)
      .hexf("rfe_type", efuse.rfe_type, 2)
      .f("mac_rows", counts.mac)
      .f("phy_rows", counts.phy)
      .f("agc_rows", counts.agc)
      .f("target_rows", counts.power_targets)
      .f("rfk_init_rows", counts.cal_init)
      .f("radio_a_rows", counts.radio_a)
      .f("radio_b_rows", counts.radio_b)
      .hexf("sys_func_en", state.sys_func_en, 4)
      .hexf("rf_ctrl", state.rf_ctrl, 2)
      .hexf("hci_opt_ctrl", state.hci_opt_ctrl, 8)
      .hexf("bb_enable", state.bb_enable, 8)
      .hexf("wire_control", state.wire_control, 8)
      .hexf("xtal_control", state.xtal_control, 8)
      .hexf("rf_a_57", state.rf_a_57, 5)
      .hexf("rf_b_57", state.rf_b_57, 5)
      .hexf("rf_a_87", state.rf_a_87, 5)
      .hexf("rf_b_87", state.rf_b_87, 5)
      .hexf("rf_a_ef", state.rf_a_ef, 5)
      .hexf("rf_b_ef", state.rf_b_ef, 5)
      .hexf("rf_b_66", state.rf_b_66, 5);
  return ok;
}

} // namespace rtl8733b
