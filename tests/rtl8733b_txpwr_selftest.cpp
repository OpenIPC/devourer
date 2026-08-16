/* Runtime TX-power offset math for the RTL8733B (devourer#1).
 *
 * On a TSSI-offset PG unit the closed loop is the TX-power control, so the
 * runtime offset moves the loop's per-rate target table rather than a TXAGC
 * index. Phy8733b::tssi_rate_offsets is pure and static, which makes the whole
 * contract testable headless: cap first, then shift what survives the cap, and
 * report the rails.
 *
 * The load-bearing cell is "shape survives the shift" — an implementation that
 * lowered the ceiling instead of shifting would pass every uniform-target case
 * here and still silently flatten the calibrated per-rate spread.
 */
#include <cstdint>
#include <cstdio>

#include "hal8733b_tables.h"
#include "rtl8733b/Phy8733b.h"

namespace {
int failures = 0;

void expect(const char *what, bool condition) {
  if (condition)
    return;
  ++failures;
  std::printf("FAIL: %s\n", what);
}

/* Synthetic 2.4 GHz targets: a flat ladder with one rate deliberately
 * calibrated below the safe ceiling, so the shape assertions have something to
 * preserve. Quarter-dBm, like the generated table. */
rtl8733b::TxPowerTargets8733b synthetic_targets(uint8_t low_rate_qdbm) {
  rtl8733b::TxPowerTargets8733b t;
  for (uint8_t path = 0; path < 2; ++path) {
    for (size_t rate = 0; rate < 20; ++rate)
      t.qdbm[0][path][rate] = 80; /* 20 dBm — above the 16 dBm ceiling */
    t.qdbm[0][path][7] = low_rate_qdbm;
    t.present[0][path] = true;
  }
  return t;
}
} // namespace

int main() {
  constexpr uint8_t kCeiling = rtl8733b::kSafeTssiTargetQdbm8733b; /* 64 */
  const rtl8733b::TxPowerTargets8733b t = synthetic_targets(60);

  /* 1. Offset 0 is the pre-knob table, byte for byte — the no-change control.
   *    Compared against the default-argument call, which is what every
   *    existing caller compiles to. */
  const auto base = rtl8733b::Phy8733b::tssi_rate_offsets(t, 0, 0, kCeiling);
  const auto base_explicit_zero =
      rtl8733b::Phy8733b::tssi_rate_offsets(t, 0, 0, kCeiling, 0);
  expect("offset 0 reproduces the pre-knob table",
         base && base_explicit_zero && *base == *base_explicit_zero);
  expect("ceiling caps a hot rate at the anchor", base && (*base)[0] == 0);
  expect("a rate calibrated below the ceiling keeps its own level",
         base && (*base)[7] == -4);

  /* 2. A uniform shift moves every capped rate by exactly the offset. */
  const auto down24 =
      rtl8733b::Phy8733b::tssi_rate_offsets(t, 0, 0, kCeiling, -24);
  expect("-24 qdB shifts the capped rates by -24",
         down24 && (*down24)[0] == -24 && (*down24)[19] == -24);

  /* 3. THE cell: the calibrated spread survives the shift. A lowered-ceiling
   *    implementation would leave rate 7 at -4 while the rest moved to -24. */
  expect("per-rate shape survives the shift",
         down24 && (*down24)[7] == -28 &&
             ((*down24)[0] - (*down24)[7]) == ((*base)[0] - (*base)[7]));

  /* 4. The floor is the int8 delta field, not the 0 dBm target. -64 qdB lands
   *    the anchor rates on a 0 dBm target and rails NOTHING: the ladder keeps
   *    its calibrated spread straight through it, because the loop was measured
   *    to keep reducing power well past that point. */
  rtl8733b::TssiOffsetSat8733b sat_clean;
  const auto floor_ok =
      rtl8733b::Phy8733b::tssi_rate_offsets(t, 0, 0, kCeiling, -64,
                                            &sat_clean);
  expect("-64 qdB is not a rail — the ladder passes through the 0 dBm target",
         floor_ok && (*floor_ok)[0] == -64 && (*floor_ok)[7] == -68 &&
             !sat_clean.low && !sat_clean.high);

  /* Well below it the spread is still intact — this is a shift, not a squeeze
   * against a floor. */
  const auto deep =
      rtl8733b::Phy8733b::tssi_rate_offsets(t, 0, 0, kCeiling, -100);
  expect("the shape survives below the 0 dBm target",
         deep && (*deep)[0] == -100 && (*deep)[7] == -104 &&
             ((*deep)[0] - (*deep)[7]) == ((*base)[0] - (*base)[7]));

  /* The real rail is the field: at -128 the hot rate sits exactly on it and the
   * colder one cannot follow, which is what saturated_low is for. */
  rtl8733b::TssiOffsetSat8733b sat_rail;
  const auto railed =
      rtl8733b::Phy8733b::tssi_rate_offsets(t, 0, 0, kCeiling, -128,
                                            &sat_rail);
  expect("-128 qdB sits on the int8 field floor",
         railed && (*railed)[0] == -128);
  expect("the colder rate rails there and is reported",
         railed && (*railed)[7] == -128 && sat_rail.low && !sat_rail.high);

  rtl8733b::TssiOffsetSat8733b sat_zero;
  const auto no_shift = rtl8733b::Phy8733b::tssi_rate_offsets(
      t, 0, 0, kCeiling, 0, &sat_zero);
  expect("offset 0 rails nothing",
         no_shift && !sat_zero.low && !sat_zero.high);

  /* 4b. The knob is symmetric: a positive offset shifts the same ladder up,
   *     shape intact, and is NOT re-clamped at the vendor's factory target.
   *     That last part is the point — a per-unit EFUSE trimmed too cold is the
   *     case an operator calibrates their own operating point for, and this is
   *     the only lever this backend gives them to command it. */
  rtl8733b::TssiOffsetSat8733b sat_up;
  const auto up16 = rtl8733b::Phy8733b::tssi_rate_offsets(t, 0, 0, kCeiling,
                                                          16, &sat_up);
  expect("+16 qdB shifts the ladder up, shape intact",
         up16 && (*up16)[0] == 16 && (*up16)[7] == 12 &&
             ((*up16)[0] - (*up16)[7]) == ((*base)[0] - (*base)[7]));
  expect("a positive offset within the field rails nothing",
         up16 && !sat_up.high && !sat_up.low);
  /* Past the factory target too: the synthetic ladder's rates are calibrated
   * at 80 qdBm, and +32 commands 96 — uncalibrated by construction, reachable
   * by design, compliance the caller's (src/TxPower.h). */
  const auto up32 =
      rtl8733b::Phy8733b::tssi_rate_offsets(t, 0, 0, kCeiling, 32);
  expect("the offset can be commanded past the factory target",
         up32 && (*up32)[0] == 32);

  /* 5. The int8 field's positive end is reachable only above the anchor, which
   *    is what the raised-ceiling default argument does. */
  rtl8733b::TxPowerTargets8733b hot;
  for (uint8_t path = 0; path < 2; ++path) {
    for (size_t rate = 0; rate < 20; ++rate)
      hot.qdbm[0][path][rate] = 254;
    hot.present[0][path] = true;
  }
  rtl8733b::TssiOffsetSat8733b sat_hot;
  const auto clipped =
      rtl8733b::Phy8733b::tssi_rate_offsets(hot, 0, 0, 0xff, 0, &sat_hot);
  expect("a target past the int8 delta field clamps and reports",
         clipped && (*clipped)[0] == 127 && sat_hot.high && !sat_hot.low);

  /* 6. The generated table, capped and shifted: parity at 0 against the values
   *    the phy-table selftest pins, then the same uniform shift. */
  rtl8733b::TxPowerTargets8733b real;
  expect("generated target-power table parses",
         rtl8733b::Phy8733b::parse_tx_power_targets(
             array_mp_8733b_phy_reg_pg, array_mp_8733b_phy_reg_pg_len, real));
  const auto real_base =
      rtl8733b::Phy8733b::tssi_rate_offsets(real, 0, 0, kCeiling);
  const auto real_down =
      rtl8733b::Phy8733b::tssi_rate_offsets(real, 0, 0, kCeiling, -24);
  expect("generated 2G table is flat at the ceiling with no offset",
         real_base && (*real_base)[0] == 0 && (*real_base)[19] == 0);
  expect("generated 2G table shifts uniformly",
         real_down && (*real_down)[0] == -24 && (*real_down)[19] == -24);

  /* kMaxPgTargetQdbm8733b is the figure the caps comment and docs quote as
   * where the vendor's calibration ends, so it must stay pinned to the
   * generated table: a regeneration that moves the highest factory target has
   * to fail here rather than let the documented number drift off the data. */
  int table_max = -1;
  for (uint8_t band = 0; band < 2; ++band)
    for (uint8_t path = 0; path < 2; ++path) {
      if (!real.present[band][path])
        continue;
      for (size_t rate = 0; rate < 20; ++rate) {
        const uint8_t v = real.qdbm[band][path][rate];
        if (v != 0xff && static_cast<int>(v) > table_max)
          table_max = v;
      }
    }
  expect("kMaxPgTargetQdbm8733b matches the generated table's highest target",
         table_max == static_cast<int>(rtl8733b::kMaxPgTargetQdbm8733b));

  /* The generated 2G table is flat against the 16 dBm clip, so a +16 qdB
   * command lands every rate at 20 dBm — at the top of the vendor's range for
   * the hottest rate and ABOVE it for the rest. That is the shape a
   * calibrating operator asks for, and it is why the positive half is not
   * re-clamped per rate. */
  const auto real_up =
      rtl8733b::Phy8733b::tssi_rate_offsets(real, 0, 0, kCeiling, 16);
  expect("+16 qdB commands the 2G ladder to the top of the PG range",
         real_up && (*real_up)[0] == 16 && (*real_up)[19] == 16);

  /* 5 GHz has no CCK targets; those four entries stay inert at the anchor
   * whatever the offset, exactly as they did before the knob existed. */
  const auto real_5g_base =
      rtl8733b::Phy8733b::tssi_rate_offsets(real, 1, 0, kCeiling);
  const auto real_5g_down =
      rtl8733b::Phy8733b::tssi_rate_offsets(real, 1, 0, kCeiling, -24);
  expect("5G CCK entries stay inert across the shift",
         real_5g_base && real_5g_down && (*real_5g_base)[0] == 0 &&
             (*real_5g_down)[0] == 0);
  expect("5G OFDM shifts and the colder MCS7 keeps its distance",
         real_5g_down && (*real_5g_down)[4] == -24 &&
             ((*real_5g_base)[4] - (*real_5g_base)[19]) ==
                 ((*real_5g_down)[4] - (*real_5g_down)[19]));

  /* 7. The packed BB plan carries the shift through to the five dwords the
   *    actuator writes, and nothing else in the plan moves with power. */
  const auto plan_base = rtl8733b::Phy8733b::tssi_bb_plan(real, 1, 0, 0,
                                                          kCeiling);
  const auto plan_down =
      rtl8733b::Phy8733b::tssi_bb_plan(real, 1, 0, 0, kCeiling, -24);
  expect("BB plan packs the shifted targets",
         plan_base && plan_down &&
             plan_base->rate_offsets != plan_down->rate_offsets &&
             plan_down->rate_offsets[0] == 0xe8e8e8e8u);
  expect("only the rate-offset dwords depend on power",
         plan_base && plan_down && plan_base->reg_4308 == plan_down->reg_4308 &&
             plan_base->reg_439c == plan_down->reg_439c &&
             plan_base->reg_43a8 == plan_down->reg_43a8);

  return failures == 0 ? 0 : 1;
}
