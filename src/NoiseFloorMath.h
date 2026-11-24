/* Verbatim ports of the two phydm_math_lib helpers the active idle-noise-floor
 * measurement needs (reference/rtl8812au/hal/phydm/phydm_math_lib.c). Kept in a
 * standalone header so the per-generation noise-floor code and a selftest share
 * one copy. See docs/rx-spectrum-sensing.md.
 *
 * These drive the Jaguar1 (8812A/8821A) active-sampling path: the 0x0FA0 debug
 * port packs rx I/Q as 10-bit signed (sign_conversion), and the per-sample power
 * is 10*log10(I^2 + Q^2) approximated as 3*log2 via the leading set bit
 * (pwdb_conversion). */
#ifndef DEVOURER_NOISE_FLOOR_MATH_H
#define DEVOURER_NOISE_FLOOR_MATH_H

#include <cstdint>

namespace devourer {
namespace nf {

/* odm_pwdb_conversion: Y = 10*log10(X) ~= 3*log2(X), read off the leading set
 * bit. `total_bit` bounds the bit search; `decimal_bit` is the fixed-point
 * fractional width of X. The measurement forms S(20,18) = I^2 + Q^2, so it is
 * called with (X, 20, 18). The next-lower bit adds a coarse ~2 dB decimal. */
inline int32_t pwdb_conversion(int32_t X, uint32_t total_bit,
                               uint32_t decimal_bit) {
  int32_t integer = 0, decimal = 0;
  if (X == 0)
    X = 1; /* log2(x): x can't be 0 */
  for (uint32_t i = (total_bit - 1); i > 0; i--) {
    if (X & (int32_t{1} << i)) {
      integer = static_cast<int32_t>(i);
      decimal = (X & (int32_t{1} << (i - 1))) ? 2 : 0;
      break;
    }
  }
  return 3 * (integer - static_cast<int32_t>(decimal_bit)) + decimal;
}

/* odm_sign_conversion: interpret the low `total_bit` bits of `value` as a
 * two's-complement signed integer (rxi/rxq are 10-bit signed on the debug
 * port). */
inline int32_t sign_conversion(int32_t value, uint32_t total_bit) {
  if (value & (int32_t{1} << (total_bit - 1)))
    value -= (int32_t{1} << total_bit);
  return value;
}

/* Absolute NHM thresholds (Jaguar3 busy-airtime NHM, IRtlRadio::ArmNhmBusy).
 * read_nhm (NhmReader.h) sets its thresholds from the live IGI, so a bucket
 * means a different dBm whenever IGI moves; these are fixed. Unit PWdB
 * U(8,1): th = 2*(dBm + 110). phydm's 11k table starts at -92 dBm, which
 * clips a real 20 MHz 5 GHz floor (~-95); this one reaches -104 in the same
 * 3 dB steps (phydm's NHM_DBG 1-dB mode shows the thresholds are free),
 * 5 dB steps above -80, top bucket = above -70. */
inline constexpr uint8_t nhm_th_from_dbm(int dbm) {
  return static_cast<uint8_t>(2 * (dbm + 110));
}

inline constexpr int8_t kNhmAbsThDbm[11] = {-104, -101, -98, -95, -92, -89,
                                             -86,  -83,  -80, -75, -70};

inline void nhm_abs_thresholds(uint8_t th[11]) {
  for (int i = 0; i < 11; i++)
    th[i] = nhm_th_from_dbm(kNhmAbsThDbm[i]);
}

/* First bucket whose lower edge is `dbm` (bucket i >= 1 starts at
 * kNhmAbsThDbm[i-1]), or -1 when `dbm` is not one of the edges: the share of
 * buckets [result..11] is the airtime above `dbm`. */
inline constexpr int nhm_abs_bucket_from(int dbm) {
  for (int i = 0; i < 11; i++)
    if (kNhmAbsThDbm[i] == dbm)
      return i + 1;
  return -1;
}

} // namespace nf
} // namespace devourer

#endif /* DEVOURER_NOISE_FLOOR_MATH_H */
