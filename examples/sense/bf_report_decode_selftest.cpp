/* Headless self-test for src/BfReportDecode.h — guards the C++ port of
 * tools/bf_report_decode.py against regressions. Registered as a ctest, so a
 * decode break fails CI instead of only surfacing on a radio.
 *
 * Checks: the LSB-first BitReader and the dequant formulas on known inputs, the
 * report header parse + fixed-split angle decode against a real captured MU
 * report (reference psi values computed offline with a fixed (8,2) split), and
 * that the split picker returns a valid split. */
#include "BfReportDecode.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace devourer::bf;

static int failures = 0;
#define CHECK(cond, msg)                                                        \
  do {                                                                          \
    if (!(cond)) {                                                              \
      std::printf("FAIL: %s\n", msg);                                           \
      ++failures;                                                               \
    }                                                                           \
  } while (0)

static bool approx(double a, double b, double tol = 1e-4) {
  return std::fabs(a - b) < tol;
}

static std::vector<uint8_t> from_hex(const std::string &h) {
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < h.size(); i += 2)
    out.push_back((uint8_t)std::strtoul(h.substr(i, 2).c_str(), nullptr, 16));
  return out;
}

/* A real VHT MU Compressed Beamforming report captured from an 8822CU beamformee
 * (20 MHz, Nr=2 Nc=1, MU). psi[0..5] below were computed offline from these bytes
 * with a fixed (b_phi=8, b_psi=2) split. */
static const char *kReportHex =
    "e000000056427505d60000e04c8822ce00000000000010001500088c04c2a98dad97b09fbe"
    "addaadc7bfccbde1c9e5cfe8cdf3cdf1d1eacfe5cff0d5eed9f0d9e6e1ffd70cd820e017d2"
    "25d61ef093f0b9daa1ec91e687dc83e093e058f417ecfbebf9ebe9e1f1db03dc08de0dda11"
    "d814de0fd808d60dd219c815c605b811b01bac20b62ac40f01f00fef00100100ff0011111001cbbf1bd5";

int main() {
  /* 1. BitReader — LSB-first. byte 0xB4 = 1011 0100b; reading 3 bits gives the
   * low three bits in LSB order = 0b100 = 4; next 5 bits = 0b10110 = 22. */
  {
    uint8_t bytes[2] = {0xB4, 0x00};
    BitReader br(bytes, 2);
    CHECK(br.read(3) == 4u, "BitReader low 3 bits");
    CHECK(br.read(5) == 22u, "BitReader next 5 bits");
  }

  /* 2. dequant — psi=(2q+1)pi/2^(b+2), phi=(2q+1)pi/2^b. */
  CHECK(approx(dequant_psi(0, 2), M_PI / 16.0), "dequant_psi q0 b2");
  CHECK(approx(dequant_psi(3, 2), 7.0 * M_PI / 16.0), "dequant_psi q3 b2");
  CHECK(approx(dequant_phi(0, 8), M_PI / 256.0), "dequant_phi q0 b8");

  /* 3. parse_report on the real MU report. */
  std::vector<uint8_t> frame = from_hex(kReportHex);
  ReportHdr hdr;
  CHECK(parse_report(frame.data(), frame.size(), hdr), "parse_report matches");
  CHECK(hdr.nc == 1 && hdr.nr == 2, "nc/nr");
  CHECK(hdr.bw == 0 && hdr.ng == 1, "bw/ng");
  CHECK(hdr.mu && hdr.vht, "mu/vht");
  CHECK(hdr.ns == 52, "ns=52");
  CHECK(hdr.per_sc_bits == 10, "per_sc_bits=10 (MU)");
  CHECK(hdr.angle_len == 65, "angle_len=65 (52*10 bits)");

  /* 4. fixed-split decode vs offline reference. */
  std::vector<double> psi;
  CHECK(decode_psi(hdr, 8, 2, psi), "decode_psi ok");
  CHECK(psi.size() == 52, "52 psi values");
  const double ref[6] = {0.589049, 1.374447, 0.589049,
                         0.981748, 0.981748, 1.374447};
  for (int k = 0; k < 6; ++k)
    CHECK(approx(psi[k], ref[k]), "psi matches reference");
  bool in_range = true;
  for (double p : psi)
    if (p < 0.0 || p > M_PI / 2.0)
      in_range = false;
  CHECK(in_range, "all psi in (0, pi/2)");

  /* 5. split picker returns the standard Givens split (b_phi = b_psi + 2),
   * i.e. (6,4) for this 10-bit/tone 2x1 report — NOT the degenerate (8,2) that a
   * naive min-variance search picks because a 2-bit psi is trivially constant. */
  {
    std::vector<ReportHdr> batch(8, hdr); /* same frame repeated is enough */
    int bphi = 0, bpsi = 0;
    CHECK(pick_split(batch, bphi, bpsi), "pick_split found a split");
    CHECK(bphi + bpsi == hdr.per_sc_bits, "split sums to per_sc_bits");
    CHECK(bphi == bpsi + 2, "split obeys Givens b_phi = b_psi + 2");
    CHECK(bphi == 6 && bpsi == 4, "10-bit/tone 2x1 split is (6,4)");
  }

  /* 6. MotionMeter — identical reports => ~zero motion energy. */
  {
    MotionMeter m(52, 8, 2, 16);
    for (int i = 0; i < 8; ++i)
      m.push(hdr);
    CHECK(m.motion_energy() < 1e-9, "static channel => zero motion energy");
  }

  if (failures == 0)
    std::printf("bf_report_decode_selftest: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
