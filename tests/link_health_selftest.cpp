/* Headless guard for the link-health classifier (src/LinkHealth.h) — the
 * sensor-tuple -> verdict mapping behind the link.health event. The cases are
 * drawn from real on-air data: the saturation-knee sweep
 * (tests/saturation_knee_sweep.sh: clean EVM ~-56 at rssi 57, collapsed EVM
 * ~-27 at rssi 72) and the AWGN interference sweep (tests/j3_dig_penalty_sweep:
 * quiet FA ~1-30, interfered FA ~200-500). A misclassification fails `ctest`
 * instead of only surfacing as bad on-air advice. */
#include <cstdio>

#include "LinkHealth.h"

static int g_fail = 0;

static const char *vname(devourer::LinkVerdict v) {
  switch (v) {
  case devourer::LinkVerdict::NoSignal: return "NoSignal";
  case devourer::LinkVerdict::Saturated: return "Saturated";
  case devourer::LinkVerdict::Interference: return "Interference";
  case devourer::LinkVerdict::Weak: return "Weak";
  case devourer::LinkVerdict::Marginal: return "Marginal";
  case devourer::LinkVerdict::Healthy: return "Healthy";
  }
  return "?";
}

static void expect(const char *name, const devourer::LinkHealthInput &in,
                   devourer::LinkVerdict want) {
  const devourer::LinkHealthVerdict got = devourer::classify_link_health(in);
  if (got.verdict == want)
    return;
  ++g_fail;
  std::printf("FAIL: %s -> %s, want %s\n", name, vname(got.verdict),
              vname(want));
}

int main() {
  using devourer::LinkHealthInput;
  using V = devourer::LinkVerdict;

  /* No frames. */
  expect("no-signal", LinkHealthInput{}, V::NoSignal);

  /* Saturation knee (measured): top-of-ladder cell — strong RSSI 72, EVM
   * collapsed to -27 (raw), SNR still "fine" at 36. The signature SNR misses. */
  {
    LinkHealthInput in;
    in.frames = 3000;
    in.rssi_raw = 72;
    in.snr_raw = 36; /* 18 dB — looks healthy */
    in.evm_raw = -27;
    in.evm_valid = true;
    expect("saturated-knee-top", in, V::Saturated);
  }
  /* Saturation corroborated by IGI pinned at the floor. */
  {
    LinkHealthInput in;
    in.frames = 2000;
    in.rssi_raw = 70;
    in.snr_raw = 34;
    in.evm_raw = -31;
    in.evm_valid = true;
    in.igi_valid = true;
    in.igi = 0x1c;
    in.igi_min = 0x1c;
    in.igi_max = 0x2a;
    expect("saturated-igi-floor", in, V::Saturated);
  }

  /* Clean strong link (measured: rssi 57, evm -56, snr 36) = healthy. */
  {
    LinkHealthInput in;
    in.frames = 3300;
    in.rssi_raw = 57;
    in.snr_raw = 36;
    in.evm_raw = -56;
    in.evm_valid = true;
    expect("healthy-clean", in, V::Healthy);
  }

  /* External interference: moderate RSSI, degraded EVM, and a false-alarm
   * rate up in the hundreds (AWGN sweep). Not strong -> not saturation. */
  {
    LinkHealthInput in;
    in.frames = 800;
    in.rssi_raw = 48;
    in.snr_raw = 14; /* 7 dB */
    in.evm_raw = -20;
    in.evm_valid = true;
    in.energy_valid = true;
    in.fa_ofdm = 380;
    expect("interference-awgn", in, V::Interference);
  }

  /* Weak far link: low RSSI, low SNR, AGC wide open. */
  {
    LinkHealthInput in;
    in.frames = 200;
    in.rssi_raw = 30; /* ~ -80 dBm */
    in.snr_raw = 12;  /* 6 dB */
    in.evm_raw = -18;
    in.evm_valid = true;
    in.igi_valid = true;
    in.igi = 0x2a;
    in.igi_min = 0x1c;
    in.igi_max = 0x2a;
    expect("weak-far", in, V::Weak);
  }

  /* A degraded-but-not-strong, low-FA cell with no clear cause = marginal. */
  {
    LinkHealthInput in;
    in.frames = 1500;
    in.rssi_raw = 50;
    in.snr_raw = 22; /* 11 dB */
    in.evm_raw = -30;
    in.evm_valid = true;
    in.energy_valid = true;
    in.fa_ofdm = 40; /* quiet — not interference */
    expect("marginal", in, V::Marginal);
  }

  /* CCK-only stream (no EVM): classification falls back to SNR. Strong + poor
   * SNR still reads as saturated. */
  {
    LinkHealthInput in;
    in.frames = 1000;
    in.rssi_raw = 70;
    in.snr_raw = 10; /* 5 dB, poor */
    in.evm_valid = false;
    expect("saturated-cck-nosnr", in, V::Saturated);
  }

  if (g_fail) {
    std::printf("%d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("link-health selftest: all OK\n");
  return 0;
}
