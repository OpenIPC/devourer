/* Headless guard for the vendor-neutral ChannelBusy conversions
 * (src/RxSense.h): RxEnergy -> ChannelBusy on the Realtek side and the
 * MediaTek MAC channel timers -> ChannelBusy on the other. Pure arithmetic;
 * no hardware.
 *
 * The case that matters most is the refusal: a counter that was never armed,
 * or a read that did not happen, must report NO READING — never 0% busy. A
 * consumer cannot tell a fabricated quiet channel from a real one, and the
 * scoring layers act on the difference. */
#include "RxSense.h"

#include <cstdint>
#include <cstdio>

using devourer::busy_from_ch_time;
using devourer::busy_from_rx_energy;
using devourer::BusySource;
using devourer::ChannelBusy;

static int g_fail = 0;

static void check(const char *what, long got, long want) {
  if (got != want) {
    std::printf("FAIL %s: got %ld want %ld\n", what, got, want);
    ++g_fail;
  }
}

int main() {
  /* --- RxEnergy -> ChannelBusy --- */
  {
    /* No CLM at all: a with_nhm=false read, or a generation with no CCX
     * engine (Kestrel, RTL8733B). Not a quiet channel. */
    RxEnergy e;
    const ChannelBusy b = busy_from_rx_energy(e);
    check("no-clm invalid", b.valid, 0);
    check("no-clm source none", static_cast<long>(b.source),
          static_cast<long>(BusySource::None));
    check("no-clm busy invalid", b.valid_busy, 0);
  }
  {
    /* CLM without NHM: airtime is real, the energy share is absent. */
    RxEnergy e;
    e.valid_clm = true;
    e.clm_ratio_pct = 34;
    e.clm_period = 500;
    const ChannelBusy b = busy_from_rx_energy(e);
    check("clm valid", b.valid, 1);
    check("clm source", static_cast<long>(b.source),
          static_cast<long>(BusySource::Clm));
    check("clm busy_pct", b.busy_pct, 34);
    check("clm window_us", b.window_us, 2000); /* 500 ticks * 4 us */
    check("clm energy invalid", b.valid_energy, 0);
    check("clm energy zeroed", b.energy_pct, 0);
  }
  {
    /* CLM + NHM: both halves present. */
    RxEnergy e;
    e.valid_clm = true;
    e.clm_ratio_pct = 6;
    e.clm_period = 12500; /* 50 ms */
    e.valid_nhm = true;
    e.nhm_env_ratio_pct = 56;
    const ChannelBusy b = busy_from_rx_energy(e);
    check("clm+nhm busy", b.busy_pct, 6);
    check("clm+nhm energy", b.energy_pct, 56);
    check("clm+nhm window_us", b.window_us, 50000);
  }
  {
    /* A ratio helper that overshot must clamp, not wrap the uint8. */
    RxEnergy e;
    e.valid_clm = true;
    e.clm_ratio_pct = 255;
    e.valid_nhm = true;
    e.nhm_env_ratio_pct = 200;
    const ChannelBusy b = busy_from_rx_energy(e);
    check("clm clamps", b.busy_pct, 100);
    check("nhm-env clamps", b.energy_pct, 100);
  }

  /* --- MediaTek channel timers -> ChannelBusy --- */
  {
    /* THE refusal: never armed, or the read failed. Not an idle channel. */
    const ChannelBusy b = busy_from_ch_time(0, 0, 1000);
    check("never-armed invalid", b.valid, 0);
    check("never-armed source none", static_cast<long>(b.source),
          static_cast<long>(BusySource::None));
    check("never-armed busy invalid", b.valid_busy, 0);
  }
  check("all-idle", busy_from_ch_time(0, 1000, 0).busy_pct, 0);
  check("all-idle valid", busy_from_ch_time(0, 1000, 0).valid, 1);
  check("all-busy", busy_from_ch_time(1000, 0, 0).busy_pct, 100);
  check("quarter busy", busy_from_ch_time(250, 750, 0).busy_pct, 25);
  check("half busy", busy_from_ch_time(500, 500, 0).busy_pct, 50);
  check("rounds to nearest", busy_from_ch_time(1, 2, 0).busy_pct, 33);
  {
    const ChannelBusy b = busy_from_ch_time(7, 3, 12345);
    check("ch_time source", static_cast<long>(b.source),
          static_cast<long>(BusySource::ChTime));
    check("ch_time window passthrough", b.window_us, 12345);
    check("ch_time energy invalid", b.valid_energy, 0);
  }
  /* window_us 0 means UNKNOWN (the first call has no previous mark), not a
   * zero-length window — a consumer filtering on a minimum must not read it
   * as "too short". Still a valid reading. */
  check("first-call window unknown", busy_from_ch_time(1, 1, 0).window_us, 0);
  check("first-call still valid", busy_from_ch_time(1, 1, 0).valid, 1);
  {
    /* Both counters near full scale: the ratio must be computed in 64-bit.
     * In 32 bits busy*100 wraps and the answer is nonsense. */
    const uint32_t big = 0xFFFFFFFFu;
    check("no 32-bit overflow (all busy)",
          busy_from_ch_time(big, 0, 0).busy_pct, 100);
    check("no 32-bit overflow (half)",
          busy_from_ch_time(big / 2, big / 2, 0).busy_pct, 50);
    check("no 32-bit overflow (quarter)",
          busy_from_ch_time(big / 4, (big / 4) * 3, 0).busy_pct, 25);
  }

  if (g_fail) {
    std::printf("channel_busy: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("channel_busy: all checks passed\n");
  return 0;
}
