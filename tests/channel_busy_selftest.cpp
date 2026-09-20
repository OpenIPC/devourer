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
    /* No CLM in the read: a with_nhm=false read, a generation with no CCX
     * engine (Kestrel), or one that implements no GetRxEnergy at all (the
     * RTL8733B, whose CLM answers only through an armed window). Not a quiet
     * channel. */
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

  /* --- the armed MediaTek window (busy_from_ch_time_window) ---
   *
   * These rules are the MediaTek half of the ArmChannelBusy contract, and the
   * backend that owns the state needs a radio, so they are tested here where
   * they are pure. Each refusal below is a reading that would otherwise be
   * valid and wrong. */
  {
    using devourer::busy_from_ch_time_window;
    using devourer::BusySpoil;
    using devourer::ChTimeWindow;

    { /* Unarmed: the sampled path, untouched by any of this. tx_now is
       * deliberately LARGE — an unarmed reading has no baseline, so a build
       * that dropped the armed check would report the lifetime TX counter as
       * frames sent inside a window that was never armed. */
      ChTimeWindow w;
      const ChannelBusy b =
          busy_from_ch_time_window(w, 300, 700, 1000, false, 9999);
      check("mt unarmed: valid", b.valid, 1);
      check("mt unarmed: pct", b.busy_pct, 30);
      check("mt unarmed: no own-tx claim", b.own_tx_in_window, 0);
      check("mt unarmed: no own-tx count", b.own_tx_frames, 0);
    }
    { /* The boundary: elapsed EXACTLY the requested window is complete, not
       * premature. `<` vs `<=` is a one-character mutation otherwise. */
      ChTimeWindow w; w.armed = true; w.window_us = 240000;
      const ChannelBusy b = busy_from_ch_time_window(w, 600, 400, 240000, false, 0);
      check("mt boundary: exactly the window is valid", b.valid, 1);
      check("mt boundary: pct", b.busy_pct, 60);
    }
    { /* One microsecond short is premature. */
      ChTimeWindow w; w.armed = true; w.window_us = 240000;
      const ChannelBusy b = busy_from_ch_time_window(w, 600, 400, 239999, false, 0);
      check("mt boundary: one us short is refused", b.valid, 0);
      check("mt boundary: reason", static_cast<long>(b.spoil),
            static_cast<long>(BusySpoil::NotElapsed));
    }
    { /* Armed and complete. */
      ChTimeWindow w; w.armed = true; w.window_us = 240000;
      const ChannelBusy b = busy_from_ch_time_window(w, 600, 400, 250000, false, 0);
      check("mt armed: valid", b.valid, 1);
      check("mt armed: pct", b.busy_pct, 60);
    }
    { /* Read before the requested window elapsed. These timers have no ready
       * bit, so without this a glance reads as a finished measurement. */
      ChTimeWindow w; w.armed = true; w.window_us = 240000;
      const ChannelBusy b = busy_from_ch_time_window(w, 600, 400, 5000, false, 0);
      check("mt early: refused", b.valid, 0);
      check("mt early: reason", static_cast<long>(b.spoil),
            static_cast<long>(BusySpoil::NotElapsed));
    }
    { /* A retune ran through it. */
      ChTimeWindow w; w.armed = true; w.spoiled = true; w.window_us = 240000;
      const ChannelBusy b = busy_from_ch_time_window(w, 600, 400, 250000, false, 0);
      check("mt retuned: refused", b.valid, 0);
      check("mt retuned: reason", static_cast<long>(b.spoil),
            static_cast<long>(BusySpoil::Retuned));
    }
    { /* The 1 Hz telemetry poll read-and-cleared the same registers, so these
       * counts are the remainder of the window, not the window. */
      ChTimeWindow w; w.armed = true; w.window_us = 240000;
      const ChannelBusy b = busy_from_ch_time_window(w, 600, 400, 250000, true, 0);
      check("mt disturbed: refused", b.valid, 0);
      check("mt disturbed: reason", static_cast<long>(b.spoil),
            static_cast<long>(BusySpoil::Interrupted));
    }
    { /* Interruption outranks "not elapsed", as on Realtek: the short
       * interval is a symptom of the theft. */
      ChTimeWindow w; w.armed = true; w.window_us = 240000;
      const ChannelBusy b = busy_from_ch_time_window(w, 600, 400, 5000, true, 0);
      check("mt precedence: interrupted beats not-elapsed",
            static_cast<long>(b.spoil),
            static_cast<long>(BusySpoil::Interrupted));
    }
    { /* A retune outranks a disturbed read: the earlier and larger fact. */
      ChTimeWindow w; w.armed = true; w.spoiled = true; w.window_us = 240000;
      const ChannelBusy b = busy_from_ch_time_window(w, 600, 400, 250000, true, 0);
      check("mt precedence: retune wins", static_cast<long>(b.spoil),
            static_cast<long>(BusySpoil::Retuned));
    }
    { /* Own transmission: these timers count it as busy, so the reading says
       * so rather than being silently corrected. */
      ChTimeWindow w; w.armed = true; w.window_us = 240000; w.tx_at_arm = 1000;
      const ChannelBusy b = busy_from_ch_time_window(w, 900, 100, 250000, false, 1450);
      check("mt own-tx: still valid", b.valid, 1);
      check("mt own-tx: flagged", b.own_tx_in_window, 1);
      check("mt own-tx: count", b.own_tx_frames, 450);
    }
    { /* A TX counter that went backwards must not underflow. */
      ChTimeWindow w; w.armed = true; w.window_us = 240000; w.tx_at_arm = 5000;
      const ChannelBusy b = busy_from_ch_time_window(w, 900, 100, 250000, false, 7);
      check("mt own-tx: reset does not underflow", b.own_tx_frames, 0);
      check("mt own-tx: reset leaves it unflagged", b.own_tx_in_window, 0);
    }
    { /* An armed window over dead counters is "no reading", never 0%. */
      ChTimeWindow w; w.armed = true; w.window_us = 240000;
      const ChannelBusy b = busy_from_ch_time_window(w, 0, 0, 250000, false, 0);
      check("mt armed over dead counters: no reading", b.valid, 0);
      check("mt armed over dead counters: reported as a lost window",
            static_cast<long>(b.spoil),
            static_cast<long>(BusySpoil::Interrupted));
    }
    { /* The negative control for the block above. The SAME dead counters
       * with no window armed are the sampled path finding a backend that is
       * not counting — "no sensor here", which is what a bare invalid
       * reading means, and it must stay bare. A build that reported a spoil
       * reason here would tell every sampled caller that its window was
       * lost, when it never armed one.
       *
       * This is what pins the `w.armed &&` half of the guard: without it the
       * reason leaks onto the sampled path and no other test notices. */
      ChTimeWindow w; /* unarmed */
      const ChannelBusy b = busy_from_ch_time_window(w, 0, 0, 250000, false, 0);
      check("mt unarmed over dead counters: no reading", b.valid, 0);
      check("mt unarmed over dead counters: and no window to have lost",
            static_cast<long>(b.spoil), static_cast<long>(BusySpoil::None));
    }
  }

  if (g_fail) {
    std::printf("channel_busy: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("channel_busy: all checks passed\n");
  return 0;
}
