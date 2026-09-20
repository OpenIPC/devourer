/* Headless guard for the CLM busy-window state machine (src/BusyWindow.h).
 *
 * The register access is mocked, so every path the hardware taught us is
 * reachable without a radio: the clamp, the arm/read register sequence, the
 * period-bounded ratio, and — the load-bearing half — the three ways a window
 * stops describing the caller's dwell. Each of those must yield NO READING.
 * A spoiled window that returns a number is worse than one that returns
 * nothing: on the 11AC map the corruption is a 3-4 point overcount, which no
 * consumer can distinguish from a real channel.
 */
#include "BusyWindow.h"

#include <cstdint>
#include <cstdio>
#include <map>
#include <vector>

using devourer::BusySource;
using devourer::BusySpoil;
using devourer::ChannelBusy;
using devourer::ClmWindow;
using devourer::kClmMaxPeriodTicks;
using devourer::kClmMinPeriodTicks;
using devourer::NhmRegs;
using devourer::nhm_regs_11ac;

static int g_fail = 0;

static void check(const char *what, long got, long want) {
  if (got != want) {
    std::printf("FAIL %s: got %ld want %ld\n", what, got, want);
    ++g_fail;
  }
}

namespace {

/* A baseband stand-in that applies the masked-write semantics the real
 * PHY_SetBBReg8812 has (value shifted by the mask's own bit position), so a
 * pre-shifted write shows up here as the wrong field, exactly as it would on
 * silicon. */
struct Write {
  uint16_t addr;
  uint32_t mask;
  uint32_t value; /* as passed, before the mask shift */
};

struct MockBb {
  std::map<uint16_t, uint32_t> regs;
  std::vector<Write> writes;
  /* What the "hardware" will report for the CLM result register. */
  uint16_t clm_ticks = 0;
  bool clm_ready = true;
  /* The register map under test, so the JGR3 addresses can be exercised too. */
  NhmRegs map = nhm_regs_11ac();

  uint32_t read32(uint16_t addr) {
    if (addr == map.clm)
      return (clm_ready ? (1u << 16) : 0u) | clm_ticks;
    return regs[addr];
  }
  void set_bb(uint16_t addr, uint32_t mask, uint32_t value) {
    int shift = 0;
    while (shift < 32 && !((mask >> shift) & 1u))
      shift++;
    regs[addr] = (regs[addr] & ~mask) | ((value << shift) & mask);
    writes.push_back(Write{addr, mask, value});
    /* Model the hardware the refusals depend on: triggering CLM starts a new
     * window, so the ready bit drops until it completes. A build where the
     * trigger did NOT clear ready would return the previous window's latched
     * value, which is what the NotElapsed refusal exists to prevent - the
     * on-air `stale` arm in tests/busy_window_probe.sh is what checks that
     * this model matches the silicon. */
    if (addr == map.ctrl && (mask & 0x1u) && value == 1)
      clm_ready = false;
  }
  /* Did anything pulse the NHM trigger (ctrl bit1) to 1? A rising edge there
   * re-arms the shared engine, which is exactly what a CLM-only arm must not
   * do - and a check on the FINAL register image cannot see a 0->1->0 pulse. */
  bool nhm_trigger_pulsed() const {
    for (const Write &w : writes)
      if (w.addr == map.ctrl && (w.mask & 0x2u) && (w.value & 0x1u))
        return true;
    return false;
  }
  ClmWindow::Read32 rd() {
    return [this](uint16_t a) { return read32(a); };
  }
  ClmWindow::SetBb wr() {
    return [this](uint16_t a, uint32_t m, uint32_t v) { set_bb(a, m, v); };
  }
};

} // namespace

int main() {
  const NhmRegs regs = nhm_regs_11ac();

  /* --- the clamp: a window nobody can program must not be pretended --- */
  {
    MockBb bb;
    ClmWindow w;
    const uint32_t armed = w.arm(regs, 10u * 1000u * 1000u, 0, bb.wr());
    check("clamp: armed window is the ceiling",
          armed, static_cast<long>(kClmMaxPeriodTicks) * 4);
    const uint32_t period = bb.regs[regs.period] & 0xffffu;
    check("clamp: period register holds the ceiling", period,
          kClmMaxPeriodTicks);
  }
  {
    MockBb bb;
    ClmWindow w;
    const uint32_t armed = w.arm(regs, 100, 0, bb.wr());
    check("clamp: a sub-millisecond request is floored",
          armed, static_cast<long>(kClmMinPeriodTicks) * 4);
  }
  /* Zero is a caller bug rather than "the minimum": nothing armed, no
   * registers touched, and a later read must not invent a window. */
  {
    MockBb bb;
    ClmWindow w;
    const uint32_t armed = w.arm(regs, 0, 0, bb.wr());
    check("zero window: refused", armed, 0);
    check("zero window: nothing armed", w.armed() ? 1 : 0, 0);
    check("zero window: no register writes",
          static_cast<long>(bb.writes.size()), 0);
    bb.clm_ticks = 30000;
    const ChannelBusy b = w.read(regs, 0, bb.rd());
    check("zero window: no reading", b.valid, 0);
  }

  /* --- the arm sequence: CLM only, NHM's half of the dword untouched --- */
  {
    MockBb bb;
    /* Pre-load NHM's half of the period dword; arming CLM must not move it.
     * Start from the state a previous read_nhm leaves behind — both triggers
     * high — so "left alone" is tested against a realistic prior state and
     * not against a conveniently zeroed register. */
    bb.regs[regs.period] = 0x01f40000u;
    bb.regs[regs.ctrl] = 0x3u;
    ClmWindow w;
    w.arm(regs, 240000, 0, bb.wr());
    check("arm: no NHM trigger edge", bb.nhm_trigger_pulsed() ? 1 : 0, 0);
    check("arm: NHM period half preserved",
          (bb.regs[regs.period] >> 16) & 0xffffu, 0x01f4);
    check("arm: CLM period half programmed",
          bb.regs[regs.period] & 0xffffu, 60000);
    check("arm: ccx_en set", (bb.regs[regs.ctrl] >> 8) & 1u, 1);
    check("arm: CLM trigger left high", bb.regs[regs.ctrl] & 1u, 1);
    /* It was 1 before the arm and the arm must not have driven it either way:
     * the final image still shows the NHM engine as the previous caller left
     * it. Paired with the pulse check above, which a final-state check alone
     * cannot make. */
    check("arm: NHM trigger left as found", (bb.regs[regs.ctrl] >> 1) & 1u, 1);
  }

  /* --- a clean window reports the ratio against its own period --- */
  {
    MockBb bb;
    ClmWindow w;
    /* 240 ms is the ceiling (kClmMaxPeriodTicks), so this is also the
     * longest window a caller can actually get. */
    w.arm(regs, 240000, 0, bb.wr());       /* 60000 ticks */
    bb.clm_ticks = 36000;                  /* 60% */
    bb.clm_ready = true;                   /* the window completed */
    const ChannelBusy b = w.read(regs, 0, bb.rd());
    check("clean: valid", b.valid, 1);
    check("clean: source is CLM", static_cast<long>(b.source),
          static_cast<long>(BusySource::Clm));
    check("clean: busy pct", b.busy_pct, 60);
    check("clean: window_us is the armed window", b.window_us, 240000);
    check("clean: no own TX", b.own_tx_in_window, 0);
    check("clean: spoil clear", static_cast<long>(w.last_spoil()),
          static_cast<long>(BusySpoil::None));
  }

  /* A result at or above the period is 100%, never an overflow artefact.
   * Overshoots the period far enough to KILL the mutant: rounding means a
   * small excess still lands on 100 either way, so the value has to exceed
   * period by more than half a percent of it. Without the clamp 61000/60000
   * yields 102, which a uint8_t percentage reports as an out-of-range
   * channel occupancy. */
  {
    MockBb bb;
    ClmWindow w;
    w.arm(regs, 240000, 0, bb.wr());
    bb.clm_ticks = 61000;
    bb.clm_ready = true;
    const ChannelBusy b = w.read(regs, 0, bb.rd());
    check("saturated: 100%", b.busy_pct, 100);
    check("saturated: window still its own", b.window_us, 240000);
  }

  /* --- an NHM read inside the window: JGR3 truncates it, 11AC inflates it,
   * and neither is a reading about this window --- */
  {
    MockBb bb;
    ClmWindow w;
    w.arm(regs, 240000, 0, bb.wr());
    w.note_nhm_read();
    bb.clm_ticks = 37500;
    bb.clm_ready = true;
    const ChannelBusy b = w.read(regs, 0, bb.rd());
    check("nhm-spoiled: no reading", b.valid, 0);
    check("nhm-spoiled: busy invalid", b.valid_busy, 0);
    check("nhm-spoiled: reason reported", static_cast<long>(w.last_spoil()),
          static_cast<long>(BusySpoil::Interrupted));
  }

  /* --- a retune inside the window blends two channels --- */
  {
    MockBb bb;
    ClmWindow w;
    w.arm(regs, 240000, 0, bb.wr());
    w.note_retune();
    bb.clm_ticks = 37500;
    bb.clm_ready = true;
    const ChannelBusy b = w.read(regs, 0, bb.rd());
    check("retune-spoiled: no reading", b.valid, 0);
    check("retune-spoiled: reason reported", static_cast<long>(w.last_spoil()),
          static_cast<long>(BusySpoil::Retuned));
  }

  /* --- an early read returns the PREVIOUS window on hardware, so the ready
   * bit being clear must be a refusal, not a zero --- */
  {
    MockBb bb;
    ClmWindow w;
    w.arm(regs, 240000, 0, bb.wr());
    bb.clm_ready = false;
    bb.clm_ticks = 37500;
    const ChannelBusy b = w.read(regs, 0, bb.rd());
    check("early: no reading", b.valid, 0);
    check("early: reason reported", static_cast<long>(w.last_spoil()),
          static_cast<long>(BusySpoil::NotElapsed));
    /* The window is still counting, so it is still armed: the read at the
     * end of the dwell is the measurement. A build that consumed it here
     * would send that read down the sampled path instead. */
    check("early: window still armed", w.armed() ? 1 : 0, 1);
    bb.clm_ready = true;
    const ChannelBusy later = w.read(regs, 0, bb.rd());
    check("early: later read is the window", later.valid, 1);
    check("early: later value", later.busy_pct, 63);
    check("early: consumed by the completed read", w.armed() ? 1 : 0, 0);
  }

  /* --- the other half of that rule: a SPOILED window IS consumed.
   *
   * "Not elapsed" keeps the window because nothing disturbed it — the caller
   * was merely early. Interrupted and Retuned are the opposite: the hardware
   * window was re-armed underneath it, or it spans two channels, so there is
   * nothing left to wait for.
   *
   * Consuming it is what hands the caller BACK to the sampled path.
   * IRtlRadio::GetChannelBusy takes the armed branch on armed(), so a spoiled
   * window left armed strands every later call there: `spoil_` is sticky
   * (only arm() clears it), so each one re-enters the spoil branch and
   * returns invalid-Retuned, and the sampled fallback is unreachable for
   * every unrelated consumer until something re-arms or resets the window.
   * Not a wrong number — a sensor that stops answering.
   *
   * Without this block the asymmetry is invisible to the suite: a build that
   * keeps the window armed on EVERY verdict passes every other test in this
   * file. It is exactly the difference a later editor removes while tidying,
   * so it is asserted rather than only described in the header. --- */
  {
    MockBb bb;
    ClmWindow w;
    w.arm(regs, 240000, 0, bb.wr());
    w.note_retune();
    const ChannelBusy spoiled = w.read(regs, 0, bb.rd());
    check("spoiled: refused", spoiled.valid, 0);
    /* On the READING, not just in last_spoil(): the block above checks the
     * accessor, so a branch that reported every spoil as Interrupted would
     * pass it. What a consumer acts on is the field. */
    check("spoiled: with its reason", static_cast<long>(spoiled.spoil),
          static_cast<long>(BusySpoil::Retuned));
    check("spoiled: window consumed", w.armed() ? 1 : 0, 0);
  }

  /* --- reading without arming is not a quiet channel --- */
  {
    MockBb bb;
    ClmWindow w;
    bb.clm_ticks = 37500;
    const ChannelBusy b = w.read(regs, 0, bb.rd());
    check("unarmed: no reading", b.valid, 0);
  }

  /* --- one arm, one reading: a second read must not re-report a window that
   * has already been consumed, because the result register still holds it --- */
  {
    MockBb bb;
    ClmWindow w;
    w.arm(regs, 240000, 0, bb.wr());
    bb.clm_ticks = 37500;
    bb.clm_ready = true;
    const ChannelBusy first = w.read(regs, 0, bb.rd());
    const ChannelBusy second = w.read(regs, 0, bb.rd());
    check("consumed: first valid", first.valid, 1);
    check("consumed: second refuses", second.valid, 0);
  }

  /* --- own transmission is carried, not corrected: the reading stays valid
   * and says it was taken hot --- */
  {
    MockBb bb;
    ClmWindow w;
    w.arm(regs, 240000, /*tx_submitted=*/1000, bb.wr());
    bb.clm_ticks = 12000;
    bb.clm_ready = true; /* 20% — the depressed reading a hot sensor gives */
    const ChannelBusy b = w.read(regs, /*tx_submitted=*/1900, bb.rd());
    check("own-tx: still a reading", b.valid, 1);
    check("own-tx: flagged", b.own_tx_in_window, 1);
    check("own-tx: frame count", b.own_tx_frames, 900);
    check("own-tx: value not silently corrected", b.busy_pct, 20);
  }

  /* A TX counter that jumped forward by more than the 32-bit field can hold
   * must SATURATE, not truncate: a truncated delta can land on a small number
   * that reads as "barely transmitting". */
  {
    MockBb bb;
    ClmWindow w;
    w.arm(regs, 240000, /*tx_submitted=*/0, bb.wr());
    bb.clm_ticks = 0;
    bb.clm_ready = true;
    const ChannelBusy b =
        w.read(regs, /*tx_submitted=*/0x1'0000'0003ull, bb.rd());
    check("own-tx: huge delta saturates", b.own_tx_frames == 0xffffffffu, 1);
    check("own-tx: huge delta still flagged", b.own_tx_in_window, 1);
  }

  /* A TX counter that went backwards (a reset between arm and read) must not
   * underflow into a huge frame count. */
  {
    MockBb bb;
    ClmWindow w;
    w.arm(regs, 240000, /*tx_submitted=*/5000, bb.wr());
    bb.clm_ticks = 0;
    bb.clm_ready = true;
    const ChannelBusy b = w.read(regs, /*tx_submitted=*/10, bb.rd());
    check("own-tx: counter reset does not underflow", b.own_tx_frames, 0);
    check("own-tx: counter reset leaves it unflagged", b.own_tx_in_window, 0);
  }

  /* --- the stale latch: arm, complete, read, arm again, read early. The
   * result register still holds the FIRST window's value, so a build that
   * trusted it would report a completed measurement for a window that has
   * barely started. This is the case the on-air `stale` arm re-checks against
   * real silicon, because it depends on the trigger clearing ready. --- */
  {
    MockBb bb;
    ClmWindow w;
    w.arm(regs, 240000, 0, bb.wr());
    bb.clm_ticks = 48000; /* 80% */
    bb.clm_ready = true;
    const ChannelBusy first = w.read(regs, 0, bb.rd());
    check("stale: first window valid", first.valid, 1);
    check("stale: first window value", first.busy_pct, 80);
    w.arm(regs, 240000, 0, bb.wr()); /* the mock drops ready, as the chip does */
    const ChannelBusy second = w.read(regs, 0, bb.rd());
    check("stale: second read refuses", second.valid, 0);
    check("stale: refusal is not-elapsed", static_cast<long>(second.spoil),
          static_cast<long>(BusySpoil::NotElapsed));
  }

  /* --- arming twice without reading: the second arm owns the window, and the
   * first one's spoilage must not leak into it --- */
  {
    MockBb bb;
    ClmWindow w;
    w.arm(regs, 240000, 0, bb.wr());
    w.note_retune();               /* spoils the FIRST window */
    w.arm(regs, 240000, 0, bb.wr()); /* a new window starts clean */
    bb.clm_ticks = 30000;
    bb.clm_ready = true;
    const ChannelBusy b = w.read(regs, 0, bb.rd());
    check("re-arm: clears the previous spoil", b.valid, 1);
    check("re-arm: reports the new window", b.busy_pct, 50);
  }

  /* --- a spoil arriving after a clean read must not taint the next window,
   * and a note with nothing armed is a no-op --- */
  {
    MockBb bb;
    ClmWindow w;
    w.arm(regs, 240000, 0, bb.wr());
    bb.clm_ticks = 30000;
    bb.clm_ready = true;
    (void)w.read(regs, 0, bb.rd());
    /* A note with nothing armed must not reach the NEXT window. (The guard
     * inside note_*() is belt-and-braces: arm() also clears the reason, so
     * this asserts the observable behaviour rather than the mechanism.) */
    w.note_nhm_read();
    check("unarmed note: nothing armed", w.armed() ? 1 : 0, 0);
    w.arm(regs, 240000, 0, bb.wr());
    bb.clm_ticks = 30000;
    bb.clm_ready = true;
    const ChannelBusy b = w.read(regs, 0, bb.rd());
    check("unarmed note: next window unaffected", b.valid, 1);
  }

  /* --- spoil precedence: the FIRST reason is kept, because it is the one
   * that describes what happened to the measurement --- */
  {
    MockBb bb;
    ClmWindow w;
    w.arm(regs, 240000, 0, bb.wr());
    w.note_nhm_read();
    w.note_retune();
    bb.clm_ticks = 30000;
    bb.clm_ready = true;
    const ChannelBusy b = w.read(regs, 0, bb.rd());
    check("precedence: invalid", b.valid, 0);
    check("precedence: first reason kept", static_cast<long>(b.spoil),
          static_cast<long>(BusySpoil::Interrupted));
  }

  /* An interrupted window that has ALSO not elapsed reports the interruption:
   * the not-elapsed part is a symptom of the re-arm, and the caller's fix is
   * the sequencing, not a longer wait. */
  {
    MockBb bb;
    ClmWindow w;
    w.arm(regs, 240000, 0, bb.wr());
    w.note_nhm_read();
    bb.clm_ready = false; /* the re-arm restarted it, so it is not done */
    const ChannelBusy b = w.read(regs, 0, bb.rd());
    check("precedence: not-elapsed does not mask the interruption",
          static_cast<long>(b.spoil),
          static_cast<long>(BusySpoil::Interrupted));
  }

  /* --- the JGR3 map: different addresses, same state machine. The map is
   * data, so a mix-up (reading CLM from the 11AC address on a Jaguar3) is a
   * silent wrong-register read, not a compile error. --- */
  {
    const NhmRegs jgr3 = devourer::nhm_regs_jgr3();
    MockBb bb;
    bb.map = jgr3;
    ClmWindow w;
    w.arm(jgr3, 240000, 0, bb.wr());
    check("jgr3: period programmed at its own address",
          bb.regs[jgr3.period] & 0xffffu, 60000);
    check("jgr3: 11AC period untouched", bb.regs[nhm_regs_11ac().period], 0);
    bb.clm_ticks = 15000;
    bb.clm_ready = true;
    const ChannelBusy b = w.read(jgr3, 0, bb.rd());
    check("jgr3: reading", b.busy_pct, 25);
  }

  if (g_fail == 0)
    std::printf("busy_window_selftest: all checks passed\n");
  return g_fail ? 1 : 0;
}
