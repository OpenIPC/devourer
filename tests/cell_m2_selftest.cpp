/* Headless guard for the M2 cell core: src/cell/Harq.h (receiver dedup and
 * window, sender ACK / NACK / timeout / attempt limit / span) and
 * src/cell/LinkAdapt.h (loss and SNR rate rules). Includes a lossy-channel
 * simulation: HARQ must deliver every frame exactly once where the no-HARQ
 * baseline loses them. No hardware. */
#include <cstdint>
#include <cstdio>
#include <set>
#include <vector>

#include "cell/Harq.h"
#include "cell/LinkAdapt.h"

using namespace devourer::cell;

static int failures = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

int main() {
  /* --- Receiver: in order, out of order, duplicates, abandoned gaps. --- */
  {
    HarqReceiver r;
    CHECK(r.on_frame(0, 0) && r.feedback(0) == (HarqFeedback{1, 0}));
    CHECK(!r.on_frame(0, 0)); /* duplicate */
    CHECK(r.on_frame(2, 1));  /* 1 missing */
    CHECK(r.feedback(0) == (HarqFeedback{1, 0b1}));
    CHECK(r.on_frame(3, 1) && r.feedback(0) == (HarqFeedback{1, 0b11}));
    CHECK(!r.on_frame(3, 1));
    CHECK(r.on_frame(1, 1)); /* fills the gap: base jumps over 2, 3 */
    CHECK(r.feedback(0) == (HarqFeedback{4, 0}));
    CHECK(!r.on_frame(2, 1)); /* below base now */
    /* The sender gives up on 4 and 5: `low` = 6 moves base past them. */
    CHECK(r.on_frame(7, 6) && r.feedback(0) == (HarqFeedback{6, 0b1}));
    CHECK(!r.on_frame(5, 6)); /* abandoned: not delivered late */
    /* Far ahead (a lost frame carried a moved `low`): catch up, keep window. */
    CHECK(r.on_frame(60, 6));
    CHECK(r.feedback(0).base == 60 - kHarqWindow && (r.feedback(0).bitmap >> 31) == 1u);
    /* Wrap. */
    HarqReceiver w;
    CHECK(w.on_frame(0xFFFFFFFEu, 0xFFFFFFFEu) && w.on_frame(0xFFFFFFFFu, 0xFFFFFFFEu));
    CHECK(w.on_frame(0, 0xFFFFFFFEu) && w.feedback(0) == (HarqFeedback{1, 0}));
    CHECK(seq_lt(0xFFFFFFFFu, 0) && !seq_lt(0, 0xFFFFFFFFu));
  }

  /* --- Sender: ACK, NACK (with the report-age rule), timeout, drop. --- */
  {
    HarqConfig c;
    c.max_attempts = 3;
    c.timeout_sf = 3;
    c.nack_min_age_sf = 2;
    HarqSender s(c);
    auto t0 = s.next(10, true);
    auto t1 = s.next(11, true);
    auto t2 = s.next(12, true);
    CHECK(t0 && t0->seq == 0 && !t0->retx && t0->low == 0 && t0->pid == 0);
    CHECK(t1 && t1->seq == 1 && t2 && t2->seq == 2 && t2->low == 0);
    /* Report generated at sf 13: 0 and 2 received, 1 missing. 1 was sent at
     * sf 11, 2 superframes before the report: NACK. */
    s.on_feedback(HarqFeedback{1, 0b1, 13});
    CHECK(s.inflight() == 1 && s.stats().acked == 2 && s.stats().nacks == 1);
    auto r1 = s.next(13, true); /* the retransmission goes first */
    CHECK(r1 && r1->retx && r1->seq == 1 && r1->attempt == 2 && r1->low == 1);
    /* A report generated at sf 14 still missing 1: its attempt (sf 13) is too
     * recent for a NACK. */
    s.on_feedback(HarqFeedback{1, 0b1, 14});
    CHECK(s.stats().nacks == 1);
    /* An OLD report (generated at sf 12, before the retransmission) that
     * arrives late, whenever, never NACKs the retransmission. */
    s.on_feedback(HarqFeedback{1, 0b1, 12});
    CHECK(s.stats().nacks == 1);
    /* No ACK: the timeout (sf 13 + 3) makes it due; the third attempt is the last. */
    auto n = s.next(14, false);
    CHECK(!n); /* not due yet, no new data */
    auto r2 = s.next(16, false);
    CHECK(r2 && r2->seq == 1 && r2->attempt == 3 && s.stats().timeouts == 1);
    auto r3 = s.next(19, false); /* times out again with attempts exhausted: dropped */
    CHECK(!r3 && s.stats().dropped == 1 && s.inflight() == 0 && s.low() == 3);
  }

  /* --- Sender: the seq span is bounded by the feedback window. --- */
  {
    HarqConfig c;
    c.max_attempts = 100;
    c.timeout_sf = 1000;
    HarqSender s(c);
    int sent = 0;
    while (s.next(0, true))
      ++sent;
    /* max_span 31 is a difference: seqs 0..31 go out, 32 does not. */
    CHECK(sent == static_cast<int>(kHarqWindow));
    /* ACK everything but seq 0 (bits 0..30 = seqs 1..31): the span is still
     * full (low stays 0). */
    s.on_feedback(HarqFeedback{0, 0x7FFFFFFFu, 5});
    CHECK(s.inflight() == 1 && s.low() == 0);
    auto t = s.next(5, true);
    CHECK(t && t->retx && t->seq == 0); /* NACKed by the later ACKs: retransmitted */
    CHECK(!s.next(5, true));            /* span full: no new data */
    s.on_feedback(HarqFeedback{kHarqWindow, 0, 7});
    CHECK(s.inflight() == 0 && s.next(7, true).has_value());
  }

  /* --- unsend: a refused submission spends no attempt. --- */
  {
    HarqConfig c;
    c.max_attempts = 2;
    c.timeout_sf = 1;
    HarqSender s(c);
    auto t = s.next(0, true);
    s.unsend(*t); /* the first transmission never left: its seq is reused */
    CHECK(s.inflight() == 0 && s.stats().new_tx == 0);
    auto t2 = s.next(1, true);
    CHECK(t2 && t2->seq == 0 && t2->attempt == 1);
    auto r = s.next(2, false); /* timed out: retransmission (attempt 2 of 2) */
    CHECK(r && r->retx && r->attempt == 2);
    s.unsend(*r); /* refused: still owed, attempt not spent */
    CHECK(s.stats().retx == 0 && s.inflight() == 1);
    auto r2 = s.next(3, false);
    CHECK(r2 && r2->retx && r2->attempt == 2 && s.stats().dropped == 0);
  }

  /* --- Implausible feedback (claims seqs never sent) is ignored whole. --- */
  {
    HarqSender s;
    for (int i = 0; i < 3; ++i)
      s.next(static_cast<uint64_t>(i), true); /* seqs 0..2 in flight */
    s.on_feedback(HarqFeedback{100, 0, 10}); /* base far past anything sent */
    CHECK(s.inflight() == 3 && s.stats().implausible == 1);
    s.on_feedback(HarqFeedback{0, 1u << 5, 10}); /* acks seq 6: never sent */
    CHECK(s.inflight() == 3 && s.stats().implausible == 2);
    s.on_feedback(HarqFeedback{3, 0, 10}); /* all three, exactly: plausible */
    CHECK(s.inflight() == 0 && s.stats().acked == 3);
  }

  /* --- Lossy channel: HARQ vs no-HARQ baseline. ---
   * One DL frame per superframe, deterministic 30% loss in each direction's
   * data path (feedback assumed delivered, one superframe late). */
  {
    auto lost = [](uint64_t k) { return (k * 2654435761u >> 7) % 10 < 3; };
    HarqConfig c;
    c.max_attempts = 6;
    c.timeout_sf = 2;
    c.nack_min_age_sf = 1;
    HarqSender s(c);
    HarqReceiver r;
    std::set<uint32_t> delivered;
    int dups = 0;
    uint64_t k = 0;
    HarqFeedback fb{};
    bool have_fb = false;
    const uint32_t N = 400;
    for (uint64_t sf = 0; sf < 2000 && (s.stats().new_tx < N || s.inflight() > 0); ++sf) {
      if (have_fb)
        s.on_feedback(fb); /* generated last superframe, applied now */
      auto tx = s.next(sf, s.stats().new_tx < N);
      if (tx && !lost(k++)) {
        if (r.on_frame(tx->seq, tx->low)) {
          if (!delivered.insert(tx->seq).second)
            ++dups;
        }
      }
      if (r.started()) {
        fb = r.feedback(sf);
        have_fb = true;
      }
    }
    CHECK(s.stats().new_tx == N);
    CHECK(dups == 0);
    /* This deterministic channel never loses one frame 6 times: every frame
     * must be delivered, exactly once, with nothing dropped. */
    CHECK(delivered.size() == N);
    CHECK(s.stats().dropped == 0);
    CHECK(s.stats().retx > 0);
    /* Baseline: one attempt each. */
    uint64_t k2 = 0, base_ok = 0;
    for (uint32_t i = 0; i < N; ++i)
      if (!lost(k2++))
        ++base_ok;
    CHECK(base_ok < N * 8 / 10);           /* the baseline really loses ~30% */
    CHECK(delivered.size() > base_ok + N / 5);
    std::printf("cell_m2: lossy channel 30%%: HARQ delivered %zu/%u (retx %llu, dropped %llu), "
                "baseline %llu/%u\n",
                delivered.size(), N, (unsigned long long)s.stats().retx,
                (unsigned long long)s.stats().dropped, (unsigned long long)base_ok, N);
  }

  /* --- Link adaptation. --- */
  {
    LinkAdapt la({}, 24);
    CHECK(la.rate_mbps() == 24);
    /* Clean, SNR supports more: up one rate after 3 clean windows. */
    CHECK(!la.window(30, true, 10, 0) && !la.window(30, true, 10, 0));
    CHECK(la.window(30, true, 10, 0) && la.rate_mbps() == 36);
    /* Heavy loss: down one, then held. */
    CHECK(la.window(30, true, 10, 5) && la.rate_mbps() == 24);
    for (int i = 0; i < 3; ++i)
      la.window(30, true, 10, 0);
    CHECK(la.rate_mbps() == 24); /* hold_windows (5) not over yet */
    for (int i = 0; i < 4; ++i)
      la.window(30, true, 10, 0);
    CHECK(la.rate_mbps() == 36); /* hold over + clean run: back up */
    /* SNR collapse: straight to the best supported rate. */
    CHECK(la.window(9, true, 10, 0) && la.rate_mbps() == 12);
    CHECK(la.window(1, true, 10, 0) && la.rate_mbps() == 6);
    CHECK(!la.window(1, true, 10, 10) && la.rate_mbps() == 6); /* floor */
    /* Up needs the next rate's threshold + hysteresis: 9M needs 6 + 2 dB. */
    LinkAdapt lb({}, 6);
    for (int i = 0; i < 10; ++i)
      lb.window(7.5, true, 10, 0);
    CHECK(lb.rate_mbps() == 6);
    for (int i = 0; i < 3; ++i)
      lb.window(8.0, true, 10, 0);
    CHECK(lb.rate_mbps() == 9);
    /* No SNR report: only the loss rule. */
    LinkAdapt lc({}, 24);
    for (int i = 0; i < 10; ++i)
      lc.window(0, false, 10, 0);
    CHECK(lc.rate_mbps() == 24);
    CHECK(lc.window(0, false, 10, 3) && lc.rate_mbps() == 18);
    /* clean, clean, idle, clean is not three consecutive clean windows. */
    LinkAdapt le({}, 24);
    le.window(30, true, 10, 0);
    le.window(30, true, 10, 0);
    le.window(30, true, 0, 0); /* idle */
    CHECK(!le.window(30, true, 10, 0) && le.rate_mbps() == 24);
    CHECK(!le.window(30, true, 10, 0));
    CHECK(le.window(30, true, 10, 0) && le.rate_mbps() == 36); /* the third in a row */
    /* An idle window (no transmissions) is not a clean window. */
    LinkAdapt ld({}, 24);
    for (int i = 0; i < 10; ++i)
      ld.window(30, true, 0, 0);
    CHECK(ld.rate_mbps() == 24);
  }

  if (failures == 0)
    std::printf("cell_m2_selftest: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
