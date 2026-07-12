# Unicast ACK + TxReport capability matrix (M0 contract 3)

A scheduled MAC's reliability layer (per-UE delivery detection, HARQ-style
retransmission, link adaptation) rests on two capabilities per generation:
an **injected unicast descriptor solicits a hardware ACK** (with autonomous
MAC retransmission until it arrives), and **`TxReport` reports the per-frame
ACK / no-ACK outcome** to the host. This measures both, per generation, plus
the report delivery rate — `tests/ack_txreport_matrix.sh` /
`tests/ack_txreport_analyze.py` (`--selftest` covers the verdict logic).

## Method

Fixed hardware-ACK responder (`SetAckResponder` on a second adapter); per TX
generation three phases: **on** (responder armed with MAC1, unicast QoS-Data
to MAC1 → expect ~100% `tx.report ok`, retries ~0), **retarget** (responder
re-armed to a different MAC2, TX to MAC2 → proves RA and responder MAC are
arbitrary), **off** (no responder → expect 0% ok, retries pinned at the
descriptor limit: the no-ACK outcome must be *visible*, per frame).
`report_coverage` = reports / frames sent (`tx.stats.submitted`); HalMAC adds
SW_DEFINE tag-echo gap counting.

TX sessions run `DEVOURER_TX_WITH_RX=thread`: CCX reports arrive on the C2H
RX path, so J1/J2 TX-only sessions never see them (measured: J2 TX-only = 0
reports; only J3 drains C2H off its coex runtime without an RX loop). A
scheduled MAC runs TX+RX anyway, so this is the relevant session shape.

## Measured (ch36, MCS3, unicast TA, 8814AU responder, ~8 s/phase)

| TX generation | on: ACK rate / mean retries | retarget | off: retries pinned | report coverage | tag gaps |
|---|---|---|---|---|---|
| Jaguar1 8812AU | 1.00 / 0.34 | 1.00 / 0.25 | yes (12) | 1.00 | n/a (8812 fmt) |
| Jaguar2 8812BU | 0.91 / 2.1 (run-to-run 0.12–0.91) | 0.64 / 5.3 | yes (12) | 0.86 | 0 |
| Jaguar3 8822CU | 1.00 / 0.24 | 1.00 / 0.13 | yes (12) | 0.96 | 0 |

Responder-side capability (same bench, J3 TX as the reference soliciting
station): **8814AU** closes the loop at retries ~0.1 (the bench responder of
choice); **8812AU** works but degraded (97% delivery at ~7 mean retries —
its SIFS ACKs only land intermittently); **8821AU never closed the loop**
(TX retries stayed pinned with it armed); the 8812BU responder was
previously proven (`tests/ack_responder_check.sh`).

## The contract

1. **Per-frame delivery detection is GO on all three generations**: the OFF
   phase pins retries at the descriptor limit with `state=1` on every report —
   a no-ACK outcome is unambiguously visible per frame, which is all M2's
   software retransmission logic needs. Report coverage 86–100% with zero
   HalMAC tag gaps (interior losses); the reliability layer must tolerate a
   ~5–15% report-less frame tail (treat missing report as "unknown", not
   "delivered").
2. **Closed-loop hardware ACK + autonomous retry is GO on Jaguar1 and
   Jaguar3** (100% delivery, retries ≈ 0.2–0.3) including retargeting an
   arbitrary UE MAC mid-session (re-arm `SetAckResponder`, change the
   descriptor RA — both fully dynamic).
3. **Jaguar2 as the soliciting TX is MARGINAL on this bench**: ACK closure
   varied 12–91% across identical runs (mean retries 2–11) against both
   8814AU and 8812AU responders, and its TX pace in the TX+RX-thread shape is
   ~24 ms/frame regardless of the requested gap (~37 fps vs J1/J3's ~150).
   As a *responder* J2 is proven good. An M1 cell should prefer J1/J3 (or the
   8821CE) for the DU role until the J2 TX anomaly is root-caused; treat it
   as open, not as silicon folklore.
4. Bench quirk recorded: the J3 report's `missed` field reads a constant 4×
   the report count while tag continuity shows zero loss — the 8822C
   `missed_rpt` offset likely decodes something else; trust `tag` gaps on J3.
