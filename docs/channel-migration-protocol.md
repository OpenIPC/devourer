# Channel-migration protocol

The coordinated, authenticated whole-link channel move between the ground
receiver and the drone video transmitter — the actuation layer under the
advisory engine (`docs/adaptive-channel-migration.md`). A migration is a rare,
deliberate event, not a per-round hop (that is FHSS, `docs/fhss.md`).

## Authority

Ground proposes; the **drone is the final schedule authority**. It validates
the target, fixes an absolute activation instant on its own TSF, and — the
load-bearing rule — **arms that activation only after it has seen the ground
echo its nonce** (the commit-ack). A commit the current ground never
acknowledged is never activated, so the drone can never strand itself on a
channel the ground is not following. The ground, symmetrically, never
unilaterally declares success: it **follows the drone's authoritative
STATUS**, reaching a channel only when the drone reports it settled there.
Those two rules are what make every failure-matrix row converge.

```
STABLE(old) -> PROPOSED -> COMMITTED -> SWITCHING -> VERIFYING -> CONFIRMED
                              (drone)                  (ground)
            \-> (drone) VALIDATING        \-> ROLLBACK / RECOVERY
```

## Wire format (`src/chanmig/MigWire.h`)

Byte-packed little-endian, 8-byte header (`"CM"` magic, version, type,
link-id) + a per-type body + a trailing 8-byte **SipHash-2-4 MAC** over
everything preceding it. The control key is derived from `DEVOURER_MIG_KEY`
(the existing hop-seed parser), domain-separated from any schedule/link key.
Every message is ≤ 96 bytes — ~120 µs of 6M air. Seven types: SwitchProposal,
SwitchCommit, SwitchStatus, SwitchConfirm, SwitchAbort, MigMarker,
TargetValidation.

Anti-replay is structural: epochs are random per process start and **never
persisted**, so a restart orphans every in-flight generation for free and
there is no persisted in-flight state at all (a boot never strands the next
run). A receiver adopts a peer epoch only from a PROPOSAL/STATUS, tracks the
max generation (a fresh epoch resets the ceiling), and binds COMMIT/CONFIRM to
the nonce it issued — so a replayed old exchange can never be acked by the
current peer.

The wire format and key derivation are pinned by known-answer vectors
(`ctest chanmig_wire_kat`): exact bytes + MAC per message, a byte-flip tamper
sweep (every flip breaks auth), truncation, and the replay-window rules.

## Transport

Control frames are their **own 802.11 frames** (canonical-SA probe-req header +
the `CM` magic + the MigWire body), never bytes buried in the video/FEC PSDU.
The `examples/chanmig` demo (`--role ground|drone`) runs the adapter in duplex
(RX video + TX control on one claimed handle; J1/J3 are GO), stamps `tx_tsf` at
send, MACs, and transmits. Ground proposals ride unicast with `tx.report` for
MAC-layer delivery evidence (a host write is never treated as peer receipt).

## Activation clock

The drone fixes `activate_tsf` once, on its own TSF, at commit creation;
`tx_tsf` is re-stamped per repeat. The ground schedules its retune from the
**relative offset** `activate_tsf − commit.tx_tsf` (both drone-clock, so the
difference is a pure duration) — no clock translation needed, error is one
frame's one-way latency, absorbed by the lead + verify window.
`src/chanmig/MigClock.h` (`PeerClock`) additionally fits the drone↔ground TSF
via the existing `TsfSync` and rings the residuals, so the activation **guard**
is composed from the measured residual p99 + TX-drain p99.9 + worst retune + a
margin — never a fixed guess. `tests/chanmig_clock_bench.sh` measures the real
numbers on-air.

## State machines (`MigProposer.h`, `MigResponder.h`)

Pure classes: inputs are decoded MigMsgs + local observations + monotonic
`now` (+ the drone's own TSF, needed once to fix the activation); outputs are
`MigAction`s the demo executes. The demo owns all crypto and I/O.

### Convergence invariants

- **I1** neither endpoint retunes before adopting a nonce+generation-matched,
  acknowledged commit.
- **I2** every post-activation path has a bounded route back to a channel in
  the recovery scan set `{old, new, rescue}`.
- **I3** the ground follows the drone's authoritative STATUS; the drone rolls
  back to source if unconfirmed by its deadline, and orphans its migration
  (rolling back to source if already moved) on a ground-epoch change.
- **I4** random boot epochs orphan in-flight generations on any restart.
- **I5** MAC + nonce binding ⇒ a replayed/forged exchange never reaches an ack.

### Failure matrix

`ctest chanmig_proto_matrix` couples the two machines through a
drop/duplicate/reorder/delay LinkSim and drives 14 rows — drop each message
type (incl. the last commit pre-activation), duplicate/reorder/replay, restart
either endpoint mid-migration, silent/illegal target, drain stall,
control-path loss, delayed old-channel frames — plus an exhaustive
drop-every-single-message sweep. Every case converges both endpoints to a
shared channel in `{old, new, rescue}` within a bounded time; **none produces
a lasting split-brain**.

## Validation

- `ctest`: `chanmig_wire_kat`, `chanmig_proto_matrix`, `chanmig_clock_math` —
  the exhaustive gate. The 14-row failure matrix + drop-every-message sweep
  prove convergence without split-brain across every message loss, duplication,
  reorder, restart, silent/illegal target, drain stall, and control-path loss.
- On-air: the wire codec is verified end-to-end on hardware — a proposal
  transmitted by the ground decodes and authenticates on the drone. Two bugs
  the on-air path surfaced and that the headless tests now pin: the RX frame
  carries a **trailing FCS** after the message (so `mig_decode` locates the MAC
  after the fixed body and ignores the trailer — every on-air frame read
  `bad_mac` before this), and the demo drives the single-threaded machines from
  the RX callback + tick loop + operator thread, so every entry point takes
  `g_sm_mu` (a lost commit was the tell).
- `tests/chanmig_bench.sh` / `chanmig_endurance.sh` — first-light + the
  ≥1,000-cycle acceptance. Both need a **decoupled** link: the two adapters
  must be far enough apart (or attenuated) that the RX front end is not
  saturated (`link.health` HEALTHY, not SATURATED), and rtw88/rtw89 must be
  unbound from both dongles (they auto-probe on every USB enumeration — the
  scripts do this, but a bare `rxdemo` run must too). On the ~30 cm bench the
  primary link runs 90 %+ lost and the reverse control path is intermittent, so
  the endurance is not meaningful there; run it on a spaced/attenuated pair.

## Autonomous policy (`src/chanmig/MigGate.h`)

`DEVOURER_MIG_MODE=off|advisory|manual|automatic` (default advisory) gates the
advisory engine's recommendation into the protocol. The gate is pure and
deterministic (no clock reads, no RNG — the same input trace replays to the
same decisions, `ctest chanmig_gate_policy` + `ChanMigReplay`), and it
proposes only when a conjunction holds: a genuine channel-attributable
impairment (not weak-signal / saturation / broad-degradation — those map to
`FAULT_*` holds), a sufficiently-confident recommendation, healthy time-synced
telemetry and a fresh scout, a verified rescue channel and enough control-link
margin to complete the exchange, and no anti-oscillation bar tripped
(`COOLDOWN`, `RESIDENCY`, `MOVE_CAP`, per-channel `HOLDDOWN_CHANNEL` with
exponential backoff after a rollback). Migrations are exceptional; stable
single-channel video is the normal state.

Anti-oscillation state is carried across decisions and kept honest by
`mig_gate_on_confirmed` / `mig_gate_on_rolledback` (residency start, move
count, rollback streak → holddown backoff). A material change in the frozen
evidence while a proposal is in flight aborts it for a rescore
(`AbortInFlight`); a probation window after a move rolls back on poor
destination delivery (`ProposeRollback`). Operator controls (stdin, echoed as
`migrate.op`): `mode`, `approve-next`, `inhibit <s>`, `pin <chan>|none`,
`rollback`, `rescue`, `status`/`why`. `DEVOURER_MIG_SHADOW` makes the gate
decide-but-not-actuate (the on-air ladder's first rung, quantifying false
moves without touching the live link — `tests/chanmig_gate_onair.sh`).
