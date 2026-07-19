# Drone-side target validation

Optional drone-side validation of a ground-proposed migration destination
before the drone commits (issue #280). Ground evidence stays authoritative for
downlink video quality; the drone's checks are a conservative veto/diagnostic
for a locally busy, unsupported, or unsafe destination — never an independent
channel-selection state machine, and never an override of good ground delivery
evidence merely because the drone's local energy differs.

## Why this is optional

The ground receiver is the endpoint whose video delivery matters, and
interference is asymmetric — a channel clean at the drone can be bad at ground
and vice versa. So drone sensing must not veto a move the ground needs merely
because the drone's local occupancy is unremarkable. The genuinely useful drone
checks are narrow: is the target legal and supported by the drone's actual
radio, and is it so locally saturated that TX would immediately fail.

## Variants (and where each lives)

- **A — validation-only baseline (default, the product answer).**
  `MigResponder::validate_target`: legality (grid), band/width vs `MigCaps`, the
  operator allowed-list, and no-IR exclusion. Synchronous, zero cost. This is
  the recommended production behaviour — an illegal/unsupported target is
  always rejected before commit, and that is the whole benefit worth its cost.
- **B — single-radio pre-commit probe (research, `DEVOURER_MIG_PROBE=1`).**
  A pure, abort-safe sub-sequence in the responder's `VALIDATING` state: pause
  the pump → drain → retune to the target → settle → sample `GetRxEnergy` →
  **always return to source before deciding** → resume → veto iff the measured
  occupancy exceeds `DEVOURER_MIG_VETO_BUSY` (0.6). Every retune has a failure
  edge (return to source; if that fails, rescue + reject), so a failed probe
  can never strand the drone off-source. The cost is a real video outage — two
  retunes plus the settle+dwell — measured by `tests/chanmig_probe_outage.sh`.
- **C — post-switch probation.** No new machinery: the #279 gate's probation
  window already samples destination delivery after a move and rolls back
  (`ProposeRollback`) on poor delivery, without a second rollback authority.
- **D — dedicated airborne scout.** Not wired as a product path — it would ride
  the stage-1 `chanscout` on a second airborne adapter, and the mass / power /
  USB / RF-self-interference cost is not assumed to transfer from the ground
  dual-radio result. Bench-only.

The drone reports its result as a `TargetValidation` control frame; the ground
logs it (`migrate.validation`) and flags a `migrate.disagree` when the drone
vetoes a target the ground's scout called clean — **reporting only.** The
ground never applies a mask or target from it: ground authority for downlink
quality, drone authority only to veto a locally-unsafe destination. `unknown`
(no valid energy) does not manufacture confidence — it follows
`DEVOURER_MIG_UNKNOWN` policy (default: proceed on ground authority).

## The decision

**Variant A is the recommended production result.** It is free, it prevents
the one class of failure the drone is uniquely positioned to catch (an illegal
or radio-unsupported target), and it never vetoes a beneficial move on
asymmetric local energy. Variant B buys a marginal reduction in bad moves at a
real, measured video-outage cost every probe, and only helps when the target is
bad *at the drone* — precisely the case ground delivery evidence and the #279
probation (variant C) already cover after the fact, without the outage. A no-go
on airborne RF sensing beyond A + C is an acceptable — and, on the measured
cost, the preferred — outcome. B and D remain in-tree as opt-in research so the
trade can be re-measured on airframes where the outage budget differs.

## Validation

- `ctest chanmig_proto_matrix` includes the probe rows: accept → migrates, veto
  → stays on source, probe-retune-failure → safe return, no split-brain.
- `tests/chanmig_probe_outage.sh` measures the per-probe video outage
  (ground-side rx.frame gap p50/p99) and the asymmetric-interference cases (a
  B210 parked at the drone-only vs ground-only) → false-veto / prevented-failure
  counts per variant.
