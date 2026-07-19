# Adaptive channel migration — scout and scoring

Slow, evidence-driven channel migration for a fixed-channel FPV link — the
complement to FHSS (`docs/fhss.md`): instead of hopping every slot, the link
normally sits on one channel and moves rarely, deliberately, and only when a
second radio has proven a better home exists. This document covers the sensing
and advisory layers; the coordinated switch protocol is
`docs/channel-migration-protocol.md`.

```text
video TX ───────────────► primary ground RX (fixed channel, delivery oracle)
                                  │ stdout → primary.jsonl
scout adapter (chanscout) ────────┴─► survey.dwell / channel.* JSONL
```

Two adapters, two processes, JSONL files as the only contract. The primary
receiver is the unmodified production RX path — a scout bug can never take
down video — and the scout structurally cannot command anything: it retunes
only itself, and the advisory engine's output is an event stream.

## The scout (`chanscout`)

`chanscout` sweeps an immutable per-process candidate plan on a second
adapter. Grammar (`DEVOURER_SCOUT_PLAN`):

```
tok[,tok...]     tok = <primary>[/<width>[u|l]][:<flags>]
                 width 5|10|20|40|80; u/l = 40 MHz secondary above/below
                 flags: b=preferred backup, p=no-IR (receive-only), d=DFS
e.g.  DEVOURER_SCOUT_PLAN="104/40l:b,36/80,132/20:p"
```

Malformed or off-grid tokens abort startup — a silently missing migration
candidate is a field hazard. On the 5 GHz grid the 40 MHz pairing and 80 MHz
quad position derive from the primary (an explicit contradicting suffix is an
error); a 2.4 GHz 40 MHz pair is genuinely ambiguous and requires `u`/`l`.
Validation is *grid geometry only* — devourer has no regulatory database, the
caller owns compliance, and `no_ir`/`dfs` are attributes carried through to
policy (a `no_ir` candidate can be surveyed but never recommended).

Wide candidates are surveyed as their 20 MHz constituent bins so every dwell
rides the cheap `FastRetune` path (0.5–3.5 ms); a candidate is later judged by
its **worst** bin — a 40/80 MHz channel is only clean if every bin is clean.
`DEVOURER_SCOUT_FULLWIDTH_MS` optionally adds a periodic real-width
verification dwell through the full `SetMonitorChannel` gate (the next bin
dwell restores 20 MHz through the full gate too — `FastRetune` is
same-width).

### Dwell hygiene

The chip's FA/CCA counters are delta-on-read (`src/RxSense.h`), so a naive
sweep charges each bin with the retune and settle of its own start plus
frames still in the USB pipeline from the previous channel. Each chanscout
dwell therefore runs:

```
FastRetune → settle sleep → DISCARD BARRIER → observe sleep → read + emit
                            (throwaway GetRxEnergy() resets the hardware
                             deltas; the frame aggregate is drained)
```

The emitted counters span exactly `observe_ms` on exactly that bin. The NHM
histogram is a synchronous ~2 ms point sample at dwell end, not a span.

### The survey record

One dwell = one versioned `survey.dwell` event (schema owned by
`src/chanmig/SurveyJsonl.h`, emit and parse in one header so producer and
consumers cannot drift):

```json
{"ev":"survey.dwell","v":1,"seq":4,"chan":"5:44/20","round":1,
 "plan":"0x73ecfa0a","start_ms":187261192,"end_ms":187261356,
 "retune_us":3520,"settle_ms":30,"observe_ms":115,
 "cca_ofdm":7,"cca_cck":0,"fa_ofdm":3,"fa_cck":0,"igi":52,
 "nhm_busy":100,"nhm_peak":2,"nhm_dur":501,"nhm":[0,0,255,0,0,0,0,0,0,0,0,0],
 "frames":5,"rssi_mean":43,"rssi_max":51,"snr_mean":43,"snr_min":41,
 "evm_mean":-51,"dvr_frames":0,"dvr_air_us":0,"oth_air_us":533,
 "flags":0,"scout_id":"0xe870b9dd","agen":2}
```

Nullable convention as in `rx.energy`: a counter the generation doesn't
expose is JSON `null`, never a fake zero. `flags` is the validity bitmask
(truncated / retune-failed / read-failed / counter-suspect / nhm-missing /
full-width) — a record that should not rank carries the reason it shouldn't.

`dvr_air_us` vs `oth_air_us` split *decoded* occupancy by the canonical
devourer SA (`57:42:75:05:d6:00`): wanted-video airtime is recognized and
reported separately, so the active channel being busy with our own signal can
never read as interference. Undecodable energy appears only in FA/CCA/NHM —
the scoring layer treats the airtime numbers as a split of explained
occupancy, never as total occupancy.

`scout_id` is the calibration-domain key (FNV over usb id + topology + chip):
raw energy units are comparable only within one scout, and the aggregator
resets rather than mixing domains.

### Scheduling

`src/chanmig/ScanPlan.h` (pure, selftested): every bin carries a revisit
deadline — backup-candidate bins the fast cadence
(`DEVOURER_SCOUT_BACKUP_MS`), the rest the background cadence
(`DEVOURER_SCOUT_BG_MS`) — and the scout always executes the most-overdue
bin, so it scans at 100 % duty while backup bins are simply visited more
often. The round counter advances only on full coverage of every bin: backup
favoritism can never starve the full rescan a scoring policy's min-rounds
gate requires. Failed dwells push their bin back `fail_retry` and the scout
continues with the rest.

### Health

`scout.health` events (state transitions only): `retune_fail` (5 consecutive
= degraded, 15 = wedged + exit 3 for the supervisor), `energy_read_fail`,
`rx_stalled` (silence after having heard traffic), `usb_congested`
(retune-latency p95 drift proxy), `thermal` (where the RF 0x42 meter exists),
`stale_survey` (a bin unobserved past `DEVOURER_SCOUT_MAX_AGE_MS`).
"Regulatory-set change" reduces to the plan hash: the plan is immutable per
process, `scout.plan`/every record carry `plan`, and consumers refuse to fold
records bearing a different hash.

### Per-generation sensor caveats

| Sensor | J1 | J2 | J3 | Kestrel |
|---|---|---|---|---|
| FA/CCA deltas | ✓ | ✓ | ✓ | ✓ |
| IGI (live) | ✓ | ✓ | static hint | ✓ |
| NHM histogram | ✓ | ✓ | ✓ | ✓ (abs floor 2.4 GHz-only) |
| Active abs noise floor | 8812A CAL | live | — | — |
| FastRetune (cached) | 1.6 ms | 0.6–2.5 ms | ~2 ms | ✓ |

On the 8822BU the quiet-channel NHM noise floor sits in bucket ~2, so the
`nhm_busy` percentage (samples above bucket 0) reads 100 even on a clean
channel — occupancy scoring keys on CCA/FA rates and the NHM peak-index
shift, with per-scout rank normalization absorbing constant offsets.

## Evidence and scoring

`src/chanmig/EvidenceStore.h` folds records into per-bin rings behind a trust
boundary: wrong plan hash, failure flags, implausible counter deltas (a
wrapped hardware counter reads orders of magnitude beyond the ~1000
events/ms physical ceiling), stale records, unknown bins, and
foreign-`scout_id` records are all rejected by counted reason. Every accepted
fold bumps a global evidence generation; decisions cite the generation they
were computed at, so any output is reproducible from the record stream.

## The recommendation engine

`src/chanmig/ChannelScore.{h,cpp}` (`RecommendEngine`, pure and selftested) is
the advisory brain. Run it in-process with `DEVOURER_SCOUT_ADVISE=1` plus
`DEVOURER_SCOUT_ACTIVE=<token>` (the live video channel) and
`DEVOURER_SCOUT_PRIMARY_FEED=<path>` (the primary receiver's JSONL, tail-
followed as a plain append-only file — never a FIFO, so a wedged scout can
never stall video). It emits `channel.recommend` / `channel.hold` /
`channel.ranking` and structurally cannot retune anything.

The recommendation law has two independent legs, kept in separate unit worlds
so nothing compares raw energy across the scout/primary boundary:

- **leg 1 (the active channel)** — the primary receiver's *delivery* is
  authoritative (scout energy on the live channel is confounded by the video's
  own airtime). `src/chanmig/ActiveLink.h` classifies each primary window as
  channel-impaired vs a fault domain that migration cannot fix: `WeakSignal`
  (range — add power), `Saturation` (near-field overload — reduce power),
  `TelemetryDown`. A recommendation needs *persistent* channel-attributable
  impairment (`impaired_windows_min` consecutive windows), reusing LinkHealth's
  EVM/RSSI verdict as the domain discriminator.
- **leg 2 (a candidate)** — the scout's occupancy is authoritative. A candidate
  must be sufficiently observed (`min_rounds` distinct rounds, `min_clean_obs_ms`
  per constituent bin, full-width coverage, fresh within `max_evidence_age_ms`),
  legal (not `no_ir`), and *materially* clean: its occupancy must clear the
  qualify bar by `improvement_margin` (hysteresis between "fine to sit on" and
  "worth escaping to"). Occupancy is worst-bin `oth_air_frac` + a bounded
  false-alarm term (NHM is excluded — its absolute floor is generation-
  dependent); an active-adjacent candidate takes an overlap penalty.

Anti-churn is structural: a broad degradation across most candidates holds
(`HoldBroadDegradation` — the interference isn't this channel's); a cooldown
rate-limits recommendations so two near-equal candidates cannot flip-flop; and
ties break deterministically by RF key. Every decision cites the evidence
generation, plan hash, and policy hash, so it is reproducible from the log.

Thresholds live in `PolicyConfig`, loadable from a bare `key value` file via
`DEVOURER_SCOUT_POLICY` (the one deliberate departure from the env-var culture:
the exact policy artifact must be versionable, hashable, and identical across
the C++ engine, the ctest scenarios, and the replay tool). The `policy_hash`
in every event ties a decision to its threshold set.

`tools/chanmig_dash.py` renders active-link health, the candidate ranking,
evidence age, and the counterfactual reason a move is / isn't recommended —
entirely from the two JSONL files. That it can is the acceptance proof that
every output is explainable from logged score components.

`tests/chanmig_replay.py --confusion` replays a captured advise-mode log and
tallies the engine's own decisions against an independent primary-impairment
label (it never re-implements the policy, so a python shadow cannot drift from
the shipping engine). The single-trace confusion matrix reports the false-move
rate and possible misses; whether a recommended destination actually delivered
better needs the manual-move validation on a *held-out* run (never tune and
score thresholds on the same captures).

## Validation

- `ctest`: `chan_def_grid` (grid legality + grammar), `scan_plan_policy`
  (coverage / cadence / starvation / determinism), `survey_aggregate`
  (fold path: counter wrap, staleness, width coverage, mixed domains +
  the JSONL round-trip).
- `tests/chanscout_bench.sh` — two-adapter smoke: full-plan coverage, dwell
  hygiene, and the attribution split (a canonical-SA flood on one candidate
  must land in `dvr_air_us` and out-rank the quiet bins).
- `tests/chanscout_stress.sh` — ≥10,000 dwell retunes in one run; asserts
  zero retune failures, no retune-latency drift (p95 tail vs head), an
  intact record stream, and a post-run `doctor` grade.
- `tests/chanmig_soak.sh` + `tests/chanmig_soak_analyze.py` — the
  three-adapter A/B: ≥1 h video + scanning vs a no-scout control arm;
  primary delivery (rx.txhit rate, seq-gap loss) must be statistically
  unchanged.
- `ctest chan_score_policy` — the engine's two-leg policy over the ten offline
  scenarios (persistent interferer → recommend; wanted-video-dominates-CCA →
  hold-healthy; weak signal → not-congestion; saturation → reduce-power;
  adjacent/overlap ordering; all-degrade → broad; burst → no-flap; stale;
  cooldown; scout-down) plus age/tie-break/generation/policy-hash edges.
- `tests/chanmig_replay.py --confusion` — shadow-mode confusion matrix over a
  captured advise run + primary log.
